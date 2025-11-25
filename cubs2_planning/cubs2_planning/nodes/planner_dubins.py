#!/usr/bin/env python3
import numpy as np
import rclpy
from cubs2_planning.planner.dubins import DubinsPathType, derive_dubins
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from racecourse_description.factory import MarkerFactory
from racecourse_description.loader import RacecourseLoader
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


class DubinsGatePlannerNode(Node):
    def __init__(self):
        super().__init__("dubins_gate_planner")

        self.declare_parameter(
            "racecourse_yaml", "package://racecourse_description/config/racecourse.yaml"
        )
        self.declare_parameter("turn_radius", 3.0)
        self.declare_parameter("sample_points", 200)
        self.declare_parameter("show_turn_circles", False)
        self.declare_parameter("show_gate_turn_circles", False)
        self.declare_parameter("show_headings", False)
        self.declare_parameter("show_path_marker", False)
        self.declare_parameter("planner.heading_spacing", 20)
        self.declare_parameter("planner.altitude", 2.0)
        self.declare_parameter(
            "planner.gate_sequence", [0, 1, 2, 4, 3, 1, 2, 4, 3, 1, 2, 4, 3, 0]
        )  # Default sequence
        self.declare_parameter("planner.velocity", 5.0)  # m/s - velocity along trajectory
        self.declare_parameter("reference_frame_id", "reference")  # TF frame name

        self.marker_pub = self.create_publisher(MarkerArray, "dubins_trajectory", 10)
        self.path_pub = self.create_publisher(Path, "dubins_path", 10)

        # TF broadcaster for reference trajectory position
        self.tf_broadcaster = TransformBroadcaster(self)

        yaml_path = self.get_parameter("racecourse_yaml").value
        self.racecourse = RacecourseLoader(yaml_path)

        self.get_logger().debug(f"Loaded {len(self.racecourse.gates)} gates")

        self.plan_fn, self.eval_fn = derive_dubins()
        self.segments = self.plan_trajectory()

        # Flatten all trajectory points for time-based lookup
        self.trajectory_points = []
        for seg in self.segments:
            self.trajectory_points.extend(seg["points"])

        # Calculate total arc length and time for each point
        self._compute_trajectory_timing()

        # Record start time
        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(1.0, self.publish_visualization)
        self.tf_timer = self.create_timer(0.02, self.publish_reference_tf)  # 50 Hz TF updates

        self.get_logger().info("started")

    def plan_trajectory(self):
        R = self.get_parameter("turn_radius").value
        n = self.get_parameter("sample_points").value
        gate_sequence = self.get_parameter("planner.gate_sequence").value

        all_gates = self.racecourse.gates

        # Build ordered gate list from sequence
        try:
            ordered_gates = [all_gates[idx] for idx in gate_sequence]
        except IndexError as e:
            self.get_logger().error(f"Invalid gate index in sequence: {e}")
            self.get_logger().error(f"Available gates: 0-{len(all_gates)-1}")
            return []

        self.get_logger().info(f'gate sequence: {" → ".join([g.name for g in ordered_gates])}')

        segments = []

        for i in range(len(ordered_gates) - 1):
            g0, g1 = ordered_gates[i], ordered_gates[i + 1]

            p0 = np.array([g0.center[0], g0.center[1]])
            p1 = np.array([g1.center[0], g1.center[1]])

            cost, ptype, a1, d, a2, tp0, tp1, c0, c1 = self.plan_fn(p0, g0.yaw, p1, g1.yaw, R)

            c0_arr = np.array(c0).flatten()
            c1_arr = np.array(c1).flatten()

            points = []
            for s in np.linspace(0, 1, n):
                x, y, psi = self.eval_fn(
                    s,
                    p0,
                    g0.yaw,
                    a1,
                    d,
                    a2,
                    np.array(tp0).flatten(),
                    np.array(tp1).flatten(),
                    c0_arr,
                    c1_arr,
                    R,
                )
                z = g0.center[2] + (g1.center[2] - g0.center[2]) * s
                points.append({"pos": [float(x), float(y), z], "psi": float(psi)})

            segments.append(
                {
                    "points": points,
                    "centers": [c0_arr, c1_arr],
                    "z": g0.center[0],
                }
            )

            self.get_logger().debug(
                f"{g0.name} → {g1.name}: type={DubinsPathType.name(ptype)}, cost={float(cost):.2f}"
            )

        return segments

    def _compute_trajectory_timing(self):
        """Calculate arc length, time, and heading rate for each trajectory point."""
        if not self.trajectory_points:
            return

        velocity = self.get_parameter("planner.velocity").value
        g = 9.81  # gravity (m/s^2)

        # Add arc length and time to each point
        cumulative_length = 0.0
        self.trajectory_points[0]["arc_length"] = 0.0
        self.trajectory_points[0]["time"] = 0.0
        self.trajectory_points[0]["psi_dot"] = 0.0
        self.trajectory_points[0]["bank"] = 0.0

        for i in range(1, len(self.trajectory_points)):
            p0 = np.array(self.trajectory_points[i - 1]["pos"])
            p1 = np.array(self.trajectory_points[i]["pos"])
            segment_length = np.linalg.norm(p1 - p0)
            cumulative_length += segment_length

            self.trajectory_points[i]["arc_length"] = cumulative_length
            self.trajectory_points[i]["time"] = cumulative_length / velocity

            # Calculate heading rate (psi_dot)
            psi0 = self.trajectory_points[i - 1]["psi"]
            psi1 = self.trajectory_points[i]["psi"]

            # Handle angle wraparound
            dpsi = psi1 - psi0
            while dpsi > np.pi:
                dpsi -= 2 * np.pi
            while dpsi < -np.pi:
                dpsi += 2 * np.pi

            # Time between points
            dt = segment_length / velocity if segment_length > 0 else 1e-6
            psi_dot = dpsi / dt

            # Calculate required bank angle: phi = atan(V * psi_dot / g)
            # For coordinated turn: bank angle relates to turn rate
            # Negative sign because positive yaw rate (left turn) requires negative roll (left bank)
            bank = -np.arctan2(velocity * psi_dot, g)

            self.trajectory_points[i]["psi_dot"] = psi_dot
            self.trajectory_points[i]["bank"] = bank

        self.total_trajectory_time = self.trajectory_points[-1]["time"]
        self.get_logger().info(
            f"trajectory: {cumulative_length:.2f}m, {self.total_trajectory_time:.2f}s @ {velocity:.2f}m/s"
        )

    def publish_reference_tf(self):
        """Publish TF frame at current reference position based on elapsed time."""
        if not self.trajectory_points:
            return

        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        # Loop the trajectory
        trajectory_time = elapsed % self.total_trajectory_time

        # Find the point at this time using binary search or linear interpolation
        ref_point = self._interpolate_trajectory(trajectory_time)

        if ref_point is None:
            return

        # Create and broadcast transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.racecourse.frame_id
        t.child_frame_id = self.get_parameter("reference_frame_id").value

        t.transform.translation.x = ref_point["pos"][0]
        t.transform.translation.y = ref_point["pos"][1]
        t.transform.translation.z = ref_point["pos"][2]

        # Convert orientation (roll, pitch, yaw) to quaternion
        roll = ref_point["bank"]
        pitch = 0.0
        yaw = ref_point["psi"]

        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def _interpolate_trajectory(self, time):
        """Interpolate trajectory point at given time."""
        # Find the two points that bracket this time
        if time <= 0:
            return self.trajectory_points[0]
        if time >= self.total_trajectory_time:
            return self.trajectory_points[-1]

        # Binary search for efficiency
        left, right = 0, len(self.trajectory_points) - 1
        while right - left > 1:
            mid = (left + right) // 2
            if self.trajectory_points[mid]["time"] < time:
                left = mid
            else:
                right = mid

        # Linear interpolation between left and right
        p0 = self.trajectory_points[left]
        p1 = self.trajectory_points[right]

        t0, t1 = p0["time"], p1["time"]
        alpha = (time - t0) / (t1 - t0) if t1 > t0 else 0.0

        # Interpolate position
        pos = [
            p0["pos"][0] + alpha * (p1["pos"][0] - p0["pos"][0]),
            p0["pos"][1] + alpha * (p1["pos"][1] - p0["pos"][1]),
            p0["pos"][2] + alpha * (p1["pos"][2] - p0["pos"][2]),
        ]

        # Interpolate heading (handle wraparound)
        psi0, psi1 = p0["psi"], p1["psi"]
        dpsi = psi1 - psi0
        # Wrap to [-pi, pi]
        while dpsi > np.pi:
            dpsi -= 2 * np.pi
        while dpsi < -np.pi:
            dpsi += 2 * np.pi
        psi = psi0 + alpha * dpsi

        # Interpolate bank angle
        bank = p0["bank"] + alpha * (p1["bank"] - p0["bank"])

        return {"pos": pos, "psi": psi, "bank": bank}

    def publish_visualization(self):
        frame = self.racecourse.frame_id
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        mid = 0

        factory = MarkerFactory(frame)

        for seg_idx, seg in enumerate(self.segments):
            if self.get_parameter("show_path_marker").value:
                mid = self._add_path_marker(markers, factory, stamp, seg, seg_idx, mid)

            if self.get_parameter("show_headings").value:
                mid = self._add_heading_arrows(markers, factory, stamp, seg, mid)

            if self.get_parameter("show_turn_circles").value:
                mid = self._add_segment_turn_circles(markers, factory, stamp, seg, seg_idx, mid)

        if self.get_parameter("show_gate_turn_circles").value:
            mid = self._add_gate_turn_circles(markers, factory, stamp, mid)

        self.marker_pub.publish(markers)
        self._publish_path(frame, stamp)

    def _add_path_marker(self, markers, factory, stamp, seg, seg_idx, mid):
        points = [Point(x=p["pos"][0], y=p["pos"][1], z=p["pos"][2]) for p in seg["points"]]
        m = factory.line_strip(
            marker_id=mid,
            points=points,
            stamp=stamp,
            ns=f"path_{seg_idx}",
            line_width=0.15,
            color=factory.color(0.0, 0.5, 1.0),
        )
        markers.markers.append(m)
        return mid + 1

    def _add_heading_arrows(self, markers, factory, stamp, seg, mid):
        spacing = self.get_parameter("planner.heading_spacing").value
        for i in range(0, len(seg["points"]), spacing):
            pt = seg["points"][i]
            m = factory.arrow_pose(
                marker_id=mid,
                position=pt["pos"],
                heading=pt["psi"],
                stamp=stamp,
                ns="headings",
                length=1.0,
                width=0.15,
                color=factory.color(0.0, 1.0, 0.0, 0.8),
            )
            markers.markers.append(m)
            mid += 1
        return mid

    def _add_segment_turn_circles(self, markers, factory, stamp, seg, seg_idx, mid):
        R = self.get_parameter("turn_radius").value
        for cidx, c in enumerate(seg["centers"]):
            m = factory.circle(
                marker_id=mid,
                center=c[:2],
                radius=R,
                z=seg["z"],
                stamp=stamp,
                ns=f"circle_{seg_idx}_{cidx}",
                line_width=0.1,
                color=factory.color(0.2, 0.2, 0.2, 1.0),
            )
            markers.markers.append(m)
            mid += 1
        return mid

    def _add_gate_turn_circles(self, markers, factory, stamp, mid):
        R = self.get_parameter("turn_radius").value
        for gate in self.racecourse.gates:
            yaw = gate.yaw
            cx_right = gate.center[0] - R * np.sin(yaw)
            cy_right = gate.center[1] + R * np.cos(yaw)
            cx_left = gate.center[0] + R * np.sin(yaw)
            cy_left = gate.center[1] - R * np.cos(yaw)

            for label, cx, cy in [("L", cx_left, cy_left), ("R", cx_right, cy_right)]:
                m = factory.circle(
                    marker_id=mid,
                    center=[cx, cy],
                    radius=R,
                    z=gate.center[2],
                    stamp=stamp,
                    ns=f"gate_circle_{gate.name}_{label}",
                    line_width=0.05,
                    color=factory.color(1.0, 0.5, 0.0, 0.4),
                )
                markers.markers.append(m)
                mid += 1
        return mid

    def _publish_path(self, frame, stamp):
        factory = MarkerFactory(frame)
        path = Path()
        path.header = factory.header(stamp)
        for seg in self.segments:
            for pt in seg["points"]:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose = factory.pose(pt["pos"], pt["psi"])
                path.poses.append(pose)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)

    dubins_planner = DubinsGatePlannerNode()

    rclpy.spin(dubins_planner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dubins_planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
