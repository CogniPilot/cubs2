#!/usr/bin/env python3
# Copyright 2025 CogniPilot Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import copy

from builtin_interfaces.msg import Time
from cubs2_control.closed_loop import closed_loop_sportcub
from cubs2_msgs.msg import AircraftControl
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class SimNode(Node):
    def __init__(self):
        super().__init__(node_name='sim_node')

        # Note: use_sim_time should NOT be set to true for the clock source
        # node

        # Default 10ms, but GUI can override
        self.declare_parameter('dt', 0.01)
        self.dt = float(self.get_parameter('dt').value)

        # Toggle force/moment visualization
        self.declare_parameter('show_forces', True)
        self.show_forces = bool(self.get_parameter('show_forces').value)

        # Trim parameters from config file
        self.declare_parameter('controller.trim.aileron', 0.0)
        self.declare_parameter('controller.trim.elevator', 0.0)
        self.declare_parameter('controller.trim.rudder', 0.0)

        # Initial pose parameters (ENU frame, yaw about +z)
        self.declare_parameter('initial_position.x', 0.0)
        self.declare_parameter('initial_position.y', 0.0)
        self.declare_parameter('initial_position.z', 0.1)
        self.declare_parameter('initial_yaw', 0.0)  # radians

        self.initial_x = float(self.get_parameter('initial_position.x').value)
        self.initial_y = float(self.get_parameter('initial_position.y').value)
        self.initial_z = float(self.get_parameter('initial_position.z').value)
        self.initial_yaw = float(self.get_parameter('initial_yaw').value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sim_time_publisher = self.create_publisher(Clock, '/clock', 10)
        self.sim_time = 0.0

        # Force/moment marker publisher
        if self.show_forces:
            self.force_marker_publisher = self.create_publisher(
                MarkerArray, '/vehicle/force_markers', 10
            )

        # Pose and velocity publishers for HUD panel
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/sportcub/pose', 10)
        self.velocity_publisher = self.create_publisher(
            TwistStamped, '/sportcub/velocity', 10)

        self.reset_subscription = self.create_subscription(
            Empty, '/reset', self.reset_topic_callback, 10
        )
        self.pause_subscription = self.create_subscription(
            Empty, '/pause', self.pause_topic_callback, 10
        )
        self.sim_speed = 1.0
        self.speed_subscription = self.create_subscription(
            Float64, '/set_speed', self.speed_topic_callback, 10
        )
        self.dt_subscription = self.create_subscription(
            Float64, '/set_dt', self.dt_topic_callback, 10
        )
        self.paused_publisher = self.create_publisher(Bool, '/sat/paused', 10)
        self.resume()

        # Initialize closed-loop model (composed aircraft + controller)
        self.model = closed_loop_sportcub()

        # TEMPORARY: Also create plant-only model for pure manual mode demo
        from cubs2_dynamics.sportcub import sportcub

        self.cl_model = sportcub()

        # State and inputs for composed model
        # For composed models, x0 is structured (x0.plant, x0.controller)
        # while x0_composed is the flat vector for integration
        # Structured state with submodel access
        self.x = copy.deepcopy(self.model.x0)
        self.u = copy.deepcopy(self.model.u0)  # External inputs
        # Parameters (empty for this model)
        self.p = copy.deepcopy(self.model.p0)

        # Apply trim parameters from config to controller submodel
        self.model._submodels['controller'].p0.trim_aileron = float(
            self.get_parameter('controller.trim.aileron').value
        )
        self.model._submodels['controller'].p0.trim_elevator = float(
            self.get_parameter('controller.trim.elevator').value
        )
        self.model._submodels['controller'].p0.trim_rudder = float(
            self.get_parameter('controller.trim.rudder').value
        )

        # Plant-only state/inputs
        self.x_cl = copy.deepcopy(self.cl_model.x0)
        self.u_cl = copy.deepcopy(self.cl_model.u0)
        self.p_cl = copy.deepcopy(self.cl_model.p0)

        # Apply initial pose to both composed and plant-only models
        self._apply_initial_pose()

        # Manual control inputs from joystick/gamepad
        self.u.ail_manual = 0.0
        self.u.elev_manual = 0.0
        self.u.rud_manual = 0.0
        self.u.thr_manual = 0.0
        self.u.mode = 0.0  # 0=manual, 1=stabilized

        # Ready-for-takeoff start (taildragger sitting on its main wheels, no initial bounce).
        # World frame ENU (z up), ground plane z=0. Main wheels located at z=-0.1 in body frame, so
        # choosing CG z ≈ 0.1 puts both main wheels exactly on the ground with zero penetration.
        # Tail wheel in current model sits at z=0 (should be below CG
        # physically); this means tail wheel is raised.
        # For a simple takeoff run this is acceptable; adjust dynamics wheel
        # geometry later for visual accuracy.

        # Set initial plant state using structured access
        # (position and yaw are handled inside _apply_initial_pose)

        # Subscribe to control messages (published by virtual joystick,
        # gamepad, keyboard)
        self.control_subscription = self.create_subscription(
            AircraftControl, '/control', self.control_callback, 10
        )

        # Subscribe to control mode (0=manual, 1=stabilized)
        self.mode_subscription = self.create_subscription(
            Float32, '/control_mode', self.mode_callback, 10
        )

        # Joint state publisher for control surface and propeller animation
        self.joint_state_publisher = self.create_publisher(
            JointState, '/vehicle/joint_states', 10)

        # Propeller rotation state
        self.propeller_angle = 0.0

        # Track last outputs for force visualization
        self.last_outputs = {}

        # Track actual control commands sent to aircraft (for visualization)
        self.ail_cmd = 0.0
        self.elev_cmd = 0.0
        self.rud_cmd = 0.0
        self.thr_cmd = 0.0

    def _apply_initial_pose(self):
        """Apply configured initial position and yaw to aircraft state."""
        # Convert yaw to quaternion (rotation about z-axis in ENU frame)
        # Quaternion stored as [w, x, y, z]
        half_yaw = self.initial_yaw / 2.0
        qw = np.cos(half_yaw)
        qx = 0.0
        qy = 0.0
        qz = np.sin(half_yaw)

        # Apply to composed model (plant portion)
        self.x.plant.p[0] = self.initial_x
        self.x.plant.p[1] = self.initial_y
        self.x.plant.p[2] = self.initial_z
        self.x.plant.r[0] = qw
        self.x.plant.r[1] = qx
        self.x.plant.r[2] = qy
        self.x.plant.r[3] = qz

        # Apply to plant-only model
        self.x_cl.p[0] = self.initial_x
        self.x_cl.p[1] = self.initial_y
        self.x_cl.p[2] = self.initial_z
        self.x_cl.r[0] = qw
        self.x_cl.r[1] = qx
        self.x_cl.r[2] = qy
        self.x_cl.r[3] = qz

    def get_sim_time_msg(self):
        sim_time_msg = Clock()
        sim_time_msg.clock = Time()
        sim_time_msg.clock.sec = int(self.sim_time)
        sim_time_msg.clock.nanosec = int(
            (self.sim_time - int(self.sim_time)) * 1e9)
        return sim_time_msg

    def control_callback(self, msg: AircraftControl):
        """Handle AircraftControl messages."""
        self.u.ail_manual = float(msg.aileron)
        self.u.elev_manual = float(msg.elevator)
        self.u.thr_manual = float(msg.throttle)
        self.u.rud_manual = float(msg.rudder)

        # TEMPORARY: Also update plant-only inputs for demo
        self.u_cl.ail = float(msg.aileron)
        self.u_cl.elev = float(msg.elevator)
        self.u_cl.thr = float(msg.throttle)
        self.u_cl.rud = float(msg.rudder)

        # Debug: Log throttle input
        if abs(self.u.thr_manual) > 0.01:
            self.get_logger().info(
                f'Control input - throttle: {self.u.thr_manual:.3f}, '
                f'ail: {self.u.ail_manual:.3f}, elev: {self.u.elev_manual:.3f}'
            )

    def mode_callback(self, msg: Float32):
        """Handle control mode messages."""
        self.u.mode = float(msg.data)

    def publish_joint_states(self):
        """Publish joint states for control surfaces and propeller animation."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # Use tracked control commands (updated in step_simulation)
        # Control surface deflections (convert from normalized [-1, 1] to radians)
        # Positive aileron command -> left aileron down, right aileron up
        aileron_deflection = self.ail_cmd * 0.4  # Max ~23 degrees
        elevator_deflection = self.elev_cmd * 0.4  # Max ~23 degrees
        rudder_deflection = self.rud_cmd * 0.4  # Max ~23 degrees

        # Propeller rotation based on throttle
        # Angular velocity proportional to throttle (rad/s)
        propeller_rpm = self.thr_cmd * 2000.0  # Max 2000 RPM at full throttle
        propeller_omega = propeller_rpm * 2.0 * np.pi / 60.0  # Convert to rad/s
        self.propeller_angle += propeller_omega * self.dt
        self.propeller_angle = self.propeller_angle % (
            2.0 * np.pi)  # Wrap to [0, 2π]

        # Joint names and positions
        joint_state.name = [
            'propeller_joint',
            'left_aileron_joint',
            'right_aileron_joint',
            'elevator_joint',
            'rudder_joint',
        ]

        joint_state.position = [
            self.propeller_angle,
            # Left aileron (negative for positive roll command)
            -aileron_deflection,
            # Right aileron (positive for positive roll command)
            aileron_deflection,
            elevator_deflection,  # Elevator
            rudder_deflection,  # Rudder
        ]

        # Set velocities for propeller
        joint_state.velocity = [propeller_omega, 0.0, 0.0, 0.0, 0.0]

        self.joint_state_publisher.publish(joint_state)

    def publish_force_moment_markers(self, outputs):
        """Publish MarkerArray for force and moment visualization.

        Creates arrow markers for aerodynamic, thrust, and weight forces.
        Creates curved arrow/arc markers for moments to distinguish from forces.
        Args:
            outputs: SportCubOutputs dataclass instance
        """
        try:
            # Extract force and moment vectors (body frame) from dataclass
            FA_b = np.array(outputs.FA_b).flatten()
            FT_b = np.array(outputs.FT_b).flatten()
            FW_b = np.array(outputs.FW_b).flatten()
            MA_b = np.array(outputs.MA_b).flatten()
            MT_b = np.array(outputs.MT_b).flatten()
            MW_b = np.array(outputs.MW_b).flatten()

            marker_array = MarkerArray()
            # Use timestamp of 0 for frame locking - markers will use latest transform
            # instead of interpolating to closest timestamped transform
            zero_time = rclpy.time.Time().to_msg()

            # Helper to create arrow marker for forces
            def create_force_arrow(
                marker_id,
                name,
                vec,
                color,
                scale=1.0,
                offset=(
                    0.0,
                    0.0,
                    0.0)):
                """Create arrow marker for force vector."""
                magnitude = np.linalg.norm(vec)

                # Skip markers for very small forces to avoid rendering issues
                if magnitude < 1e-6:
                    return None

                marker = Marker()
                marker.header.frame_id = 'vehicle'
                marker.header.stamp = zero_time  # Frame locking enabled
                marker.ns = 'forces'
                marker.id = marker_id
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.frame_locked = True  # Enable frame locking in RViz2

                # Arrow from origin to scaled endpoint
                start = Point(x=offset[0], y=offset[1], z=offset[2])
                end = Point(
                    x=offset[0] + vec[0] * scale,
                    y=offset[1] + vec[1] * scale,
                    z=offset[2] + vec[2] * scale,
                )
                marker.points = [start, end]

                # Arrow shaft and head dimensions - 3x wider than before
                marker.scale.x = 0.03  # Shaft diameter (3cm)
                marker.scale.y = 0.06  # Head diameter (6cm)
                marker.scale.z = 0.0  # Not used for ARROW type with points

                marker.color = color
                marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

                return marker

            # Helper to create circular arc marker for moments
            def create_moment_arc(marker_id, name, vec, color, scale=1.0):
                """Create circular arc marker for moment vector using LINE_STRIP.

                The arc is drawn perpendicular to the moment vector (right-hand rule).
                Radius proportional to moment magnitude.
                """
                magnitude = np.linalg.norm(vec)

                # Skip markers for very small moments
                if magnitude < 1e-6:
                    return None

                marker = Marker()
                marker.header.frame_id = 'vehicle'
                marker.header.stamp = zero_time
                marker.ns = 'moments'
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.frame_locked = True

                # Normalize moment vector to get rotation axis
                axis = vec / magnitude

                # Find two perpendicular vectors to create the arc plane
                # Choose an arbitrary vector not parallel to axis
                if abs(axis[0]) < 0.9:
                    arbitrary = np.array([1.0, 0.0, 0.0])
                else:
                    arbitrary = np.array([0.0, 1.0, 0.0])

                # Create orthonormal basis in the plane perpendicular to axis
                u = np.cross(axis, arbitrary)
                u = u / np.linalg.norm(u)
                v = np.cross(axis, u)

                # Arc radius proportional to moment magnitude
                radius = magnitude * scale

                # Create arc points (270 degrees, showing rotation direction)
                num_points = 30
                angle_range = 3.0 * np.pi / 2.0  # 270 degrees

                points = []
                for i in range(num_points):
                    angle = (i / (num_points - 1)) * angle_range
                    # Point on circle in the perpendicular plane
                    point = radius * (np.cos(angle) * u + np.sin(angle) * v)
                    points.append(
                        Point(
                            x=float(
                                point[0]), y=float(
                                point[1]), z=float(
                                point[2])))

                # Add arrowhead at the end by creating small segments
                last_point = points[-1]
                # Direction tangent to arc at end point
                tangent = -np.sin(angle_range) * u + np.cos(angle_range) * v
                tangent = tangent / np.linalg.norm(tangent)

                # Arrow tip perpendicular to tangent
                arrow_left = np.cross(axis, tangent) * radius * 0.1
                arrow_right = -arrow_left

                # Add arrow tip lines
                tip_center = np.array(
                    [last_point.x, last_point.y, last_point.z])
                points.append(
                    Point(
                        x=float(tip_center[0] + arrow_left[0]),
                        y=float(tip_center[1] + arrow_left[1]),
                        z=float(tip_center[2] + arrow_left[2]),
                    )
                )
                points.append(last_point)
                points.append(
                    Point(
                        x=float(tip_center[0] + arrow_right[0]),
                        y=float(tip_center[1] + arrow_right[1]),
                        z=float(tip_center[2] + arrow_right[2]),
                    )
                )

                marker.points = points

                # Line width - make thicker for visibility
                marker.scale.x = 0.03  # Line width (3cm, increased from 1.5cm)

                marker.color = color
                marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

                return marker

            # Color definitions
            blue = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            orange = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
            purple = ColorRGBA(r=0.8, g=0.0, b=0.8, a=0.8)

            # Force markers (at CG) - 5x longer than before (was 0.2, now 1.0)
            force_scale = 1.0  # 1m per Newton (5x increase from 20cm)
            markers_to_add = [
                create_force_arrow(0, 'force_aero', FA_b, blue, force_scale),
                create_force_arrow(1, 'force_thrust', FT_b,
                                   orange, force_scale),
                create_force_arrow(2, 'force_weight', FW_b,
                                   purple, force_scale),
            ]

            # Moment markers as curved arcs - increased scale for visibility
            moment_scale = 2.0  # 2m radius per N·m (increased from 0.5m)
            markers_to_add.extend(
                [
                    create_moment_arc(3, 'moment_aero', MA_b,
                                      blue, moment_scale),
                    create_moment_arc(4, 'moment_thrust',
                                      MT_b, orange, moment_scale),
                    create_moment_arc(5, 'moment_weight',
                                      MW_b, purple, moment_scale),
                ]
            )

            # Add non-None markers to array
            for marker in markers_to_add:
                if marker is not None:
                    marker_array.markers.append(marker)

            self.force_marker_publisher.publish(marker_array)

        except Exception as e:
            self.get_logger().debug(
                f'Failed to publish force/moment markers: {e}')

    def step_simulation(self):
        """Step the simulation and publish transforms and diagnostics."""
        self.sim_time += self.dt
        sim_time_msg = self.get_sim_time_msg()
        self.sim_time_publisher.publish(sim_time_msg)

        self.get_logger().debug(f'sim time: {self.sim_time:.2f}s')

        # TEMPORARY: Use plant-only model in manual mode (mode=0) for demo
        use_plant_only = self.u.mode < 0.5

        if use_plant_only:
            # Step plant-only model (pure manual, no controller)
            x_vec = self.x_cl.as_vec()
            u_vec = self.u_cl.as_vec()
            p_vec = self.p_cl.as_vec()

            x_next = self.cl_model.f_step(
                x=x_vec, u=u_vec, p=p_vec, dt=self.dt)
            x_next_vec = x_next['x_next']

            # Check for NaN or invalid values
            if np.any(np.isnan(x_next_vec)) or np.any(np.isinf(x_next_vec)):
                self.get_logger().error(
                    f'NaN or Inf detected in plant state at t={
                        self.sim_time:.3f}s!\n'
                    'Pausing simulation.'
                )
                self.pause()
                return

            # Update plant state - manually unpack vector into state fields
            from dataclasses import fields

            offset = 0
            for field_obj in fields(self.x_cl):
                field_val = getattr(self.x_cl, field_obj.name)
                field_size = field_val.shape[0] if hasattr(
                    field_val, 'shape') else 1
                setattr(self.x_cl, field_obj.name,
                        x_next_vec[offset: offset + field_size])
                offset += field_size

            # Sync plant state to composed model (for when mode switches)
            self.x.plant = copy.deepcopy(self.x_cl)

            # Compute outputs from plant
            if hasattr(self.cl_model, 'f_y'):
                result = self.cl_model.f_y(x=x_vec, u=u_vec, p=p_vec)
                output_vec = result['y']

                from cubs2_dynamics.sportcub import SportCubOutputs

                outputs = SportCubOutputs.from_vec(output_vec)

                # Extract control commands (pass-through in manual mode)
                self.ail_cmd = float(self.u_cl.ail)
                self.elev_cmd = float(self.u_cl.elev)
                self.rud_cmd = float(self.u_cl.rud)
                self.thr_cmd = float(self.u_cl.thr)

                # Store forces/moments
                self.last_outputs = outputs

        else:
            # Use closed-loop model (aircraft + controller)
            # Sync plant state from manual mode if switching
            if hasattr(self, '_was_plant_only') and self._was_plant_only:
                self.x.plant = copy.deepcopy(self.x_cl)

            # Convert structured state to vector for integration
            x_vec = self.model._state_to_vec(self.x)
            u_vec = self.u.as_vec()
            p_vec = self.p.as_vec()

            x_next = self.model.f_step(x=x_vec, u=u_vec, p=p_vec, dt=self.dt)
            x_next_vec = x_next['x_next']

            # Check for NaN or invalid values
            if np.any(np.isnan(x_next_vec)) or np.any(np.isinf(x_next_vec)):
                self.get_logger().error(
                    f'NaN or Inf detected in state at t={
                        self.sim_time:.3f}s!\n'
                    'Pausing simulation.'
                )
                self.pause()
                return

            # Convert vector back to structured state
            self.x = self.model._vec_to_state(x_next_vec)

            # Sync back to plant state
            self.x_cl = copy.deepcopy(self.x.plant)

            # Compute outputs from composed model
            if hasattr(self.model, 'f_y'):
                try:
                    result = self.model.f_y(x=x_vec, u=u_vec, p=p_vec)
                    output_vec = result['y']

                    from cubs2_control.closed_loop import ClosedLoopOutputs

                    outputs = ClosedLoopOutputs.from_vec(output_vec)

                    # Extract control commands for joint state visualization
                    self.ail_cmd = float(outputs.ail)
                    self.elev_cmd = float(outputs.elev)
                    self.rud_cmd = float(outputs.rud)
                    self.thr_cmd = float(outputs.thr)

                    # Store forces/moments for visualization
                    self.last_outputs = type(
                        'obj',
                        (object,),
                        {
                            'FA_b': outputs.F,
                            'FT_b': np.zeros(3),
                            'FW_b': np.zeros(3),
                            'MA_b': outputs.M,
                            'MT_b': np.zeros(3),
                            'MW_b': np.zeros(3),
                        },
                    )()
                except Exception as e:
                    import traceback

                    self.get_logger().warn(
                        f'Failed to compute outputs: {
                            e}\n{traceback.format_exc()}'
                    )

        self._was_plant_only = use_plant_only

        # Publish joint states every step
        self.publish_joint_states()

        # Publish force/moment markers every step
        if self.show_forces and self.last_outputs:
            self.publish_force_moment_markers(self.last_outputs)

        # Publish pose using tf (position + orientation)
        pos = self.x.plant.p  # position in world frame
        q = self.x.plant.r  # quaternion (world->body) stored as [w, x, y, z]
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'vehicle'  # Match URDF parent frame
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        # Map internal quaternion [w,x,y,z] -> geometry_msgs (x,y,z,w)
        try:
            t.transform.rotation.x = float(q[1])
            t.transform.rotation.y = float(q[2])
            t.transform.rotation.z = float(q[3])
            t.transform.rotation.w = float(q[0])
        except Exception:
            # Fallback to identity if something unexpected occurs
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # Publish pose for HUD panel
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])
        pose_msg.pose.orientation.x = float(q[1])
        pose_msg.pose.orientation.y = float(q[2])
        pose_msg.pose.orientation.z = float(q[3])
        pose_msg.pose.orientation.w = float(q[0])
        self.pose_publisher.publish(pose_msg)

        # Publish velocity for HUD panel
        vel = self.x.plant.v  # velocity in world frame
        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = self.get_clock().now().to_msg()
        velocity_msg.header.frame_id = 'map'
        velocity_msg.twist.linear.x = float(vel[0])
        velocity_msg.twist.linear.y = float(vel[1])
        velocity_msg.twist.linear.z = float(vel[2])
        # Angular velocity (w) in body frame
        w = self.x.plant.w
        velocity_msg.twist.angular.x = float(w[0])
        velocity_msg.twist.angular.y = float(w[1])
        velocity_msg.twist.angular.z = float(w[2])
        self.velocity_publisher.publish(velocity_msg)

        # x1 = self.sim.f_step(
        #     x=self.x0.as_vec(),
        #     u=self.sim.u0.as_vec(),
        #     p=self.sim.p0.as_vec(),
        #     dt=self.dt,
        # )

    def publish_visuals(self):
        """Publish the current state (clock + TF + joint states) without advancing physics.

        Useful after a reset while paused so RViz immediately reflects the new
        initial conditions instead of waiting for the user to press play.
        """
        # Publish current (reset) clock value
        sim_time_msg = self.get_sim_time_msg()
        self.sim_time_publisher.publish(sim_time_msg)

        # Publish joint states
        self.publish_joint_states()

        # Publish TF for current pose with orientation
        pos = self.x.plant.p
        q = self.x.plant.r
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'vehicle'
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = float(pos[2])
        try:
            t.transform.rotation.x = float(q[1])
            t.transform.rotation.y = float(q[2])
            t.transform.rotation.z = float(q[3])
            t.transform.rotation.w = float(q[0])
        except Exception:
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def pause(self):
        """Pause the simulation."""
        self.paused = True
        try:
            self.timer.cancel()
        except Exception:
            pass
        self.get_logger().info('paused')
        try:
            self.paused_publisher.publish(Bool(data=True))
        except Exception:
            pass

    def resume(self):
        """Resume the simulation."""
        self.timer = self.create_timer(
            self.dt / self.sim_speed, self.step_simulation)
        self.get_logger().info('running')
        self.paused = False
        try:
            self.paused_publisher.publish(Bool(data=False))
        except Exception:
            pass

    def pause_topic_callback(self, msg):
        """Toggle simulation pause/resume when /pause topic is received."""
        if self.paused:
            self.resume()
        else:
            self.pause()

    def speed_topic_callback(self, msg):
        """Update sim speed multiplier."""
        self.pause()
        self.sim_speed = max(0.01, float(msg.data))
        self.get_logger().info(f'sim speed set to {self.sim_speed:.2f}x')
        self.resume()

    def dt_topic_callback(self, msg):
        """Update simulation time step (dt)."""
        self.pause()
        new_dt = max(0.001, float(msg.data))  # Minimum 1ms time step
        self.dt = new_dt
        self.get_logger().info(f'time step set to {self.dt:.3f}s')
        self.resume()

    def reset_topic_callback(self, msg):
        """Reset the  initial conditions (topic interface for RViz)."""
        self.get_logger().info('resetting to initial conditions (via /reset topic)...')
        was_running = not getattr(self, 'paused', True)
        self.pause()

        # Reset clock
        self.sim_time = 0.0

        # Reset closed-loop model state to initial conditions
        self.x = copy.deepcopy(self.model.x0)
        self.u = copy.deepcopy(self.model.u0)
        self.p = copy.deepcopy(self.model.p0)

        # Reset plant-only model state
        self.x_cl = copy.deepcopy(self.cl_model.x0)
        self.u_cl = copy.deepcopy(self.cl_model.u0)
        self.p_cl = copy.deepcopy(self.cl_model.p0)

        # Reapply takeoff-ready initial conditions (aircraft state portion)
        self._apply_initial_pose()

        self.u.thr_manual = 0.0
        self.u.elev_manual = 0.0

        # Reset plant-only inputs
        self.u_cl.thr = 0.0
        self.u_cl.elev = 0.0
        self.u_cl.ail = 0.0
        self.u_cl.rud = 0.0

        # Publish visuals immediately so RViz reflects new initial state while
        # paused
        self.publish_visuals()
        self.get_logger().info('reset complete (clock restarted at t=0.0s)')

        # Resume only if we were running before reset
        if was_running:
            self.get_logger().info('auto-resuming simulation after reset (was running before)')
            self.resume()
        else:
            self.get_logger().info('simulation remains paused after reset')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SimNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
