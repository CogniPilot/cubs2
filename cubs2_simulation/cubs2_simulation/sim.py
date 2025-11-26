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

from beartype import beartype
from builtin_interfaces.msg import Time
import casadi as ca
from cubs2_control.closed_loop import closed_loop_sportcub
from cubs2_dynamics.sportcub import sportcub
from cubs2_msgs.msg import AircraftControl
from cubs2_simulation.markers import create_force_arrow
from cubs2_simulation.markers import create_moment_arc
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
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster
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

        # Model selection parameter
        self.declare_parameter('model', 'direct')  # 'direct' or 'closed_loop'

        # Trim parameters from config file
        self.declare_parameter('controller.trim.aileron', 0.0)
        self.declare_parameter('controller.trim.elevator', 0.0)
        self.declare_parameter('controller.trim.rudder', 0.0)

        # Initial pose parameters (ENU frame, yaw about +z)
        self.declare_parameter('initial_position.x', 0.0)
        self.declare_parameter('initial_position.y', 0.0)
        self.declare_parameter('initial_position.z', 0.1)
        self.declare_parameter('initial_yaw_deg', -30.0)  # degrees

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

        # Initialize model based on parameter
        self.model_type = str(self.get_parameter('model').value)

        if self.model_type == 'closed_loop':
            self.get_logger().info(
                'Initializing closed-loop model '
                '(aircraft + autolevel controller)'
            )
            self.model = closed_loop_sportcub()
        else:
            self.get_logger().info('Initializing direct sportcub model (no controller)')
            self.model = sportcub()

        self.y = self.model.y_current  # computed at end of each step

        # Apply initial state (position, orientation, and control inputs)
        self.apply_initial_state()

        # Subscribe to control messages (published by virtual joystick,
        # gamepad, keyboard)
        self.control_subscription = self.create_subscription(
            AircraftControl, '/control', self.control_callback, 10
        )

        # Joint state publisher for control surface and propeller animation
        self.joint_state_publisher = self.create_publisher(
            JointState, '/vehicle/joint_states', 10)

        # Propeller rotation state
        self.propeller_angle = 0.0

        # Initialize to running state
        self.resume()

    @beartype
    def step_simulation(self) -> None:
        """Step the simulation and publish transforms and diagnostics."""
        self.sim_time += self.dt

        # Use closed-loop model (aircraft + controller)
        # Simulate one step forward using the simulate method

        try:
            self.model.simulate(
                t0=self.sim_time - self.dt,
                tf=self.sim_time,
                dt=self.dt,
                u_func=self._get_control_inputs,
                in_place=True,
            )

        except RuntimeError as e:
            # cyecca detected NaN or Inf
            self.get_logger().error(f'{e}\nPausing simulation.')
            self.pause()
            return

        # Update propeller angle for animation
        if self.model_type == 'closed_loop':
            thr_value = self.model.u0.thr_manual
        else:
            thr_value = self.model.u0.thr
        propeller_rpm = thr_value * 2000.0  # Max 2000 RPM at full throttle
        propeller_omega = propeller_rpm * 2.0 * np.pi / 60.0  # Convert to rad/s
        self.propeller_angle += propeller_omega * self.dt
        self.propeller_angle = self.propeller_angle % (2.0 * np.pi)  # Wrap to [0, 2π]

        self.publish_state()

    def _get_control_inputs(self, t, model):
        """
        Get control inputs based on model type.

        For closed-loop: 5 inputs (ail_manual, elev_manual, rud_manual,
                                   thr_manual, mode)
        For direct: 4 inputs (ail, elev, rud, thr)
        """
        if self.model_type == 'closed_loop':
            return ca.vertcat(
                model.u0.ail_manual,
                model.u0.elev_manual,
                model.u0.rud_manual,
                model.u0.thr_manual,
                model.u0.mode
            )
        else:
            return ca.vertcat(
                model.u0.ail,
                model.u0.elev,
                model.u0.rud,
                model.u0.thr
            )

    @beartype
    def control_callback(self, msg: AircraftControl) -> None:
        """Handle AircraftControl messages."""
        # Update actual model inputs based on model type
        if self.model_type == 'closed_loop':
            self.model.u0.ail_manual = float(msg.aileron)
            self.model.u0.elev_manual = float(msg.elevator)
            self.model.u0.thr_manual = float(msg.throttle)
            # self.model.u0.rud_manual = float(msg.rudder)
        else:
            self.model.u0.ail = float(msg.aileron)
            self.model.u0.elev = float(msg.elevator)
            self.model.u0.thr = float(msg.throttle)
            # self.model.u0.rud = float(msg.rudder)

    @beartype
    def pause_topic_callback(self, msg: Empty) -> None:
        """Toggle simulation pause/resume when /pause topic is received."""
        if self.paused:
            self.resume()
        else:
            self.pause()

    @beartype
    def speed_topic_callback(self, msg: Float64) -> None:
        """Update sim speed multiplier."""
        self.pause()
        self.sim_speed = max(0.01, float(msg.data))
        self.get_logger().info(f'sim speed set to {self.sim_speed:.2f}x')
        self.resume()

    @beartype
    def dt_topic_callback(self, msg: Float64) -> None:
        """Update simulation time step (dt)."""
        self.pause()
        new_dt = max(0.001, float(msg.data))  # Minimum 1ms time step
        self.dt = new_dt
        self.get_logger().info(f'time step set to {self.dt:.3f}s')
        self.resume()

    @beartype
    def reset_topic_callback(self, msg: Empty) -> None:
        """Reset the  initial conditions (topic interface for RViz)."""
        self.get_logger().info('resetting to initial conditions (via /reset topic)...')
        was_running = not getattr(self, 'paused', True)
        self.pause()

        # Reset clock
        self.sim_time = 0.0

        # Reapply takeoff-ready initial conditions (aircraft state portion)
        self.apply_initial_state()

        # Publish state immediately so RViz reflects new initial state while paused
        self.publish_state()
        self.get_logger().info('reset complete (clock restarted at t=0.0s)')

        # Resume only if we were running before reset
        if was_running:
            self.get_logger().info('auto-resuming simulation after reset (was running before)')
            self.resume()
        else:
            self.get_logger().info('simulation remains paused after reset')

    @beartype
    def apply_initial_state(self) -> None:
        """Apply configured initial position and yaw to aircraft state."""
        # Get initial pose parameters
        initial_x = float(self.get_parameter('initial_position.x').value)
        initial_y = float(self.get_parameter('initial_position.y').value)
        initial_z = float(self.get_parameter('initial_position.z').value)
        initial_yaw_deg = float(
            self.get_parameter('initial_yaw_deg').value
        )

        # Convert yaw from degrees to radians, then to quaternion
        # (rotation about z-axis in ENU frame)
        # Quaternion stored as [w, x, y, z]
        initial_yaw_rad = np.deg2rad(initial_yaw_deg)
        half_yaw = initial_yaw_rad / 2.0
        qw = np.cos(half_yaw)
        qx = 0.0
        qy = 0.0
        qz = np.sin(half_yaw)

        # Apply to aircraft state
        self.model.x0.p[0] = initial_x
        self.model.x0.p[1] = initial_y
        self.model.x0.p[2] = initial_z

        # Reset velocity to zero
        self.model.x0.v[0] = 0.0
        self.model.x0.v[1] = 0.0
        self.model.x0.v[2] = 0.0

        # Set attitude quaternion
        self.model.x0.r[0] = qw
        self.model.x0.r[1] = qx
        self.model.x0.r[2] = qy
        self.model.x0.r[3] = qz

        # Reset angular velocity to zero
        self.model.x0.w[0] = 0.0
        self.model.x0.w[1] = 0.0
        self.model.x0.w[2] = 0.0

        # Manual control inputs from joystick/gamepad
        if self.model_type == 'closed_loop':
            self.model.u0.ail_manual = 0.0
            self.model.u0.elev_manual = 0.0
            self.model.u0.rud_manual = 0.0
            self.model.u0.thr_manual = 0.0
            self.model.u0.mode = float(AircraftControl.MODE_MANUAL)
        else:
            self.model.u0.ail = 0.0
            self.model.u0.elev = 0.0
            self.model.u0.rud = 0.0
            self.model.u0.thr = 0.0

    @beartype
    def get_sim_time_msg(self) -> Clock:
        sim_time_msg = Clock()
        sim_time_msg.clock = Time()
        sim_time_msg.clock.sec = int(self.sim_time)
        sim_time_msg.clock.nanosec = int(
            (self.sim_time - int(self.sim_time)) * 1e9)
        return sim_time_msg

    @beartype
    def publish_state(self) -> None:
        """
        Publish the current state (clock + TF + joint states) without advancing physics.

        Useful after a reset while paused so RViz immediately reflects the new
        initial conditions instead of waiting for the user to press play.

        """
        # Publish current (reset) clock value
        sim_time_msg = self.get_sim_time_msg()
        self.sim_time_publisher.publish(sim_time_msg)
        self.get_logger().debug(f'sim time: {self.sim_time:.2f}s')

        # Use simulation time for all timestamps
        stamp = sim_time_msg.clock

        # Publish force/moment markers every step
        self.y = self.model.y_current
        if self.show_forces:
            self.publish_force_moment_markers()

        # Extract state as numpy arrays for publishing
        pos = np.array(self.model.x0.p).flatten()
        q = np.array(self.model.x0.r).flatten()
        vel = np.array(self.model.x0.v).flatten()
        w = np.array(self.model.x0.w).flatten()

        # Publish pose using tf (position + orientation)
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'vehicle'  # Match URDF parent frame
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Publish pose for HUD panel
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = pos[0]
        pose_msg.pose.position.y = pos[1]
        pose_msg.pose.position.z = pos[2]
        pose_msg.pose.orientation.w = q[0]
        pose_msg.pose.orientation.x = q[1]
        pose_msg.pose.orientation.y = q[2]
        pose_msg.pose.orientation.z = q[3]
        self.pose_publisher.publish(pose_msg)

        # Publish velocity for HUD panel
        velocity_msg = TwistStamped()
        velocity_msg.header.stamp = stamp
        velocity_msg.header.frame_id = 'map'
        velocity_msg.twist.linear.x = vel[0]
        velocity_msg.twist.linear.y = vel[1]
        velocity_msg.twist.linear.z = vel[2]
        velocity_msg.twist.angular.x = w[0]
        velocity_msg.twist.angular.y = w[1]
        velocity_msg.twist.angular.z = w[2]
        self.velocity_publisher.publish(velocity_msg)

    @beartype
    def pause(self) -> None:
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

    @beartype
    def resume(self) -> None:
        """Resume the simulation."""
        self.timer = self.create_timer(
            self.dt / self.sim_speed, self.step_simulation)
        self.get_logger().info('running')
        self.paused = False
        try:
            self.paused_publisher.publish(Bool(data=False))
        except Exception:
            pass

    @beartype
    def publish_force_moment_markers(self) -> None:
        """
        Publish MarkerArray for force and moment visualization.

        Creates arrow markers for aerodynamic, thrust, and weight forces.
        Creates curved arrow/arc markers for moments to distinguish from forces.

        """
        try:
            # Extract force and moment vectors (body frame) from dataclass
            FA_b = np.array(self.y.FA_b).flatten()
            FT_b = np.array(self.y.FT_b).flatten()
            FW_b = np.array(self.y.FW_b).flatten()
            MA_b = np.array(self.y.MA_b).flatten()
            MT_b = np.array(self.y.MT_b).flatten()
            MW_b = np.array(self.y.MW_b).flatten()

            # Color definitions
            blue = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            orange = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.8)
            purple = ColorRGBA(r=0.8, g=0.0, b=0.8, a=0.8)

            # Create force markers (at CG)
            force_scale = 1.0  # 1m per Newton
            markers_to_add = [
                create_force_arrow(0, 'force_aero', FA_b, blue, force_scale),
                create_force_arrow(1, 'force_thrust', FT_b, orange, force_scale),
                create_force_arrow(2, 'force_weight', FW_b, purple, force_scale),
            ]

            # Create moment markers as curved arcs
            moment_scale = 2.0  # 2m radius per N·m
            markers_to_add.extend([
                create_moment_arc(3, 'moment_aero', MA_b, blue, moment_scale),
                create_moment_arc(4, 'moment_thrust', MT_b, orange, moment_scale),
                create_moment_arc(5, 'moment_weight', MW_b, purple, moment_scale),
            ])

            # Publish non-None markers
            marker_array = MarkerArray()
            for marker in markers_to_add:
                if marker is not None:
                    marker_array.markers.append(marker)

            self.force_marker_publisher.publish(marker_array)

        except Exception as e:
            self.get_logger().debug(
                f'Failed to publish force/moment markers: {e}')


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
