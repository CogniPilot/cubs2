#!/usr/bin/env python3
from cubs2_msgs.msg import AircraftControl
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from pathlib import Path
from cubs2_dynamics.sportcub import sportcub
import yaml
import numpy as np
from types import SimpleNamespace
import casadi as ca
from cyecca.lie import SO3Quat, SO3EulerB321
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class AutoControlNode(Node):
    def __init__(self) -> None:
        super().__init__("auto_control")

        # Publishers
        self.pub_control = self.create_publisher(AircraftControl, "/control_auto", 10)

        # Current state
        self.aileron = 0.0
        self.elevator = 0.0
        self.throttle = 0.0
        self.rudder = 0.0
        self.mode = 0  # 0 = manual, 1 = stabilized

        # Trim values (applied as offsets to stick inputs)
        self.trim_aileron = 0.0
        self.trim_elevator = 0.0
        self.trim_throttle = 0.0
        self.trim_rudder = 0.0


        self.TW = 0.47 # Thrust to weight ratio

        # Reference trajectory subscriber
        self.ref_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "reference_pose_ahead",
            self.reference_pose_callback,
            10,
        )

        # True state subscriber (for feedback control)
        self.actual_pose_sub = self.create_subscription(
            PoseStamped, "/sportcub/pose", self.actual_pose_callback, 10
        )

        self.velocity_sub = self.create_subscription(
            TwistStamped, "/sportcub/velocity", self.speed_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, "/sportcub/imu", self.imu_callback, 10
        )
        self.imu_received = False

        self.actual_data = {
            "x_est": 0.0,
            "y_est": 0.0,
            "z_est": 0.0,
            "roll_est": 0.0,
            "pitch_est": 0.0,
            "yaw_est": 0.0,
            "vx_est": 0.0,
            "vy_est": 0.0,
            "vz_est": 0.0,
            "v_est": 0.0,
            "gamma_est": 0.0,
            "vdot_est": 0.0,
            "p_est": 0.0,
            "q_est": 0.0,
            "r_est": 0.0,
            "beta_est": 0.0,
            "ax_est": 0.0,
            "ay_est": 0.0,
            "az_est": 0.0,
        }

        self.ref_data = {
            "des_v": 0.0,
            "des_gamma": 0.0,
            "des_heading": 0.0,
            "des_a": 0.0,
            "des_phi": 0.0,
            "des_phi_dot": 0.0,
            "des_psi_dot": 0.0,
            "des_p": 0.0,
        }

        # Store reference pose
        self.ref_pose = None

        # Flight mode and timing
        self.prev_speed = 0
        self.flight_mode = "takeoff"
        self.dt = 0.01
        self.g = 9.81
        self.thr_max = 0.3 #7.5  # 4.5 #Maximum Thrust

        self.args = "sim"  # Vehicle selection
        this_file = Path(__file__).resolve()
        self.base_dir = this_file.parent / "param"

        # TECS controller state
        self.error_norm_Es_dot_integral = 0
        self.error_dist_term_integral = 0
        self.error_pitch_integral = 0
        self.error_r_integral = 0
        self.error_r_last = 0
        self.error_xtrack_integral = 0

        # Integrator for roll rate error in takeoff yaw control
        self.p_integral = 0.0

        # Roll controller (options: "stabilized" | "phi_stick" | "direct")
        self.roll_mode = "stabilized"
        self._phi_cmd = 0.0
        self._e_phi_int = 0.0

        # Takeoff yaw rate controller (maintain zero yaw)
        self._e_r_int = 0.0
        self._K_r_p = 0.5
        self._K_r_i = 0.1
        self._r_int_max = 0.3

        self.timer = self.create_timer(self.dt, self.control_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.time = 0
        self.takeoff_time = 0

        self.reload_gains()

    def reload_gains(self):
        self.get_logger().info(f"[TECSControl] Loading gains from: {self.args}.yaml")
        gain_path = self.base_dir / f"{self.args}.yaml"

        if not gain_path.exists():
            raise FileNotFoundError(f"[TECSControl] Gain file not found: {gain_path}")

        with open(gain_path, "r") as f:
            raw = yaml.safe_load(f)

        self.param = SimpleNamespace(**raw)

        self.phi_lim = np.deg2rad(self.param.phi_lim_deg)
        self.chi_deadband = np.deg2rad(self.param.chi_deadband_deg)
        self.phi_dot_lim = np.deg2rad(self.param.phi_dot_lim_deg_s)

        self.mass = self.param.mass
        self.weight = self.mass * self.g

        # Load trim values from parameter file
        self.trim_aileron = getattr(self.param, "trim_aileron", 0.0)
        self.trim_elevator = getattr(self.param, "trim_elevator", 0.0)
        self.trim_throttle = getattr(self.param, "trim_throttle", 0.0)
        self.trim_rudder = getattr(self.param, "trim_rudder", 0.0)

        # Initialize PID controllers for throttle and elevator commands
        self.pid_thr_kp = getattr(self.param, "pid_thr_kp", 0.1)
        self.pid_thr_ki = getattr(self.param, "pid_thr_ki", 0.01)
        self.pid_thr_kd = getattr(self.param, "pid_thr_kd", 0.05)
        self.pid_elv_kp = getattr(self.param, "pid_elv_kp", 0.2)
        self.pid_elv_ki = getattr(self.param, "pid_elv_ki", 0.02)
        self.pid_elv_kd = getattr(self.param, "pid_elv_kd", 0.1)
        self.pid_thr_integral = 0.0
        self.pid_thr_prev_error = 0.0
        self.pid_elv_integral = 0.0
        self.pid_elv_prev_error = 0.0

        # self.get_logger().debug(f"Gains loaded from: {gain_path}")

    def compute_thrust_pitch(
        self, x, y, z, ref_data, vx_est, vy_est, vz_est, V_est, gamma_est, vdot_est
    ):
        # ref data in function of time
        ref_airspeed = ref_data["des_v"]
        ref_gamma = ref_data["des_gamma"]  # Glide slope angle
        # ref_xtrack_err = ref_data['xtrack_err']
        ref_accel = ref_data["des_a"]

        r_V = float(ref_airspeed)  # desired body-frame speed
        r_gamma = float(ref_gamma)  # desired flight path angle
        r_V_dot = float(ref_accel)  # desired acceleration

        # Envelope protection: clip desired acceleration
        drag = 1.0
        r_V_dot = np.clip(
            r_V_dot, -drag / self.weight, (self.thr_max - drag) / self.weight
        )

        # -------------------Desired Thrust-------------------#
        # Specific energy rate error
        error_norm_Es_dot = (r_gamma - gamma_est) + (r_V_dot - vdot_est) / self.g
        thrust_unsat = self.param.trim_thrust + self.weight * (
            self.param.K_thrustp * (gamma_est + vdot_est / self.g)
            + self.param.K_thrusti * self.error_norm_Es_dot_integral
        )

        thrust = float(np.clip(thrust_unsat, 0.0, self.thr_max))

        # Thrust anti-windup
        # If at upper limit and error > 0, integrating would push further into sat -> freeze integral.
        # If at lower limit and error < 0, freeze integral.
        allow_I = True
        if thrust >= self.thr_max - 1e-9 and error_norm_Es_dot > 0.0:
            allow_I = False
        if thrust <= 0.0 + 1e-9 and error_norm_Es_dot < 0.0:
            allow_I = False

        if allow_I:
            self.error_norm_Es_dot_integral += error_norm_Es_dot * self.dt
            self.error_norm_Es_dot_integral = np.clip(
                self.error_norm_Es_dot_integral,
                -self.param.norm_Es_dot_integral_max,
                self.param.norm_Es_dot_integral_max,
            )

        # -------------------Desired Pitch-------------------#
        # Energy rate distribution term error
        error_dist_term = (r_gamma - gamma_est) - (r_V_dot - vdot_est) / self.g
        pitch_unsat = (
            self.param.K_pitchi * self.error_dist_term_integral
            - self.param.K_pitchp * (gamma_est - vdot_est / self.g)
        )

        pitch = float(np.clip(pitch_unsat, np.deg2rad(-20), np.deg2rad(20)))

        allow_I = True
        if pitch >= np.deg2rad(20) - 1e-9 and error_dist_term > 0.0:
            allow_I = False
        if pitch <= np.deg2rad(-20) + 1e-9 and error_dist_term < 0.0:
            allow_I = False

        if allow_I:
            self.error_dist_term_integral += error_dist_term * self.dt
            self.error_dist_term_integral = np.clip(
                self.error_dist_term_integral,
                -self.param.dist_term_integral_max,
                self.param.dist_term_integral_max,
            )
        # self.get_logger().debug(f"r_gamma: {r_gamma:5.2f}, gamma_est: {gamma_est:5.2f}, r_V_dot: {r_V_dot:5.2f}, vdot_est: {vdot_est:5.2f}")

        return thrust, pitch
    

    def get_T_alpha_desired(self, phi, v, gamma):
        # print(gamma)
        # gamma = np.deg2rad(5)
        # print("v ", v)
        m = 1.55
        S = 0.68
        rho = 1.225
        g = 9.81
        CD0 = 0.04
        k = 0.0783
        
        wing_incidence = 0

        CL0 = 0.3
        CLa = 4.79

        L = m*g/np.cos(phi)*np.cos(gamma)
        CL = L/(0.5 * rho * v**2 * S)

        alpha = (CL - CL0)/CLa - wing_incidence

        D = 1/2 * rho * v**2 * S * (CD0 + k * CL**2)

        T = m*g*np.sin(gamma) + D


        return T, alpha


    def compute_control(self, ref_data, actual_data, ref_thrust=None, ref_pitch=None):
        # actual data
        x = actual_data["x_est"]
        y = actual_data["y_est"]
        z = actual_data["z_est"]
        roll = actual_data["roll_est"]
        pitch = actual_data["pitch_est"]
        yaw = actual_data["yaw_est"]
        vx_est = actual_data["vx_est"]
        vy_est = actual_data["vy_est"]
        vz_est = actual_data["vz_est"]
        V_est = actual_data["v_est"]
        gamma_est = actual_data["gamma_est"]
        vdot_est = actual_data["vdot_est"]
        p_est = actual_data["p_est"]
        q_est = actual_data["q_est"]
        r_est = actual_data["r_est"]

        # Gains
        K_phi = 2 # Proportional gain for roll angle error to roll rate command
        K_p = 1 # Proportional gain for roll rate error to aileron command
        K_i = 0.1 # Integral gain for roll rate error to aileron command
        K_pff = 0.1 # Feed forward gain for roll rate from planner

        K_ay = 0.5

        # Turn outer loop
        print("--- Control Computation ---")
        print("des_phi: %.2f deg" % np.rad2deg(ref_data['des_phi']))
        phi_err = ref_data['des_phi'] - roll

        # inner loop 
        p_cmd = K_phi * phi_err
        # Simple P controller for aileron

        PID_p_err = p_cmd - p_est

        self.p_integral = self.p_integral + PID_p_err * self.dt

        ail_cmd = (K_p * (p_cmd - p_est) + K_i * self.p_integral +  K_pff * ref_data['des_p'])  # Add feed-forward from planner

        elv_cmd = 0.1
        thr_cmd = 0.4
        rud_cmd = -K_ay * actual_data['ay_est']

        print(f"ay_est: {actual_data['ay_est']:.2f} m/s^2")

        print(f"phi_err: {np.rad2deg(phi_err):.2f} deg, p_err {np.rad2deg(p_cmd - p_est):.2f} deg/s, ail_cmd: {ail_cmd:.3f}")

        self.aileron = np.clip(ail_cmd, -1, 1)
        self.elevator = np.clip(elv_cmd, -1, 1)
        self.throttle = np.clip(thr_cmd, -1, 1)
        self.rudder = np.clip(rud_cmd, -1, 1)

    def reference_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback for reference trajectory pose."""
        self.ref_pose = msg

    def control_callback(self):
        """Publish current control state as AircraftControl message."""

        ################################### FLIGHT MODE ####################################
        flight_mode_msg = String()
        if self.actual_data["z_est"] <= 0.5:
            new_mode = "takeoff"
        else:
            new_mode = "airborne"

        if new_mode != self.flight_mode:
            self.get_logger().info(
                "Flight mode changed from: %s to %s" % (self.flight_mode, new_mode)
            )
            self.flight_mode = new_mode
            flight_mode_msg.data = new_mode
            
            # Reset controllers when entering takeoff mode
            if new_mode == "takeoff":
                self._e_r_int = 0.0  # Reset yaw rate integrator

        if flight_mode_msg.data == "":
            flight_mode_msg.data = self.flight_mode

        self.time += self.dt

        if self.flight_mode == "takeoff":
            self.takeoff_time += self.dt

            # Throttle ramp with floor/ceiling
            self.throttle = ca.fmin(1.00, ca.fmax(0.7, self.throttle + 2.0 * self.dt)) # Cancel out trim in command

            self.aileron = 0.0  # Wings-level during takeoff

            # Yaw rate control: maintain zero yaw rate using rudder
            # Get current yaw rate (r_est in rad/s)
            r_est = self.actual_data.get("r_est", 0.0)
            # Desired yaw rate is zero
            e_r = 0.0 - r_est  # yaw rate error
            
            # Yaw rate PI controller
            self._e_r_int += e_r * self.dt
            self._e_r_int = float(
                np.clip(self._e_r_int, -self._r_int_max, self._r_int_max)
            )
            
            rud_cmd = (
                self._K_r_p * e_r
                + self._K_r_i * self._e_r_int
            )
            self.rudder = float(np.clip(rud_cmd, -1.0, 1.0))

            # Elevator schedule: pitch up as airspeed increases
            v_to = 0.5
            e_down = -0.02
            e_up = 0.15
            e_rate = 0.40
            if self.actual_data["v_est"] == None:
                self.actual_data["v_est"] = (
                    0.0  # Initialize V_est, assume start at stationary
                )

            self.elevator = ca.if_else(
                self.actual_data["v_est"] < v_to,
                e_down,
                ca.fmin(e_up, self.elevator + e_rate * self.dt),
            )

        if self.flight_mode == "airborne":

            ## CONSTANTS
            # V
            planner_v = 6.0 # Desired airspeed along trajectory (TODO - publish from planner)
            
            # gamma
            K_rc = 0.5  # Rate of climb gain
            max_gamma = np.deg2rad(30) # Maximum climb angle

            # heading
            # acceleration 
            # bank angle
            # bank anlge rate
            # psi_dot


            ## READ REFERENCE TRAJECTORY
            pose_q = np.array([
                self.ref_pose.pose.pose.orientation.w,
                self.ref_pose.pose.pose.orientation.x,
                self.ref_pose.pose.pose.orientation.y,
                self.ref_pose.pose.pose.orientation.z,
            ])

            ref_phi_dot = self.ref_pose.pose.covariance[1]
            ref_psi_dot = self.ref_pose.pose.covariance[2]

            SO3_pose = SO3Quat.elem(ca.horzcat(pose_q))
            eulers = SO3EulerB321.from_Quat(SO3_pose).param
            ref_psi, ref_theta, ref_phi = float(eulers[0]), float(eulers[1]), float(eulers[2])
            
            ## REFERENCE GENERATION
            # Reference velocity (straight from planner)
            des_v = planner_v

            # Reference gamma
            # RC = V sin(gamma) -> gamma = arcsin(RC / V)
            RC_des = K_rc * (self.ref_pose.pose.pose.position.z - self.actual_data["z_est"])
            des_gamma = ca.fmin(ca.arcsin(RC_des / self.actual_data["v_est"]), max_gamma)

            # Reference Heading 
            des_heading = ref_psi

            # Reference acceleration (simple P controller on velocity error for now, can add feed-forward from planner if we want)
            K_V = 1.0
            des_a = K_V * (planner_v - np.abs(self.actual_data["v_est"]))

            # Reference phi 
            des_phi = ca.arctan(ref_psi_dot * self.actual_data["v_est"] / self.g - self.TW * ca.sin(self.actual_data['beta_est']))

            des_p = ref_phi_dot - ref_psi_dot * ca.sin(self.actual_data["pitch_est"])

            # Reference phi_dot and psi_dot (feed-forward from planner)
            des_phi_dot = ref_phi_dot
            des_psi_dot = ref_psi_dot 

            # Test case
            R = 8
            des_psi_dot = self.actual_data["v_est"] / R
            des_phi = ca.arctan(ref_psi_dot * self.actual_data["v_est"] / self.g - self.TW * ca.sin(self.actual_data['beta_est']))
            des_p = 0
            des_phi_dot = 0


            des_v = 6.0 # m/s
            des_heading = -1 # NA
            
            self.ref_data = {
                "des_v": des_v,#
                "des_gamma": des_gamma, #
                "des_heading": des_heading,#
                "des_a": des_a,#
                "des_phi": des_phi,#
                "des_phi_dot": des_phi_dot,#
                "des_psi_dot": des_psi_dot,#
                "des_p": des_p, #
            }

            self.compute_control(self.ref_data, self.actual_data)

        msg = AircraftControl()

        # send message
        msg.header.stamp = self.get_clock().now().to_msg()
        # Only apply trim in airborne mode
        if self.flight_mode == "takeoff":
            msg.aileron = float(self.aileron)
            msg.elevator = float(self.elevator)
            msg.throttle = float(self.throttle)
            msg.rudder = float(self.rudder)
        else:
            msg.aileron = float(self.aileron) + self.trim_aileron
            msg.elevator = float(self.elevator) + self.trim_elevator
            msg.throttle = float(self.throttle) + self.trim_throttle
            msg.rudder = float(self.rudder) + self.trim_rudder
        msg.mode = int(self.mode)
        self.pub_control.publish(msg)

    def speed_callback(self, msg: TwistStamped):
        """Update velocity estimates from twist message."""
        msg = msg.twist
        self.actual_data["vx_est"] = msg.linear.x
        self.actual_data["vy_est"] = msg.linear.y
        self.actual_data["vz_est"] = msg.linear.z
        v = np.linalg.norm([msg.linear.x, msg.linear.y, msg.linear.z])
        self.actual_data["v_est"] = v

        # Estimate flight path angle
        gamma_new = np.arctan(np.clip(msg.linear.z / msg.linear.x, -1.0, 1.0))
        # self.get_logger().debug(f"Gamma: {gamma_new:.3f}")
        self.actual_data["gamma_est"] = gamma_new

        # Angular rates fall back to twist if IMU is unavailable.
        if not self.imu_received:
            self.actual_data["p_est"] = msg.angular.x
            self.actual_data["q_est"] = msg.angular.y
            self.actual_data["r_est"] = msg.angular.z

        # TODO: (check) Low-pass filter for acceleration estimate
        fc = 100.0
        alpha = np.exp(-2 * np.pi * fc * self.dt)
        vdot_new = v - self.prev_speed
        self._lpf_many({"vdot": vdot_new}, alpha)
        self.prev_speed = v

    def imu_callback(self, msg: Imu):
        """Update attitude and angular-rate estimates from IMU."""
        self.imu_received = True

        self.actual_data["p_est"] = msg.angular_velocity.x
        self.actual_data["q_est"] = msg.angular_velocity.y
        self.actual_data["r_est"] = msg.angular_velocity.z
        self.actual_data["ax_est"] = msg.linear_acceleration.x
        self.actual_data["ay_est"] = msg.linear_acceleration.y
        self.actual_data["az_est"] = msg.linear_acceleration.z

        pose_q = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ])

        SO3_pose = SO3Quat.elem(ca.horzcat(pose_q))
        SO3_321 = SO3EulerB321.from_Quat(SO3_pose).param

        self.actual_data["roll_est"] = float(SO3_321[2])
        self.actual_data["pitch_est"] = float(SO3_321[1])
        self.actual_data["yaw_est"] = float(SO3_321[0])

    def actual_pose_callback(self, msg: PoseStamped):
        """Update position and attitude estimates from pose message."""
        self.actual_data["x_est"] = msg.pose.position.x
        self.actual_data["y_est"] = msg.pose.position.y
        self.actual_data["z_est"] = msg.pose.position.z

        # TODO: check
        pose_q = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ])

        SO3_pose = SO3Quat.elem(ca.horzcat(pose_q))
        SO3_321 = SO3EulerB321.from_Quat(SO3_pose).param

        self.actual_data["roll_est"] = float(SO3_321[2])
        self.actual_data["pitch_est"] = float(SO3_321[1])
        self.actual_data["yaw_est"] = float(SO3_321[0])

    def _lpf(self, name: str, new_value, alpha: float):
        """
        Exponential low-pass update for <name>.
        Uses/creates attributes: <name>_est and <name>_est_last.
        """
        if not (0.0 <= alpha <= 1.0):
            raise ValueError("alpha must be in [0, 1]")
        last_name = f"{name}_est_last"
        est_name = f"{name}_est"

        last = getattr(self, last_name, None)
        if last is None:
            last = new_value

        est = alpha * new_value + (1.0 - alpha) * last
        setattr(self, est_name, est)
        setattr(self, last_name, est)
        return est

    def _lpf_many(self, mapping: dict, alpha: float):
        """Batch low-pass filter updates."""
        for k, v in mapping.items():
            self._lpf(k, v, alpha)


def main(args=None):
    rclpy.init(args=args)
    node = AutoControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down auto control node")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
