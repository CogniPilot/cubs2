#!/usr/bin/env python3
from cubs2_msgs.msg import AircraftControl
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TwistStamped
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from pathlib import Path
import yaml
import numpy as np
from types import SimpleNamespace
from std_msgs.msg import String
import casadi as ca
from cyecca.lie import SO3Quat, SO3EulerB321
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

def _wrap_pi(a):
    return np.arctan2(np.sin(a), np.cos(a))

class AutoControlNode(Node):
    def __init__(self) -> None:
        super().__init__('auto_control')

        # Publishers
        self.pub_control = self.create_publisher(
            AircraftControl, '/control_auto', 10)
        self.pub_reset = self.create_publisher(Empty, '/reset', 10)
        self.pub_pause = self.create_publisher(Empty, '/pause', 10)

        # Current state
        self.aileron = 0.0
        self.elevator = 0.0
        self.throttle = 0.0
        self.rudder = 0.0
        self.mode = 0  # 0 = manual, 1 = stabilized

        # Trim values (applied as offsets to stick inputs)
        self.trim_aileron = 0.0
        self.trim_elevator = 0.0
        self.trim_throttle = 0.35
        self.trim_rudder = 0.0

        # Reference trajectory subscriber
        self.ref_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'reference_pose',
            self.reference_pose_callback,
            10)

        # True state subscriber (for feedback control)
        self.actual_pose_sub = self.create_subscription(
            PoseStamped,
            '/sportcub/pose',
            self.actual_pose_callback,
            10)

        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/sportcub/velocity',
            self.speed_callback,
            10
        )

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
            'gamma_est': 0.0,
            'vdot_est': 0.0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
            "p_est": 0.0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
            "q_est": 0.0,
            "r_est": 0.0,
        }

        self.ref_data = {
            "des_v": 0.0,
            "des_gamma": 0.0,
            "des_heading": 0.0,
            "des_a": 0.0,
        }

        # Store reference pose
        self.ref_pose = None


        # FROM TECSControl
        self.prev_speed = 0 
        self.flight_mode = "takeoff"
        self.prev_x = 0
        self.prev_y = 0
        self.prev_z = 0
        self.prev_V = 0
        self.prev_roll = 0
        self.prev_pitch = 0
        self.prev_yaw = 0
        self.prev_ref_yaw = 0
        self.dt = 0.01
        self.g = 9.81
        self.thr_max = 7.5 # 4.5 #Maximum Thrust

        self.args = "sim" #Vehicle selection
        this_file = Path(__file__).resolve()
        self.base_dir = this_file.parent / "param"

        self.error_norm_Es_dot_integral = 0 #Integral of error of specific error rate normalized by velocity
        self.error_dist_term_integral = 0 #Integral of error of (V_E_dot/g - gamma_E) term from energy rate distribution adjustment
        self.error_pitch_integral = 0 #Integral of error of pitch
        self.error_thrust_integral = 0 #Integral of error of thrust
        self.error_r_integral = 0 #Integral of error of yaw rate
        self.error_r_last = 0 #Integral of error of yaw rate
        self.error_roll_integral =0 #integral of error of roll
        self.throttle_cmd = None # throttle command
        self.error_xtrack_integral = 0 # Integral of error in side acceleration
        self.error_xtrack_last = 0 # Last value of error in side acceleration

        self.roll_mode = "stabilized"   # choices: "stabilized" | "phi_stick" | "direct"
        self._phi_cmd = 0.0         # last commanded bank
        self._e_phi_int = 0.0       # roll PID integrator

        # self._last_argchi_err = 0.0


        self.timer = self.create_timer(self.dt, self.control_callback)

        self.time = 0
        self.takeoff_time = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.reload_gains()

    def reload_gains(self):
        print(f"[TECSControl] Using gain path: {self.base_dir / f"{self.args}.yaml"}")

        gain_path = self.base_dir / f"{self.args}.yaml"

        if not gain_path.exists():
            raise FileNotFoundError(f"[TECSControl] Gain file not found: {gain_path}")

        with open(gain_path, "r") as f:
            raw = yaml.safe_load(f)

        self.param = SimpleNamespace(**raw)

        self.phi_lim       = np.deg2rad(self.param.phi_lim_deg)
        self.chi_deadband  = np.deg2rad(self.param.chi_deadband_deg)
        self.phi_dot_lim   = np.deg2rad(self.param.phi_dot_lim_deg_s)
        # self.k_chi_rate    = np.deg2rad(self.param.phi_dot_per_rad_deg_s)


        self.mass = self.param.mass
        self.weight = self.mass * self.g
        
        print(f"[TECSControl] Gains reloaded from: {gain_path}")
    

    def compute_thrust_pitch(self, x, y, z, ref_data, vx_est, vy_est, vz_est, V_est, gamma_est, vdot_est):
        # ref data in function of time
        ref_airspeed = ref_data['des_v']
        ref_gamma = ref_data['des_gamma'] #Glide slope angle
        # ref_xtrack_err = ref_data['xtrack_err']
        ref_accel = ref_data['des_a']

        r_V = float(ref_airspeed)  # desired body-frame speed
        r_gamma = float(ref_gamma) # desired flight path angle
        r_V_dot = float(ref_accel) # desired acceleration


        #######################################################
        # Envelope Protection Feature
        drag = 1.0 # Maximum Drag estimate
        # r_gamma = np.clip(r_gamma, -drag/weight, (thr_max-drag)/weight)
        r_V_dot = np.clip(r_V_dot, -drag/self.weight, (self.thr_max-drag)/self.weight)
        # print(f"desired_gamma: {r_gamma:5.2f}, gamma_est: {gamma_est:5.2f} Vdot_ref:{r_V_dot/self.g:5.2f} Vdot_actual:{vdot_est/self.g:5.2f} err_vdot:{(r_V_dot-vdot_est)/self.g:5.2f}")

        #######################################################

        #-------------------Desired Thrust-------------------#
        #Specific energy rate error
        error_norm_Es_dot = (r_gamma - gamma_est) + (r_V_dot - vdot_est) / self.g

        #Desired thrust
        # thrust = self.param.trim_thrust +self.weight * (self.param.K_thrustp * (gamma_est + vdot_est / self.g) + self.param.K_thrusti * self.error_norm_Es_dot_integral)
        thrust_unsat = ( self.param.trim_thrust
               + self.weight * ( self.param.K_thrustp * (gamma_est + vdot_est / self.g)
                                 + self.param.K_thrusti * self.error_norm_Es_dot_integral ) )

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
                -self.param.norm_Es_dot_integral_max, self.param.norm_Es_dot_integral_max
            )

        #-------------------Desired Pitch-------------------#
        #Energy rate distribution term error
        error_dist_term = (r_gamma - gamma_est) - (r_V_dot - vdot_est) / self.g

        #Desired pitch
        pitch_unsat = self.param.K_pitchi * self.error_dist_term_integral - self.param.K_pitchp * (gamma_est - vdot_est / self.g)

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
                -self.param.dist_term_integral_max, self.param.dist_term_integral_max
            )
        # print("r_gamma: {:5.2f} gamma_est: {:5.2f}, r_V_dot:{:5.2f}; vdot_est:{:5.2f}".format(r_gamma, gamma_est,r_V_dot,vdot_est))

        return thrust, pitch
    
    def compute_control(self, ref_data, actual_data, ref_thrust= None, ref_pitch=None):
        # actual data
        x = actual_data['x_est']
        y= actual_data['y_est']
        z = actual_data['z_est']
        roll = actual_data['roll_est']
        pitch = actual_data['pitch_est']
        yaw = actual_data['yaw_est']
        vx_est = actual_data['vx_est']
        vy_est = actual_data['vy_est']
        vz_est = actual_data['vz_est']
        V_est = actual_data['v_est']
        gamma_est = actual_data['gamma_est']
        vdot_est =actual_data['vdot_est']
        p_est = actual_data['p_est']
        q_est = actual_data['q_est']
        r_est = actual_data['r_est']

        #-------------------Compute Reference Outer Loop and Heading------------------#
        #Get desired thrust and pitch (we can remove this if we want to fully separate the two functions during implementation)
        if ref_thrust == None or ref_pitch == None:
            ref_thrust, ref_pitch = self.compute_thrust_pitch(x, y, z, ref_data, vx_est, vy_est, vz_est, V_est, gamma_est, vdot_est) #Outer loop TECS controller
 
        ref_heading = ref_data['des_heading']

        transform = self.tf_buffer.lookup_transform(
            'reference',          # target frame
            'error_relative_to_reference',    # source frame
            rclpy.time.Time()
        )
        cross_track = transform.transform.translation.y
        K_cc = 5
        Heading_max = np.deg2rad(65)
        r_heading = float(ref_heading) - min(cross_track/K_cc * Heading_max,Heading_max) #desired heading yaw rate

        #-------------------Elevator Control-------------------#
        #Compute errors
        pitch = -1 * pitch # remaps nose-up-negative to nose-up-positive (NED)
        error_pitch = _wrap_pi(ref_pitch - pitch)

        q_turn = np.sin(roll) * np.cos(pitch) * np.tan(roll) * self.g / V_est
        error_q = q_turn - q_est # turning pitch
        error_q  = (error_q + np.pi) % (2 * np.pi) - np.pi

        nz_excess = (1.0/np.cos(roll)) - 1.0 # Steady-turn feed-forward
        ele_ff_phi = self.param.K_phi_elev * nz_excess   

        #Integral of pitch error
        self.error_pitch_integral += error_pitch * self.dt
        if self.error_pitch_integral > self.param.pitch_integral_max:
            self.error_pitch_integral = self.param.pitch_integral_max
        elif self.error_pitch_integral < -self.param.pitch_integral_max:
            self.error_pitch_integral = -self.param.pitch_integral_max
        
        #Control commands for elevator
        elev_cmd = self.param.trim_elev + (self.param.K_elevp * error_pitch + self.param.K_elevi * self.error_pitch_integral) + self.param.K_q * error_q
        elev_cmd += ele_ff_phi # feed-forward elevator wrt to roll angle
        elev_cmd = np.clip(elev_cmd, -1,1 ) #Saturation

        #-------------------Throttle Control-------------------#
        thr_cmd = np.clip(ref_thrust / self.thr_max, 0.0 ,1.0) # This bypasses the throttle and assume throttle level is the thrust percentage

        # #-------------------Lateral Heading Control using WP_NAV-------------------#
        chi     = np.arctan2(vy_est, vx_est) # current ground course track angle
        chi_ref = r_heading                  # navigation desired heading
        chi_err = _wrap_pi(chi_ref-chi)*-1
        if abs(chi_err) < self.chi_deadband:   # small deadband to prevent sudden flip near wrap
            chi_err = 0.0

        chi_dot_des = self.param.k_chi * chi_err  # desired yaw rate to correct heading error
        Vg = max(V_est, 0.05) #ground speed, avoid div by zero
        phi_des = np.arctan2(Vg * chi_dot_des , self.g) # Balmer, "Modelling and Control of a Fixed-wing UAV for Landings on Mobile Landing Platforms" (eqn 3.3)

        phi_des = float(np.clip(phi_des, -self.phi_lim, self.phi_lim)) 
        dphi_max = self.phi_dot_lim * self.dt
        phi_des = np.clip(phi_des - self._phi_cmd, -dphi_max, dphi_max) + self._phi_cmd
        self._phi_cmd = float(np.clip(phi_des, -self.phi_lim, self.phi_lim))

        # innter loop roll control modes
        if self.roll_mode == "stabilized":
            # Emulate onboard roll stabilizer in sim: PD on (phi, p) -> aileron
            e_phi = _wrap_pi(self._phi_cmd - roll)
            # Integrator with clamp
            self._e_phi_int += e_phi * self.dt
            self._e_phi_int = float(np.clip(self._e_phi_int, -self.param.i_phi_max, self.param.i_phi_max))

            # D on measured roll-rate
            d_term = - self.param.K_phi_d * p_est    # p_est in rad/s

            ail_cmd = ( self.param.trim_ail
                + self.param.K_phi_p * e_phi
                + self.param.K_phi_i * self._e_phi_int
                + d_term )

            ail_cmd = float(np.clip(ail_cmd, -self.param.da_max, self.param.da_max))

        elif self.roll_mode == "phi_stick": # heading error -> desired bank -> aileron cmd
            # Output bank stick directly (designed to command on real onboard gyro)
            # maps [-phi_lim, +phi_lim] -> [-1, +1]
            ail_cmd = float(np.clip(phi_des / self.phi_lim, -1.0, 1.0))

        else:  # "direct" = yaw error --> aileron cmd
            err_yaw = (r_heading - yaw) # error of haeding angle
            err_yaw  = (err_yaw + np.pi) % (2 * np.pi) - np.pi
            error_r_deriv = (err_yaw - self.error_r_last)/self.dt
            self.error_r_last = err_yaw
            self.error_r_integral += err_yaw*self.dt
            if self.error_r_integral > self.param.r_integral_max:
                self.error_r_integral = self.param.r_integral_max
            elif self.error_r_integral < -self.param.r_integral_max:
                self.error_r_integral = -self.param.r_integral_max

            ail_cmd = (self.param.trim_ail
                   + self.param.K_deltap * err_yaw
                   + self.param.K_deltai * self.error_r_integral
                   + self.param.K_deltad * error_r_deriv)
            ail_cmd = float(np.clip(ail_cmd, -1.0, 1.0))


        # #-------------------Coordinated Turn Control-------------------#
        # TODO?
        rud_cmd = 0
        rud_cmd = np.clip(rud_cmd,-1,1)
        #Set history variables
        self.prev_x = x
        self.prev_y = y
        self.prev_z = z
        self.prev_pitch = pitch
        self.prev_yaw = yaw

        # Set control outputs
        self.aileron =  ail_cmd
        self.elevator = elev_cmd
        self.throttle =  thr_cmd
        self.rudder = rud_cmd

    def reference_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback for reference trajectory pose."""
        self.ref_pose = msg




    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to axis value."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def control_callback(self):
        """Publish current control state as AircraftControl message."""

        ################################### FLIGHT MODE ####################################
        flight_mode_msg = String()
        if (self.actual_data["z_est"] <= 1.0):
            new_mode = "takeoff"
        else:
            new_mode = "airborne"

        if new_mode != self.flight_mode:
            self.get_logger().info(
                "Flight mode changed from: %s to %s" % (self.flight_mode, new_mode)
            )
            self.flight_mode = new_mode
            flight_mode_msg.data = new_mode

        if flight_mode_msg.data == "":  # initialize flight mode
            flight_mode_msg.data = self.flight_mode

        # self.pub_flight_mode.publish(flight_mode_msg)  # Publish Flight mode

        ###########################################################################################

        self.time += self.dt

        if self.flight_mode == "takeoff":
            self.takeoff_time += self.dt

            # Throttle ramp with floor/ceiling
            self.throttle = ca.fmin(1.0, ca.fmax(0.7, self.throttle + 2.0 * self.dt))

            self.rudder = 0.0  # No yaw during takeoff
            self.aileron = 0.0  # Wings-level during takeoff

            # Elevator schedule (taildragger hold-down, then smooth pitch-up)
            v_to = 0.5  # takeoff speed threshold
            e_down = -0.02  # elevator up while accelerating (tail on ground)
            e_up = 0.15  # target pitch-up elevator
            e_rate = 0.40  # max elevator change per second
            if self.actual_data["v_est"] == None:
                self.actual_data["v_est"] = 0.0  # Initialize V_est, assume start at stationary

            self.elevator = ca.if_else(
                self.actual_data["v_est"] < v_to, e_down, ca.fmin(e_up, self.elevator + e_rate * self.dt)
            )

        if self.flight_mode == "airborne":
            planner_v = 5.0
            K_V = 1.0
            des_a = K_V * (
                planner_v - np.abs(self.actual_data["v_est"])
            )  # Desired Acceleration from current velocity

            pose_q = np.array([self.ref_pose.pose.pose.orientation.w, self.ref_pose.pose.pose.orientation.x, self.ref_pose.pose.pose.orientation.y, self.ref_pose.pose.pose.orientation.z])

            SO3_pose = SO3Quat.elem(ca.horzcat(pose_q))
            SO3_321 = SO3EulerB321.from_Quat(SO3_pose).param

            des_heading = float(SO3_321[0]) 


            self.ref_data= {
                "des_v": planner_v,
                "des_gamma": 0,
                "des_heading": des_heading,
                "des_a": des_a,
            }

            # print(self.actual_data)

            # print(self.ref_data)

            self.compute_control(self.ref_data, self.actual_data)

        msg = AircraftControl()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.aileron = float(self.aileron) + self.trim_aileron
        msg.elevator = float(self.elevator) + self.trim_elevator
        msg.throttle = float(self.throttle) + self.trim_throttle
        msg.rudder = float(self.rudder) + self.trim_rudder
        msg.mode = int(self.mode)
        self.pub_control.publish(msg)

    # make callback for speed subscription
    def speed_callback(self, msg: TwistStamped):
        msg = msg.twist
        self.actual_data["vx_est"] = msg.linear.x
        self.actual_data["vy_est"] = msg.linear.y
        self.actual_data["vz_est"] = msg.linear.z
        v = np.linalg.norm([msg.linear.x, msg.linear.y, msg.linear.z])
        self.actual_data["v_est"] = v

        eps = 1e-5
        denom = max(v, eps)
        gamma_new = np.arccos(
            np.clip(msg.linear.x / denom, -1.0, 1.0)
        )

        self.actual_data["gamma_est"] = gamma_new

        self.actual_data["p_est"] = msg.angular.x
        self.actual_data["q_est"] = msg.angular.y
        self.actual_data["r_est"] = msg.angular.z

        # TODO: odd
        fc = 100.0  # Hz
        alpha = np.exp(-2 * np.pi * fc * self.dt)

        vdot_new = (
            v - self.prev_speed
        )# / self.dt # Acceleration magnitutde diff

        self._lpf_many({
            "vdot" : vdot_new
        }, alpha)

        self.prev_speed = v

    
    def actual_pose_callback(self, msg: PoseStamped):
        self.actual_data["x_est"] = msg.pose.position.x
        self.actual_data["y_est"] = msg.pose.position.y
        self.actual_data["z_est"] = msg.pose.position.z

        # TODO Check
        pose_q = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])

        SO3_pose = SO3Quat.elem(ca.horzcat(pose_q))
        SO3_321 = SO3EulerB321.from_Quat(SO3_pose).param

        self.actual_data["roll_est"] = float(SO3_321[2])
        self.actual_data["pitch_est"] = float(SO3_321[1])
        self.actual_data["yaw_est"] = float(SO3_321[0])

        pass

    
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
            last = new_value  # initialize on first call

        est = alpha * new_value + (1.0 - alpha) * last
        setattr(self, est_name, est)
        setattr(self, last_name, est)
        return est

    def _lpf_many(self, mapping: dict, alpha: float):
        """Batch low-pass updates: mapping = {name: new_value}."""
        for k, v in mapping.items():
            self._lpf(k, v, alpha)



def main(args=None):
    rclpy.init(args=args)
    node = AutoControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down auto control node')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()