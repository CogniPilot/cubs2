"""PID rate controller for aircraft."""

import casadi as ca
from beartype import beartype
from cubs2_dynamics.model import ModelSX, input_var, output_var, param, state, symbolic


@symbolic
class PIDStates:
    i_roll: ca.SX = state(1, 0.0, "roll rate integral")
    i_pitch: ca.SX = state(1, 0.0, "pitch rate integral")
    i_yaw: ca.SX = state(1, 0.0, "yaw rate integral")
    i_speed: ca.SX = state(1, 0.0, "speed integral")


@symbolic
class PIDInputs:
    roll_ref: ca.SX = input_var(desc="roll rate ref (rad/s)")
    pitch_ref: ca.SX = input_var(desc="pitch rate ref (rad/s)")
    yaw_ref: ca.SX = input_var(desc="yaw rate ref (rad/s)")
    speed_ref: ca.SX = input_var(desc="speed ref (m/s)")
    roll_meas: ca.SX = input_var(desc="roll rate meas (rad/s)")
    pitch_meas: ca.SX = input_var(desc="pitch rate meas (rad/s)")
    yaw_meas: ca.SX = input_var(desc="yaw rate meas (rad/s)")
    speed_meas: ca.SX = input_var(desc="speed meas (m/s)")
    d_roll_meas: ca.SX = input_var(desc="d/dt roll rate (rad/s^2)")
    d_pitch_meas: ca.SX = input_var(desc="d/dt pitch rate (rad/s^2)")
    d_yaw_meas: ca.SX = input_var(desc="d/dt yaw rate (rad/s^2)")
    d_speed_meas: ca.SX = input_var(desc="d/dt speed (m/s^2)")


@symbolic
class PIDParams:
    Kp_roll: ca.SX = param(2.5, "P gain roll")
    Ki_roll: ca.SX = param(0.8, "I gain roll")
    Kd_roll: ca.SX = param(0.2, "D gain roll")
    Kp_pitch: ca.SX = param(3.0, "P gain pitch")
    Ki_pitch: ca.SX = param(0.9, "I gain pitch")
    Kd_pitch: ca.SX = param(0.25, "D gain pitch")
    Kp_yaw: ca.SX = param(1.8, "P gain yaw")
    Ki_yaw: ca.SX = param(0.5, "I gain yaw")
    Kd_yaw: ca.SX = param(0.15, "D gain yaw")
    Kp_speed: ca.SX = param(0.7, "P gain speed")
    Ki_speed: ca.SX = param(0.3, "I gain speed")
    Kd_speed: ca.SX = param(0.05, "D gain speed")
    ail_min: ca.SX = param(-0.5, "ail min (rad)")
    ail_max: ca.SX = param(0.5, "ail max (rad)")
    elev_min: ca.SX = param(-0.5, "elev min (rad)")
    elev_max: ca.SX = param(0.5, "elev max (rad)")
    rud_min: ca.SX = param(-0.5, "rud min (rad)")
    rud_max: ca.SX = param(0.5, "rud max (rad)")
    thr_min: ca.SX = param(0.0, "thr min")
    thr_max: ca.SX = param(1.0, "thr max")


@symbolic
class PIDOutputs:
    ail: ca.SX = output_var(desc="aileron (rad)")
    elev: ca.SX = output_var(desc="elevator (rad)")
    rud: ca.SX = output_var(desc="rudder (rad)")
    thr: ca.SX = output_var(desc="throttle")


@beartype
def pid_controller() -> ModelSX:
    """Create PID rate controller.

    A multi-axis PID controller for aircraft rate control.
    Controls roll, pitch, yaw rates and airspeed using PID feedback.

    Returns:
        ModelSX: PID controller model with integral states and control outputs
    """
    model = ModelSX.create(PIDStates, PIDInputs, PIDParams, output_type=PIDOutputs)

    x, u, p, y = model.x, model.u, model.p, model.y

    e_roll = u.roll_ref - u.roll_meas
    e_pitch = u.pitch_ref - u.pitch_meas
    e_yaw = u.yaw_ref - u.yaw_meas
    e_speed = u.speed_ref - u.speed_meas

    f_x = ca.vertcat(e_roll, e_pitch, e_yaw, e_speed)

    y.ail = model.saturate(
        p.Kp_roll * e_roll + p.Ki_roll * x.i_roll - p.Kd_roll * u.d_roll_meas,
        p.ail_min,
        p.ail_max,
    )
    y.elev = model.saturate(
        p.Kp_pitch * e_pitch + p.Ki_pitch * x.i_pitch - p.Kd_pitch * u.d_pitch_meas,
        p.elev_min,
        p.elev_max,
    )
    y.rud = model.saturate(
        p.Kp_yaw * e_yaw + p.Ki_yaw * x.i_yaw - p.Kd_yaw * u.d_yaw_meas,
        p.rud_min,
        p.rud_max,
    )
    y.thr = model.saturate(
        p.Kp_speed * e_speed + p.Ki_speed * x.i_speed - p.Kd_speed * u.d_speed_meas,
        p.thr_min,
        p.thr_max,
    )

    model.build(f_x=f_x, f_y=y.as_vec(), integrator="euler")
    return model


__all__ = [
    "pid_controller",
    "PIDStates",
    "PIDInputs",
    "PIDParams",
    "PIDOutputs",
]
