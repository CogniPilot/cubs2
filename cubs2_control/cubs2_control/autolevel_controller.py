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
"""Autolevel controller for aircraft - SAFE/AS3X style stabilized autopilot."""
from beartype import beartype
import casadi as ca
from cyecca.dynamics import input_var
from cyecca.dynamics import ModelSX
from cyecca.dynamics import output_var
from cyecca.dynamics import param
from cyecca.dynamics import state
from cyecca.dynamics import symbolic
import numpy as np


@symbolic
class AutolevelStates:
    i_p: ca.SX = state(1, 0.0, 'roll rate integral')
    i_q: ca.SX = state(1, 0.0, 'pitch rate integral')


@symbolic
class AutolevelInputs:
    q: ca.SX = input_var(4, desc='quaternion [w,x,y,z]')
    omega: ca.SX = input_var(3, desc='angular velocity body frame (rad/s)')
    v: ca.SX = input_var(3, desc='velocity earth frame ENU (m/s)')
    # Manual mode inputs (pass-through)
    ail_manual: ca.SX = input_var(desc='manual aileron (rad)')
    elev_manual: ca.SX = input_var(desc='manual elevator (rad)')
    rud_manual: ca.SX = input_var(desc='manual rudder (rad)')
    thr_manual: ca.SX = input_var(desc='manual throttle')
    mode: ca.SX = input_var(desc='mode: 0=manual, 1=stabilized')


@symbolic
class AutolevelParams:
    # Outer loop: angle control (slow, 1-3 Hz)
    Kp_phi: ca.SX = param(2.5, 'P gain roll angle (per rad)')  # 1.5-4.0
    Kp_theta: ca.SX = param(2.0, 'P gain pitch angle (per rad)')  # 1.5-4.0

    # Inner loop: rate damping (fast, 15-25 rad/s)
    # 0.4-0.8 per deg/s * 57.3
    Kp_p: ca.SX = param(0.6, 'P gain roll rate (per rad/s)')
    Ki_p: ca.SX = param(0.0, 'I gain roll rate')  # typically tiny or zero
    Kp_q: ca.SX = param(0.6, 'P gain pitch rate (per rad/s)')
    Ki_q: ca.SX = param(0.0, 'I gain pitch rate')

    # Yaw damping (passive)
    Kp_r: ca.SX = param(0.3, 'P gain yaw rate damping')

    # Speed control
    Kp_speed: ca.SX = param(0.7, 'P gain speed')
    speed_ref: ca.SX = param(20.0, 'reference speed (m/s)')

    # Trim offsets (controller-level trim for hands-off flight)
    trim_aileron: ca.SX = param(0.0, 'aileron trim offset (rad)')
    trim_elevator: ca.SX = param(0.0, 'elevator trim offset (rad)')
    trim_rudder: ca.SX = param(0.0, 'rudder trim offset (rad)')

    # Stick to attitude mapping (for stabilized mode)
    stick_to_phi: ca.SX = param(
        np.deg2rad(45), 'aileron stick to roll ref (rad)'
    )  # ±45° at full stick
    stick_to_theta: ca.SX = param(
        np.deg2rad(20), 'elevator stick to pitch ref (rad)'
    )  # ±20° at full stick

    # Limits
    phi_max: ca.SX = param(np.deg2rad(50), 'max bank angle (rad)')
    theta_max: ca.SX = param(np.deg2rad(30), 'max pitch angle (rad)')
    ail_min: ca.SX = param(-0.5, 'ail min (rad)')
    ail_max: ca.SX = param(0.5, 'ail max (rad)')
    elev_min: ca.SX = param(-0.5, 'elev min (rad)')
    elev_max: ca.SX = param(0.5, 'elev max (rad)')
    rud_min: ca.SX = param(-0.5, 'rud min (rad)')
    rud_max: ca.SX = param(0.5, 'rud max (rad)')
    thr_min: ca.SX = param(0.0, 'thr min')
    thr_max: ca.SX = param(1.0, 'thr max')


@symbolic
class AutolevelOutputs:
    ail: ca.SX = output_var(desc='aileron (rad)')
    elev: ca.SX = output_var(desc='elevator (rad)')
    rud: ca.SX = output_var(desc='rudder (rad)')
    thr: ca.SX = output_var(desc='throttle')


@beartype
def autolevel_controller() -> ModelSX:
    """Create SAFE/AS3X style autolevel controller.

    A cascaded gyro-based stabilization system like SAFE/AS3X trainers:
    - Inner loop: rate damping (fast, gyro-based)
    - Outer loop: attitude hold (slow, uses complementary filter φ/θ)

    Roll axis:
        p_cmd = -Kφ * (φ - φ_target)
        δ_a = -Kp_p * (p - p_cmd) - Ki_p * ∫p dt

    Pitch axis:
        q_cmd = -Kθ * (θ - θ_target)
        δ_e = -Kp_q * (q - q_cmd) - Ki_q * ∫q dt

    Yaw axis (passive damping):
        δ_r = -Kr * r

    Maximum bank angle limited to 50 degrees.

    Returns:
        ModelSX: Autolevel controller model with integral states and control outputs
    """
    model = ModelSX.create(
        AutolevelStates,
        AutolevelInputs,
        AutolevelParams,
        output_type=AutolevelOutputs)

    x, u, p, y = model.x, model.u, model.p, model.y

    # Extract quaternion and compute Euler angles
    qw, qx, qy, qz = u.q[0], u.q[1], u.q[2], u.q[3]

    # Roll (phi) - aerospace ZYX sequence
    phi = ca.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))

    # Pitch (theta)
    theta = ca.asin(2.0 * (qw * qy - qz * qx))

    # Extract angular rates
    p_meas = u.omega[0]
    q_meas = u.omega[1]
    r_meas = u.omega[2]

    # Airspeed from velocity (input is in earth frame ENU, compute magnitude
    # directly)
    speed_meas = ca.norm_2(u.v)

    # In stabilized mode, map stick inputs to attitude references
    # In manual mode, this is ignored (mode=0 uses manual inputs directly)
    phi_ref_from_stick = u.ail_manual * p.stick_to_phi
    theta_ref_from_stick = u.elev_manual * p.stick_to_theta

    # Saturate reference angles to max limits
    phi_ref_sat = ca.fmin(ca.fmax(phi_ref_from_stick, -p.phi_max), p.phi_max)
    theta_ref_sat = ca.fmin(
        ca.fmax(theta_ref_from_stick, -p.theta_max), p.theta_max)

    # Outer loop: angle errors -> rate commands
    e_phi = phi_ref_sat - phi
    e_theta = theta_ref_sat - theta

    p_cmd = p.Kp_phi * e_phi
    q_cmd = p.Kp_theta * e_theta

    # Inner loop: rate errors
    e_p = p_cmd - p_meas
    e_q = q_cmd - q_meas

    # Integral states (rate errors)
    # Only accumulate integrals in stabilized mode (mode=1), not manual mode
    # (mode=0)
    f_x = ca.vertcat(e_p * u.mode, e_q * u.mode)

    # Stabilized mode control outputs
    ail_stabilized = model.saturate(
        p.Kp_p * e_p + p.Ki_p * x.i_p, p.ail_min, p.ail_max)
    elev_stabilized = model.saturate(
        p.Kp_q * e_q + p.Ki_q * x.i_q, p.elev_min, p.elev_max)
    rud_stabilized = model.saturate(-p.Kp_r * r_meas, p.rud_min, p.rud_max)
    e_speed = p.speed_ref - speed_meas
    thr_stabilized = model.saturate(p.Kp_speed * e_speed, p.thr_min, p.thr_max)

    # Mode switch: 0 = manual (pass-through), 1 = stabilized (autopilot)
    # Using smooth transition: mode is expected to be 0.0 or 1.0
    # Apply trim offsets to all outputs (both manual and stabilized modes)
    y.ail = (u.ail_manual * (1.0 - u.mode) +
             ail_stabilized * u.mode) + p.trim_aileron
    y.elev = (u.elev_manual * (1.0 - u.mode) +
              elev_stabilized * u.mode) + p.trim_elevator
    y.rud = (u.rud_manual * (1.0 - u.mode) +
             rud_stabilized * u.mode) + p.trim_rudder
    y.thr = u.thr_manual * (1.0 - u.mode) + thr_stabilized * u.mode

    model.build(f_x=f_x, f_y=y.as_vec(), integrator='euler')
    return model


__all__ = [
    'autolevel_controller',
    'AutolevelStates',
    'AutolevelInputs',
    'AutolevelParams',
    'AutolevelOutputs',
]
