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
from cyecca.dynamics.explicit import explicit
from cyecca.dynamics.explicit import input_var
from cyecca.dynamics.explicit import Model
from cyecca.dynamics.explicit import output_var
from cyecca.dynamics.explicit import param
from cyecca.dynamics.explicit import state
import cyecca.sym as cy
import numpy as np


@explicit
class AutolevelController:
    """Unified autolevel controller with all states, inputs, params, outputs."""

    # === States ===
    i_p: float = state(1, 0.0, 'roll rate integral')
    i_q: float = state(1, 0.0, 'pitch rate integral')

    # === Inputs ===
    q: float = input_var(4, desc='quaternion [w,x,y,z]')
    omega: float = input_var(3, desc='angular velocity body frame (rad/s)')
    vel: float = input_var(3, desc='velocity earth frame ENU (m/s)')
    # Manual mode inputs (pass-through)
    ail_manual: float = input_var(desc='manual aileron (rad)')
    elev_manual: float = input_var(desc='manual elevator (rad)')
    rud_manual: float = input_var(desc='manual rudder (rad)')
    thr_manual: float = input_var(desc='manual throttle')
    mode: float = input_var(desc='mode: 0=manual, 1=stabilized')

    # === Parameters ===
    # Outer loop: angle control (slow, 1-3 Hz)
    Kp_phi: float = param(2.5, desc='P gain roll angle (per rad)')
    Kp_theta: float = param(2.0, desc='P gain pitch angle (per rad)')

    # Inner loop: rate damping (fast, 15-25 rad/s)
    Kp_p: float = param(0.6, desc='P gain roll rate (per rad/s)')
    Ki_p: float = param(0.0, desc='I gain roll rate')
    Kp_q: float = param(0.6, desc='P gain pitch rate (per rad/s)')
    Ki_q: float = param(0.0, desc='I gain pitch rate')

    # Yaw damping (passive)
    Kp_r: float = param(0.3, desc='P gain yaw rate damping')

    # Speed control
    Kp_speed: float = param(0.7, desc='P gain speed')
    speed_ref: float = param(20.0, desc='reference speed (m/s)')

    # Trim offsets
    trim_aileron: float = param(0.0, desc='aileron trim offset (rad)')
    trim_elevator: float = param(0.0, desc='elevator trim offset (rad)')
    trim_rudder: float = param(0.0, desc='rudder trim offset (rad)')

    # Stick to attitude mapping
    stick_to_phi: float = param(np.deg2rad(45), desc='aileron stick to roll ref (rad)')
    stick_to_theta: float = param(np.deg2rad(20), desc='elevator stick to pitch ref (rad)')

    # Limits
    phi_max: float = param(np.deg2rad(50), desc='max bank angle (rad)')
    theta_max: float = param(np.deg2rad(30), desc='max pitch angle (rad)')
    ail_min: float = param(-0.5, desc='ail min (rad)')
    ail_max: float = param(0.5, desc='ail max (rad)')
    elev_min: float = param(-0.5, desc='elev min (rad)')
    elev_max: float = param(0.5, desc='elev max (rad)')
    rud_min: float = param(-0.5, desc='rud min (rad)')
    rud_max: float = param(0.5, desc='rud max (rad)')
    thr_min: float = param(0.0, desc='thr min')
    thr_max: float = param(1.0, desc='thr max')

    # === Outputs ===
    ail: float = output_var(desc='aileron (rad)')
    elev: float = output_var(desc='elevator (rad)')
    rud: float = output_var(desc='rudder (rad)')
    thr: float = output_var(desc='throttle')


def _saturate(val, low, high):
    """Saturate value between low and high."""
    return cy.fmin(cy.fmax(val, low), high)


@beartype
def autolevel_controller() -> Model:
    """
    Create SAFE/AS3X style autolevel controller.

    A cascaded gyro-based stabilization system:
    - Inner loop: rate damping (fast, gyro-based)
    - Outer loop: attitude hold (slow, uses quaternion-derived φ/θ)

    Returns
    -------
    Model
        Autolevel controller model with integral states and control outputs

    """
    model = Model(AutolevelController)
    m = model.v  # Unified namespace

    # Extract quaternion and compute Euler angles
    qw = m.q.sym[0]
    qx = m.q.sym[1]
    qy = m.q.sym[2]
    qz = m.q.sym[3]

    # Roll (phi) - aerospace ZYX sequence
    phi = cy.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))

    # Pitch (theta)
    theta = cy.asin(2.0 * (qw * qy - qz * qx))

    # Extract angular rates
    p_meas = m.omega.sym[0]
    q_meas = m.omega.sym[1]
    r_meas = m.omega.sym[2]

    # Airspeed from velocity
    speed_meas = cy.norm_2(m.vel.sym)

    # In stabilized mode, map stick inputs to attitude references
    phi_ref_from_stick = m.ail_manual.sym * m.stick_to_phi.sym
    theta_ref_from_stick = m.elev_manual.sym * m.stick_to_theta.sym

    # Saturate reference angles to max limits
    phi_ref_sat = _saturate(phi_ref_from_stick, -m.phi_max.sym, m.phi_max.sym)
    theta_ref_sat = _saturate(theta_ref_from_stick, -m.theta_max.sym, m.theta_max.sym)

    # Outer loop: angle errors -> rate commands
    e_phi = phi_ref_sat - phi
    e_theta = theta_ref_sat - theta

    p_cmd = m.Kp_phi.sym * e_phi
    q_cmd = m.Kp_theta.sym * e_theta

    # Inner loop: rate errors
    e_p = p_cmd - p_meas
    e_q = q_cmd - q_meas

    # ODEs: Integral states (only accumulate in stabilized mode)
    model.ode(m.i_p, e_p * m.mode.sym)
    model.ode(m.i_q, e_q * m.mode.sym)

    # Stabilized mode control outputs
    ail_stabilized = _saturate(
        m.Kp_p.sym * e_p + m.Ki_p.sym * m.i_p.sym,
        m.ail_min.sym, m.ail_max.sym
    )
    elev_stabilized = _saturate(
        m.Kp_q.sym * e_q + m.Ki_q.sym * m.i_q.sym,
        m.elev_min.sym, m.elev_max.sym
    )
    rud_stabilized = _saturate(-m.Kp_r.sym * r_meas, m.rud_min.sym, m.rud_max.sym)
    e_speed = m.speed_ref.sym - speed_meas
    thr_stabilized = _saturate(m.Kp_speed.sym * e_speed, m.thr_min.sym, m.thr_max.sym)

    # Mode switch: 0 = manual (pass-through), 1 = stabilized (autopilot)
    # Apply trim offsets to all outputs
    ail_out = (
        m.ail_manual.sym * (1.0 - m.mode.sym) + ail_stabilized * m.mode.sym
    ) + m.trim_aileron.sym
    elev_out = (
        m.elev_manual.sym * (1.0 - m.mode.sym) + elev_stabilized * m.mode.sym
    ) + m.trim_elevator.sym
    rud_out = (
        m.rud_manual.sym * (1.0 - m.mode.sym) + rud_stabilized * m.mode.sym
    ) + m.trim_rudder.sym
    thr_out = m.thr_manual.sym * (1.0 - m.mode.sym) + thr_stabilized * m.mode.sym

    # Define outputs
    model.output(m.ail, ail_out)
    model.output(m.elev, elev_out)
    model.output(m.rud, rud_out)
    model.output(m.thr, thr_out)

    model.build()
    return model


__all__ = [
    'autolevel_controller',
    'AutolevelController',
]
