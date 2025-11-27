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
"""PID rate controller for aircraft with unified namespace Model API."""
from beartype import beartype
from cyecca.dynamics.explicit import explicit
from cyecca.dynamics.explicit import input_var
from cyecca.dynamics.explicit import Model
from cyecca.dynamics.explicit import output_var
from cyecca.dynamics.explicit import param
from cyecca.dynamics.explicit import state
import cyecca.sym as cy


@explicit
class PIDController:
    """Unified PID controller with all states, inputs, params, outputs."""

    # === States ===
    i_roll: float = state(1, 0.0, 'roll rate integral')
    i_pitch: float = state(1, 0.0, 'pitch rate integral')
    i_yaw: float = state(1, 0.0, 'yaw rate integral')
    i_speed: float = state(1, 0.0, 'speed integral')

    # === Inputs ===
    roll_ref: float = input_var(desc='roll rate ref (rad/s)')
    pitch_ref: float = input_var(desc='pitch rate ref (rad/s)')
    yaw_ref: float = input_var(desc='yaw rate ref (rad/s)')
    speed_ref: float = input_var(desc='speed ref (m/s)')
    roll_meas: float = input_var(desc='roll rate meas (rad/s)')
    pitch_meas: float = input_var(desc='pitch rate meas (rad/s)')
    yaw_meas: float = input_var(desc='yaw rate meas (rad/s)')
    speed_meas: float = input_var(desc='speed meas (m/s)')
    d_roll_meas: float = input_var(desc='d/dt roll rate (rad/s^2)')
    d_pitch_meas: float = input_var(desc='d/dt pitch rate (rad/s^2)')
    d_yaw_meas: float = input_var(desc='d/dt yaw rate (rad/s^2)')
    d_speed_meas: float = input_var(desc='d/dt speed (m/s^2)')

    # === Parameters ===
    Kp_roll: float = param(2.5, desc='P gain roll')
    Ki_roll: float = param(0.8, desc='I gain roll')
    Kd_roll: float = param(0.2, desc='D gain roll')
    Kp_pitch: float = param(3.0, desc='P gain pitch')
    Ki_pitch: float = param(0.9, desc='I gain pitch')
    Kd_pitch: float = param(0.25, desc='D gain pitch')
    Kp_yaw: float = param(1.8, desc='P gain yaw')
    Ki_yaw: float = param(0.5, desc='I gain yaw')
    Kd_yaw: float = param(0.15, desc='D gain yaw')
    Kp_speed: float = param(0.7, desc='P gain speed')
    Ki_speed: float = param(0.3, desc='I gain speed')
    Kd_speed: float = param(0.05, desc='D gain speed')
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
def pid_controller() -> Model:
    """
    Create PID rate controller.

    A multi-axis PID controller for aircraft rate control.
    Controls roll, pitch, yaw rates and airspeed using PID feedback.

    Returns
    -------
    Model
        PID controller model with integral states and control outputs

    """
    model = Model(PIDController)
    m = model.v  # Unified namespace

    # Errors
    e_roll = m.roll_ref.sym - m.roll_meas.sym
    e_pitch = m.pitch_ref.sym - m.pitch_meas.sym
    e_yaw = m.yaw_ref.sym - m.yaw_meas.sym
    e_speed = m.speed_ref.sym - m.speed_meas.sym

    # ODEs: integral states
    model.ode(m.i_roll, e_roll)
    model.ode(m.i_pitch, e_pitch)
    model.ode(m.i_yaw, e_yaw)
    model.ode(m.i_speed, e_speed)

    # PID outputs
    ail_out = _saturate(
        (m.Kp_roll.sym * e_roll + m.Ki_roll.sym * m.i_roll.sym
         - m.Kd_roll.sym * m.d_roll_meas.sym),
        m.ail_min.sym,
        m.ail_max.sym,
    )
    elev_out = _saturate(
        (m.Kp_pitch.sym * e_pitch + m.Ki_pitch.sym * m.i_pitch.sym
         - m.Kd_pitch.sym * m.d_pitch_meas.sym),
        m.elev_min.sym,
        m.elev_max.sym,
    )
    rud_out = _saturate(
        m.Kp_yaw.sym * e_yaw + m.Ki_yaw.sym * m.i_yaw.sym
        - m.Kd_yaw.sym * m.d_yaw_meas.sym,
        m.rud_min.sym,
        m.rud_max.sym,
    )
    thr_out = _saturate(
        (m.Kp_speed.sym * e_speed + m.Ki_speed.sym * m.i_speed.sym
         - m.Kd_speed.sym * m.d_speed_meas.sym),
        m.thr_min.sym,
        m.thr_max.sym,
    )

    # Define outputs
    model.output(m.ail, ail_out)
    model.output(m.elev, elev_out)
    model.output(m.rud, rud_out)
    model.output(m.thr, thr_out)

    model.build()
    return model


__all__ = [
    'pid_controller',
    'PIDController',
]
