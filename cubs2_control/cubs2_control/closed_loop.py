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
"""Closed-loop aircraft model with autolevel controller."""
from beartype import beartype
import casadi as ca
from cubs2_control.autolevel_controller import autolevel_controller
from cubs2_dynamics.sportcub import sportcub
from cyecca.dynamics import input_var
from cyecca.dynamics import ModelSX
from cyecca.dynamics import output_var
from cyecca.dynamics import symbolic


@symbolic
class ClosedLoopInputs:
    """External inputs to closed-loop system (manual control commands)."""

    ail_manual: ca.SX = input_var(desc='manual aileron command [-1, 1]')
    elev_manual: ca.SX = input_var(desc='manual elevator command [-1, 1]')
    rud_manual: ca.SX = input_var(desc='manual rudder command [-1, 1]')
    thr_manual: ca.SX = input_var(desc='manual throttle command [0, 1]')
    mode: ca.SX = input_var(desc='control mode (0=manual, 1=stabilized)')


@symbolic
class ClosedLoopOutputs:
    """Outputs from closed-loop system (actuator commands and forces/moments)."""

    ail: ca.SX = output_var(desc='aileron command to aircraft [-1, 1]')
    elev: ca.SX = output_var(desc='elevator command to aircraft [-1, 1]')
    rud: ca.SX = output_var(desc='rudder command to aircraft [-1, 1]')
    thr: ca.SX = output_var(desc='throttle command to aircraft [0, 1]')
    F: ca.SX = output_var(3, desc='total force on aircraft (N)')
    M: ca.SX = output_var(3, desc='total moment on aircraft (N⋅m)')


@beartype
def closed_loop_sportcub() -> ModelSX:
    """Create closed-loop aircraft model with autolevel controller.

    Uses ModelSX hierarchical composition to combine the aircraft plant
    and autolevel controller into a single integrated model.

    Returns:
        ModelSX: Composed closed-loop system model
    """
    # Create submodels
    plant = sportcub()
    controller = autolevel_controller()

    # Compose into parent model with explicit input/output types
    parent = ModelSX.compose(
        {'plant': plant, 'controller': controller},
        input_type=ClosedLoopInputs,
        output_type=ClosedLoopOutputs,
    )

    # Connect controller inputs (from plant state and parent inputs)
    parent.connect('controller.u.q', 'plant.x.r')
    parent.connect('controller.u.omega', 'plant.x.w')
    # ENU velocity - controller computes airspeed magnitude
    parent.connect('controller.u.v', 'plant.x.v')

    parent.connect('controller.u.ail_manual', 'u.ail_manual')
    parent.connect('controller.u.elev_manual', 'u.elev_manual')
    parent.connect('controller.u.rud_manual', 'u.rud_manual')
    parent.connect('controller.u.thr_manual', 'u.thr_manual')
    parent.connect('controller.u.mode', 'u.mode')

    # Connect plant inputs (from controller outputs)
    parent.connect('plant.u.ail', 'controller.y.ail')
    parent.connect('plant.u.elev', 'controller.y.elev')
    parent.connect('plant.u.rud', 'controller.y.rud')
    parent.connect('plant.u.thr', 'controller.y.thr')

    # Connect parent outputs
    parent.connect('y.ail', 'controller.y.ail')
    parent.connect('y.elev', 'controller.y.elev')
    parent.connect('y.rud', 'controller.y.rud')
    parent.connect('y.thr', 'controller.y.thr')
    parent.connect('y.F', 'plant.y.F_b')
    parent.connect('y.M', 'plant.y.M_b')

    # Build the composed model with single integration loop
    parent.build_composed(integrator='rk4')

    return parent


__all__ = ['closed_loop_sportcub']
