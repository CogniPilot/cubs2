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
    """Outputs from closed-loop system (actuator commands and all aircraft outputs)."""

    # Control commands
    ail: ca.SX = output_var(desc='aileron command to aircraft [-1, 1]')
    elev: ca.SX = output_var(desc='elevator command to aircraft [-1, 1]')
    rud: ca.SX = output_var(desc='rudder command to aircraft [-1, 1]')
    thr: ca.SX = output_var(desc='throttle command to aircraft [0, 1]')

    # Aircraft outputs (from SportCubOutputs)
    Vt: ca.SX = output_var(desc='airspeed (m/s)')
    alpha: ca.SX = output_var(desc='angle of attack (rad)')
    beta: ca.SX = output_var(desc='sideslip (rad)')
    qbar: ca.SX = output_var(desc='dynamic pressure (Pa)')
    q: ca.SX = output_var(4, desc='quaternion (w,x,y,z) for ROS2 compatibility')
    CL: ca.SX = output_var(desc='lift coefficient')
    CD: ca.SX = output_var(desc='drag coefficient')
    FA_b: ca.SX = output_var(3, desc='aero force body (N)')
    FG_b: ca.SX = output_var(3, desc='ground force body (N)')
    FT_b: ca.SX = output_var(3, desc='thrust force body (N)')
    FW_b: ca.SX = output_var(3, desc='weight force body (N)')
    F_b: ca.SX = output_var(3, desc='total force body (N)')
    MA_b: ca.SX = output_var(3, desc='aero moment body (N·m)')
    MG_b: ca.SX = output_var(3, desc='ground moment body (N·m)')
    MT_b: ca.SX = output_var(3, desc='thrust moment body (N·m)')
    MW_b: ca.SX = output_var(3, desc='weight moment body (N·m)')
    M_b: ca.SX = output_var(3, desc='total moment body (N·m)')


@beartype
def closed_loop_sportcub() -> ModelSX:
    """
    Create closed-loop aircraft model with autolevel controller.

    Uses ModelSX hierarchical composition to combine the aircraft plant
    and autolevel controller into a single integrated model.

    Returns
    -------
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

    # Connect parent outputs - control commands from controller
    parent.connect('y.ail', 'controller.y.ail')
    parent.connect('y.elev', 'controller.y.elev')
    parent.connect('y.rud', 'controller.y.rud')
    parent.connect('y.thr', 'controller.y.thr')

    # Connect parent outputs - all aircraft outputs from plant
    parent.connect('y.Vt', 'plant.y.Vt')
    parent.connect('y.alpha', 'plant.y.alpha')
    parent.connect('y.beta', 'plant.y.beta')
    parent.connect('y.qbar', 'plant.y.qbar')
    parent.connect('y.q', 'plant.y.q')
    parent.connect('y.CL', 'plant.y.CL')
    parent.connect('y.CD', 'plant.y.CD')
    parent.connect('y.FA_b', 'plant.y.FA_b')
    parent.connect('y.FG_b', 'plant.y.FG_b')
    parent.connect('y.FT_b', 'plant.y.FT_b')
    parent.connect('y.FW_b', 'plant.y.FW_b')
    parent.connect('y.F_b', 'plant.y.F_b')
    parent.connect('y.MA_b', 'plant.y.MA_b')
    parent.connect('y.MG_b', 'plant.y.MG_b')
    parent.connect('y.MT_b', 'plant.y.MT_b')
    parent.connect('y.MW_b', 'plant.y.MW_b')
    parent.connect('y.M_b', 'plant.y.M_b')

    # Build the composed model with single integration loop
    parent.build_composed(integrator='rk4')

    return parent


__all__ = ['closed_loop_sportcub']
