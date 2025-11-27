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
"""SportCub 6-DOF aircraft dynamics with unified namespace model API."""
from typing import Literal

from beartype import beartype

AttitudeRep = Literal['quat', 'euler']
import casadi as ca
from cyecca.dynamics.explicit import explicit
from cyecca.dynamics.explicit import input_var
from cyecca.dynamics.explicit import Model
from cyecca.dynamics.explicit import output_var
from cyecca.dynamics.explicit import param
from cyecca.dynamics.explicit import state
import cyecca.lie as lie
from cyecca.lie.group_so3 import SO3DcmLieGroupElement
import numpy as np

# ============================================================================
# Unified SportCub Model Classes
# ============================================================================

@explicit
class _Base:
    # States
    #==========================================================
    p: float = state(3, [0, 0, 0], 'position in earth frame ENU (m)')
    v: float = state(3, [0, 0, 0], 'velocity in earth frame ENU (m/s)')
    w: float = state(3, [0, 0, 0], 'angular velocity in body frame FLU (rad/s)')
    # rotation will be defined in subclasses

    # Inputs
    #==========================================================
    ail: float = input_var(desc='aileron (normalized -1 to 1)')
    elev: float = input_var(desc='elevator (normalized -1 to 1)')
    rud: float = input_var(desc='rudder (normalized -1 to 1)')
    thr: float = input_var(desc='throttle (normalized 0 to 1)')

    # Physical properties
    thr_max: float = param(0.30, desc='maximum thrust (N)')
    m: float = param(0.065, desc='mass (kg)')
    S: float = param(0.055, desc='wing area (m^2)')
    rho: float = param(1.225, desc='air density (kg/m^3)')
    g: float = param(9.81, desc='gravity (m/s^2)')

    # Inertias
    Jx: float = param(8.0e-4, desc='roll inertia (kg·m^2)')
    Jy: float = param(1.2e-3, desc='pitch inertia (kg·m^2)')
    Jz: float = param(1.8e-3, desc='yaw inertia (kg·m^2)')
    Jxz: float = param(1.0e-4, desc='product of inertia (kg·m^2)')

    # Geometry
    cbar: float = param(0.09, desc='mean chord (m)')
    span: float = param(0.617, desc='wingspan (m)')
    wing_incidence: float = param(np.deg2rad(6.0), desc='wing incidence angle (rad)')

    # Pitch coefficients
    Cm0: float = param(0.0, desc='pitch moment coeff')
    Cma: float = param(-0.8, desc='pitch moment slope (1/rad)')
    Cmq: float = param(-12.0, desc='pitch damping (1/rad)')

    # Lift & drag
    CL0: float = param(0.5, desc='lift coeff at zero AoA')
    CLa: float = param(4.7, desc='lift slope (1/rad)')
    CD0: float = param(0.06, desc='parasitic drag')
    k_ind: float = param(0.09, desc='induced drag factor')
    CD0_fp: float = param(0.30, desc='flat plate drag')
    CY_fp: float = param(0.50, desc='flat plate sideforce')

    # Control effectiveness
    Clda: float = param(0.05, desc='aileron roll (1/rad)')
    Cldr: float = param(0.006, desc='rudder roll (1/rad)')
    Cmde: float = param(0.3, desc='elevator pitch (1/rad)')
    Cndr: float = param(0.015, desc='rudder yaw (1/rad)')
    Cnda: float = param(0.006, desc='aileron yaw (1/rad)')
    CYda: float = param(0.004, desc='aileron sideforce (1/rad)')
    CYdr: float = param(-0.015, desc='rudder sideforce (1/rad)')

    # Stability & damping
    Cnb: float = param(0.06, desc='yaw stiffness (1/rad)')
    CYb: float = param(-0.50, desc='sideslip sideforce (1/rad)')
    CYr: float = param(0.20, desc='yaw rate sideforce')
    CYp: float = param(-0.15, desc='roll rate sideforce')
    Clb: float = param(-0.25, desc='dihedral effect (1/rad)')
    Clp: float = param(-0.50, desc='roll damping')
    Clr: float = param(0.15, desc='yaw-roll coupling')
    Cnr: float = param(-0.15, desc='yaw damping')
    Cnp: float = param(0.010, desc='roll-yaw coupling')

    # Limits
    blend_width: float = param(np.deg2rad(5), desc='stall blend width (rad)')
    max_defl_ail: float = param(np.deg2rad(30), desc='max aileron (rad)')
    max_defl_elev: float = param(np.deg2rad(24), desc='max elevator (rad)')
    max_defl_rud: float = param(np.deg2rad(20), desc='max rudder (rad)')
    alpha_stall: float = param(np.deg2rad(20), desc='stall AoA (rad)')

    disable_aero: float = param(0.0, desc='disable aero (debug)')
    disable_gf: float = param(0.0, desc='disable ground forces (debug)')

    # Ground contact
    ground_wn: float = param(350.0, desc='ground frequency (rad/s)')
    ground_zeta: float = param(0.6, desc='ground damping ratio')
    ground_c_xy: float = param(0.05, desc='lateral damping (N·s/m)')
    ground_mu: float = param(0.15, desc='friction coefficient')
    ground_max_force_per_wheel: float = param(20.0, desc='max normal force per wheel (N)')
    tailwheel_steer_gain: float = param(0.03, desc='tail wheel steering effectiveness (moment/rad)')

    # Outputs
    #==========================================================
    Vt: float = output_var(desc='airspeed (m/s)')
    alpha: float = output_var(desc='angle of attack (rad)')
    beta: float = output_var(desc='sideslip (rad)')
    qbar: float = output_var(desc='dynamic pressure (Pa)')
    q: float = output_var(4, desc='quaternion (w,x,y,z) for ROS2 compatibility')
    CL: float = output_var(desc='lift coefficient')
    CD: float = output_var(desc='drag coefficient')
    FA_b: float = output_var(3, desc='aero force body (N)')
    FG_b: float = output_var(3, desc='ground force body (N)')
    FT_b: float = output_var(3, desc='thrust force body (N)')
    FW_b: float = output_var(3, desc='weight force body (N)')
    F_b: float = output_var(3, desc='total force body (N)')
    MA_b: float = output_var(3, desc='aero moment body (N·m)')
    MG_b: float = output_var(3, desc='ground moment body (N·m)')
    MT_b: float = output_var(3, desc='thrust moment body (N·m)')
    MW_b: float = output_var(3, desc='weight moment body (N·m)')
    M_b: float = output_var(3, desc='total moment body (N·m)')

@explicit
class SportCubQuat(_Base):
    """Quaternion attitude representation."""
    r: float = state(4, [1, 0, 0, 0], 'quaternion [w,x,y,z] (ROS2 convention)')

@explicit
class SportCubEuler(_Base):
    """Euler angle attitude representation."""
    r: float = state(3, [0, 0, 0], 'Euler angles [psi,theta,phi] (rad)')


# ============================================================================
# Helper Functions
# ============================================================================


@beartype
def clamp(val: ca.SX, low: ca.SX | int | float,
          high: ca.SX | int | float) -> ca.SX:
    return ca.fmin(ca.fmax(val, low), high)


@beartype
def casadi_min_withcargo(
        costs: list[ca.SX], cargos: list[ca.SX]) -> tuple[ca.SX, ca.SX]:
    """Branch-free CasADi minimum with associated cargo."""
    if len(costs) == 1:
        return costs[0], cargos[0]

    current_min_cost = costs[0]
    current_mincargo = cargos[0]

    for i in range(1, len(costs)):
        is_lower = costs[i] < current_min_cost
        current_min_cost = ca.if_else(is_lower, costs[i], current_min_cost)
        current_mincargo = ca.if_else(is_lower, cargos[i], current_mincargo)

    return current_min_cost, current_mincargo


@beartype
def flu_to_frd(v_flu: ca.SX) -> ca.SX:
    """Convert ROS FLU to classical FRD."""
    return ca.vertcat(v_flu[0], -v_flu[1], -v_flu[2])


@beartype
def frd_to_flu(v_frd: ca.SX) -> ca.SX:
    """Convert classical FRD back to ROS FLU."""
    return ca.vertcat(v_frd[0], -v_frd[1], -v_frd[2])


@beartype
def wind_axes_from_velocity_frd(
    v_frd: ca.SX, eps: float = 1e-6
) -> tuple[ca.SX, ca.SX, ca.SX, ca.SX]:
    """Build wind-frame DCM from velocity vector (FRD convention)."""
    U, V, W = v_frd[0], v_frd[1], v_frd[2]
    Vt = ca.norm_2(v_frd) + eps
    w_x = v_frd / Vt

    b_x = ca.SX([1, 0, 0])
    b_z = ca.SX([0, 0, 1])

    cost_z = ca.fabs(ca.dot(w_x, b_z))
    cost_x = ca.fabs(ca.dot(w_x, b_x))

    _, ref = casadi_min_withcargo([cost_z, cost_x], [b_z, b_x])

    ref_parallel = ca.dot(ref, w_x) * w_x
    w_z_temp = ref - ref_parallel
    w_z = w_z_temp / (ca.norm_2(w_z_temp) + eps)
    w_y = ca.cross(w_z, w_x)

    R_b_wind = ca.horzcat(w_x, w_y, w_z)

    V_xz = ca.sqrt(U * U + W * W) + eps
    alpha = ca.atan2(W, U)
    beta = ca.atan2(V, V_xz)

    return R_b_wind, Vt, alpha, beta


def _compute_aero_coefficients(
    Vt, alpha, beta,
    w_sym,  # angular velocity (w)
    ail_sym, elev_sym, rud_sym,  # control inputs
    # parameters
    span, cbar, alpha_stall, blend_width,
    CL0, CLa, CD0, k_ind, CD0_fp, CY_fp,
    Clda, Cldr, Clb, Clp, Clr,
    Cm0, Cma, Cmde, Cmq,
    Cnb, Cndr, Cnda, Cnp, Cnr,
    CYb, CYda, CYdr, CYp, CYr,
    max_defl_ail, max_defl_elev, max_defl_rud,
) -> dict[str, ca.SX]:
    """Compute aerodynamic coefficients with smooth stall model."""
    # Convert body angular rates to FRD convention
    w_b_frd = flu_to_frd(w_sym)
    P, Q, R = w_b_frd[0], w_b_frd[1], w_b_frd[2]

    # Convert normalized control inputs to radians
    ail_rad = clamp(max_defl_ail * ail_sym, -max_defl_ail, max_defl_ail)
    elev_rad = clamp(max_defl_elev * elev_sym, -max_defl_elev, max_defl_elev)
    rud_rad = clamp(max_defl_rud * rud_sym * -1, -max_defl_rud, max_defl_rud)

    sigma = (1 + ca.tanh((alpha - alpha_stall) / blend_width)) / 2

    CL_lin = CL0 + CLa * alpha
    CL_fp = 2 * ca.sin(alpha) * ca.cos(alpha)
    CL = (1 - sigma) * CL_lin + sigma * CL_fp

    CD_lin = CD0 + k_ind * CL_lin**2
    CD_fp_val = CD0_fp + 2 * ca.sin(alpha) ** 2
    CD = (1 - sigma) * CD_lin + sigma * CD_fp_val

    CY_lin = (
        CYb * beta +
        CYda * ail_rad +
        CYdr * rud_rad +
        CYp * (span / (2 * Vt)) * P +
        CYr * (span / (2 * Vt)) * R
    )
    CY_fp_val = CY_fp * ca.sin(beta) * ca.cos(alpha)
    CY = (1 - sigma) * CY_lin + sigma * CY_fp_val

    Cl = (
        Clda * ail_rad +
        Cldr * rud_rad +
        Clb * beta +
        Clp * (span / (2 * Vt)) * P +
        Clr * (span / (2 * Vt)) * R
    )
    Cm = Cm0 + Cma * alpha + Cmde * elev_rad + Cmq * (cbar / (2 * Vt)) * Q
    Cn = (
        Cnb * beta +
        Cndr * rud_rad +
        Cnda * ail_rad +
        Cnp * (span / (2 * Vt)) * P +
        Cnr * (span / (2 * Vt)) * R
    )

    return {'CL': CL, 'CD': CD, 'CY': CY, 'Cl': Cl, 'Cm': Cm, 'Cn': Cn}


def _compute_ground_forces(
    p_sym, v_sym, w_sym,  # state: position, velocity, angular velocity
    rud_sym,  # control: rudder
    R_eb: SO3DcmLieGroupElement,
    # parameters
    m, ground_wn, ground_zeta, ground_c_xy, ground_mu,
    ground_max_force_per_wheel, max_defl_rud, tailwheel_steer_gain,
) -> tuple[ca.SX, ca.SX]:
    """Compute ground reaction forces and moments."""
    left_wheel_b = ca.SX([0.1, 0.1, -0.1])
    right_wheel_b = ca.SX([0.1, -0.1, -0.1])
    tail_wheel_b = ca.SX([-0.4, 0.0, 0.0])

    wheel_b_list = [left_wheel_b, right_wheel_b, tail_wheel_b]
    R_be = R_eb.inverse()

    v_b = R_be @ v_sym

    ground_k = m * ground_wn**2
    ground_c_vert = 2.0 * ground_zeta * m * ground_wn

    ground_force_e_list = []
    MG_b = ca.SX.zeros(3)

    for i, wheel_b in enumerate(wheel_b_list):
        wheel_e = R_eb @ wheel_b
        pos_wheel_e = p_sym + wheel_e
        vel_wheel_b = v_b + ca.cross(w_sym, wheel_b)
        vel_wheel_e = R_eb @ vel_wheel_b
        penetration = -pos_wheel_e[2]

        normal_force_unclamped = penetration * ground_k - vel_wheel_e[2] * ground_c_vert
        normal_force = clamp(normal_force_unclamped, 0, ground_max_force_per_wheel)

        lateral_damp = ca.vertcat(-vel_wheel_e[0] * ground_c_xy, -vel_wheel_e[1] * ground_c_xy)
        lateral_mag = ca.norm_2(lateral_damp) + 1e-9
        max_lateral = ground_mu * normal_force
        scale_lat = ca.if_else(lateral_mag > max_lateral, max_lateral / lateral_mag, 1.0)
        lateral_limited = scale_lat * lateral_damp

        force_contact_e = ca.vertcat(lateral_limited[0], lateral_limited[1], normal_force)
        force_e = ca.if_else(pos_wheel_e[2] < 0.0, force_contact_e, ca.vertcat(0, 0, 0))
        ground_force_e_list.append(force_e)

        force_b = R_be @ force_e
        MG_b += ca.cross(wheel_b, force_b)

        # Tail wheel steering
        if i == 2:
            rud_rad = max_defl_rud * clamp(rud_sym, -1, 1)
            forward_speed = vel_wheel_e[0]
            speed_sq = forward_speed * forward_speed
            lateral_force_factor = ca.sqrt(speed_sq + 1.0) - 1.0
            tailwheel_moment_z = tailwheel_steer_gain * rud_rad * lateral_force_factor
            tailwheel_moment_b = ca.if_else(
                pos_wheel_e[2] < 0.0,
                ca.vertcat(0, 0, tailwheel_moment_z),
                ca.vertcat(0, 0, 0),
            )
            MG_b += tailwheel_moment_b

    FG_b = R_be @ ca.sum2(ca.horzcat(*ground_force_e_list))

    return FG_b, MG_b


# ============================================================================
# SportCub Factory
# ============================================================================


@beartype
def sportcub(attitude_rep: AttitudeRep = 'quat') -> Model:
    """
    Create SportCub 6-DOF aircraft model.

    Frames: e=earth (ENU inertial), b=body (FLU), wind=aligned with velocity

    Args:
        attitude_rep: 'quat' (default) for quaternions (ROS2 compatible, no singularities)
                     'euler' for Euler angles (better for linearization analysis)

    Returns:
        Model instance with selected attitude representation
    """
    # Select state class based on attitude representation
    if attitude_rep == 'quat':
        model_type = SportCubQuat
    elif attitude_rep == 'euler':
        model_type = SportCubEuler
    else:
        raise ValueError(f'Invalid attitude representation: {attitude_rep}')
    
    model = Model(model_type)
    
    # Shortcuts for typed views
    x = model.x  # states
    u = model.u  # inputs
    p = model.p  # parameters
    y = model.y  # outputs

    # Build inertia tensor
    J = ca.SX.zeros(3, 3)
    J[0, 0] = p.Jx.sym
    J[1, 1] = p.Jy.sym
    J[2, 2] = p.Jz.sym
    J[0, 2] = p.Jxz.sym
    J[2, 0] = p.Jxz.sym

    xAxis = ca.vertcat(1, 0, 0)
    zAxis = ca.vertcat(0, 0, 1)

    # Get rotation matrix and attitude element based on representation
    if attitude_rep == 'quat':
        R_eb = lie.SO3Dcm.from_Quat(lie.SO3Quat.elem(x.r.sym))
        att = lie.SO3Quat.elem(x.r.sym)
    else:  # euler
        R_eb = lie.SO3Dcm.from_Euler(lie.SO3EulerB321.elem(x.r.sym))
        att = lie.SO3EulerB321.elem(x.r.sym)

    R_be = R_eb.inverse()

    # Convert earth velocity to body frame for aero calculations
    v_b = R_be @ x.v.sym
    v_frd = flu_to_frd(v_b)
    R_b_wind, Vt, alpha_body, beta = wind_axes_from_velocity_frd(v_frd)
    alpha = alpha_body + p.wing_incidence.sym

    # Compute aerodynamic coefficients
    coeff = _compute_aero_coefficients(
        Vt, alpha, beta,
        x.w.sym,
        u.ail.sym, u.elev.sym, u.rud.sym,
        p.span.sym, p.cbar.sym, p.alpha_stall.sym, p.blend_width.sym,
        p.CL0.sym, p.CLa.sym, p.CD0.sym, p.k_ind.sym, p.CD0_fp.sym, p.CY_fp.sym,
        p.Clda.sym, p.Cldr.sym, p.Clb.sym, p.Clp.sym, p.Clr.sym,
        p.Cm0.sym, p.Cma.sym, p.Cmde.sym, p.Cmq.sym,
        p.Cnb.sym, p.Cndr.sym, p.Cnda.sym, p.Cnp.sym, p.Cnr.sym,
        p.CYb.sym, p.CYda.sym, p.CYdr.sym, p.CYp.sym, p.CYr.sym,
        p.max_defl_ail.sym, p.max_defl_elev.sym, p.max_defl_rud.sym,
    )

    qbar = 0.5 * p.rho.sym * Vt**2

    # Aerodynamic forces
    FA_wind_frd = qbar * p.S.sym * ca.vertcat(-coeff['CD'], coeff['CY'], -coeff['CL'])
    FA_body_frd = R_b_wind @ FA_wind_frd
    FA_b = frd_to_flu(FA_body_frd)

    # Aerodynamic moments
    MA_frd = qbar * p.S.sym * ca.vertcat(
        p.span.sym * coeff['Cl'],
        p.cbar.sym * coeff['Cm'],
        p.span.sym * coeff['Cn'],
    )
    MA_b = frd_to_flu(MA_frd)

    # Ground forces
    FG_b, MG_b = _compute_ground_forces(
        x.p.sym, x.v.sym, x.w.sym,
        u.rud.sym,
        R_eb,
        p.m.sym, p.ground_wn.sym, p.ground_zeta.sym, p.ground_c_xy.sym, p.ground_mu.sym,
        p.ground_max_force_per_wheel.sym, p.max_defl_rud.sym, p.tailwheel_steer_gain.sym,
    )

    # Thrust
    FT_b = p.thr_max.sym * clamp(u.thr.sym, 0, 1) * xAxis
    MT_b = ca.SX.zeros(3)

    # Weight
    FW_b = R_be @ (-p.m.sym * p.g.sym * zAxis)
    MW_b = ca.SX.zeros(3)

    # Total forces and moments
    F_b = (1.0 - p.disable_aero.sym) * FA_b + (1.0 - p.disable_gf.sym) * FG_b + FT_b + FW_b
    M_b = (1.0 - p.disable_aero.sym) * MA_b + (1.0 - p.disable_gf.sym) * MG_b + MT_b + MW_b

    # Define outputs
    model.output(y.Vt, Vt)
    model.output(y.alpha, alpha)
    model.output(y.beta, beta)
    model.output(y.qbar, qbar)
    model.output(y.CL, coeff['CL'])
    model.output(y.CD, coeff['CD'])
    model.output(y.FA_b, FA_b)
    model.output(y.FG_b, FG_b)
    model.output(y.FT_b, FT_b)
    model.output(y.FW_b, FW_b)
    model.output(y.F_b, F_b)
    model.output(y.MA_b, MA_b)
    model.output(y.MG_b, MG_b)
    model.output(y.MT_b, MT_b)
    model.output(y.MW_b, MW_b)
    model.output(y.M_b, M_b)

    # Quaternion output for ROS2 compatibility
    if attitude_rep == 'quat':
        model.output(y.q, x.r.sym)
    else:  # euler
        quat_group = lie.SO3Quat.from_Euler(att)
        model.output(y.q, quat_group.param)

    # Dynamics equations
    p_dot = x.v.sym
    F_e = R_eb @ F_b
    v_dot = F_e / p.m.sym

    # Attitude derivative
    att_dot = att.right_jacobian() @ x.w.sym

    # Angular velocity derivative
    w_dot = ca.inv(J) @ (M_b - ca.cross(x.w.sym, J @ x.w.sym))

    # Define ODEs
    model.ode(x.p, p_dot)
    model.ode(x.v, v_dot)
    model.ode(x.r, att_dot)
    model.ode(x.w, w_dot)

    model.build(integrator='rk4', integrator_options={'N': 100})
    return model


__all__ = [
    'sportcub',
    'StatesQuat',
    'StatesEuler',
    'Inputs',
    'Parameters',
    'Outputs',
]
