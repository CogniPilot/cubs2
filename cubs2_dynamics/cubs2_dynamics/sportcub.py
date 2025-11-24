"""SportCub 6-DOF aircraft dynamics with type-safe symbolic framework."""

from typing import ClassVar, Literal, Union

import casadi as ca
import cyecca.lie as lie
import numpy as np
from beartype import beartype
from cubs2_dynamics.model import ModelSX, input_var, output_var, param, state, symbolic
from cubs2_dynamics.trim_fixed_wing import (
    classify_aircraft_modes,
    find_trim_fixed_wing,
    print_mode_summary,
)
from cyecca.lie.group_so3 import SO3DcmLieGroupElement

# ============================================================================
# State, Input, Parameter, Output Definitions
# ============================================================================

# SO3 attitude representation type
AttitudeRep = Literal["quat", "euler"]


@symbolic
class SportCubStatesQuat:
    """Quaternion attitude representation."""

    attitude_rep: ClassVar[AttitudeRep] = "quat"  # Class attribute, not part of state vector

    p: ca.SX = state(3, [0, 0, 0], "position in earth frame (m)")
    v: ca.SX = state(3, [0, 0, 0], "velocity in earth frame (m/s)")
    r: ca.SX = state(4, [1, 0, 0, 0], "attitude quaternion (w,x,y,z) earth to body")
    w: ca.SX = state(3, [0, 0, 0], "angular velocity in body frame (rad/s)")


@symbolic
class SportCubStatesEuler:
    """Euler angle attitude representation."""

    attitude_rep: ClassVar[AttitudeRep] = "euler"  # Class attribute, not part of state vector

    p: ca.SX = state(3, [0, 0, 0], "position in earth frame (m)")
    v: ca.SX = state(3, [0, 0, 0], "velocity in earth frame (m/s)")
    r: ca.SX = state(3, [0, 0, 0], "Euler angles 3-2-1 (phi,theta,psi) rad")
    w: ca.SX = state(3, [0, 0, 0], "angular velocity in body frame (rad/s)")


# Union type for functions that work with both representations
SportCubStates = Union[SportCubStatesQuat, SportCubStatesEuler]


@symbolic
class SportCubInputs:
    ail: ca.SX = input_var(desc="aileron (normalized -1 to 1)")
    elev: ca.SX = input_var(desc="elevator (normalized -1 to 1)")
    rud: ca.SX = input_var(desc="rudder (normalized -1 to 1)")
    thr: ca.SX = input_var(desc="throttle (normalized 0 to 1)")


@symbolic
class SportCubParams:
    # Physical properties
    thr_max: ca.SX = param(0.60, "maximum thrust (N)")
    m: ca.SX = param(0.065, "mass (kg)")  # 65g actual weight
    S: ca.SX = param(0.055, "wing area (m^2)")  # Moderate wing area
    rho: ca.SX = param(1.225, "air density (kg/m^3)")
    g: ca.SX = param(9.81, "gravity (m/s^2)")

    # Inertias
    Jx: ca.SX = param(8.0e-4, "roll inertia (kg·m^2)")
    Jy: ca.SX = param(9.0e-4, "pitch inertia (kg·m^2)")
    Jz: ca.SX = param(1.5e-3, "yaw inertia (kg·m^2)")
    Jxz: ca.SX = param(0.0e-4, "product of inertia (kg·m^2)")

    # Geometry
    cbar: ca.SX = param(0.09, "mean chord (m)")
    span: ca.SX = param(0.617, "wingspan (m)")
    # Wing mounting angle
    wing_incidence: ca.SX = param(np.deg2rad(3.0), "wing incidence angle (rad)")

    # Pitch coefficients
    Cm0: ca.SX = param(0.05, "pitch moment coeff")
    Cma: ca.SX = param(-0.5, "pitch moment slope (1/rad)")
    Cmq: ca.SX = param(-3.0, "pitch damping (1/rad)")

    # Lift & drag
    CL0: ca.SX = param(0.12, "lift coeff at zero AoA")  # Increased for higher CL in glide
    CLa: ca.SX = param(4.2, "lift slope (1/rad)")
    CD0: ca.SX = param(0.045, "parasitic drag")  # Low drag for efficient glider
    k_ind: ca.SX = param(0.09, "induced drag factor")
    CD0_fp: ca.SX = param(0.30, "flat plate drag")
    CY_fp: ca.SX = param(0.50, "flat plate sideforce")

    # Control effectiveness
    Clda: ca.SX = param(0.070, "aileron roll (1/rad)")
    Cldr: ca.SX = param(0.018, "rudder roll (1/rad)")
    Cmde: ca.SX = param(0.35, "elevator pitch (1/rad)")
    Cndr: ca.SX = param(0.045, "rudder yaw (1/rad)")
    Cnda: ca.SX = param(0.009, "aileron yaw (1/rad)")
    CYda: ca.SX = param(0.006, "aileron sideforce (1/rad)")
    CYdr: ca.SX = param(-0.045, "rudder sideforce (1/rad)")

    # Stability & damping
    Cnb: ca.SX = param(0.06, "yaw stiffness (1/rad)")
    CYb: ca.SX = param(-0.50, "sideslip sideforce (1/rad)")
    CYr: ca.SX = param(0.20, "yaw rate sideforce")
    CYp: ca.SX = param(-0.15, "roll rate sideforce")
    Clb: ca.SX = param(-0.25, "dihedral effect (1/rad)")
    Clp: ca.SX = param(-0.50, "roll damping")
    Clr: ca.SX = param(0.15, "yaw-roll coupling")
    Cnr: ca.SX = param(-0.15, "yaw damping")
    Cnp: ca.SX = param(0.010, "roll-yaw coupling")

    # Limits
    blend_width: ca.SX = param(np.deg2rad(5), "stall blend width (rad)")
    max_defl_ail: ca.SX = param(np.deg2rad(30), "max aileron (rad)")
    max_defl_elev: ca.SX = param(np.deg2rad(24), "max elevator (rad)")
    max_defl_rud: ca.SX = param(np.deg2rad(20), "max rudder (rad)")
    alpha_stall: ca.SX = param(np.deg2rad(20), "stall AoA (rad)")

    disable_aero: ca.SX = param(0.0, "disable aero (debug)")
    disable_gf: ca.SX = param(0.0, "disable ground forces (debug)")

    # Ground contact
    ground_wn: ca.SX = param(350.0, "ground frequency (rad/s)")
    ground_zeta: ca.SX = param(0.6, "ground damping ratio")
    ground_c_xy: ca.SX = param(0.05, "lateral damping (N·s/m)")
    ground_mu: ca.SX = param(0.15, "friction coefficient")  # Reduced from 0.6 (4x reduction)
    ground_max_force_per_wheel: ca.SX = param(20.0, "max normal force per wheel (N)")
    tailwheel_steer_gain: ca.SX = param(0.03, "tail wheel steering effectiveness (moment/rad)")


@symbolic
class SportCubOutputs:
    Vt: ca.SX = output_var(desc="airspeed (m/s)")
    alpha: ca.SX = output_var(desc="angle of attack (rad)")
    beta: ca.SX = output_var(desc="sideslip (rad)")
    qbar: ca.SX = output_var(desc="dynamic pressure (Pa)")
    q: ca.SX = output_var(4, desc="quaternion (w,x,y,z) for ROS2 compatibility")
    CL: ca.SX = output_var(desc="lift coefficient")
    CD: ca.SX = output_var(desc="drag coefficient")
    FA_b: ca.SX = output_var(3, desc="aero force body (N)")
    FG_b: ca.SX = output_var(3, desc="ground force body (N)")
    FT_b: ca.SX = output_var(3, desc="thrust force body (N)")
    FW_b: ca.SX = output_var(3, desc="weight force body (N)")
    F_b: ca.SX = output_var(3, desc="total force body (N)")
    MA_b: ca.SX = output_var(3, desc="aero moment body (N·m)")
    MG_b: ca.SX = output_var(3, desc="ground moment body (N·m)")
    MT_b: ca.SX = output_var(3, desc="thrust moment body (N·m)")
    MW_b: ca.SX = output_var(3, desc="weight moment body (N·m)")
    M_b: ca.SX = output_var(3, desc="total moment body (N·m)")


# ============================================================================
# Helper Functions
# ============================================================================


@beartype
def clamp(val: ca.SX, low: ca.SX | int | float, high: ca.SX | int | float) -> ca.SX:
    return ca.fmin(ca.fmax(val, low), high)


@beartype
def casadi_min_withcargo(costs: list[ca.SX], cargos: list[ca.SX]) -> tuple[ca.SX, ca.SX]:
    """Branch-free CasADi minimum with associated cargo.

    Finds minimum cost and returns the corresponding cargo value using
    branch-free operations suitable for symbolic computation.
    """
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
    """Convert ROS FLU (+x forward, +y left, +z up)
    to classical FRD (+x forward, +y right, +z down)."""
    return ca.vertcat(v_flu[0], -v_flu[1], -v_flu[2])


@beartype
def frd_to_flu(v_frd: ca.SX) -> ca.SX:
    """Convert classical FRD back to ROS FLU."""
    return ca.vertcat(v_frd[0], -v_frd[1], -v_frd[2])


@beartype
def wind_axes_from_velocity_frd(
    v_frd: ca.SX, eps: float = 1e-6
) -> tuple[ca.SX, ca.SX, ca.SX, ca.SX]:
    """Build wind-frame DCM from velocity vector (FRD convention).

    Constructs a wind-aligned coordinate frame where x-axis points along velocity.
    Prefers to keep z-axis aligned with body z-axis for normal flight, but switches
    to x-axis reference when velocity is nearly vertical (to avoid singularity).

    Args:
        v_frd: Velocity vector in FRD frame
        eps: Small value to prevent division by zero

    Returns:
        R_b_wind: DCM from body to wind frame (columns are wind axes in body frame)
        Vt: Total airspeed (with eps added to prevent division by zero)
        alpha: Angle of attack (rad)
        beta: Sideslip angle (rad)
    """
    U, V, W = v_frd[0], v_frd[1], v_frd[2]
    Vt = ca.norm_2(v_frd) + eps
    w_x = v_frd / Vt

    # Prefer b_z as reference (normal flight), switch to b_x when nearly vertical
    b_x = ca.SX([1, 0, 0])
    b_z = ca.SX([0, 0, 1])

    # Use b_z unless velocity is nearly aligned with it (vertical flight)
    cost_z = ca.fabs(ca.dot(w_x, b_z))  # How aligned with vertical
    cost_x = ca.fabs(ca.dot(w_x, b_x))  # How aligned with forward

    # Select reference with minimum alignment (most perpendicular)
    _, ref = casadi_min_withcargo([cost_z, cost_x], [b_z, b_x])

    # Build orthonormal frame
    ref_parallel = ca.dot(ref, w_x) * w_x
    w_z_temp = ref - ref_parallel
    w_z = w_z_temp / (ca.norm_2(w_z_temp) + eps)
    w_y = ca.cross(w_z, w_x)

    R_b_wind = ca.horzcat(w_x, w_y, w_z)

    # Compute aerodynamic angles
    V_xz = ca.sqrt(U * U + W * W) + eps
    alpha = ca.atan2(W, U)
    beta = ca.atan2(V, V_xz)

    return R_b_wind, Vt, alpha, beta


@beartype
def aero_coefficients(
    Vt: ca.SX,
    alpha: ca.SX,
    beta: ca.SX,
    x: Union[SportCubStatesQuat, SportCubStatesEuler],
    u: SportCubInputs,
    p: SportCubParams,
) -> dict[str, ca.SX]:
    """Compute aerodynamic coefficients with smooth stall model.

    Uses tanh blending between linear (pre-stall) and flat-plate (post-stall) models
    to create a smooth transition through stall. The blending factor sigma transitions
    from 0 (linear) to 1 (flat-plate) as alpha exceeds alpha_stall.

    Args:
        Vt: Total airspeed (m/s) - must include small epsilon to avoid division by zero
        alpha: Angle of attack (rad)
        beta: Sideslip angle (rad)
        x: Aircraft state (for angular rates)
        u: Control surface inputs
        p: Aircraft parameters

    Returns:
        Dictionary with keys: CL, CD, CY, Cl, Cm, Cn
    """
    # Convert body angular rates to FRD convention
    w_b_frd = flu_to_frd(x.w)
    P, Q, R = w_b_frd[0], w_b_frd[1], w_b_frd[2]

    ail_rad = p.max_defl_ail * clamp(u.ail, -1, 1)
    elev_rad = p.max_defl_elev * clamp(u.elev, -1, 1)
    rud_rad = p.max_defl_rud * clamp(u.rud, -1, 1) * -1  # Rudder sign convention

    sigma = (1 + ca.tanh((alpha - p.alpha_stall) / p.blend_width)) / 2

    CL_lin = p.CL0 + p.CLa * alpha
    CL_fp = 2 * ca.sin(alpha) * ca.cos(alpha)
    CL = (1 - sigma) * CL_lin + sigma * CL_fp

    CD_lin = p.CD0 + p.k_ind * CL_lin**2
    CD_fp = p.CD0_fp + 2 * ca.sin(alpha) ** 2
    CD = (1 - sigma) * CD_lin + sigma * CD_fp

    CY_lin = (
        p.CYb * beta
        + p.CYda * ail_rad
        + p.CYdr * rud_rad
        + p.CYp * (p.span / (2 * Vt)) * P
        + p.CYr * (p.span / (2 * Vt)) * R
    )
    CY_fp = p.CY_fp * ca.sin(beta) * ca.cos(alpha)
    CY = (1 - sigma) * CY_lin + sigma * CY_fp

    # Roll, pitch, yaw moments with damping terms
    Cl = (
        p.Clda * ail_rad
        + p.Cldr * rud_rad
        + p.Clb * beta
        + p.Clp * (p.span / (2 * Vt)) * P
        + p.Clr * (p.span / (2 * Vt)) * R
    )
    Cm = p.Cm0 + p.Cma * alpha + p.Cmde * elev_rad + p.Cmq * (p.cbar / (2 * Vt)) * Q
    Cn = (
        p.Cnb * beta
        + p.Cndr * rud_rad
        + p.Cnda * ail_rad
        + p.Cnp * (p.span / (2 * Vt)) * P
        + p.Cnr * (p.span / (2 * Vt)) * R
    )

    return {"CL": CL, "CD": CD, "CY": CY, "Cl": Cl, "Cm": Cm, "Cn": Cn}


@beartype
def aerodynamic_forces_and_moments(
    x: Union[SportCubStatesQuat, SportCubStatesEuler],
    R_eb: SO3DcmLieGroupElement,
    u: SportCubInputs,
    p: SportCubParams,
) -> dict:
    """Compute aerodynamic forces and moments in body FLU frame.

    Args:
        x: Aircraft state
        R_eb: Rotation from earth to body frame
        u: Control inputs
        p: Aircraft parameters

    Returns:
        Dictionary with keys: FA_b, MA_b, Vt, alpha, beta, qbar, CL, CD
    """
    # Convert earth velocity to body frame
    R_be = R_eb.inverse()
    v_b = R_be @ x.v

    # Convert to FRD and compute wind axes
    v_frd = flu_to_frd(v_b)
    R_b_wind, Vt, alpha_body, beta = wind_axes_from_velocity_frd(v_frd)

    # Add wing incidence angle to get effective angle of attack
    alpha = alpha_body + p.wing_incidence

    coeff = aero_coefficients(Vt, alpha, beta, x, u, p)
    qbar = 0.5 * p.rho * Vt**2

    FA_wind_frd = qbar * p.S * ca.vertcat(-coeff["CD"], coeff["CY"], -coeff["CL"])
    FA_body_frd = R_b_wind @ FA_wind_frd
    FA_b = frd_to_flu(FA_body_frd)

    MA_frd = (
        qbar
        * p.S
        * ca.vertcat(
            p.span * coeff["Cl"],
            p.cbar * coeff["Cm"],
            p.span * coeff["Cn"],
        )
    )
    MA_b = frd_to_flu(MA_frd)

    return {
        "FA_b": FA_b,
        "MA_b": MA_b,
        "Vt": Vt,
        "alpha": alpha,  # Return wing alpha (includes incidence)
        "beta": beta,
        "qbar": qbar,
        "CL": coeff["CL"],
        "CD": coeff["CD"],
    }


@beartype
def ground_forces_and_moments(
    x: Union[SportCubStatesQuat, SportCubStatesEuler],
    R_eb: SO3DcmLieGroupElement,
    u: SportCubInputs,
    p: SportCubParams,
) -> dict:
    """Compute ground reaction forces and moments from wheel contact.

    Implements a 3-wheel tricycle landing gear with spring-damper-friction model.
    Wheel positions are hardcoded in body FLU frame:
    - Left wheel: (0.1, 0.1, -0.1) - front left
    - Right wheel: (0.1, -0.1, -0.1) - front right
    - Tail wheel: (-0.4, 0.0, 0.0) - aft center (steerable via rudder)

    Forces computed in earth frame, then transformed to body frame for moments.
    Tail wheel steering: rudder input creates a yaw moment when tail wheel is on ground.

    Args:
        x: Aircraft state
        R_eb: Rotation from earth to body frame
        u: Control inputs (for rudder-linked tail wheel steering)
        p: Aircraft parameters

    Returns:
        Dictionary with keys: FG_b, MG_b
    """
    left_wheel_b = ca.SX([0.1, 0.1, -0.1])
    right_wheel_b = ca.SX([0.1, -0.1, -0.1])
    tail_wheel_b = ca.SX([-0.4, 0.0, 0.0])

    wheel_b_list = [left_wheel_b, right_wheel_b, tail_wheel_b]
    R_be = R_eb.inverse()

    # Convert earth velocity to body frame
    v_b = R_be @ x.v

    ground_k = p.m * p.ground_wn**2
    ground_c_vert = 2.0 * p.ground_zeta * p.m * p.ground_wn

    ground_force_e_list = []
    MG_b = ca.SX.zeros(3)

    for i, wheel_b in enumerate(wheel_b_list):
        wheel_e = R_eb @ wheel_b
        pos_wheel_e = x.p + wheel_e
        vel_wheel_b = v_b + ca.cross(x.w, wheel_b)
        vel_wheel_e = R_eb @ vel_wheel_b
        penetration = -pos_wheel_e[2]

        normal_force_unclamped = penetration * ground_k - vel_wheel_e[2] * ground_c_vert
        normal_force = clamp(normal_force_unclamped, 0, p.ground_max_force_per_wheel)

        lateral_damp = ca.vertcat(-vel_wheel_e[0] * p.ground_c_xy, -vel_wheel_e[1] * p.ground_c_xy)
        lateral_mag = ca.norm_2(lateral_damp) + 1e-9
        max_lateral = p.ground_mu * normal_force
        scale_lat = ca.if_else(lateral_mag > max_lateral, max_lateral / lateral_mag, 1.0)
        lateral_limited = scale_lat * lateral_damp

        force_contact_e = ca.vertcat(lateral_limited[0], lateral_limited[1], normal_force)
        force_e = ca.if_else(pos_wheel_e[2] < 0.0, force_contact_e, ca.vertcat(0, 0, 0))
        ground_force_e_list.append(force_e)

        force_b = R_be @ force_e
        MG_b += ca.cross(wheel_b, force_b)

        # Tail wheel steering moment (index 2 = tail wheel)
        if i == 2:
            # Bicycle steering model: steered wheel creates lateral force
            # Lateral force creates yaw moment: M = F_lateral × arm_length
            # F_lateral is proportional to slip angle, which depends on:
            #   - Steering angle (rudder input)
            #   - Forward velocity (higher speed = more lateral force)
            # But at very low speeds, the force saturates (friction limited)

            rud_rad = p.max_defl_rud * clamp(u.rud, -1, 1)
            forward_speed = vel_wheel_e[0]  # Can be positive or negative

            # Lateral force coefficient increases with speed but saturates
            # At low speeds: friction-limited steering (proportional to normal force)
            # At high speeds: aerodynamic-like steering (proportional to velocity)
            speed_sq = forward_speed * forward_speed
            # Smooth transition: F_lat ∝ sqrt(v²) * steer for high speed behavior
            #                    but limited by friction at low speed
            lateral_force_factor = ca.sqrt(speed_sq + 1.0) - 1.0  # Smooth, starts at 0

            # Moment = lateral_force_factor * steer_angle * gain
            # The gain should account for wheelbase (moment arm) and cornering stiffness
            tailwheel_moment_z = p.tailwheel_steer_gain * rud_rad * lateral_force_factor

            # Apply moment only when wheel is on ground
            tailwheel_moment_b = ca.if_else(
                pos_wheel_e[2] < 0.0,
                ca.vertcat(0, 0, tailwheel_moment_z),
                ca.vertcat(0, 0, 0),
            )
            MG_b += tailwheel_moment_b

    FG_b = R_be @ ca.sum2(ca.horzcat(*ground_force_e_list))

    return {
        "FG_b": FG_b,
        "MG_b": MG_b,
    }


# ============================================================================
# SportCub Factory
# ============================================================================


@beartype
def sportcub(attitude_rep: AttitudeRep = "quat") -> ModelSX:
    """Create SportCub 6-DOF aircraft model.

    Frames: e=earth (ENU inertial), b=body (FLU), wind=aligned with velocity

    Args:
        attitude_rep: "quat" (default) for quaternions (ROS2 compatible, no singularities)
                     "euler" for Euler angles (better for linearization analysis)

    Returns:
        ModelSX instance with selected attitude representation
    """

    if attitude_rep == "quat":
        model = ModelSX.create(
            SportCubStatesQuat,
            SportCubInputs,
            SportCubParams,
            output_type=SportCubOutputs,
        )
    elif attitude_rep == "euler":
        model = ModelSX.create(
            SportCubStatesEuler,
            SportCubInputs,
            SportCubParams,
            output_type=SportCubOutputs,
        )
    else:
        raise ValueError(f"Invalid attitude representation: {attitude_rep}")

    x, u, p, y = model.x, model.u, model.p, model.y

    # Get attitude representation from the state class
    attitude_rep = model.state_type.attitude_rep

    J = ca.SX.zeros(3, 3)
    J[0, 0] = p.Jx
    J[1, 1] = p.Jy
    J[2, 2] = p.Jz
    J[0, 2] = p.Jxz
    J[2, 0] = p.Jxz

    xAxis = ca.vertcat(1, 0, 0)
    zAxis = ca.vertcat(0, 0, 1)

    # Get rotation matrix based on attitude representation
    if attitude_rep == "quat":
        R_eb = lie.SO3Dcm.from_Quat(lie.SO3Quat.elem(x.r))
        att_group = lie.SO3Quat.elem(x.r)
    elif attitude_rep == "euler":
        R_eb = lie.SO3Dcm.from_Euler(lie.SO3EulerB321.elem(x.r))
        att_group = lie.SO3EulerB321.elem(x.r)
    else:
        raise ValueError(f"Invalid attitude representation: {attitude_rep}")

    R_be = R_eb.inverse()

    # Compute forces and moments (pure functions returning dicts)
    aero = aerodynamic_forces_and_moments(x, R_eb, u, p)
    ground = ground_forces_and_moments(x, R_eb, u, p)

    # Extract forces and moments
    FA_b = aero["FA_b"]
    MA_b = aero["MA_b"]
    FG_b = ground["FG_b"]
    MG_b = ground["MG_b"]

    FT_b = p.thr_max * clamp(u.thr, 0, 1) * xAxis
    MT_b = ca.SX.zeros(3)

    FW_b = R_be @ (-p.m * p.g * zAxis)
    MW_b = ca.SX.zeros(3)

    # Total forces and moments
    F_b = (1.0 - p.disable_aero) * FA_b + (1.0 - p.disable_gf) * FG_b + FT_b + FW_b
    M_b = (1.0 - p.disable_aero) * MA_b + (1.0 - p.disable_gf) * MG_b + MT_b + MW_b

    # Assign all outputs
    y.Vt = aero["Vt"]
    y.alpha = aero["alpha"]
    y.beta = aero["beta"]
    y.qbar = aero["qbar"]
    y.CL = aero["CL"]
    y.CD = aero["CD"]
    y.FA_b = FA_b
    y.FG_b = FG_b
    y.FT_b = FT_b
    y.FW_b = FW_b
    y.F_b = F_b
    y.MA_b = MA_b
    y.MG_b = MG_b
    y.MT_b = MT_b
    y.MW_b = MW_b
    y.M_b = M_b

    # Always output quaternion for ROS2 compatibility
    if attitude_rep == "quat":
        y.q = x.r
    else:  # euler
        quat_group = lie.SO3Quat.from_Euler(att_group)
        y.q = quat_group.param

    # Dynamics equations
    p_dot = x.v
    F_e = R_eb @ F_b
    v_dot = F_e / p.m  # No additional gravity term - already in F_b

    # Attitude derivative depends on representation
    if attitude_rep == "quat":
        # Quaternion: use right Jacobian
        att_dot = att_group.right_jacobian() @ x.w
    else:  # euler
        # Euler B321 (state order: psi, theta, phi)
        # Body angular velocity in FLU: [p, q, r] = [wx, wy, wz]
        theta, phi = x.r[1], x.r[2]
        p_body, q_body, r_body = x.w[0], x.w[1], x.w[2]

        # Euler angle rates for B321 sequence
        psi_dot = (q_body * ca.sin(phi) + r_body * ca.cos(phi)) / ca.cos(theta)
        theta_dot = q_body * ca.cos(phi) - r_body * ca.sin(phi)
        phi_dot = p_body + (q_body * ca.sin(phi) + r_body * ca.cos(phi)) * ca.tan(theta)

        att_dot = ca.vertcat(psi_dot, theta_dot, phi_dot)

    w_dot = ca.inv(J) @ (M_b - ca.cross(x.w, J @ x.w))

    f_x = ca.vertcat(p_dot, v_dot, att_dot, w_dot)
    f_y = y.as_vec()

    model.build(f_x=f_x, f_y=f_y, integrator="rk4", integrator_options={"N": 100})
    return model


__all__ = [
    "sportcub",
]
