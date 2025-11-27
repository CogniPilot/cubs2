"""Fixed-wing aircraft trim and mode analysis utilities."""
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
from beartype import beartype
import casadi as ca
from cyecca.dynamics.explicit import Model
from cyecca.dynamics.explicit.linearize import (
    find_trim, get_input_index, get_state_index
)
import numpy as np


@beartype
def find_trim_fixed_wing(
    model: Model,
    V_target: float | None = 3.0,
    gamma: float | None = 0.0,
    print_progress: bool = True,
    fix_throttle: bool = True,
    verbose: bool = False,
    ipopt_print_level: int = 5,
):
    """
    Fixed-wing aircraft trim for steady level or descending flight.

    This is a convenience wrapper around the generic trim solver with
    aircraft-specific cost function and constraints for level flight.

    Parameters
    ----------
    model : Model
        Aircraft model instance
    V_target : float, optional
        Target airspeed (m/s). If None, solver finds natural equilibrium speed.
        For glide (fix_throttle=True), natural equilibrium corresponds to max L/D.
    gamma : float, optional
        Flight path angle (rad). If None, solver finds natural equilibrium angle.
    print_progress : bool
        Print optimization progress
    fix_throttle : bool
        If True, throttle is fixed at 0 (glide)
    verbose : bool
        Print detailed trim solution
    ipopt_print_level : int
        IPOPT solver verbosity (0-12)

    Returns
    -------
    v_trim : dataclass instance
        Trim point with all state, input, param, output fields.
    stats : dict
        Solver statistics.

    Note:
    ----
        When finding glide trim (fix_throttle=True) with V_target=None and gamma=None,
        the solver finds the natural equilibrium which corresponds to maximum L/D ratio.
        This is the best glide condition - no explicit L/D optimization is needed.

    """
    # Use defaults if not specified
    if gamma is not None:
        gamma = float(gamma)
    if V_target is None:
        V_target_guess = 5.0  # default guess for initial conditions when not specified
    else:
        V_target_guess = V_target

    # Create initial guesses with better aerodynamic setup
    model_type = model.model_type
    v_guess = model_type.numeric()

    # Copy defaults from model.v0
    for field_name in model_type._field_info.keys():
        val = getattr(model.v0, field_name)
        if isinstance(val, np.ndarray):
            setattr(v_guess, field_name, val.copy())
        else:
            setattr(v_guess, field_name, val)

    # Set position
    v_guess.p = np.array([0.0, 0.0, 10.0])

    # Initial velocity guess: assume small pitch angle, velocity mostly forward
    pitch_guess = np.deg2rad(5.0)

    # Handle both quaternion and euler representations
    r_info = model_type._field_info.get('r', None)
    if r_info is not None:
        if r_info['dim'] == 4:
            # Quaternion representation
            v_guess.r = np.array(
                [np.cos(pitch_guess / 2.0), 0.0, np.sin(pitch_guess / 2.0), 0.0])
        elif r_info['dim'] == 3:
            # Euler representation
            v_guess.r = np.array([0.0, pitch_guess, 0.0])

    # Velocity in body frame
    v_guess.v = np.array(
        [
            V_target_guess * np.cos(pitch_guess),
            0.0,
            V_target_guess * np.sin(pitch_guess),
        ]
    )
    v_guess.w = np.array([0.0, 0.0, 0.0])

    # Set input guesses
    v_guess.ail = 0.0
    v_guess.elev = -0.1
    v_guess.rud = 0.0
    v_guess.thr = 0.0 if fix_throttle else 0.3

    # Get indices for state/input fields
    v_start, v_end = get_state_index(model, 'v')  # velocity
    w_start, w_end = get_state_index(model, 'w')  # angular velocity
    p_start, p_end = get_state_index(model, 'p')  # position
    r_start, r_end = get_state_index(model, 'r')  # rotation

    ail_start, ail_end = get_input_index(model, 'ail')
    elev_start, elev_end = get_input_index(model, 'elev')
    rud_start, rud_end = get_input_index(model, 'rud')
    thr_start, thr_end = get_input_index(model, 'thr')

    def aircraft_cost_fn(x_opt, u_opt, xdot_opt, model) -> ca.MX:
        """
        Aircraft-specific cost for level flight trim.

        Parameters
        ----------
        x_opt : ca.MX
            State optimization variable vector.
        u_opt : ca.MX
            Input optimization variable vector.
        xdot_opt : ca.MX
            State derivative vector.
        model : Model
            The model instance.

        Returns
        -------
        cost : ca.MX
            Cost to minimize.

        """
        # Extract velocity and angular velocity derivatives
        xdot_v = xdot_opt[v_start:v_end]
        xdot_w = xdot_opt[w_start:w_end]

        # PRIMARY: Zero accelerations for steady-state trim
        obj = 10000.0 * ca.sumsqr(xdot_v) + 10000.0 * ca.sumsqr(xdot_w)

        # Get velocity from state
        v_vel = x_opt[v_start:v_end]
        w_rate = x_opt[w_start:w_end]

        # SECONDARY: Match desired speed (if specified)
        if V_target is not None:
            speed = ca.norm_2(v_vel)
            obj += 1000.0 * (speed - V_target) ** 2

        # TERTIARY: Match desired flight path angle (if specified)
        if gamma is not None:
            xdot_p = xdot_opt[p_start:p_end]
            horiz_speed = ca.sqrt(xdot_p[0] ** 2 + xdot_p[1] ** 2) + 1e-6
            gamma_actual = ca.atan2(xdot_p[2], horiz_speed)
            obj += 1000.0 * (gamma_actual - gamma) ** 2

        # Coordinated flight: minimize sideslip and angular rates
        obj += 10.0 * v_vel[1] ** 2  # lateral velocity (sideslip)
        obj += 10.0 * ca.sumsqr(w_rate)  # angular rates

        # Get input values
        u_ail = u_opt[ail_start]
        u_rud = u_opt[rud_start]
        u_thr = u_opt[thr_start]

        obj += 0.1 * (u_ail**2 + u_rud**2)  # minimize aileron and rudder

        if not fix_throttle:
            obj += 0.1 * u_thr**2

        return obj

    def aircraft_constraints_fn(opti, x_opt, u_opt, model):
        """
        Aircraft-specific constraints for level flight trim.

        Parameters
        ----------
        opti : ca.Opti
            The optimizer instance.
        x_opt : ca.MX
            State optimization variable vector.
        u_opt : ca.MX
            Input optimization variable vector.
        model : Model
            The model instance.

        """
        # Check for rotation representation
        r_info = model_type._field_info.get('r', None)
        if r_info is not None:
            r_sym = x_opt[r_start:r_end]
            if r_info['dim'] == 4:
                # Quaternion representation
                opti.subject_to(ca.sumsqr(r_sym) == 1.0)
                opti.subject_to(r_sym[0] >= 0.5)
            elif r_info['dim'] == 3:
                # Euler angle representation
                opti.subject_to(r_sym[0] >= -np.pi)  # psi lower
                opti.subject_to(r_sym[0] <= np.pi)   # psi upper
                opti.subject_to(r_sym[1] >= -np.pi / 2)  # theta lower
                opti.subject_to(r_sym[1] <= np.pi / 2)   # theta upper
                opti.subject_to(r_sym[2] >= -np.pi)  # phi lower
                opti.subject_to(r_sym[2] <= np.pi)   # phi upper

        # Velocity bounds
        v_sym = x_opt[v_start:v_end]
        for i in range(3):
            opti.subject_to(v_sym[i] >= -20.0)
            opti.subject_to(v_sym[i] <= 20.0)

        # Angular velocity bounds
        w_sym = x_opt[w_start:w_end]
        for i in range(3):
            opti.subject_to(w_sym[i] >= -5.0)
            opti.subject_to(w_sym[i] <= 5.0)

        # Control surface bounds
        opti.subject_to(u_opt[ail_start] >= -1)
        opti.subject_to(u_opt[ail_start] <= 1)
        opti.subject_to(u_opt[elev_start] >= -1)
        opti.subject_to(u_opt[elev_start] <= 1)
        opti.subject_to(u_opt[rud_start] >= -1)
        opti.subject_to(u_opt[rud_start] <= 1)

        # Throttle constraints
        if fix_throttle:
            opti.subject_to(u_opt[thr_start] == 0.0)
        else:
            opti.subject_to(u_opt[thr_start] >= 0.0)
            opti.subject_to(u_opt[thr_start] <= 1.0)

    # Call the generic trim solver
    v_trim, stats = find_trim(
        model,
        v_guess=v_guess,
        cost_fn=aircraft_cost_fn,
        constraints_fn=aircraft_constraints_fn,
        verbose=verbose,
        print_level=ipopt_print_level,
    )

    # Custom output messages
    if print_progress:
        if stats['success']:
            print(f'  ✓ Aircraft trim converged for V={V_target} m/s')
            if verbose:
                print('    Ipopt stats:')
                for k, val in stats.items():
                    if isinstance(val, (float, int, str)):
                        print(f'      {k}: {val}')
        else:
            print(f'  ⚠ Aircraft trim had issues for V={V_target} m/s')

    if verbose:
        print_trim_details(model, v_trim)

    return v_trim, stats


def print_trim_details(model, v_trim, header=None):
    """Print detailed trim solution."""
    if header:
        print(f'\n{header}')
        print('=' * 80)

    print('Trim State:')
    for field_name in model._state_fields:
        val = getattr(v_trim, field_name)
        print(f'  {field_name}: {val}')

    print('Trim Input:')
    for field_name in model._input_fields:
        val = getattr(v_trim, field_name)
        print(f'  {field_name}: {val}')

    print('Parameters:')
    for field_name in model._param_fields:
        val = getattr(v_trim, field_name)
        print(f'  {field_name}: {val}')


@beartype
def classify_aircraft_modes(
        modes: list[dict], state_names: list[str]) -> dict[str, dict | list]:
    """Fixed-wing aircraft mode classification (short-period, phugoid, etc.)."""
    def contrib(mode: dict, pattern: str) -> float:
        return sum(
            mag for name,
            mag in mode.get(
                'participation',
                {}).items() if pattern in name)

    def try_classify(mode: dict) -> str | None:
        """Return mode type key if mode matches, else None."""
        v_c = contrib(mode, 'v')
        w_c = contrib(mode, 'w')
        p_c = contrib(mode, 'p')
        osc = mode['type'] == 'complex'
        freq = mode.get('omega_n', 0) / (2 * np.pi)
        real = abs(mode['real'])
        stable = mode['stability'] == 'stable'

        # Get specific angular rate contributions for mode classification
        # w[0] = roll rate (p), w[1] = pitch rate (q), w[2] = yaw rate (r)
        participation = mode.get('participation', {})
        w0_c = participation.get('w[0]', 0)  # roll rate
        w1_c = participation.get('w[1]', 0)  # pitch rate
        w2_c = participation.get('w[2]', 0)  # yaw rate

        # Short period: pitch-dominated (w[1] > roll/yaw), fast
        if w1_c > 0.3 and w1_c > w0_c:
            if (osc and freq > 0.3) or (not osc and real > 5.0 and stable):
                return 'short_period'

        # Roll damping: roll rate dominated, fast real mode
        if not osc and w0_c > 0.5 and real > 5.0 and stable:
            return 'roll_damping'

        # Phugoid: velocity-dominated, slow oscillation
        if v_c > 0.4 and w_c < 0.5:
            if (osc and freq < 1.0) or (not osc and real < 15.0):
                return 'phugoid'

        # Dutch roll: lateral oscillation (sideslip + yaw rate)
        if osc and (w2_c > 0.2 or (v_c > 0.2 and w0_c > 0.1)):
            return 'dutch_roll'

        # Spiral: slow position drift
        if not osc and p_c > 0.5 and real < 1.0:
            return 'spiral'

        return None

    classified = {
        key: None for key in [
            'short_period',
            'phugoid',
            'dutch_roll',
            'roll_damping',
            'spiral']}
    classified['other'] = []

    for mode in modes:
        if abs(mode['real']) < 1e-6 and abs(mode['imag']) < 1e-6:
            continue

        mode_type = try_classify(mode)
        if mode_type and classified[mode_type] is None:
            classified[mode_type] = mode
        else:
            classified['other'].append(mode)

    return classified


@beartype
def print_mode_summary(modes: list[dict], state_names: list[str]) -> None:
    """Print summary of fixed-wing aircraft dynamic modes."""
    classified = classify_aircraft_modes(modes, state_names)
    print('\n' + '=' * 80)
    print('AIRCRAFT DYNAMIC MODES ANALYSIS')
    print('=' * 80)
    mode_types = [
        ('short_period', 'Short Period (Pitch Oscillation)'),
        ('phugoid', 'Phugoid (Speed/Altitude Oscillation)'),
        ('dutch_roll', 'Dutch Roll (Lateral Oscillation)'),
        ('roll_damping', 'Roll Damping'),
        ('spiral', 'Spiral Mode'),
    ]
    for key, title in mode_types:
        mode = classified[key]
        print(f'\n{title}:')
        print('-' * 80)
        if mode is None:
            print('  NOT IDENTIFIED')
            continue
        print(f"  Eigenvalue:      {mode['eigenvalue']:.4f}")
        print(f"  Time constant:   {mode['time_constant']:.3f} s")
        print(f"  Damping ratio:   {mode['zeta']:.4f}")
        if mode['type'] == 'complex':
            print(f"  Frequency:       {mode['omega_n']/(2*np.pi):.4f} Hz")
            print(f"  Period:          {mode['period']:.3f} s")
        print(f"  Stable:          {mode['stability']}")
        if 'participation' in mode:
            print('  Dominant states:')
            sorted_states = sorted(mode['participation'].items(), key=lambda x: -x[1])
            for name, mag in sorted_states[:5]:
                print(f'    {name:12s}  {mag:.4f}')
    if classified['other']:
        print(f"\nOther modes ({len(classified['other'])}):")
        print('-' * 80)
        for mode in classified['other']:
            stable_str = mode['stability']
            if mode['type'] == 'complex':
                freq_hz = mode['omega_n'] / (2 * np.pi)
                print(
                    f"  λ = {mode['real']:.4f} ± {mode['imag']:.4f}j  "
                    f'({freq_hz:.4f} Hz, {stable_str})'
                )
            else:
                tc = mode['time_constant']
                print(f"  λ = {mode['real']:.4f}  (τ = {tc:.3f} s, {stable_str})")
    print('=' * 80)


__all__ = [
    'find_trim_fixed_wing',
    'classify_aircraft_modes',
    'print_mode_summary',
    'print_trim_details',
]
