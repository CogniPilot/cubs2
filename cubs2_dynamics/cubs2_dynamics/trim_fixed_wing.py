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
from cyecca.dynamics import ModelSX
import numpy as np


@beartype
def find_trim_fixed_wing(
    model: ModelSX,
    V_target: float | None = 3.0,
    gamma: float | None = 0.0,
    print_progress: bool = True,
    fix_throttle: bool = True,
    verbose: bool = False,
    ipopt_print_level: int = 5,
) -> tuple[np.ndarray, np.ndarray, dict | None]:
    """
    Fixed-wing aircraft trim for steady level or descending flight.

    This is a convenience wrapper around the generic trim solver with
    aircraft-specific cost function and constraints for level flight.

    Parameters
    ----------
    model : ModelSX
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

    Note:
    ----
        When finding glide trim (fix_throttle=True) with V_target=None and gamma=None,
        the solver finds the natural equilibrium which corresponds to maximum L/D ratio.
        This is the best glide condition - no explicit L/D optimization is needed.

    """
    import copy

    from cyecca.dynamics.linearize import find_trim as find_trim_generic
    from cyecca.dynamics.linearize import print_trim_details

    # Use defaults if not specified
    if gamma is not None:
        gamma = float(gamma)
    if V_target is None:
        V_target_guess = 5.0  # default guess for initial conditions when not specified
    else:
        V_target_guess = V_target

    # Create initial guesses with better aerodynamic setup
    s_guess = copy.deepcopy(model.x0)
    s_guess.p = np.array([0.0, 0.0, 10.0])
    # Initial velocity guess: assume small pitch angle, velocity mostly forward
    # in body frame, then rotate to earth frame for initial guess
    pitch_guess = np.deg2rad(5.0)

    # Handle both quaternion and euler representations
    if hasattr(s_guess, 'q'):
        s_guess.q = np.array(
            [np.cos(pitch_guess / 2.0), 0.0, np.sin(pitch_guess / 2.0), 0.0])

    # Velocity in earth frame: mostly east with slight upward component
    s_guess.v = np.array(
        [
            V_target_guess * np.cos(pitch_guess),
            0.0,
            V_target_guess * np.sin(pitch_guess),
        ]
    )
    s_guess.w = np.array([0.0, 0.0, 0.0])

    i_guess = copy.deepcopy(model.u0)
    i_guess.ail = 0.0
    i_guess.elev = -0.1
    i_guess.rud = 0.0
    i_guess.thr = 0.0 if fix_throttle else 0.3

    def aircraft_cost_fn(
        model: ModelSX,
        x,  # State dataclass instance (symbolic)
        u,  # Input dataclass instance (symbolic)
        p,  # Parameter dataclass instance (symbolic)
        x_dot,  # State derivative dataclass instance (symbolic)
    ) -> ca.MX | ca.SX:
        """
        Aircraft-specific cost for level flight trim.

        All state/input variables are already wrapped in dataclass instances!

        """
        # Calculate outputs for aerodynamic coefficients
        y = model.output_type.from_vec(
            model.f_y(x.as_vec(), u.as_vec(), p.as_vec()))

        # PRIMARY: Zero accelerations for steady-state trim
        obj = 10000.0 * ca.sumsqr(x_dot.v) + 10000.0 * ca.sumsqr(x_dot.w)

        # Encourage high L/D when both V_target and gamma are None
        if V_target is None and gamma is None:
            # Minimize drag-to-lift ratio (equivalent to maximizing L/D)
            obj += 5.0 * y.CD / (y.CL + 1e-5)

        # SECONDARY: Match desired speed (if specified)
        if V_target is not None:
            speed = ca.norm_2(x.v)
            obj += 1000.0 * (speed - V_target) ** 2

        # TERTIARY: Match desired flight path angle (if specified)
        if gamma is not None:
            horiz_speed = ca.sqrt(x_dot.p[0] ** 2 + x_dot.p[1] ** 2) + 1e-6
            gamma_actual = ca.atan2(x_dot.p[2], horiz_speed)
            obj += 1000.0 * (gamma_actual - gamma) ** 2

        # Coordinated flight: minimize sideslip and angular rates
        obj += 10.0 * x.v[1] ** 2  # lateral velocity (sideslip)
        obj += 10.0 * ca.sumsqr(x.w)  # angular rates
        obj += 0.1 * (u.ail**2 + u.rud**2)  # minimize aileron and rudder

        if not fix_throttle:
            obj += 0.1 * u.thr**2

        return obj

    def aircraft_constraints_fn(
        model: ModelSX,
        x,  # State dataclass instance (symbolic)
        u,  # Input dataclass instance (symbolic)
        p,  # Parameter dataclass instance (symbolic)
        x_dot,  # State derivative dataclass instance (symbolic)
    ) -> list:
        """
        Aircraft-specific constraints for level flight trim.

        All state/input variables are already wrapped in dataclass instances!
        Returns list of constraint expressions.

        """
        constraints = []

        # Quaternion unit norm constraint (check shape for quaternion vs euler)
        if hasattr(x.r, 'shape') and x.r.shape[0] == 4:
            # Quaternion representation (r is 4x1)
            constraints.append(ca.sumsqr(x.r) == 1.0)
            # keep quaternion in positive hemisphere
            constraints.append(x.r[0] >= 0.5)
        elif hasattr(x.r, 'shape') and x.r.shape[0] == 3:
            # Euler angle representation (r is 3x1)
            # Bounds: psi, theta, phi
            constraints.append(x.r[0] >= -np.pi)  # psi lower
            constraints.append(x.r[0] <= np.pi)  # psi upper
            constraints.append(x.r[1] >= -np.pi / 2)  # theta lower
            constraints.append(x.r[1] <= np.pi / 2)  # theta upper
            constraints.append(x.r[2] >= -np.pi)  # phi lower
            constraints.append(x.r[2] <= np.pi)  # phi upper

        # Velocity bounds (reasonable range for this aircraft)
        for i in range(3):
            constraints.append(x.v[i] >= -20.0)
            constraints.append(x.v[i] <= 20.0)

        # Angular velocity bounds (prevent unrealistic solutions)
        for i in range(3):
            constraints.append(x.w[i] >= -5.0)
            constraints.append(x.w[i] <= 5.0)

        # Control surface bounds
        constraints.append(u.ail >= -1)
        constraints.append(u.ail <= 1)
        constraints.append(u.elev >= -1)
        constraints.append(u.elev <= 1)
        constraints.append(u.rud >= -1)
        constraints.append(u.rud <= 1)

        # Throttle constraints
        if fix_throttle:
            constraints.append(u.thr == 0.0)
        else:
            constraints.append(u.thr >= 0.0)
            constraints.append(u.thr <= 1.0)

        return constraints

    # Call the generic trim solver
    x_trim, u_trim, stats = find_trim_generic(
        model,
        x_guess=s_guess,
        u_guess=i_guess,
        cost_fn=aircraft_cost_fn,
        constraints_fn=aircraft_constraints_fn,
        print_progress=print_progress,
        verbose=False,  # We'll handle our own verbose output
        ipopt_print_level=ipopt_print_level,
        max_iter=2000,
    )

    # Custom output messages and verbose printing
    if print_progress and stats is not None:
        print(f'  ✓ Aircraft trim converged for V={V_target} m/s')
        if verbose:
            print('    Ipopt stats:')
            for k, v in stats.items():
                if isinstance(v, (float, int, str)):
                    print(f'      {k}: {v}')
    elif print_progress:
        print(f'  ⚠ Aircraft trim had issues for V={V_target} m/s')

    header = None
    if verbose:
        header = (
            'AIRCRAFT TRIM SOLUTION (VERBOSE)' if stats else 'AIRCRAFT TRIM (FAILED / BEST GUESS)'
        )
    print_trim_details(model, x_trim, u_trim, model.p0.as_vec(), header=header)

    return x_trim, u_trim, stats


@beartype
def classify_aircraft_modes(
        modes: list[dict], state_names: list[str]) -> dict[str, dict | list]:
    """Fixed-wing aircraft mode classification (short-period, phugoid, etc.)."""
    def contrib(mode: dict, pattern: str) -> float:
        return sum(
            mag for name,
            mag in mode.get(
                'dominant_states',
                []) if pattern in name)

    def try_classify(mode: dict) -> str | None:
        """Return mode type key if mode matches, else None."""
        v_c = contrib(mode, 'v[')
        w_c = contrib(mode, 'w[')
        p_c = contrib(mode, 'p[')
        osc = mode['is_oscillatory']
        freq = mode.get('frequency_hz', 0)
        real = abs(mode['real'])
        stable = mode['stable']

        # Short period: pitch-dominated, fast
        if contrib(mode, 'w[1]') > 0.5 and w_c > 0.5:
            if (osc and freq > 0.3) or (not osc and real > 5.0 and stable):
                return 'short_period'

        # Phugoid: velocity-dominated, slow oscillation
        if (contrib(mode, 'v[0]') > 0.3 or contrib(
                mode, 'v[2]') > 0.3) and v_c > 0.4:
            if contrib(mode, 'w[0]') < 0.2 and contrib(mode, 'w[2]') < 0.5:
                if (osc and freq < 1.0) or (not osc and real < 15.0):
                    return 'phugoid'

        # Dutch roll: lateral oscillation (sideslip + yaw)
        if osc and (
            contrib(
                mode,
                'v[1]') > 0.2 or contrib(
                mode,
                'w[2]') > 0.3):
            if contrib(mode, 'v[1]') + contrib(mode, 'w[2]') > 0.5:
                return 'dutch_roll'

        # Roll damping: fast roll convergence
        if not osc and contrib(mode, 'w[0]') > 0.3 and real > 5.0 and stable:
            return 'roll_damping'

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
        print(f"  Eigenvalue:      {mode['eigenvalue_continuous']:.4f}")
        print(f"  Time constant:   {mode['time_constant']:.3f} s")
        print(f"  Damping ratio:   {mode['damping_ratio']:.4f}")
        if mode['is_oscillatory']:
            print(f"  Frequency:       {mode['frequency_hz']:.4f} Hz")
            print(f"  Period:          {mode['period']:.3f} s")
        print(f"  Stable:          {mode['stable']}")
        if 'dominant_states' in mode:
            print('  Dominant states:')
            for name, mag in mode['dominant_states']:
                print(f'    {name:12s}  {mag:.4f}')
    if classified['other']:
        print(f"\nOther modes ({len(classified['other'])}):")
        print('-' * 80)
        for mode in classified['other']:
            stable_str = 'stable' if mode['stable'] else 'UNSTABLE'
            if mode['is_oscillatory']:
                freq_hz = mode['frequency_hz']
                print(
                    f"  λ = {mode['real']:.4f} ± {mode['imag']:.4f}j  "
                    f'({freq_hz:.4f} Hz, {stable_str})'
                )
            else:
                tc = mode['time_constant']
                print(f"  λ = {mode['real']:.4f}  (τ = {
                      tc:.3f} s, {stable_str})")
    print('=' * 80)


__all__ = [
    'find_trim_fixed_wing',
    'classify_aircraft_modes',
    'print_mode_summary',
]
