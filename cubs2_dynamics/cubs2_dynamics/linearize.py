#!/usr/bin/env python3
"""
Linearize sportcub dynamics and perform modal analysis.

Extracts linearized state-space model at a trim condition (e.g., steady level flight at 3 m/s)
and computes eigenvalues/eigenvectors to identify aircraft dynamic modes:
- Short period (pitch oscillation)
- Phugoid (long-period altitude/speed oscillation)
- Dutch roll (yaw/roll oscillation)
- Roll damping
- Spiral mode

Usage:
    python3 linearize.py
"""

import casadi as ca
import numpy as np

from scipy import linalg


# ---------------------------------------------------------------------------
# Generic Trim Solver
# ---------------------------------------------------------------------------
def find_trim(
    model,
    x_guess=None,
    u_guess=None,
    cost_fn=None,
    constraints_fn=None,
    print_progress=True,
    verbose=False,
    ipopt_print_level=1,
    max_iter=2000,
    return_dataclasses=False,
):
    """Generic trim solver for any ModelSX/MX-based aircraft or dynamic model.

    This function provides a flexible trim solution by allowing callers to specify:
      - Initial guesses (dataclass instance, list/np.ndarray, or None for model defaults)
      - A custom cost function callback
      - A custom constraints callback

    The objective is solved as a nonlinear program using CasADi Opti/Ipopt.

    Parameters
    ----------
    model : ModelSX/MX instance
        Built model exposing `f_x` and dataclasses (`x0`, `u0`, `p0`).
    x_guess : Sequence | dataclass | None
        Initial state guess. If None uses `model.x0`. If dataclass, will be converted
        using `.as_vec()`. If sequence length must match state dimension.
    u_guess : Sequence | dataclass | None
        Initial input guess. Defaults to `model.u0` if None.
    cost_fn : callable(opti, model, x, u, x_dot, p_vec) -> expression
        Builds the objective expression where x, u, x_dot are dataclass instances.
        If None, a default is used: minimize( ||x_dot||^2 + 1e-3*||u||^2 )
    constraints_fn : callable(opti, model, x, u, x_dot, p_vec) -> None
        Adds problem-specific constraints where x, u, x_dot are dataclass instances.
        If None, no constraints are added (fully generic).
    print_progress : bool
        Print high-level optimization progress.
    verbose : bool
    If True, calls `print_trim_details` on success.
    ipopt_print_level : int
        Ipopt iteration verbosity level (0-5).
    max_iter : int
        Maximum Ipopt iterations.
    return_dataclasses : bool
        If True returns (State instance, Input instance, stats) instead of raw arrays.

    Returns
    -------
    x_trim, u_trim, stats : np.ndarray | dataclass, np.ndarray | dataclass, dict | None
        Trim solution and solver stats. Stats may be None if unavailable.

    Notes
    -----
    The generic solver is intentionally lightweight; callers define physics-aware
    objectives and constraints externally. All callbacks receive structured dataclass
    instances (x, u, x_dot) for clean, type-safe access to model variables.
    """
    # Prepare numeric defaults
    x0_vec = np.array(model.x0.as_vec()).flatten()
    u0_vec = np.array(model.u0.as_vec()).flatten()
    p_vec = model.p0.as_vec()

    n_x = len(x0_vec)
    n_u = len(u0_vec)

    # Resolve initial guesses
    def _resolve_guess(g, fallback, expected):
        if g is None:
            return fallback
        if hasattr(g, "as_vec"):
            arr = np.array(g.as_vec()).flatten()
        else:
            arr = np.array(g).flatten()
        if len(arr) != expected:
            raise ValueError(f"Initial guess length {len(arr)} != expected {expected}")
        return arr

    x_init = _resolve_guess(x_guess, x0_vec, n_x)
    u_init = _resolve_guess(u_guess, u0_vec, n_u)

    # Optimization problem
    opti = ca.Opti()
    x_var = opti.variable(n_x)
    u_var = opti.variable(n_u)

    # Continuous dynamics evaluation
    f_x_result = model.f_x(x_var, u_var, p_vec)
    x_dot_vec = f_x_result["dx_dt"] if isinstance(f_x_result, dict) else f_x_result

    # Wrap variables in dataclass instances for structured access
    x = model.state_type.from_vec(x_var)
    u = model.input_type.from_vec(u_var)
    x_dot = model.state_type.from_vec(x_dot_vec)

    # Default cost if none provided - generic: minimize state derivatives and control effort
    if cost_fn is None:
        obj = ca.sumsqr(x_dot_vec) + 1e-3 * ca.sumsqr(u_var)
    else:
        obj = cost_fn(model, x, u, x_dot, p_vec)
        if obj is None:
            raise ValueError("cost_fn returned None; must return CasADi expression")

    opti.minimize(obj)

    # Apply constraints - only use custom constraints function
    # Generic linearization should not assume specific model structure
    if constraints_fn is not None:
        constraints = constraints_fn(model, x, u, x_dot, p_vec)
        if constraints is not None:
            for constraint in constraints:
                opti.subject_to(constraint)

    # Solver options
    opts = {
        "ipopt.print_level": int(ipopt_print_level),
        "ipopt.max_iter": int(max_iter),
        "ipopt.tol": 1e-9,
        "ipopt.acceptable_tol": 1e-6,
        "print_time": bool(print_progress),
    }
    opti.solver("ipopt", opts)

    # Set initial guesses
    opti.set_initial(x_var, x_init)
    opti.set_initial(u_var, u_init)

    if print_progress:
        print("  Optimizing trim...")

    try:
        sol = opti.solve()
        x_trim = np.array(sol.value(x_var)).flatten()
        u_trim = np.array(sol.value(u_var)).flatten()
        stats = None
        try:
            stats = sol.stats()
        except Exception:
            stats = None
        if print_progress:
            print(f"  ✓ Trim converged. Final objective: {sol.value(obj):.6e}")
        if verbose:
            print_trim_details(model, x_trim, u_trim, p_vec, header="TRIM (VERBOSE)")
    except Exception as e:
        if print_progress:
            print(f"  ⚠ Trim failed: {e}")
        x_trim = np.array(opti.debug.value(x_var)).flatten()
        u_trim = np.array(opti.debug.value(u_var)).flatten()
        stats = None
        if verbose:
            print_trim_details(model, x_trim, u_trim, p_vec, header="TRIM (FAILED)")

    if return_dataclasses:
        x_dc = model.State.from_vec(x_trim) if hasattr(model, "State") else x_trim
        u_dc = model.Input.from_vec(u_trim) if hasattr(model, "Input") else u_trim
        return x_dc, u_dc, stats
    return x_trim, u_trim, stats


def print_trim_details(model, x_trim, u_trim, p_vec, header="TRIM DETAILS"):
    """Lightweight trim printer using dataclass repr - fully generic."""
    print("\n" + "=" * 80)
    print(header)
    print("=" * 80)

    # Use dataclass repr for clean, automatic printing
    x_dc = model.state_type.from_vec(x_trim)
    u_dc = model.input_type.from_vec(u_trim)

    print("States:")
    print(f"  {x_dc}")
    print("\nInputs:")
    print(f"  {u_dc}")

    if hasattr(model, "f_y") and hasattr(model, "output_type"):
        y_vec = model.f_y(x_trim, u_trim, p_vec)
        y_dc = model.output_type.from_vec(y_vec)
        print("\nOutputs:")
        print(f"  {y_dc}")

    print("=" * 80)


def linearize_dynamics(model, x_trim, u_trim, p=None):
    """
    Linearize dynamics around trim point.

    Returns:
        A: State matrix (n_x × n_x)
        B: Input matrix (n_x × n_u)
    """
    if p is None:
        p = model.p0.as_vec()

    # Convert to numpy if needed and get dimensions
    x_trim = np.array(x_trim).flatten()
    u_trim = np.array(u_trim).flatten()
    p = np.array(p).flatten()

    n_x = len(x_trim)
    n_u = len(u_trim)
    n_p = len(p)

    # Create symbolic state and input
    x_sym = ca.SX.sym("x", n_x)
    u_sym = ca.SX.sym("u", n_u)
    p_sym = ca.SX.sym("p", n_p)

    # Dynamics function (model.f_x is the CasADi Function)
    f_x_result = model.f_x(x_sym, u_sym, p_sym)
    if isinstance(f_x_result, dict):
        f_x = f_x_result["dx_dt"]
    else:
        f_x = f_x_result

    # Compute Jacobians
    A_sym = ca.jacobian(f_x, x_sym)
    B_sym = ca.jacobian(f_x, u_sym)

    # Evaluate at trim point
    A_func = ca.Function("A", [x_sym, u_sym, p_sym], [A_sym])
    B_func = ca.Function("B", [x_sym, u_sym, p_sym], [B_sym])

    A = np.array(A_func(x_trim, u_trim, p)).astype(float)
    B = np.array(B_func(x_trim, u_trim, p)).astype(float)

    return A, B


def analyze_modes(A, state_names=None, dt=None):
    """
    Perform eigenvalue analysis to identify dynamic modes.

    Args:
        A: State matrix (continuous or discrete)
        state_names: Optional list of state variable names for dominant state identification
        dt: Time step (if discrete-time system)

    Returns:
        modes: List of dicts with mode characteristics
    """
    # Compute eigenvalues and eigenvectors
    eigvals, eigvecs = linalg.eig(A)

    modes = []
    processed = set()

    for i, (lam, vec) in enumerate(zip(eigvals, eigvecs.T)):
        if i in processed:
            continue

        # Convert discrete eigenvalue to continuous if needed
        if dt is not None:
            lam_c = np.log(lam) / dt
        else:
            lam_c = lam

        real_part = np.real(lam_c)
        imag_part = np.imag(lam_c)

        mode = {
            "index": i,
            "eigenvalue": lam,
            "eigenvalue_continuous": lam_c,
            "real": real_part,
            "imag": imag_part,
            "eigenvector": vec,
        }

        # Time constant (1/|real_part|)
        if abs(real_part) > 1e-9:
            mode["time_constant"] = 1.0 / abs(real_part)
            mode["damping_ratio"] = -real_part / np.sqrt(real_part**2 + imag_part**2)
        else:
            mode["time_constant"] = np.inf
            mode["damping_ratio"] = 0.0

        # Natural frequency and period for oscillatory modes
        if abs(imag_part) > 1e-6:
            mode["frequency_hz"] = abs(imag_part) / (2 * np.pi)
            mode["period"] = 2 * np.pi / abs(imag_part)
            mode["is_oscillatory"] = True

            # Find complex conjugate pair
            for j in range(i + 1, len(eigvals)):
                if np.abs(eigvals[j] - np.conj(lam)) < 1e-9:
                    processed.add(j)
                    break
        else:
            mode["frequency_hz"] = 0.0
            mode["period"] = np.inf
            mode["is_oscillatory"] = False

        # Stability
        if dt is not None:
            mode["stable"] = abs(lam) < 1.0
        else:
            mode["stable"] = real_part < 0

        # Dominant states (largest magnitude in eigenvector)
        if state_names is not None:
            state_mag = np.abs(vec)
            dominant_idx = np.argsort(state_mag)[::-1][:3]
            mode["dominant_states"] = [(state_names[idx], state_mag[idx]) for idx in dominant_idx]

        modes.append(mode)

    return modes

    # sportcub-specific trim and mode classification removed for generic module usage.
