"""
Piecewise Polynomial Trajectory Optimization

This module solves optimal trajectory planning problems using piecewise polynomials
with arbitrary boundary conditions. The optimization minimizes a weighted sum of 
derivative norms (position, velocity, acceleration, jerk, snap, etc.) subject to:
  - Continuity constraints between segments
  - Boundary value constraints (positions, velocities, accelerations, etc.)
  - Continuity discontinuities (e.g., acceleration jumps)

The problem is formulated as a quadratic program:
  minimize: p^T Q p
  subject to: A p = b
"""

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt


# ============================================================================
# Matrix Construction Functions
# ============================================================================

def A_0_matrix(order):
    """
    Construct the A_0 matrix for evaluating polynomial derivatives at t=0.
    
    A_0 is a diagonal matrix with falling factorials on the diagonal:
    A_0[r,r] = r! (the r-th derivative of t^n at t=0)
    A_0[r,n] = 0 for r ≠ n
    
    This represents: d^r P(0)/dt^r = r! * a_r for polynomial P(t) = Σ a_n t^n
    
    Args:
        order: Polynomial degree (matrix will be (order+1) x (order+1))
    
    Returns:
        numpy array: A_0 matrix with shape (order+1, order+1)
    """
    size = order + 1
    A_0 = np.zeros((size, size))
    
    for r in range(size):
        # Compute falling factorial r! = r * (r-1) * ... * 1
        factorial = 1
        for m in range(r):
            factorial *= (r - m)
        A_0[r, r] = factorial
    
    return A_0

def A_tau_matrix(order, tau):
    """
    Construct the A_tau matrix for evaluating polynomial derivatives at t=tau.
    
    A_τ represents derivatives of polynomials evaluated at t=τ:
    A_τ[r,n] = (falling factorial) * τ^(n-r) for n >= r, else 0
    
    This represents: d^r P(τ)/dt^r = Σ_{n=r}^{order} (falling fact.) * a_n * τ^(n-r)
    
    Args:
        order: Polynomial degree
        tau: Time point at which to evaluate (segment duration)
    
    Returns:
        numpy array: A_tau matrix with shape (order+1, order+1)
    """
    size = order + 1
    A_tau = np.zeros((size, size))
    
    for r in range(size):
        for n in range(size):
            if n >= r:
                # Compute falling factorial: (n) * (n-1) * ... * (n-r+1)
                falling_fact = 1
                for m in range(r):
                    falling_fact *= (n - m)
                
                # Multiply by τ^(n-r)
                A_tau[r, n] = falling_fact * (tau ** (n - r))
    
    return A_tau

def get_A_continuity(order, tau, K):
    """
    Build the continuity constraint matrix for piecewise polynomials.
    
    This enforces C^n continuity (all derivatives continuous) between segments.
    Creates (K-1) * (order+1) constraints for K segments of order-degree polynomials.
    
    The constraint enforces: P_k(τ) = P_{k+1}(0) for all k
    Which means: A_τ * [coeffs_k] = A_0 * [coeffs_{k+1}]
    
    Args:
        order: Polynomial degree
        tau: Segment duration
        K: Number of segments
    
    Returns:
        CasADi DM matrix: Continuity constraint matrix with shape ((K-1)*(order+1), K*(order+1))
    """
    size = order + 1
    A_rows = []
    
    for i in range(K - 1):
        # Build row enforcing: -A_tau * p_k + A_0 * p_{k+1} = 0
        if i == 0:
            # First segment: [-A_tau | A_0 | 0 ... 0]
            row = ca.horzcat(
                -A_tau_matrix(order, tau[i]),
                A_0_matrix(order),
                np.zeros((order + 1, (K - 2 - i) * (order + 1)))
            )
        else:
            # Middle segments: [0 ... 0 | -A_tau | A_0 | 0 ... 0]
            row = ca.horzcat(
                np.zeros((order + 1, i * (order + 1))),
                -A_tau_matrix(order, tau[i]),
                A_0_matrix(order),
                np.zeros((order + 1, (K - 2 - i) * (order + 1)))
            )
        A_rows.append(np.array(row)[0:5])
    
    A = ca.vertcat(*A_rows)
    return A


# ============================================================================
# Cost Matrix Construction Functions
# ============================================================================

def get_Q_matrix_entries(r, i, l, tau):
    """
    Compute a single entry Q_r^{il} of the cost matrix for derivative order r.
    
    The cost matrix penalizes the r-th derivative:
    Q_r^{il} = 2 * (falling fact.) * τ^{i+l-2r+1} / (i+l-2r+1)
    
    This comes from ∫₀^τ [d^r P(t)/dt^r]² dt integrated over a polynomial basis.
    
    Args:
        r: Derivative order (0=position, 1=velocity, 2=acceleration, etc.)
        i: Row index of polynomial coefficient
        l: Column index of polynomial coefficient
        tau: Segment duration
    
    Returns:
        float: Q_r^{il} entry
    """
    # Zero out if either index is less than derivative order
    if i < r or l < r:
        return 0.0
    
    # Compute falling factorial: (i)(i-1)...(i-r+1) * (l)(l-1)...(l-r+1)
    falling_fact = 1.0
    for m in range(r):
        falling_fact *= (i - m) * (l - m)
    
    # Compute scaling: τ^(i+l-2r+1)
    exponent = tau ** (i + l - 2 * r + 1)
    
    # Compute denominator
    denominator = i + l - 2 * r + 1
    
    return 2.0 * falling_fact * exponent / denominator

def Q_total_matrix(order, tau, weights=None):
    """
    Construct the weighted cost matrix Q for a single segment.
    
    Q = Σ_{r=0}^{order} c_r * Q_r
    
    where Q_r is the cost matrix for penalizing the r-th derivative, and c_r
    are the penalty weights (0=no penalty, high value=strong penalty).
    
    Args:
        order: Polynomial degree
        tau: Segment duration
        weights: List of weights [c_0, c_1, c_2, ...] for each derivative.
                Default: uniform weights of 1.0
    
    Returns:
        numpy array: Q matrix with shape (order+1, order+1)
    """
    size = order + 1
    Q = np.zeros((size, size))
    
    # Use uniform weights if not provided
    if weights is None:
        weights = np.ones(size)
    
    # Sum weighted cost matrices for each derivative order
    for r in range(size):
        Q_r = np.zeros((size, size))
        for i in range(size):
            for l in range(size):
                Q_r[i, l] = get_Q_matrix_entries(r, i, l, tau)
        
        # Add weighted contribution
        Q += weights[r] * Q_r
    
    return Q

def Q_block_diagonal(order, tau, segments, weights=None):
    """
    Create block diagonal cost matrix for all segments.
    
    Constructs a block diagonal matrix where each block is the cost matrix Q
    for that segment. Total size: (segments*(order+1)) x (segments*(order+1))
    
    Args:
        order: Polynomial degree
        tau: Segment duration (same for all segments)
        segments: Number of segments
        weights: Penalty weights for derivatives
    
    Returns:
        numpy array: Block diagonal Q matrix
    """
    
    size = order + 1
    total_size = segments * size
    Q_block = np.zeros((total_size, total_size))
    
    # Place Q matrix on diagonal for each segment
    for k in range(segments):
        Q_single = Q_total_matrix(order, tau[k], weights=weights)
        start = k * size
        end = (k + 1) * size
        Q_block[start:end, start:end] = Q_single
    
    return Q_block

def get_A_constraints(order, tau, segments):
    """
    Build the full constraint matrix (structure varies by application).
    
    This is a utility function for building diagonal constraint matrices.
    Note: Not actively used in current implementation; kept for reference.
    
    Args:
        order: Polynomial degree
        tau: Segment duration
        segments: Number of segments
    
    Returns:
        numpy array: Constraint matrix
    """
    size = order + 1
    total_size = segments * size
    A_constraints = np.zeros((total_size, total_size))
    
    # Diagonal blocks with A_0 matrices
    for k in range(segments - 1):
        start = k * size
        end = (k + 1) * size
        A_constraints[start:end, start:end] = A_0_matrix(order)
    
    # Last block
    last_start = (segments - 1) * size
    last_end = segments * size
    A_constraints[last_start:last_end, last_start:last_end] = A_0_matrix(order)
    
    # Append A_tau constraint row for final continuity
    A_tau_constraint = A_tau_matrix(order, tau[-1])
    A_constraints = np.vstack([
        A_constraints,
        np.hstack([np.zeros((size, (segments - 1) * size)), A_tau_constraint])
    ])
    
    return A_constraints


# ============================================================================
# Plotting Functions
# ============================================================================

def plot_piecewise_polynomials(polynomials, segment_times, t_eval=None, title="Piecewise Polynomial"):
    """
    Plot a piecewise polynomial trajectory.
    
    Each polynomial is evaluated over its segment time, with different colors
    for different segments.
    
    Args:
        polynomials: Array of shape (segments, order+1) with polynomial coefficients [a_0, a_1, ...]
        segment_times: List of segment durations [τ_0, τ_1, ...]
        t_eval: Time points to evaluate (auto-generated if None)
        title: Plot title
    
    Returns:
        tuple: (figure, axes) matplotlib objects
    """
    # Compute cumulative time at segment boundaries
    gamma = [0]
    for tau in segment_times:
        gamma.append(gamma[-1] + tau)
    
    # Auto-generate evaluation points if not provided
    if t_eval is None:
        t_eval = np.linspace(0, gamma[-1], 500)
    
    # Evaluate polynomials at each time point
    T_values = np.zeros_like(t_eval)
    segment_indices = np.zeros_like(t_eval, dtype=int)
    
    for i, t in enumerate(t_eval):
        for k in range(len(polynomials)):
            if gamma[k] < t <= gamma[k + 1]:
                t_local = t - gamma[k]
                # Evaluate polynomial P_k at local time
                T_values[i] = np.polyval(polynomials[k][::-1], t_local)
                segment_indices[i] = k
                break
        else:
            # Handle t=0 case
            if t == gamma[0] and len(polynomials) > 0:
                T_values[i] = np.polyval(polynomials[0][::-1], 0)
                segment_indices[i] = 0
    
    # Plot with different colors per segment
    fig, ax = plt.subplots(figsize=(10, 6))
    colors = plt.cm.tab10(np.linspace(0, 1, len(polynomials)))
    
    for k in range(len(polynomials)):
        mask = segment_indices == k
        ax.plot(t_eval[mask], T_values[mask], color=colors[k], 
                linewidth=2, label=f'Segment {k}')
    
    # Mark segment boundaries
    for g in gamma[1:-1]:
        ax.axvline(x=g, color='k', linestyle='--', alpha=0.3, linewidth=1)
    
    ax.set_xlabel('Time (t)', fontsize=12)
    ax.set_ylabel('T(t)', fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    return fig, ax

def compute_polynomial_derivative(coefficients, derivative_order=1):
    """
    Compute polynomial coefficients for the derivative.
    
    Given polynomial P(t) = a_0 + a_1*t + a_2*t² + ... + a_n*t^n,
    returns coefficients of d^k P/dt^k
    
    Args:
        coefficients: Array [a_0, a_1, a_2, ...]
        derivative_order: Which derivative to compute (1, 2, 3, ...)
    
    Returns:
        numpy array: Derivative polynomial coefficients
    """
    result = np.array(coefficients, dtype=float)
    
    for _ in range(derivative_order):
        if len(result) <= 1:
            return np.array([0.0])
        # Apply power rule: d/dt(a_n * t^n) = n * a_n * t^(n-1)
        result = np.array([result[i] * i for i in range(1, len(result))])
    
    return result if len(result) > 0 else np.array([0.0])

def plot_derivatives(polynomials, segment_times, title="Trajectory Derivatives"):
    """
    Plot position and its derivatives (velocity, acceleration, jerk).
    
    Creates a 4-subplot figure showing:
      - Position (0th derivative)
      - Velocity (1st derivative)
      - Acceleration (2nd derivative)
      - Jerk (3rd derivative)
    
    Args:
        polynomials: Array of shape (segments, order+1) with coefficients
        segment_times: List of segment durations
        title: Plot title
    
    Returns:
        tuple: (figure, axes) matplotlib objects
    """
    # Compute segment boundaries
    gamma = [0]
    for tau in segment_times:
        gamma.append(gamma[-1] + tau)
    
    # Evaluation points
    t_eval = np.linspace(0, gamma[-1], 1000)
    
    # Initialize output arrays
    position = np.zeros_like(t_eval)
    velocity = np.zeros_like(t_eval)
    acceleration = np.zeros_like(t_eval)
    jerk = np.zeros_like(t_eval)
    
    # Pre-compute derivatives for all segments
    derivatives = [None] * len(polynomials)
    for k in range(len(polynomials)):
        derivatives[k] = [
            np.array(polynomials[k]),                           # 0th: position
            compute_polynomial_derivative(polynomials[k], 1),   # 1st: velocity
            compute_polynomial_derivative(polynomials[k], 2),   # 2nd: acceleration
            compute_polynomial_derivative(polynomials[k], 3)    # 3rd: jerk
        ]
    
    # Evaluate at each time point
    for i, t in enumerate(t_eval):
        for k in range(len(polynomials)):
            if gamma[k] <= t <= gamma[k + 1]:
                t_local = t - gamma[k]
                position[i] = np.polyval(derivatives[k][0][::-1], t_local)
                velocity[i] = np.polyval(derivatives[k][1][::-1], t_local) if len(derivatives[k][1]) > 0 else 0
                acceleration[i] = np.polyval(derivatives[k][2][::-1], t_local) if len(derivatives[k][2]) > 0 else 0
                jerk[i] = np.polyval(derivatives[k][3][::-1], t_local) if len(derivatives[k][3]) > 0 else 0
                break
    
    # Create 4-subplot figure
    fig, axes = plt.subplots(4, 1, figsize=(12, 10))
    
    # Plot each derivative
    axes[0].plot(t_eval, position, 'b-', linewidth=2)
    axes[0].set_ylabel('Position', fontsize=11)
    axes[0].set_title(title, fontsize=13, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    
    axes[1].plot(t_eval, velocity, 'g-', linewidth=2)
    axes[1].set_ylabel('Velocity (1st deriv.)', fontsize=11)
    axes[1].grid(True, alpha=0.3)
    
    axes[2].plot(t_eval, acceleration, 'r-', linewidth=2)
    axes[2].set_ylabel('Acceleration (2nd deriv.)', fontsize=11)
    axes[2].grid(True, alpha=0.3)
    
    axes[3].plot(t_eval, jerk, 'purple', linewidth=2)
    axes[3].set_ylabel('Jerk (3rd deriv.)', fontsize=11)
    axes[3].set_xlabel('Time (t)', fontsize=11)
    axes[3].grid(True, alpha=0.3)
    
    # Mark segment boundaries on all plots
    for g in gamma[1:-1]:
        for ax in axes:
            ax.axvline(x=g, color='k', linestyle='--', alpha=0.3, linewidth=0.8)
    
    plt.tight_layout()
    return fig, axes


# ============================================================================
# Optimization Function
# ============================================================================

def run_poly_optimization(order, tau, segments, weights, boundary_conditions, continuity):
    """
    Solve the piecewise polynomial trajectory optimization problem.
    
    Minimizes: p^T Q p (weighted sum of derivative norms)
    Subject to:
      - Continuity constraints: P_k(τ) = P_{k+1}(0)
      - Boundary constraints: specified derivatives at segment boundaries
      - Continuity discontinuities: specified jumps in derivatives
    
    Args:
        order: Polynomial degree (e.g., 7 for 8 coefficients)
        tau: Segment duration (same for all segments)
        segments: Number of segments
        weights: Penalty weights [c_0, c_1, ...] for derivatives
        boundary_conditions: List of boundary constraint specifications
                            Each element: [values for each derivative at boundaries]
        continuity: List of [constraint_index, jump_value] for discontinuities
    
    Returns:
        dict: {
            'polys': (segments, order+1) array of polynomial coefficients,
            'order': polynomial order,
            'segments': number of segments,
            'tau': segment duration
        }
    """
    size = order + 1
    
    # Extract boundary values from the boundary_conditions structure
    b_vals = []
    
    # boundary_conditions[deriv_order][0] gives list of constraint values
    for i in range(segments + 1):
        for condition in boundary_conditions:
            b_vals.append(condition[0][i])
    

    # Get constraint matrices
    A_constrain = get_A_constraints(order, tau, segments)
    A_continuity = get_A_continuity(order, tau, segments)
    
    # Build boundary constraint matrix: include only non-None values
    A = []
    b = []
    for value, row in zip(b_vals, A_constrain):
        if value is not None:
            b.append(value)
            A.append(row)
    
    # Combine boundary and continuity constraints
    A = ca.vertcat(ca.DM(A), A_continuity)

    
    # Set up continuity jump constraints
    b_continuity = ca.DM.zeros(A_continuity.shape[0], 1)
    if continuity:
        for constraint_idx, jump_value in continuity:
            b_continuity[constraint_idx] = jump_value
    
    b = ca.vertcat(np.array(b).T, b_continuity)
    
    # Decision variables: polynomial coefficients for all segments
    p = ca.SX.sym('p', (order + 1) * segments)
    
    # Build cost matrix and objective function
    Q = Q_block_diagonal(order, tau, segments, weights=weights)
    objective = ca.mtimes(p.T, ca.mtimes(Q, p))
    
    # Set up and solve NLP
    opts = dict(verbose=False)
    nlp = {'x': p, 'f': objective, 'g': ca.mtimes(A, p) - b}
    solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

    solution = solver(
        x0=ca.DM([0] * p.shape[0]),
        lbg=ca.DM([0] * b.shape[0]),
        ubg=ca.DM([0] * b.shape[0]),
        
    )
    
    x_opt = solution['x']
    
    # Reshape solution into polynomial coefficients
    polys = np.array(x_opt).T.reshape(segments, order + 1)
    
    return {
        'polys': polys,
        'order': order,
        'segments': segments,
        'tau': tau
    }   


# ============================================================================
# Main Example
# ============================================================================

if __name__ == '__main__':
    # Define boundary conditions for each derivative order
    # Structure: boundary_conditions[deriv_order] = [[values at boundaries for each segment]]
    boundary_positions = [[0, 1, 2, 0]]      # Position: start=0, end=0
    boundary_velocities = [[None, None, None, None]]       # Velocity: start=0, others free
    boundary_accelerations = [[None, None, None, None]]    # Acceleration: start=0
    boundary_jerk = [[None, None, None, None]]             # Jerk: start=0
    boundary_snap = [[None, None, None, None]]             # Snap: start=0
    boundary_crackle = [[None, None, None, None]]             # Snap: start=0
    boundary_pop = [[None, None, None, None]]             # Snap: start=0
    boundary_lock = [[None, None, None, None]]             # Snap: start=0
    
    boundary_conditions = [
        boundary_positions,
        boundary_velocities,
        boundary_accelerations,
        boundary_jerk,
        boundary_snap,
        boundary_crackle,
        boundary_pop,
        boundary_lock
    ]
    
    # Define acceleration discontinuities (jumps in 2nd derivative)
    continuity_changes = [[2, -0.125], [7, 0.125]]
    tau=[0.3,0.5,0.2]
    
    # Solve optimization problem
    outputs = run_poly_optimization(
        order=7,
        tau=tau,
        segments=3,
        weights=[0, 1, 1, 1, 100.0, 1, 1, 1],  # Minimize snap (4th derivative)
        boundary_conditions=boundary_conditions,
        continuity=continuity_changes
    )
    
    # Plot results
    
    fig1, ax1 = plot_piecewise_polynomials(
        outputs['polys'],
        tau,
        title="Optimized Piecewise Polynomial Trajectory"
    )
    plt.tight_layout()
    plt.show()
    
    fig2, axes2 = plot_derivatives(
        outputs['polys'],
        tau,
        title="Trajectory Derivatives: Position, Velocity, Acceleration, Jerk"
    )
    plt.show()





