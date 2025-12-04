import numpy as np
import matplotlib.pyplot as plt
from cyecca.planning.dubins import derive_dubins, DubinsPathType
from polynomial_optimization import run_poly_optimization, plot_piecewise_polynomials, plot_derivatives
from numpy.lib.stride_tricks import sliding_window_view


plan_fn, eval_fn = derive_dubins()

# Plan Dubins path using ca functions 
def plan_dubins_path(start_pos, start_yaw, end_pos, end_yaw, turn_radius):
    """
    Plan a Dubins path between two poses.
    
    Args:
        start_pos: [x, y] starting position
        start_yaw: starting heading (radians)
        end_pos: [x, y] ending position
        end_yaw: ending heading (radians)
        turn_radius: minimum turning radius
    
    Returns:
        Dictionary with path information
    """
    p0 = np.array([start_pos[0], start_pos[1]])
    p1 = np.array([end_pos[0], end_pos[1]])
    
    cost, ptype, a1, d, a2, tp0, tp1, c0, c1 = plan_fn(
        p0, start_yaw, p1, end_yaw, turn_radius
    )
    
    return {
        'cost': float(cost),
        'type': DubinsPathType.name(ptype),
        'a1': a1,
        'd': d,
        'a2': a2,
        'tp0': np.array(tp0).flatten(),
        'tp1': np.array(tp1).flatten(),
        'c0': np.array(c0).flatten(),
        'c1': np.array(c1).flatten()
    }

# Get points for dubins path
def evaluate_dubins_path(plan_info, start_pos, start_yaw, end_pos, end_yaw, 
                        turn_radius, num_points=1000):
    """
    Evaluate a planned Dubins path to generate sample points.
    
    Args:
        plan_info: Dictionary returned from plan_dubins_path
        start_pos: [x, y] starting position
        start_yaw: starting heading (radians)
        end_pos: [x, y] ending position
        end_yaw: ending heading (radians)
        turn_radius: minimum turning radius
        num_points: number of points to sample along path
    
    Returns:
        Arrays of (x, y, psi) points along the path
    """
    p0 = np.array([start_pos[0], start_pos[1]])
    p1 = np.array([end_pos[0], end_pos[1]])
    
    points = []
    for s in np.linspace(0, 1, num_points):
        x, y, psi = eval_fn(
            s,
            p0,
            start_yaw,
            plan_info['a1'],
            plan_info['d'],
            plan_info['a2'],
            plan_info['tp0'],
            plan_info['tp1'],
            plan_info['c0'],
            plan_info['c1'],
            turn_radius
        )
        points.append([float(x), float(y), float(psi)])
    
    return np.array(points)

# Calculate curvature of a curve
def compute_curvature_from_xy(path_xy):
    """
    Compute SIGNED curvature from x,y positions using the geometric curvature formula.
    Sign is determined by the orientation (left turn positive, right turn negative).
    """

    # Handle both Nx2 and Nx3 inputs
    if path_xy.shape[1] >= 3:
        xy = path_xy[:, :2]
    else:
        xy = path_xy

    n = len(xy)
    curvature = np.zeros(n)

    for i in range(n):
        if i == 0 or i == n - 1:
            if i == 0 and n > 2:
                p0, p1, p2 = xy[0], xy[1], xy[2]
            elif i == n - 1 and n > 2:
                p0, p1, p2 = xy[n-3], xy[n-2], xy[n-1]
            else:
                curvature[i] = 0.0
                continue
        else:
            p0, p1, p2 = xy[i-1], xy[i], xy[i+1]

        # Side lengths
        a = np.linalg.norm(p1 - p2)
        b = np.linalg.norm(p0 - p2)
        c = np.linalg.norm(p0 - p1)

        # 2D cross product gives twice signed area:
        cross = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0])
        area = 0.5 * abs(cross)

        # unsigned curvature
        denom = a * b * c
        if denom > 1e-12:
            kappa = 4.0 * area / denom
        else:
            kappa = 0.0

        # apply sign
        sign = np.sign(cross)  # positive = CCW turn
        curvature[i] = sign * kappa

    return curvature

# Helper Function to plot curvature
def plot_curvature(path_points, curvature, title=''):
    """
    Plot the path and curvature distribution.
    
    Args:
        path_points: Nx3 array of [x, y, psi] points
        curvature: Array of curvature values
        title: Title for the plot
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # Plot path
    ax1.plot(path_points[:, 0], path_points[:, 1], 'b-', linewidth=2)
    ax1.plot(path_points[0, 0], path_points[0, 1], 'go', markersize=10, label='Start')
    ax1.plot(path_points[-1, 0], path_points[-1, 1], 'r*', markersize=20, label='End')
    ax1.set_xlabel('X (m)', fontsize=12)
    ax1.set_ylabel('Y (m)', fontsize=12)
    ax1.set_title('Path', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.legend()
    
    # Plot curvature
    arc_length = np.zeros(len(curvature))
    for i in range(1, len(curvature)):
        arc_length[i] = arc_length[i-1] + np.linalg.norm(path_points[i, :2] - path_points[i-1, :2])
    
    ax2.plot(arc_length/arc_length[-1], curvature, 'b-', linewidth=2)
    ax2.fill_between(arc_length/arc_length[-1], curvature, alpha=0.3)
    ax2.set_xlabel('Arc Length (m)', fontsize=12)
    ax2.set_ylabel('Curvature (1/m)', fontsize=12)
    ax2.set_title('Curvature Distribution', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    if title:
        fig.suptitle(title, fontsize=14, fontweight='bold', y=1.02)
    
    plt.tight_layout()
    return fig, (ax1, ax2)

# Helper to plot Dubins Path
def plot_dubins_path(ax, path_points, plan_info, start_pos, end_pos, 
                    turn_radius, label=''):
    """
    Plot a Dubins path on a matplotlib axis.
    """
    x = path_points[:, 0]
    y = path_points[:, 1]
    
    # Plot the path
    ax.plot(x, y, 'b-', linewidth=2, label=f'Path {label}' if label else 'Path')
    
    # Plot start and end points
    ax.plot(start_pos[0], start_pos[1], 'go', markersize=10, label='Start')
    ax.plot(end_pos[0], end_pos[1], 'r*', markersize=20, label='End')
    
    # Plot turn circles
    circle1 = plt.Circle(plan_info['c0'], turn_radius, fill=False, 
                         linestyle='--', color='gray', alpha=0.5)
    circle2 = plt.Circle(plan_info['c1'], turn_radius, fill=False, 
                         linestyle='--', color='gray', alpha=0.5)
    ax.add_patch(circle1)
    ax.add_patch(circle2)
    
    # Plot circle centers
    ax.plot(plan_info['c0'][0], plan_info['c0'][1], 'k+', markersize=8)
    ax.plot(plan_info['c1'][0], plan_info['c1'][1], 'k+', markersize=8)

# Helper to extract dubins segments
def extract_dubins_segments(plan_info, turn_radius):
    """
    Extract the three segments of a Dubins path and compute their arc lengths.
    
    A Dubins path consists of: Arc1 -> Straight -> Arc2
    This function computes the length and normalized time for each segment.
    
    Args:
        plan_info: Dictionary from plan_dubins_path() with keys: a0, a1, a2, c0, c1
                   a0, a1, a2: Angles (in radians) for turn1, straight, turn2
                   c0, c1: Centers of the two circular arcs
        turn_radius: Vehicle turning radius (meters)
    
    Returns:
        dict: {
            'segment_1_arc': Arc length of first turn (meters)
            'segment_2_straight': Arc length of straight segment (meters)
            'segment_3_arc': Arc length of second turn (meters)
            'total_length': Total path length (meters)
            'segment_times_normalized': [τ₁, τ₂, τ₃] normalized so τ₁+τ₂+τ₃=1
        }
    """
    # Extract angles from plan
    a1 = float(plan_info['a1'])  # First arc angle
    a2 = float(plan_info['a2'])  # Second arc angle
    
    # Segment 1: First circular arc
    segment_1_arc = abs(a1) * turn_radius
    
    # Segment 3: Second circular arc
    segment_3_arc = abs(a2) * turn_radius
    
    # Segment 2: Straight line segment
    # Use the straight segment length from plan_info if available, else compute from geometry
    if 'd' in plan_info:
        segment_2_straight = float(plan_info['d'])
    else:
        # Compute distance between arc endpoints
        c0 = np.array(plan_info['c0'])
        c1 = np.array(plan_info['c1'])
        segment_2_straight = np.linalg.norm(c1 - c0)
    
    # Total length
    total_length = segment_1_arc + segment_2_straight + segment_3_arc
    
    # Normalized segment times (for polynomial trajectory optimization)
    segment_times_normalized = [
        segment_1_arc / total_length,
        segment_2_straight / total_length,
        segment_3_arc / total_length
    ]
    
    return {
        'segment_1_arc': segment_1_arc,
        'segment_2_straight': segment_2_straight,
        'segment_3_arc': segment_3_arc,
        'total_length': total_length,
        'segment_times_normalized': segment_times_normalized
    }

# Ofset polynomial from dubins curve
def apply_polynomial_offset_to_dubins(dubins_path, polynomial_coeffs, tau_durations):
    """
    Apply a polynomial trajectory as a perpendicular offset to a Dubins curve.
    
    The polynomial defines the lateral (perpendicular) displacement from the curve:
    - Positive values: offset to the left (perpendicular to heading, counterclockwise)
    - Negative values: offset to the right (perpendicular to heading, clockwise)
    
    Args:
        dubins_path: Nx3 array of [x, y, psi] points along the Dubins path
        polynomial_coeffs: (segments, order+1) array of polynomial coefficients
        tau_durations: List of segment durations [τ₁, τ₂, τ₃]
    
    Returns:
        tuple: (offsetted_path, arc_lengths, lateral_offsets, seg_boundaries)
            - offsetted_path: Nx3 array of [x, y, psi] with lateral offset applied
            - arc_lengths: Array of cumulative arc lengths
            - lateral_offsets: Array of lateral offset values at each point
            - seg_boundaries: Dict with 'seg1_end' and 'seg2_end'
    """
    # Compute total path length
    total_length = 0
    arc_lengths = [0]
    for i in range(1, len(dubins_path)):
        ds = np.linalg.norm(dubins_path[i, :2] - dubins_path[i-1, :2])
        total_length += ds
        arc_lengths.append(total_length)
    
    arc_lengths = np.array(arc_lengths)
    normalized_arc = arc_lengths

    print(normalized_arc)
    
    # Convert polynomial segment durations to arc length boundaries
    seg1_end = tau_durations[0]
    seg2_end = seg1_end + tau_durations[1]

    seg_ends = []

    for value in tau_durations:
        seg_ends.append(value + (seg_ends[-1] if seg_ends else 0))
    
    # Create offsetted path and lateral offsets
    offsetted_path = np.zeros_like(dubins_path)
    lateral_offsets = []
    
    for i, (s_normalized, point) in enumerate(zip(normalized_arc, dubins_path)):
        x, y, psi = point

        bigger = [val > s_normalized for val in seg_ends]

        seg_idx = bigger.index(True)
        s_local = s_normalized - (seg_idx > 0) * seg_ends[seg_idx-1] 
        
        # Find which segment this point belongs to
        # if s_normalized <= seg1_end:
        #     seg_idx = 0
        #     s_local = s_normalized
        # elif s_normalized <= seg2_end:
        #     seg_idx = 1
        #     s_local = s_normalized - seg1_end
        # else:
        #     seg_idx = 2
        #     s_local = s_normalized - seg2_end
        
        
        # # Evaluate polynomial P(t) = Σ a_n * t^n
        offset = np.polyval(polynomial_coeffs[seg_idx][::-1], s_local)
        lateral_offsets.append(offset)
        
        # Compute perpendicular direction (left is positive, right is negative)
        # If heading is psi, perpendicular direction is psi + π/2 (90° counterclockwise)
        perp_direction = psi + np.pi / 2
        
        # Apply offset
        offset_x = offset * np.cos(perp_direction)
        offset_y = offset * np.sin(perp_direction)
        
        offsetted_path[i, 0] = x + offset_x
        offsetted_path[i, 1] = y + offset_y
        offsetted_path[i, 2] = psi  # Heading remains the same
    
    lateral_offsets = np.array(lateral_offsets)
    
    return offsetted_path, arc_lengths, lateral_offsets

# Clean output
def rolling_median_replace(x, window=7, thresh=3.0, replace_with='median'):
    x = np.asarray(x, dtype=float)
    if window % 2 == 0:
        raise ValueError("window must be odd")
    k = (window - 1) // 2
    sw = sliding_window_view(x, window)
    med = np.median(sw, axis=1)
    med_full = np.pad(med, (k, k), mode='edge')

    residual = np.abs(x - med_full)
    sigma_est = np.median(np.abs(x - np.median(x))) * 1.4826
    if sigma_est == 0:
        sigma_est = np.std(x) + 1e-12
    outliers = residual > (thresh * sigma_est)

    x_clean = x.copy()
    if replace_with == 'median':
        x_clean[outliers] = med_full[outliers]
    else:  # interpolate across outliers
        good = ~outliers
        x_clean[outliers] = np.interp(np.flatnonzero(outliers),
                                      np.flatnonzero(good),
                                      x[good])
    return x_clean


if __name__ == '__main__':
    # Multi-Waypoint Dubins Path
    # Connect 6 waypoints with Dubins trajectories, showing detailed segment breakdown

    # Define 6 waypoints: each with position (x, y) and heading (yaw)
    waypoints = [
        {'pos': [0, 0], 'yaw': np.pi/4, 'name': 'Start'},
        {'pos': [20, 20], 'yaw': np.pi/2, 'name': 'WP 1'},
        {'pos': [-20, 20], 'yaw': -np.pi/2, 'name': 'WP 2'},
        {'pos': [20, -20], 'yaw': -np.pi/2, 'name': 'WP 3'},
        {'pos': [-20, -20], 'yaw': np.pi/2, 'name': 'WP 4'},
        {'pos': [0, 0], 'yaw': np.pi/4, 'name': 'End'}
    ]

    turn_radius = 8.0
    num_points_per_segment = 1000

    # Plan Dubins paths between consecutive waypoints
    dubins_paths = []
    dubins_plans = []

    segment_L = []
    segment_direction = []

    for i in range(len(waypoints) - 1):
        wp_start = waypoints[i]
        wp_end = waypoints[i + 1]
        
        # Plan Dubins path
        plan = plan_dubins_path(
            wp_start['pos'],
            wp_start['yaw'],
            wp_end['pos'],
            wp_end['yaw'],
            turn_radius
        )
        dubins_plans.append(plan)
        
        # Evaluate path
        path_points = evaluate_dubins_path(
            plan,
            wp_start['pos'],
            wp_start['yaw'],
            wp_end['pos'],
            wp_end['yaw'],
            turn_radius,
            num_points=num_points_per_segment
        )
        dubins_paths.append(path_points)
        
        # Calculate segment lengths from evaluated path
        path_type = plan['type']
        total_seg_length = 0
        for j in range(1, len(path_points)):
            total_seg_length += np.linalg.norm(path_points[j, :2] - path_points[j-1, :2])
        
        # Use the plan's a1, d, a2 values (angles and straight distance)
        # Convert to scalars if they're arrays
        a1 = float(np.atleast_1d(plan['a1'])[0])
        d = float(np.atleast_1d(plan['d'])[0])
        a2 = float(np.atleast_1d(plan['a2'])[0])
        
        # Calculate arc lengths from angles
        L1 = a1 * turn_radius      # Arc 1 length
        L2 = d                     # Straight length
        L3 = a2 * turn_radius      # Arc 2 length

        segment_L.append(L1)
        segment_L.append(L2)
        segment_L.append(L3)
        
        
        a1_deg = float(np.degrees(a1))
        a2_deg = float(np.degrees(a2))

        segment_direction.append('L' if path_type[0] == 'L' else 'R')
        segment_direction.append('S')
        segment_direction.append('L' if path_type[2] == 'L' else 'R')
        
        print(f"\n{'='*70}")
        print(f"Segment {i+1}: {wp_start['name']} → {wp_end['name']}")
        print(f"{'='*70}")
        print(f"Path Type: {path_type}")
        print(f"  Arc 1 (θ={a1_deg:7.2f}°, R={turn_radius:.1f}m):  {L1:8.4f} m")
        print(f"  Straight (distance):         {L2:8.4f} m")
        print(f"  Arc 2 (θ={a2_deg:7.2f}°, R={turn_radius:.1f}m):  {L3:8.4f} m")
        print(f"  {'─'*47}")
        print(f"  Total Length (computed):     {total_seg_length:8.4f} m")
        print(f"  Cost (scaled):               {plan['cost']:8.4f}")

    # Combine all segments into one continuous path
    full_path = np.vstack(dubins_paths)

    # Plot the multi-waypoint path
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))

    # Left plot: Path with waypoints
    ax1.plot(full_path[:, 0], full_path[:, 1], 'b-', linewidth=2.5, label='Dubins Path')

    # Plot waypoints
    for i, wp in enumerate(waypoints):
        ax1.plot(wp['pos'][0], wp['pos'][1], 'ro', markersize=12, zorder=5)
        ax1.text(wp['pos'][0] + 1, wp['pos'][1] + 1, wp['name'], fontsize=11, fontweight='bold')
        
        # Draw heading arrows
        arrow_length = 3
        dx = arrow_length * np.cos(wp['yaw'])
        dy = arrow_length * np.sin(wp['yaw'])
        ax1.arrow(wp['pos'][0], wp['pos'][1], dx, dy, 
                head_width=1.5, head_length=1, fc='red', ec='red', alpha=0.7)

    # Plot turn circles for each segment
    for i, plan in enumerate(dubins_plans):
        circle1 = plt.Circle(plan['c0'], turn_radius, fill=False, linestyle='--', 
                            color='gray', alpha=0.3, linewidth=1)
        circle2 = plt.Circle(plan['c1'], turn_radius, fill=False, linestyle='--', 
                            color='gray', alpha=0.3, linewidth=1)
        ax1.add_patch(circle1)
        ax1.add_patch(circle2)

    ax1.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ax1.set_title('Multi-Waypoint Dubins Path (6 Waypoints)', fontsize=13, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    ax1.legend(fontsize=11, loc='best')

    # Right plot: Heading angle vs arc length
    arc_lengths = np.zeros(len(full_path))
    for i in range(1, len(full_path)):
        arc_lengths[i] = arc_lengths[i-1] + np.linalg.norm(full_path[i, :2] - full_path[i-1, :2])

    arc_lengths_norm = arc_lengths #/ arc_lengths[-1]

    ax2.plot(arc_lengths_norm, np.degrees(full_path[:, 2]), 'b-', linewidth=2.5)
    ax2.set_xlabel('Normalized Arc Length', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Heading (degrees)', fontsize=12, fontweight='bold')
    ax2.set_title('Heading Angle Along Path', fontsize=13, fontweight='bold')
    ax2.grid(True, alpha=0.3)

    # Mark waypoint transitions
    segment_boundaries = []
    current_pos = 0
    # for i, path in enumerate(dubins_paths[:-1]):
    #     current_pos += len(path) #/ len(full_path)
    #     ax2.axvline(current_pos, color='red', linestyle='--', linewidth=1.5, alpha=0.5)
    #     ax2.text(current_pos, ax2.get_ylim()[1] * 0.95, f'WP {i+1}', 
    #             rotation=90, fontsize=10, ha='right', color='red')

    plt.tight_layout()
    plt.show()




    segments = (len(waypoints)-1) * 3
    R = 8.0#/(42.7781*7.5)  # Turning radius

    boundary_positions = [[*([0, None, None]*len(waypoints)),0]]      # Position: start=0, end=
    boundary_velocities = [[0, *([None]*(segments-1)), 0]]       # Velocity: start=0, others free

    if segment_direction[0] == 'R':
        start_condition = 1/R
    elif segment_direction[0] == 'L':
        start_condition = -1/R
    else:
        assert -1 > 0 # "PANNIC"

    if segment_direction[-1] == 'R':
        end_condition = 1/R
    elif segment_direction[-1] == 'L':
        end_condition = -1/R
    else:
        assert -1 > 0 # "PANNIC"


    boundary_accelerations = [[start_condition, *([None]*(segments-1)), end_condition]]    # Acceleration: start=0

    boundary_jerk = [[None]*(segments+1)]             # Jerk: start=0
    boundary_snap =  [[None]*(segments+1)]              # Snap: start=0
    boundary_crackle = [[None]*(segments+1)]            # Snap: start=0
    boundary_pop = [[None]*(segments+1)]             # Snap: start=0
    boundary_lock = [[None]*(segments+1)]            # Snap: start=0

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

    continuity_changes = []
    i = 0
    for segment in segment_direction[0:-1]:

        current_segment = segment
        next_segment = segment_direction[i+1]

        value = None
        print(current_segment, next_segment)
        match current_segment:
            case 'R':
                if next_segment == 'R':
                    value = 0
                elif next_segment == 'L':
                    value = -2/R
                elif next_segment == 'S':
                    value = -1/R
            case 'L':
                if next_segment == 'R':
                    value = 2/R
                elif next_segment == 'L':
                    value = 0
                elif next_segment == 'S':
                    value = 1/R
            case 'S':
                if next_segment == 'R':
                    value = 1/R
                elif next_segment == 'L':
                    value = -1/R
                elif next_segment == 'S':
                    value = 0
            case _:
                assert -1 > 0
        print(value)
        if value != 0:
            continuity_changes.append([2 + i*5, value])  # Default to 0
        i += 1

    tau = np.abs(segment_L)  # Segment durations

    # Solve optimization problem
    outputs = run_poly_optimization(
        order=7,
        tau=tau,
        segments=segments,
        weights=[0.1, 0.1, 0.1, 0.1, 100000, 0.1, 0.1, 0.1],  # Minimize snap (4th derivative)
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