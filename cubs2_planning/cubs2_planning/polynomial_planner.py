import numpy as np
import matplotlib.pyplot as plt
from polynomial_optimization import run_poly_optimization, plot_piecewise_polynomials, plot_derivatives
from polynomial_optimization_clean import plot_xyz


# X boundary conditions

def plan_path(V, m, S, rho, g, CL0, CLa, CD0, k):

    # Path setup
    seg_distances = [20, 10 * np.pi * 2/4 *1.25, 10 * np.pi * 2/4 *1.25]  # Segment durations

    for j in range(1):
        x_boundary_positions = [[0, 0, 10, 20]]
        x_boundary_velocities = [[0, None, None, None]]
        x_boundary_accelerations = [[0, None, None, 0]]
        x_boundary_jerk = [[None, None, None, None]]
        x_boundary_snap = [[None, None, None, None]]
        x_boundary_crackle = [[None, None, None, None]]
        x_boundary_pop = [[None, None, None, None]]
        x_boundary_lock = [[None, None, None, None]]
        x_boundary_conditions = [
            x_boundary_positions,
            x_boundary_velocities,
            x_boundary_accelerations,
            x_boundary_jerk,
            x_boundary_snap,
            x_boundary_crackle,
            x_boundary_pop,
            x_boundary_lock
        ]

        # Y boundary conditions
        y_boundary_positions = [[0, 20, 30, 20]]
        y_boundary_velocities = [[V, None, None, None]]
        y_boundary_accelerations = [[None, None, None, 0]]
        y_boundary_jerk = [[None, None, None, None]]
        y_boundary_snap = [[None, None, None, None]]
        y_boundary_crackle = [[None, None, None, None]]
        y_boundary_pop = [[None, None, None, None]]
        y_boundary_lock = [[None, None, None, None]]
        y_boundary_conditions = [
            y_boundary_positions,
            y_boundary_velocities,
            y_boundary_accelerations,
            y_boundary_jerk,
            y_boundary_snap,
            y_boundary_crackle,
            y_boundary_pop,
            y_boundary_lock
        ]

        # Z boundary conditions
        z_boundary_positions = [[0, None, None, 0]]
        z_boundary_velocities = [[None, None, None, None]]
        z_boundary_accelerations = [[None, None, None, None]]
        z_boundary_jerk = [[None, None, None, None]]
        z_boundary_snap = [[None, None, None, None]]
        z_boundary_crackle = [[None, None, None, None]]
        z_boundary_pop = [[None, None, None, None]]
        z_boundary_lock = [[None, None, None, None]]
        z_boundary_conditions = [
            z_boundary_positions,
            z_boundary_velocities,
            z_boundary_accelerations,
            z_boundary_jerk,
            z_boundary_snap,
            z_boundary_crackle,
            z_boundary_pop,
            z_boundary_lock
        ]

        # Define acceleration discontinuities (jumps in 2nd derivative)
        continuity_changes = None
        tau = [seg_distance/V for seg_distance in seg_distances]

        # Solve optimization problem
        outputsx = run_poly_optimization(
            order=7,
            tau=tau,
            segments=3,
            weights=[0, 0, 0.1, 0.1, 1000, 0.1, 0.1, 0.1],  # Minimize snap (4th derivative)
            boundary_conditions=x_boundary_conditions,
            continuity=continuity_changes
        )

        outputsy = run_poly_optimization(
            order=7,
            tau=tau,
            segments=3,
            weights=[0, 0, 0.1, 0.1, 1000, 0.1, 0.1, 0.1],  # Minimize snap (4th derivative)
            boundary_conditions=y_boundary_conditions,
            continuity=continuity_changes
        )

        outputsz = run_poly_optimization(
            order=7,
            tau=tau,
            segments=3,
            weights=[0, 0, 0.1, 0.1, 1000, 0.1, 0.1, 0.1],  # Minimize snap (4th derivative)
            boundary_conditions=z_boundary_conditions,
            continuity=continuity_changes
        )

        # fig, axes, x, y,z, vx , vy , vz, ax, ay, az, jx, jy , jz, t= plot_xyz(outputsx['polys'],outputsy['polys'],outputsz['polys'], tau)

        # xyz = np.stack([x, y, z], axis=1)
        # sample_distances = np.linalg.norm(np.diff(xyz, axis=0), axis=1)

        # seg_boundaries = np.concatenate([[0], np.cumsum(tau)])
        # seg_distances = []
        # for t_start, t_end in zip(seg_boundaries[:-1], seg_boundaries[1:]):
        #     mask = (t[:-1] >= t_start) & (t[:-1] < t_end)
        #     seg_distances.append(sample_distances[mask].sum())

        # for i, d in enumerate(seg_distances):
        #     print(f"Segment {i+1} ({tau[i]:.2f}s): {d:.3f} m")
        # print(f"Total: {sum(seg_distances):.3f} m")


    fig, ax3d, x, y, z, vx, vy, vz, accx, accy, accz, jerkx, jerky, jerkz, t = plot_xyz(outputsx['polys'], outputsy['polys'], outputsz['polys'], tau)

    # V (x wind)
    V_analytic = np.sqrt(np.array(vx)**2 + np.array(vy)**2 + np.array(vz)**2)

    # V dot (x wind)
    V_dot = np.array(accx)*np.array(vx) + np.array(accy)*np.array(vy) + np.array(accz)*np.array(vz)
    V_dot = V_dot / V_analytic

    '''Angles Math'''
    # Chi (course angle)
    chi = np.arctan2(vy, vx)

    # Chi dot 
    vh2 = np.array(vx)**2 + np.array(vy)**2
    chi_dot = (np.array(vx) * np.array(accy) - np.array(vy) * np.array(accx)) / vh2

    # Chi dot dot
    chi_dot_dot = (accx * accy + vx * jerky - accx * accy - vy * jerkx) * (vx**2 + vy**2)- (vx * accy - vy * accx) * (2 * vx * accx + 2 * vy * accy)
    chi_dot_dot = chi_dot_dot / (vx**2 + vy**2)**2

    # Mu
    mu = np.arctan(V_analytic * chi_dot / g)

    # Mu dot
    mu_dot = 1/((V_analytic * chi_dot / g)**2 + 1) * (V_analytic * chi_dot_dot / g + V_dot * chi_dot / g)


    # Alpha (needs iterated)
    alpha0 = ((m*g / np.cos(mu)) / (1/2 * rho * V_analytic**2 * S) - CL0) / CLa

    # Alpha dot
    alpha_dot = np.gradient(alpha0, t)


    '''Forces'''
    # X force
    X_force = V_dot * m
    Y_force = m*g * np.sin(mu)
    Z_force = -m*g * np.sin(mu) * np.tan(mu)
    



    # Thrust
    Cl = CLa * alpha0 + CL0
    Cd = CD0 + k * Cl**2
    T0 = (X_force + 1/2 * rho * V_analytic**2 * S * Cd)/np.cos(alpha0)

    # omega
    q = alpha_dot - Z_force/m/V_analytic
    r = Y_force * np.cos(alpha0) / m / V_analytic + mu_dot * np.sin(alpha0)
    p = mu_dot / np.cos(alpha0) - r * np.tan(alpha0)

    # Zero since coordinated and no climb
    beta = np.zeros_like(alpha0)
    gamma = np.zeros_like(alpha0)


    return t, x, y ,z, V_analytic, chi, mu, gamma, alpha0, beta, p, q, r, T0
