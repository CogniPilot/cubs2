import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from polynomial_planner import plan_path
from cyecca.lie import group_se23, SO3Dcm
SE23DCM = group_se23.SE23LieGroup(SO3 = SO3Dcm)


def dynamics(x_vec, omega, T):
    x = x_vec[0]
    y = x_vec[1]
    z = x_vec[2]
    V = x_vec[3]
    alpha = x_vec[4]
    beta = x_vec[5]
    chi = x_vec[6]
    gamma = x_vec[7]
    mu = x_vec[8]

    p = omega[0]
    q = omega[1]
    r = omega[2]

    # parameters
    thr_max = 0.3
    m = 0.065
    S = 0.055
    rho = 1.225
    g = 9.81
    CL0 = 0.5
    CLa = 4.7
    CD0 = 0.06
    k = 0.09
    CYb = -0.50

    # Forces
    CL = CL0 + CLa * alpha
    CD = CD0 + k * CL**2
    CY = CYb * beta

    X = 1/2 * rho * V**2 * S * (-CD) + T * ca.cos(alpha) * ca.cos(beta) - m*g * ca.sin(gamma)
    Y = 1/2 * rho * V**2 * S * CY    + T * ca.cos(alpha) * ca.sin(beta) + m*g * ca.cos(gamma) * ca.sin(mu)
    Z = 1/2 * rho * V**2 * S * (-CL) - T * ca.sin(alpha) +  m*g * ca.cos(gamma) * ca.cos(mu)

    print("Forces")
    print(X, Y, Z)

    xdot = V * ca.cos(gamma) * ca.cos(chi)
    ydot = V * ca.cos(gamma) * ca.sin(chi)
    zdot = -V * ca.sin(gamma)
    Vdot = X / m
    alphadot = (-p * ca.cos(alpha) * ca.cos(beta) + q * ca.cos(beta) - r * ca.sin(alpha) * ca.sin(beta) + Z / (m * V))/ca.cos(beta)
    betadot = p * ca.sin(alpha) - r * ca.cos(alpha) + Y / (m * V)
    gammadot =  -Y * ca.sin(mu) / (m * V) - Z * ca.cos(mu) / (m * V)
    chidot =  (Y * ca.cos(mu) - Z * ca.sin(mu)) / (m * V * ca.cos(gamma))
    mudot = (p * ca.cos(alpha) + r * ca.sin(alpha) + Y * ca.cos(mu) * ca.tan(gamma) * ca.cos(beta) / (m * V) - Z * (ca.sin(mu) * ca.tan(gamma) * ca.cos(beta) + ca.sin(beta)) / (m * V))/ca.cos(beta)

    return ca.vertcat(xdot, ydot, zdot, Vdot, alphadot, betadot, chidot, gammadot, mudot)


# ── Numeric wrapper (compile once at import) ───────────────────────────────────
_x_sym     = ca.SX.sym('x', 9)
_omega_sym = ca.SX.sym('omega', 3)
_T_sym     = ca.SX.sym('T')
_f = ca.Function('f', [_x_sym, _omega_sym, _T_sym],
                      [dynamics(_x_sym, _omega_sym, _T_sym)])

def _f_num(x, omega, T):
    return np.array(_f(x, omega, float(T))).flatten()

def _rk4(x, omega, T, dt):
    k1 = _f_num(x,               omega, T)
    k2 = _f_num(x + dt/2 * k1,   omega, T)
    k3 = _f_num(x + dt/2 * k2,   omega, T)
    k4 = _f_num(x + dt    * k3,  omega, T)
    return x + dt / 6 * (k1 + 2*k2 + 2*k3 + k4)


def convert_x_to_group(x_vec):
    x = x_vec[0]
    y = x_vec[1]
    z = x_vec[2]
    V = x_vec[3]
    alpha = x_vec[4]
    beta = x_vec[5]
    chi = x_vec[6]
    gamma = x_vec[7]
    mu = x_vec[8]

    Rbw = R(alpha, axis=2) @ R(-beta, axis=3) @ R(mu, axis=1) @ R(-gamma, axis=2) @ R(-chi, axis=3)
        
        # Takes stuff from body to world frame
    Rwb = Rbw.T

    v_body = R(alpha, axis=2) @ R(-beta, axis=3) @np.array([V, 0, 0])
    v_world = Rwb @ v_body    


        
    # X = np.eye(5)
    # X[:3, :3] = Rwb
    # X[:3, 3] = v_world
    # X[:3, 4] = np.array([x, y, z])
    X_ca = SE23DCM.elem(ca.vertcat(x, y, z, v_world, Rwb.flatten()))

    return X_ca

def calculate_control(X_bar, X_group, omega_bar, T_bar, omega_tilde_prev, T_tilde_prev, Kp, Kv, Kr):
    eta_i = X_bar.inverse()*X_group
    xi = eta_i.log()  # log map to Lie algebra
    xi_param = xi.param

    Jlinv = xi.left_jacobian_inv()

    c1 = Jlinv[:3,:3]
    c2 = Jlinv[:3,3:6]
    c3 = Jlinv[:3,6:9]
    c4 = Jlinv[3:6,6:9]


    xi_p = xi_param[0:3]  # position error, expressed in reference frame
    xi_v = xi_param[3:6]  # velocity error, expressed in reference frame
    xi_R = xi_param[6:9]  # attitude error (so(3) coordinates), rotates from body (a) to reference (b) v_b = R_ba v_a

    R_ba = eta_i.R
    R_ab = R_ba.inverse()

    xi_v_des_dot = ca.vertcat(
        0, 0, 0
    )  # desired acceleration feedforward (simplified)
    xi_R_des_dot = ca.vertcat(
        0, 0, 0
    )  # desired attitude feedforward (simplified)

    xi_v_des = -c3 @ omega_tilde_prev - (c2/m) @ T_tilde_prev - Kp @ xi_p
    e_v = xi_v - xi_v_des

    a_des = (  # desired acceleration, expressed in reference frame
                    - Kv @ e_v  # velocity feedback, expressed in reference frame
                    - xi_v_des_dot  # desired acceleration feedforward, expressed in reference frame
                    - ca.cross(
                        omega_bar, xi_v_des
                    )
                    - c3 @ omega_tilde_prev
                )

    T_tilde = ca.dot(a_des, ca.vertcat(1,0,0))/(c1/m)[0,0] 


    Tau = -T_bar[0]/m
    xi_rd_x = 0
    xi_rd_y = (ca.dot(a_des, ca.vertcat(0,0,1)) - T_tilde * (c1/m)[2,0]) / Tau
    xi_rd_z = (ca.dot(a_des, ca.vertcat(0,1,0)) - T_tilde * (c1/m)[1,0]) / (-Tau)
    xi_R_des = ca.vertcat(xi_rd_x, xi_rd_y, xi_rd_z)

    e_R = xi_R - xi_R_des

    omega_tilde = ca.inv(c1)@(ca.cross(omega_bar, xi_R) + xi_R_des_dot - Kr @ e_R)

    T = T_bar + ca.vertcat(T_tilde,0,0)
    omega = omega_bar + omega_tilde
    omega_tilde_prev =omega_tilde
    T_tilde_prev = ca.vertcat(T_tilde, 0, 0)

    return T, omega, omega_tilde_prev, T_tilde_prev


def simulate(t_vec, omega_vec, T_vec, x0, x_ref, lie_control=False):
    """
    Integrate aircraft dynamics with RK4 (zero-order hold on inputs).

    Parameters
    ----------
    t_vec    : (N,)   time vector [s]
    omega_vec: (N, 3) body rates [p, q, r] [rad/s]
    T_vec    : (N,)   thrust [N]
    x0       : (9,)   initial state [x, y, z, V, alpha, beta, chi, gamma, mu]

    Returns
    -------
    t, x, y, z, chi, mu, gamma, alpha, beta  — each a (N,) numpy array
    """
    t_vec     = np.asarray(t_vec,     dtype=float)
    omega_vec = np.asarray(omega_vec, dtype=float)
    T_vec     = np.asarray(T_vec,     dtype=float)
    x0        = np.asarray(x0,        dtype=float)

    N = len(t_vec)
    X = np.zeros((N, 9))
    X[0] = x0


    omega_tilde = np.zeros(3)
    T_tilde = np.zeros(3)

    Ts = []
    omegas = []
    T_tildes = []
    omega_tildes = []

    for i in range(N - 1):
        dt = t_vec[i + 1] - t_vec[i]


        if lie_control:
            X_bar = convert_x_to_group(x_ref[i])
            X_group = convert_x_to_group(X[i])

            T, omega, omega_tilde, T_tilde = calculate_control(X_bar, X_group, omega_vec[i], np.array([T_vec[i], 0, 0]), omega_tilde, T_tilde, Kp=np.eye(3)*0.1, Kv=np.eye(3)*1.5, Kr=np.eye(3)*0.1) 

            T = np.array(ca.DM(T)).flatten()
            omega = np.array(ca.DM(omega)).flatten()
            omega_tilde = np.array(ca.DM(omega_tilde)).flatten()
            T_tilde = np.array(ca.DM(T_tilde)).flatten()

            Ts.append(T[0])
            omegas.append(omega)
            omega_tildes.append(omega_tilde)
            T_tildes.append(T_tilde)
        else:
            T = [T_vec[i]]
            omega = omega_vec[i]
    
        T = max(min(T[0], 0.3),0)
        omega = np.clip(omega, -1, 1)

        X[i + 1] = _rk4(X[i], omega, T, dt)

    # state indices: x y z V alpha beta chi gamma mu
    #                0 1 2 3   4     5    6    7    8
    return (
        t_vec,
        X[:, 0],  # x
        X[:, 1],  # y
        X[:, 2],  # z
        X[:, 3],  # V
        X[:, 6],  # chi
        X[:, 8],  # mu
        X[:, 7],  # gamma
        X[:, 4],  # alpha
        X[:, 5],  # beta
        Ts,
        omegas,
        T_tildes,
        omega_tildes
    )

def R(angle, axis):
    c, s = np.cos(angle), np.sin(angle)
    if axis == 1:  # x
        return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    elif axis == 2:  # y
        return np.array([[c,0,s],[0,1,0],[-s,0,c]])
    elif axis == 3:  # z
        return np.array([[c,-s,0],[s,c,0],[0,0,1]])
    else:
        raise ValueError("Axis must be 1 (x), 2 (y), or 3 (z)")

if __name__ == '__main__':


    thr_max = 0.3
    m = 0.065
    S = 0.055
    rho = 1.225
    g = 9.81
    CL0 = 0.5
    CLa = 4.7
    CD0 = 0.06
    k = 0.09
    CYb = -0.50
    t, x, y, z, V_val, chi, mu, gamma, alpha0, beta, p, q, r, T0 = plan_path(8, m, S, rho, g, CL0, CLa, CD0, k)

    x_ref = np.array([x, y, z, V_val, alpha0, beta, chi, gamma, mu]).T

    Rs = []
    v_worlds = []

# 
    omega = np.stack([p, q, r], axis=1)
    T = T0
    x0 = [x[0], y[0], z[0], V_val[0], alpha0[0], beta[0], chi[0], gamma[0], mu[0]]

    # ── Run ───────────────────────────────────────────────────────────────────
    t_s, x_s, y_s, z_s, V_s, chi_s, mu_s, gamma_s, alpha_s, beta_s, Ts, omegas, T_tildes, omega_tildes = simulate(t, omega, T, x0, x_ref, lie_control=True)
    t_s_nocontrol, x_s_nocontrol, y_s_nocontrol, z_s_nocontrol, V_s_nocontrol, chi_s_nocontrol, mu_s_nocontrol, gamma_s_nocontrol, alpha_s_nocontrol, beta_s_nocontrol, Ts_nocontrol, omegas_nocontrol, T_tildes_nocontrol, omega_tildes_nocontrol = simulate(t, omega, T, x0, x_ref, lie_control=False)


    # print(Ts[:][0])
    fig, axes = plt.subplots(2, 1) 
    print(t_s[1:])
    axes[0].plot(t_s[1:], Ts)
    axes[0].plot(t_s[1:], T_tildes)
    axes[0].set_ylim([-0.1,0.3])

    axes[1].plot(t_s[1:], omegas)
    axes[1].plot(t_s[1:], omega_tildes)
    axes[1].set_ylim([-1,1])
    plt.show()

    for a_loop, b_loop, chi_loop, gamma_loop, mu_loop, V_loop in zip(alpha_s, beta_s, chi_s, gamma_s, mu_s, V_s):    
        # Takes stuff from world to body frame
        Rbw = R(a_loop, axis=2) @ R(-b_loop, axis=3) @ R(mu_loop, axis=1) @ R(-gamma_loop, axis=2) @ R(-chi_loop, axis=3)
        
        # Takes stuff from body to world frame
        Rwb = Rbw.T
        Rs.append(Rwb)

        v_body = R(a_loop, axis=2) @ R(-b_loop, axis=3) @np.array([V_loop, 0, 0])
        v_world = Rwb @ v_body
        v_worlds.append(v_world)

    # plt.plot(x_s, y_s, z_s)
    # plt.show()
    # print("Final position error (m):", np.linalg.norm([x_s[-1] - x[-1], y_s[-1] - y[-1], z_s[-1] - z[-1]]))

    # ── Plot ──────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 9))
    gs  = plt.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    ax3 = fig.add_subplot(gs[:, 0], projection='3d')
    ax3.plot(x_s, y_s, z_s, 'b-', linewidth=2)
    ax3.plot(x_s_nocontrol, y_s_nocontrol, z_s_nocontrol, 'gray', linestyle='--', linewidth=1)
    ax3.plot(x, y, z, 'r--', linewidth=2)
    ax3.scatter(x_s[0],  y_s[0],  z_s[0],  color='g', s=60, zorder=5, label='start')
    ax3.scatter(x_s[-1], y_s[-1], z_s[-1], color='r', s=60, zorder=5, label='end')
    half = max(np.ptp(x_s), np.ptp(y_s), max(np.ptp(z_s), 1.0)) / 2
    for set_lim, mid in zip([ax3.set_xlim, ax3.set_ylim, ax3.set_zlim],
                            [(x_s.max()+x_s.min())/2,
                             (y_s.max()+y_s.min())/2,
                             (z_s.max()+z_s.min())/2]):
        set_lim(mid - half, mid + half)
    ax3.set_xlabel('X (m)'); ax3.set_ylabel('Y (m)'); ax3.set_zlabel('Z (m)')
    ax3.set_title('3D Trajectory'); ax3.legend(fontsize=8)

    ax1 = fig.add_subplot(gs[0, 1])
    ax1.plot(t_s, np.degrees(chi_s),   label='χ chi')
    ax1.plot(t, np.degrees(chi), label='χ chi (analytic)')
    ax1.plot(t_s, np.degrees(mu_s),    label='μ mu')
    ax1.plot(t, np.degrees(mu), label='μ mu (analytic)')
    ax1.plot(t_s, np.degrees(gamma_s), label='γ gamma')
    ax1.plot(t, np.degrees(gamma), label='γ gamma (analytic)')
    ax1.set_ylabel('deg'); ax1.set_title('Course / Bank / Climb')
    ax1.legend(fontsize=8); ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs[1, 1])
    ax2.plot(t_s, np.degrees(alpha_s), label='α alpha')
    ax2.plot(t_s, np.degrees(beta_s),  label='β beta')
    ax2.plot(t, np.degrees(alpha0), label='α alpha (analytic)')
    ax2.set_ylabel('deg'); ax2.set_title('Aero Angles')
    ax2.legend(fontsize=8); ax2.grid(True, alpha=0.3)

    ax4 = fig.add_subplot(gs[2, 1])
    ax4.plot(t_s, V_s, 'purple')
    ax4.plot(t, V_val, 'purple', linestyle='--')
    ax4.set_xlabel('time (s)'); ax4.set_ylabel('m')
    ax4.set_title('Velocity'); ax4.grid(True, alpha=0.3)

    fig.suptitle('Simulator: trim flight + coordinated turn (t=3–9s)', fontsize=12)
    # plt.tight_layout()
    # plt.show()

    from matplotlib.animation import FuncAnimation
    import matplotlib.gridspec as gridspec
    from IPython.display import HTML
    import matplotlib as mpl
    mpl.rcParams['animation.embed_limit'] = 50  # MB

    # ── Rotation matrices (ENU, z-up, right-hand) ─────────────────────────────────
    def Rx(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[1,0,0],[0,c,-s],[0,s,c]])

    def Ry(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c,0,s],[0,1,0],[-s,0,c]])

    def Rz(a):
        c, s = np.cos(a), np.sin(a)
        return np.array([[c,-s,0],[s,c,0],[0,0,1]])

    def body_frame_axes(chi_v, gamma_v, mu_v, alpha_v, beta_v):
        R_iw = Rz(chi_v) @ Ry(-gamma_v) @ Rx(mu_v)
        R_wb = Ry(alpha_v) @ Rz(-beta_v)
        return R_iw @ R_wb

    def aircraft_segments(pos, R_ib, sc):
        xb, yb, zb = R_ib[:,0], R_ib[:,1], R_ib[:,2]
        nose = pos + sc * xb
        tail = pos - sc * 0.7 * xb
        wc   = pos - sc * 0.05 * xb
        return [
            (tail,             nose,             'k',         2.5),
            (wc - sc*0.85*yb,  wc + sc*0.85*yb,  'steelblue', 2.0),
            (tail - sc*0.3*yb, tail + sc*0.3*yb,  'steelblue', 1.5),
            (tail,             tail + sc*0.35*zb,  'crimson',   1.5),
        ]

    # ── Arrays from plan_path ──────────────────────────────────────────────────────
    t_arr   = np.array(t)
    x_arr, y_arr, z_arr = np.array(x_s), np.array(y_s), np.array(z_s)
    chi_arr, mu_arr, gam_arr = np.array(chi_s), -np.array(mu_s), np.array(gamma_s)
    alp_arr, bet_arr         = np.array(alpha_s), np.array(beta_s)

    step = max(1, len(t_arr) // 80)
    sl = slice(None, None, step)
    t_a, x_a, y_a, z_a = t_arr[sl], x_arr[sl], y_arr[sl], z_arr[sl]
    chi_a, mu_a, gam_a  = chi_arr[sl], mu_arr[sl], gam_arr[sl]
    alp_a, bet_a        = alp_arr[sl], bet_arr[sl]

    sc_traj = max(np.ptp(x_arr), np.ptp(y_arr), max(np.ptp(z_arr), 0.1)) * 0.1
    sc_ac   = 1.5

    # ── Figure layout ──────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 7))
    gs  = gridspec.GridSpec(2, 2, figure=fig, width_ratios=[1.2, 1], hspace=0.38, wspace=0.32)

    ax_traj = fig.add_subplot(gs[:, 0], projection='3d')
    ax_ac   = fig.add_subplot(gs[0, 1], projection='3d')
    ax_ang  = fig.add_subplot(gs[1, 1])

    ax_traj.plot(x_arr, y_arr, z_arr, 'b-', alpha=0.25, linewidth=1.5)
    ax_traj.plot(x, y, z, 'r--', alpha=0.25, linewidth=1)
    ax_traj.set_xlabel('X (m)'); ax_traj.set_ylabel('Y (m)'); ax_traj.set_zlabel('Z (m)')
    ax_traj.set_title('Trajectory')

    # Equal axis ranges so the aircraft shape isn't distorted
    half = max(np.ptp(x_arr), np.ptp(y_arr), max(np.ptp(z_arr), 1.0)) / 2
    mid_x = (np.max(x_arr) + np.min(x_arr)) / 2
    mid_y = (np.max(y_arr) + np.min(y_arr)) / 2
    mid_z = (np.max(z_arr) + np.min(z_arr)) / 2
    ax_traj.set_xlim(mid_x - half, mid_x + half)
    ax_traj.set_ylim(mid_y - half, mid_y + half)
    ax_traj.set_zlim(mid_z - half, mid_z + half)

    pos_dot, = ax_traj.plot([], [], [], 'ro', markersize=8, zorder=5)
    traj_lines = [ax_traj.plot([], [], [], color=c, linewidth=w)[0]
                for (_, _, c, w) in aircraft_segments(np.zeros(3), np.eye(3), sc_traj)]

    lim = sc_ac * 1.4
    ax_ac.set_xlim(-lim, lim); ax_ac.set_ylim(-lim, lim); ax_ac.set_zlim(-lim, lim)
    ax_ac.set_xlabel('X'); ax_ac.set_ylabel('Y'); ax_ac.set_zlabel('Z')
    ax_ac.set_title('Body Frame Orientation')
    for v, c, lbl in [(np.array([1,0,0]),'r','N'),(np.array([0,1,0]),'g','E'),(np.array([0,0,1]),'b','Up')]:
        ax_ac.quiver(0,0,0, *(v*lim*0.55), color=c, alpha=0.2, arrow_length_ratio=0.18)
        ax_ac.text(*(v*lim*0.65), lbl, color=c, alpha=0.4, fontsize=8)

    ac_lines = [ax_ac.plot([], [], [], color=c, linewidth=w)[0]
                for (_, _, c, w) in aircraft_segments(np.zeros(3), np.eye(3), sc_ac)]

    ax_ang.set_xlim(t_a[0], t_a[-1])
    ax_ang.set_ylim(-190, 190)
    ax_ang.set_xlabel('Time (s)'); ax_ang.set_ylabel('deg')
    ax_ang.set_title('Orientation History')
    ax_ang.grid(True, alpha=0.3)
    l_chi,  = ax_ang.plot([], [], 'b-', lw=2, label='χ  chi')
    l_mu,   = ax_ang.plot([], [], 'r-', lw=2, label='μ  mu')
    l_gam,  = ax_ang.plot([], [], 'g-', lw=2, label='γ  gamma')
    cursor  = ax_ang.axvline(t_a[0], color='k', ls='--', lw=1.2)
    ax_ang.legend(fontsize=9, loc='upper right')

    # ── Animation ─────────────────────────────────────────────────────────────────
    def _seg(ln, p1, p2):
        ln.set_data([p1[0], p2[0]], [p1[1], p2[1]])
        ln.set_3d_properties([p1[2], p2[2]])

    def update(i):
        pos = np.array([x_a[i], y_a[i], z_a[i]])
        Rib = body_frame_axes(chi_a[i], gam_a[i], mu_a[i], alp_a[i], bet_a[i])

        pos_dot.set_data([pos[0]], [pos[1]])
        pos_dot.set_3d_properties([pos[2]])
        for ln, (p1, p2, _, _) in zip(traj_lines, aircraft_segments(pos, Rib, sc_traj)):
            _seg(ln, p1, p2)
        for ln, (p1, p2, _, _) in zip(ac_lines, aircraft_segments(np.zeros(3), Rib, sc_ac)):
            _seg(ln, p1, p2)

        l_chi.set_data(t_a[:i+1], np.degrees(chi_a[:i+1]))
        l_mu.set_data(t_a[:i+1], np.degrees(mu_a[:i+1]))
        l_gam.set_data(t_a[:i+1], np.degrees(gam_a[:i+1]))
        cursor.set_xdata([t_a[i], t_a[i]])

        fig.suptitle(
            f't = {t_a[i]:.2f}s  |  '
            f'χ = {np.degrees(chi_a[i]):.1f}°  '
            f'μ = {np.degrees(mu_a[i]):.1f}°  '
            f'γ = {np.degrees(gam_a[i]):.1f}°  '
            f'α = {np.degrees(alp_a[i]):.1f}°',
            fontsize=11
        )

    anim = FuncAnimation(fig, update, frames=len(t_a), interval=50)
    plt.show()


    fig3, ax = plt.subplots(figsize=(14, 9))

    vx_w = [v[0] for v in v_worlds]
    vy_w = [v[1] for v in v_worlds]
    vz_w = [v[2] for v in v_worlds]

    vx_w_n = [max(min(val, 10), -10) for val in np.gradient(x_s, t_s)]
    vy_w_n = [max(min(val, 10), -10) for val in np.gradient(y_s, t_s)]
    vz_w_n = [max(min(val, 10), -10) for val in np.gradient(z_s, t_s)]

    ax.plot(t_s, vx_w, label='vx world (from Rwb)')
    ax.plot(t_s, vy_w, label='vy world (from Rwb)')
    ax.plot(t_s, vz_w, label='vz world (from Rwb)')
    ax.plot(t_s, vx_w_n, "--", label='vx world (numerical)')
    ax.plot(t_s, vy_w_n, "--", label='vy world (numerical)')
    ax.plot(t_s, vz_w_n, "--", label='vz world (numerical)')
    ax.legend()
    plt.show()





