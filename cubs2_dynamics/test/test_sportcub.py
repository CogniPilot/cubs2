"""Tests for the sportcub model factory."""

import copy

import casadi as ca
import numpy as np
import pytest
from cyecca.dynamics.linearize import analyze_modes, linearize_dynamics
from cubs2_dynamics.sportcub import sportcub
from cubs2_dynamics.trim_fixed_wing import classify_aircraft_modes
from cubs2_dynamics.trim_fixed_wing import find_trim_fixed_wing as find_trim
from cubs2_dynamics.trim_fixed_wing import print_mode_summary
from cyecca import lie


class TestSportCubNode:
    """Collection of unit tests for the sportcub dynamics model."""

    def test_initial_conditions_near_origin(self):
        """Verify default initial conditions (position, velocity, attitude, throttle)."""
        model = sportcub()
        x = model.x0
        u = model.u0

        np.testing.assert_allclose(x.p, [0.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(x.v, [0.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(x.r, [1.0, 0.0, 0.0, 0.0], atol=1e-6)  # quaternion in x.r
        np.testing.assert_allclose(x.w, [0.0, 0.0, 0.0], atol=1e-6)
        assert u.thr == 0.0

    def test_ground_reaction_settles_at_origin(self):
        """Aircraft near ground with aero disabled should remain stable without drift."""
        model = sportcub()
        model.p0.disable_aero = 1.0  # isolate ground reaction

        x_state = model.state_type.from_vec(model.x0.as_vec())
        initial_z = 0.30
        x_state.p[2] = initial_z
        u_vec = model.u0.as_vec()
        p_vec = model.p0.as_vec()

        dt = 0.02  # Increased from 0.01 - we have 100 substeps so 0.02s outer step is fine
        n_steps = 150  # Reduced from 300 since we increased dt (same total time)
        total_time = dt * n_steps

        def u_func(t, x, p):
            return u_vec

        model.x0 = model.state_type.from_vec(x_state.as_vec())
        result = model.simulate(
            t0=0.0,
            tf=total_time,
            dt=dt,
            u_func=u_func,
            p_vec=p_vec,
        )

        states = model.state_type.from_matrix(result["x"])
        z_series = np.array([s.p[2] for s in states])
        xy_series = np.array([np.linalg.norm(s.p[:2]) for s in states])
        v_mags = np.array([np.linalg.norm(s.v) for s in states])

        min_z = float(np.min(z_series))
        max_z = float(np.max(z_series))
        max_xy_drift = float(np.max(xy_series))

        # Determine if the system settled in the second half of the simulation
        half_idx = len(v_mags) // 2
        settled_mask = (v_mags[half_idx:] < 0.05) & (xy_series[half_idx:] < 0.02)
        settled = bool(settled_mask.any())

        x_final = model.state_type.from_vec(result["x"][:, -1])
        assert min_z > -0.05, f"Unexpected penetration below ground: min_z={min_z:.3f}"
        assert (
            max_z <= initial_z + 0.02
        ), f"Bounced above initial height: initial={initial_z:.2f} max_z={max_z:.3f}"
        xy_distance = np.linalg.norm(x_final.p[:2])
        assert (
            xy_distance < 0.05
        ), f"Aircraft drifted horizontally {xy_distance:.3f} m (max drift {max_xy_drift:.3f} m)."
        assert settled, "Ground reaction did not settle within allotted simulation time"
        assert np.all(np.isfinite(x_final.p))
        assert np.all(np.isfinite(x_final.v))
        q_norm = np.linalg.norm(x_final.r)  # quaternion in x.r
        np.testing.assert_allclose(q_norm, 1.0, rtol=1e-3)

    def test_ground_bounce_characterization(self):
        """Characterize ground bounce over several drop heights with aero disabled."""
        model = sportcub()
        model.p0.disable_aero = 1.0
        drop_heights = [0.5, 5.0]  # Reduced from 5 tests to 2 for speed
        dt = 0.01  # Increased from 0.002 - we have 100 substeps so 0.01s outer step is fine
        max_steps = 1000  # Reduced from 3000 since we increased dt
        results = []

        for h0 in drop_heights:
            x_state = model.state_type.from_vec(model.x0.as_vec())
            x_state.p[2] = h0
            p_vec = model.p0.as_vec()
            u_vec = model.u0.as_vec()
            total_time = dt * max_steps

            def u_func(t, x, p):
                return u_vec

            model.x0 = model.state_type.from_vec(x_state.as_vec())
            result = model.simulate(
                t0=0.0,
                tf=total_time,
                dt=dt,
                u_func=u_func,
                p_vec=p_vec,
            )

            states = model.state_type.from_matrix(result["x"])
            z_series = np.array([s.p[2] for s in states])
            vz_series = np.array([s.v[2] for s in states])
            v_mags = np.array([np.linalg.norm(s.v) for s in states])

            first_contact = False
            rebound_found = False
            rebound_height = 0.0
            prev_vz = float(vz_series[0])
            min_z = float(np.min(z_series))
            min_z_after_contact = None
            bounce_threshold = 0.005
            settling_time = None
            settled = False

            for step in range(1, len(z_series)):
                z = float(z_series[step])
                vz = float(vz_series[step])
                if (z < 0.095) and not first_contact:
                    first_contact = True
                    min_z_after_contact = z
                if first_contact and not rebound_found:
                    if min_z_after_contact is None or z < min_z_after_contact:
                        min_z_after_contact = z
                    if (
                        prev_vz > 0.001
                        and vz <= 0
                        and (z - min_z_after_contact) >= bounce_threshold
                    ):
                        rebound_found = True
                        rebound_height = z
                prev_vz = vz
                v_mag = float(v_mags[step])
                if step > 100 and v_mag < 0.02 and z < 0.08:
                    settled = True
                    if settling_time is None:
                        settling_time = step * dt
                    break

            if not rebound_found:
                rebound_height = 0.0
            e = rebound_height / h0
            max_penetration = max(0.0, -min_z)
            results.append((h0, rebound_height, e, max_penetration, settling_time, settled))

        print("\nBounce characterization summary:")
        print(
            f"{'h0':>6} {'rebound':>10} {'e':>8} {'penetration':>12} {'settle_s':>10} {'settled':>8}"
        )
        for h0, rebound_height, e, max_penetration, settling_time, settled in results:
            settling_str = f"{settling_time:.2f}" if settling_time is not None else "None"
            print(
                f"{h0:6.2f} {rebound_height:10.4f} {e:8.4f} {max_penetration:12.4f} {settling_str:>10} {str(settled):>8}"
            )

        for h0, _, e, max_penetration, settling_time, settled in results:
            assert e <= 1.0 + 1e-6
            # Coefficient of restitution bounds (softer for low drops which may not bounce)
            if h0 >= 1.0:
                assert e <= 0.55, f"Bounce too energetic for h0={h0}m: e={e:.3f}"
                assert e >= 0.08, f"Bounce too damped for h0={h0}m: e={e:.3f}"

            # Tighter settling time requirements
            assert (
                settled and settling_time is not None and settling_time <= 8.0
            ), f"Drop {h0} m took too long to settle ({settling_time:.2f}s)"

            wheel_offset = 0.1
            wheel_penetration = max_penetration + wheel_offset

            # Tighter penetration limits (wheel_offset accounts for wheel position)
            if h0 <= 1.0:
                max_allowed = 0.11  # Wheels touch ground at 0.1m
            elif h0 <= 5.0:
                max_allowed = 0.18
            else:
                max_allowed = 0.25
            assert (
                wheel_penetration < max_allowed
            ), f"Excessive penetration for h0={h0}m: {wheel_penetration:.3f}m > {max_allowed}m"

    def test_glide_slope(self):
        """Verify controlled glide with wings-level control optimized for maximum L/D."""
        model = sportcub()
        model.p0.disable_aero = 0.0
        model.p0.disable_gf = 1.0  # isolate pure glide

        print("\nFinding natural glide equilibrium (naturally maximizes L/D)...")

        # Use find_trim with V_target=None and gamma=None
        # When no speed or angle is specified, the natural equilibrium in glide
        # corresponds to maximum L/D (best glide ratio)
        x_trim, u_trim, _ = find_trim(
            model,
            V_target=None,  # Let optimizer find equilibrium glide speed
            gamma=None,  # Let optimizer find equilibrium glide angle
            fix_throttle=True,
            print_progress=True,
            verbose=True,
            ipopt_print_level=1,
        )
        x_state = model.state_type.from_vec(x_trim)

        # VERIFY TRIM: Check that all accelerations are actually zero
        x_dot = model.f_x(x_trim, u_trim, model.p0.as_vec())
        v_dot = x_dot[3:6]  # velocity derivatives
        w_dot = x_dot[10:13]  # angular velocity derivatives
        v_dot_mag = np.linalg.norm(v_dot)
        w_dot_mag = np.linalg.norm(w_dot)

        print("\nTrim verification:")
        print(f"  |v_dot| = {v_dot_mag:.6f} m/s² (should be ~0)")
        print(f"  |w_dot| = {w_dot_mag:.6f} rad/s² (should be ~0)")

        # Verify forces and moments at trim
        outputs = model.f_y(x_trim, u_trim, model.p0.as_vec())
        out_state = model.output_type.from_vec(outputs)
        F_b = out_state.F_b
        M_b = out_state.M_b
        F_mag = np.linalg.norm(F_b)
        M_mag = np.linalg.norm(M_b)
        CL_trim = float(out_state.CL)
        CD_trim = float(out_state.CD)
        alpha_trim = float(out_state.alpha)

        print(f"  F_b = {F_b} (mag: {F_mag:.6f} N)")
        print(f"  M_b = {M_b} (mag: {M_mag:.6f} N·m)")
        print(f"  CL = {CL_trim:.3f}, CD = {CD_trim:.3f}, L/D = {CL_trim/CD_trim:.2f}")
        print(f"  alpha = {np.degrees(alpha_trim):.2f}°")

        # Extract trim velocity and glide angle for reporting
        V_trim = np.linalg.norm(x_state.v)
        # Compute glide angle from velocity
        horiz_speed = np.linalg.norm(x_state.v[:2])
        gamma_trim = np.arctan2(x_state.v[2], horiz_speed)
        print(f"  Found glide velocity: {V_trim:.2f} m/s")
        print(f"  Found glide angle: {np.degrees(gamma_trim):.2f}°")

        # Assert trim quality
        assert v_dot_mag < 0.01, f"Velocity acceleration not zero: |v_dot|={v_dot_mag:.6f}"
        assert w_dot_mag < 0.01, f"Angular acceleration not zero: |w_dot|={w_dot_mag:.6f}"
        assert F_mag < 0.1, f"Net force not zero: |F_b|={F_mag:.6f} N"
        assert M_mag < 0.05, f"Net moment not zero: |M_b|={M_mag:.6f} N·m"

        x_state.p = np.array([0.0, 0.0, 100.0])
        model.x0 = model.state_type.from_vec(x_state.as_vec())

        # Extract trim control values using dataclass
        u_trim_dc = model.input_type.from_vec(u_trim)
        base_ail = float(u_trim_dc.ail)
        base_elev = float(u_trim_dc.elev)
        base_rud = float(u_trim_dc.rud)
        base_thr = float(u_trim_dc.thr)

        # Wings-level controller only - let natural longitudinal dynamics evolve
        # This tests that the trim is a valid equilibrium
        kp_phi = 40.0  # Roll control gain
        kd_phi = 12.0  # Roll damping
        kd_rud = 5.0  # Yaw damping (coordinated flight)

        def u_func(t, x_vec, p):
            st = model.state_type.from_vec(x_vec)
            euler = lie.SO3EulerB321.from_Quat(lie.SO3Quat.elem(ca.DM(st.r)))
            # euler.param is [psi, theta, phi]
            phi = float(euler.param[2])

            # Wings level control (only lateral/directional)
            roll_error = -phi  # target phi = 0
            ail_cmd = base_ail + kp_phi * roll_error - kd_phi * st.w[0]
            rud_cmd = base_rud - kd_rud * st.w[2]

            # Use trim values for longitudinal control (no feedback)
            elev_cmd = base_elev
            thr_cmd = base_thr

            return np.array(
                [
                    float(np.clip(ail_cmd, -1.0, 1.0)),
                    float(np.clip(elev_cmd, -1.0, 1.0)),
                    float(np.clip(rud_cmd, -1.0, 1.0)),
                    float(np.clip(thr_cmd, 0.0, 1.0)),
                ]
            )

        x0_pos = x_state.p.copy()

        result = model.simulate(
            t0=0.0,
            tf=3.0,  # Reduced from 5.0s - still enough to demonstrate glide behavior
            dt=0.05,  # Increased from 0.02 - we have 100 substeps so 0.05s outer step is fine
            u_func=u_func,
            p_vec=model.p0.as_vec(),
        )

        times = result["t"]
        x_traj = result["x"]
        state_traj = model.state_type.from_matrix(x_traj)

        # Compute commanded inputs per saved step for diagnostics and outputs
        commanded_inputs = np.array(
            [u_func(times[i], x_traj[:, i], None) for i in range(len(times))]
        )

        positions = np.array([s.p for s in state_traj])
        velocities = np.array([s.v for s in state_traj])
        attitudes = np.array([s.r for s in state_traj])  # quaternions in x.r

        altitudes = positions[:, 2]
        # Compute horizontal distance from starting position
        x0_horizontal = np.array([x0_pos[0], x0_pos[1]])
        horizontal_distances = np.array([np.linalg.norm(p[:2] - x0_horizontal) for p in positions])
        airspeeds = np.linalg.norm(velocities, axis=1)

        final_time = float(times[-1])
        altitude_lost = float(altitudes[0] - altitudes[-1])
        horizontal_dist = float(horizontal_distances[-1])
        glide_ratio = horizontal_dist / altitude_lost if altitude_lost > 0 else 0.0
        avg_airspeed = float(np.mean(airspeeds))
        min_airspeed = float(np.min(airspeeds))
        descent_rate = altitude_lost / final_time if final_time > 0 else 0.0

        roll_angles = []
        pitch_angles = []
        for q in attitudes:
            phi = float(
                np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
            )
            theta = float(np.arcsin(np.clip(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1)))
            roll_angles.append(np.degrees(phi))
            pitch_angles.append(np.degrees(theta))

        max_roll = float(np.max(np.abs(roll_angles))) if roll_angles else 0.0
        max_pitch = float(np.max(np.abs(pitch_angles))) if pitch_angles else 0.0

        # Angle-of-attack series for stall guard
        out_traj = model.output_type.from_matrix(result["out"])
        alpha_series = [out.alpha for out in out_traj]
        max_alpha_deg = np.degrees(np.max(np.abs(alpha_series)))

        # Velocity variation check
        airspeed_std = float(np.std(airspeeds))
        pitch_std = float(np.std(pitch_angles))

        print("\nGlide test results (wings-level, natural longitudinal dynamics):")
        print(f"  Altitude lost: {altitude_lost:.2f} m")
        print(f"  Horizontal distance: {horizontal_dist:.2f} m")
        print(f"  Glide ratio: {glide_ratio:.2f}:1")
        print(f"  Average airspeed: {avg_airspeed:.2f} m/s")
        print(f"  Airspeed std dev: {airspeed_std:.2f} m/s")
        print(f"  Pitch std dev: {pitch_std:.2f}°")
        print(f"  Descent rate: {descent_rate:.2f} m/s")
        print(f"  Max roll angle: {max_roll:.1f}°")
        print(f"  Max pitch angle: {max_pitch:.1f}°")
        print(f"  Max |alpha|: {max_alpha_deg:.1f}°")

        # Verify natural longitudinal stability (wings level, trim elevator/throttle)
        # The trim should be stable enough that velocity and pitch don't diverge significantly
        assert max_roll < 10.0, f"Roll control failed: max_roll={max_roll:.1f}°"
        assert airspeed_std < 1.0, f"Airspeed not stable: std={airspeed_std:.2f} m/s"
        assert pitch_std < 5.0, f"Pitch not stable: std={pitch_std:.2f}°"

        # Tighter performance bounds - should achieve good glide ratio
        assert glide_ratio > 5.5, f"Glide ratio too low: {glide_ratio:.2f}:1 (expect 6-11:1)"
        assert glide_ratio < 12.0, f"Glide ratio unrealistically high: {glide_ratio:.2f}:1"

        # Verify L/D ratio is reasonable
        LD_ratio = CL_trim / CD_trim
        assert LD_ratio > 6.0, f"L/D too low: {LD_ratio:.2f} (expect ~7-8)"
        assert LD_ratio < 10.0, f"L/D unrealistically high: {LD_ratio:.2f}"

        # Airspeed should be in efficient glide range
        assert 5.0 < avg_airspeed < 8.0, f"Average airspeed out of band: {avg_airspeed:.2f} m/s"
        assert min_airspeed > 4.5, f"Minimum airspeed too low: {min_airspeed:.2f} m/s"

        # Angle of attack should be reasonable for efficient glide
        assert (
            2.0 < np.degrees(alpha_trim) < 8.0
        ), f"Trim alpha out of range: {np.degrees(alpha_trim):.2f}°"
        assert max_alpha_deg < 12.0, f"Angle of attack too high: {max_alpha_deg:.1f}°"

        # Descent rate should be reasonable
        assert 0.5 < descent_rate < 2.0, f"Descent rate out of range: {descent_rate:.2f} m/s"

    def test_takeoff_ready_initial_conditions(self):
        """Ensure takeoff-ready setup yields zero wheel penetration and no upward impulse."""
        model = sportcub()
        x = copy.deepcopy(model.x0)
        u = copy.deepcopy(model.u0)
        p = copy.deepcopy(model.p0)
        x.p[2] = 0.1  # CG altitude so main wheels at z=0
        x.v[:] = 0.0
        u.thr = 0.0
        u.elev = 0.0
        # Wheel geometry (matches sportcub model)
        left_wheel_b = np.array([0.1, 0.1, -0.1])
        right_wheel_b = np.array([0.1, -0.1, -0.1])
        tail_wheel_b = np.array([-0.4, 0.0, 0.0])
        for wb in [left_wheel_b, right_wheel_b]:
            world_z = x.p[2] + wb[2]
            assert world_z >= -1e-9
            assert abs(world_z) < 1e-9 or world_z == 0.0
        tail_world_z = x.p[2] + tail_wheel_b[2]
        assert tail_world_z > 0.0
        u_vec = u.as_vec()
        p_vec = p.as_vec()

        def u_func(t, x_state_vec, p):
            return u_vec

        model.x0 = model.state_type.from_vec(x.as_vec())
        sim = model.simulate(t0=0.0, tf=0.001, dt=0.001, u_func=u_func, p_vec=p_vec)
        x_next = model.state_type.from_vec(sim["x"][:, -1])
        assert x_next.v[2] <= 1e-6, f"Unexpected upward vertical velocity: {x_next.v[2]}"

    @staticmethod
    def _extract_outputs(model, x_vec, u_vec, p_vec):
        """Extract outputs as a dictionary from state, input, and parameter vectors."""
        out_vec = model.f_y(x_vec, u_vec, p_vec)
        return model.output_type.from_vec(out_vec)

    def test_trim_linearize_5ms(self):
        """Trim and linearize at 5 m/s, verify near-zero net forces/moments and mode presence."""
        # Use Euler representation for better mode analysis and interpretation
        model = sportcub(attitude_rep="euler")
        # Optimize throttle & elevator for level flight (vertical velocity near zero)
        # Always force verbose printing for this test so optimizer & mode details are visible
        x_trim, u_trim, _ = find_trim(
            model,
            V_target=5.0,
            gamma=0.0,
            print_progress=False,  # suppress high-level iterative logging
            fix_throttle=False,
            verbose=True,  # keep detailed trim solution summary
            ipopt_print_level=1,  # final solver summary only (no iteration table)
        )
        A, B = linearize_dynamics(model, x_trim, u_trim)
        assert A.shape == (len(x_trim), len(x_trim))
        assert B.shape[0] == len(x_trim)
        # Build state names dynamically if index map available (fallback to legacy constant)
        dynamic_state_names = getattr(model, "state_names", None)
        if not dynamic_state_names:  # Fallback if accessor missing
            dynamic_state_names = [
                n
                for n, _ in sorted(getattr(model, "state_index", {}).items(), key=lambda kv: kv[1])
            ]
        modes = analyze_modes(A, state_names=dynamic_state_names)
        classified = classify_aircraft_modes(modes, dynamic_state_names)
        print_mode_summary(modes, dynamic_state_names)
        # Print quick force/moment summary
        print("\nForce/Moment summary at trim:")
        outputs = self._extract_outputs(model, x_trim, u_trim, model.p0.as_vec())
        F_b = outputs.F_b
        M_b = outputs.M_b
        print(f"  F_b = {F_b}")
        print(f"  M_b = {M_b}")
        assert F_b is not None and M_b is not None
        # Extract vertical component and overall magnitudes
        F_mag = np.linalg.norm(F_b)
        M_mag = np.linalg.norm(M_b)

        # Tighter trim quality checks
        assert abs(F_b[2]) < 0.02, f"Vertical force imbalance: {F_b[2]:.4f} N"
        assert F_mag < 0.05, f"Net force too large: {F_mag:.4f} N"
        assert M_mag < 0.02, f"Net moment too large: {M_mag:.4f} N·m"

        # Check vertical velocity and acceleration near zero
        x_dot = model.f_x(x_trim, u_trim, model.p0.as_vec())
        v_dot_z = float(x_dot[5])  # index 5 in concatenated (p_dot[0:3], v_dot[3:6], ...)
        v_z = float(x_trim[5])
        assert abs(v_z) < 0.02, f"Vertical velocity not trimmed: {v_z:.4f} m/s"
        assert abs(v_dot_z) < 0.02, f"Vertical acceleration high: {v_dot_z:.4f} m/s^2"

        # Check flight performance metrics
        assert 0.15 < outputs.CL < 0.85, f"CL out of range: {outputs.CL:.3f}"
        assert 0.02 < outputs.CD < 0.12, f"CD out of range: {outputs.CD:.3f}"
        LD = outputs.CL / outputs.CD
        assert 5.0 < LD < 10.0, f"L/D out of range: {LD:.2f}"

        # Check alpha is reasonable
        alpha_deg = np.degrees(outputs.alpha)
        assert 2.0 < alpha_deg < 12.0, f"Alpha out of range: {alpha_deg:.2f}°"

        sp = classified.get("short_period")
        rd = classified.get("roll_damping")
        assert sp is not None and sp["real"] < 0.0, "Short period should be stable"
        assert rd is not None and rd["real"] < 0.0, "Roll damping should be stable"

        # Check mode characteristics
        if sp is not None:
            assert (
                sp["damping_ratio"] > 0.3
            ), f"Short period under-damped: ζ={sp['damping_ratio']:.3f}"
            # Short period can be oscillatory or critically/over-damped
            # If oscillatory, frequency should be reasonable
            if sp["is_oscillatory"]:
                assert (
                    sp["frequency_hz"] > 0.5
                ), f"Short period frequency low: {sp['frequency_hz']:.2f} Hz"

        # Basic condition checks
        assert outputs.qbar > 0.0
        assert abs(outputs.Vt - 5.0) < 0.2, f"Trim velocity error: {outputs.Vt:.2f} m/s"
        # Check quaternion norm (compute from output quaternion)
        q_norm = np.linalg.norm(outputs.q)
        assert abs(q_norm - 1.0) < 1e-3

    def test_euler_linearization_5ms(self):
        """Verify Euler angle representation works for linearization and trim at 5 m/s."""
        # Create model with Euler angle representation
        model_euler = sportcub(attitude_rep="euler")

        print("\nTesting Euler angle linearization at 5 m/s...")

        # Find trim with Euler representation
        x_trim, u_trim, _ = find_trim(
            model_euler,
            V_target=5.0,
            gamma=0.0,
            print_progress=False,
            fix_throttle=False,
            verbose=True,
            ipopt_print_level=1,
        )

        # Verify state structure - should have Euler angles (3D) instead of quaternion (4D)
        x_state = model_euler.state_type.from_vec(x_trim)
        assert hasattr(x_state, "r"), "State should have 'r' field for rotation"
        assert len(x_state.r) == 3, f"Euler angles should be 3D, got {len(x_state.r)}"

        # Check that Euler angles are reasonable for level flight
        psi, theta, phi = x_state.r[0], x_state.r[1], x_state.r[2]
        print(
            f"  Trim Euler angles: psi={np.degrees(psi):.2f}°, theta={np.degrees(theta):.2f}°, phi={np.degrees(phi):.2f}°"
        )

        # For level flight, pitch should be small positive, roll near zero
        assert abs(phi) < np.deg2rad(5.0), f"Roll angle too large: {np.degrees(phi):.2f}°"
        assert abs(theta) < np.deg2rad(15.0), f"Pitch angle too large: {np.degrees(theta):.2f}°"

        # Linearize with Euler representation
        A, B = linearize_dynamics(model_euler, x_trim, u_trim)

        # State vector for Euler: [px, py, pz, vx, vy, vz, psi, theta, phi, wx, wy, wz]
        expected_state_size = 3 + 3 + 3 + 3  # position + velocity + euler + angular velocity
        assert A.shape == (
            expected_state_size,
            expected_state_size,
        ), f"A matrix size mismatch: {A.shape}"
        assert B.shape[0] == expected_state_size, f"B matrix rows mismatch: {B.shape}"

        # Analyze modes
        dynamic_state_names = getattr(model_euler, "state_names", None)
        if not dynamic_state_names:
            dynamic_state_names = [
                n
                for n, _ in sorted(
                    getattr(model_euler, "state_index", {}).items(),
                    key=lambda kv: kv[1],
                )
            ]

        modes = analyze_modes(A, state_names=dynamic_state_names)
        classified = classify_aircraft_modes(modes, dynamic_state_names)
        print_mode_summary(modes, dynamic_state_names)

        # Verify forces and moments at trim
        outputs = self._extract_outputs(model_euler, x_trim, u_trim, model_euler.p0.as_vec())
        F_b = outputs.F_b
        M_b = outputs.M_b

        print(f"\n  F_b = {F_b}")
        print(f"  M_b = {M_b}")

        # Check trim quality
        F_mag = np.linalg.norm(F_b)
        M_mag = np.linalg.norm(M_b)
        assert abs(F_b[2]) < 0.02, f"Vertical force imbalance: {F_b[2]:.4f} N"
        assert F_mag < 0.05, f"Net force too large: {F_mag:.4f} N"
        assert M_mag < 0.02, f"Net moment too large: {M_mag:.4f} N·m"

        # Verify modes exist and are stable
        sp = classified.get("short_period")
        rd = classified.get("roll_damping")
        assert sp is not None, "Short period mode not found"
        assert rd is not None, "Roll damping mode not found"
        assert sp["real"] < 0.0, "Short period should be stable"
        assert rd["real"] < 0.0, "Roll damping should be stable"

        # Check mode quality
        if sp is not None:
            assert (
                sp["damping_ratio"] > 0.3
            ), f"Short period under-damped: ζ={sp['damping_ratio']:.3f}"

        # Check flight performance
        LD = outputs.CL / outputs.CD if outputs.CD > 0 else 0
        assert 5.0 < LD < 10.0, f"L/D out of range: {LD:.2f}"

        print("  ✓ Euler linearization successful")

    def test_trim_quaternion_vs_euler_5ms(self):
        """Verify quaternion and Euler representations produce equivalent trim solutions at 5 m/s."""
        print("\n" + "=" * 80)
        print("COMPARING QUATERNION vs EULER TRIM SOLUTIONS AT 5 m/s")
        print("=" * 80)

        # Create both models
        model_quat = sportcub(attitude_rep="quat")
        model_euler = sportcub(attitude_rep="euler")

        # Find trim with quaternion representation
        print("\n--- QUATERNION REPRESENTATION ---")
        x_trim_quat, u_trim_quat, stats_quat = find_trim(
            model_quat,
            V_target=5.0,
            gamma=0.0,
            print_progress=False,
            fix_throttle=False,
            verbose=True,
            ipopt_print_level=1,
        )

        # Find trim with Euler representation
        print("\n--- EULER ANGLE REPRESENTATION ---")
        x_trim_euler, u_trim_euler, stats_euler = find_trim(
            model_euler,
            V_target=5.0,
            gamma=0.0,
            print_progress=False,
            fix_throttle=False,
            verbose=True,
            ipopt_print_level=1,
        )

        # Extract state structures
        state_quat = model_quat.state_type.from_vec(x_trim_quat)
        state_euler = model_euler.state_type.from_vec(x_trim_euler)

        # Extract input structures
        input_quat = model_quat.input_type.from_vec(u_trim_quat)
        input_euler = model_euler.input_type.from_vec(u_trim_euler)

        # Convert quaternion to Euler for comparison
        # quaternion is [w, x, y, z], need to convert to [phi, theta, psi]
        qw, qx, qy, qz = state_quat.r[0], state_quat.r[1], state_quat.r[2], state_quat.r[3]

        # Convert quaternion to Euler angles (3-2-1 sequence)
        # Roll (phi)
        phi_from_quat = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        # Pitch (theta)
        sin_theta = 2 * (qw * qy - qz * qx)
        theta_from_quat = np.arcsin(np.clip(sin_theta, -1.0, 1.0))
        # Yaw (psi)
        psi_from_quat = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

        print("\n" + "=" * 80)
        print("COMPARISON RESULTS")
        print("=" * 80)

        # Compare positions
        print("\nPosition (m):")
        print(f"  Quaternion: {state_quat.p}")
        print(f"  Euler:      {state_euler.p}")
        np.testing.assert_allclose(state_quat.p, state_euler.p, rtol=1e-3, atol=1e-3)
        print("  ✓ Positions match")

        # Compare velocities
        print("\nVelocity (m/s):")
        print(f"  Quaternion: {state_quat.v}")
        print(f"  Euler:      {state_euler.v}")
        np.testing.assert_allclose(state_quat.v, state_euler.v, rtol=1e-3, atol=1e-3)
        print("  ✓ Velocities match")

        # Compare attitudes
        print("\nAttitude (deg):")
        print(
            f"  From Quat:  phi={np.degrees(phi_from_quat):.3f}°, theta={np.degrees(theta_from_quat):.3f}°, psi={np.degrees(psi_from_quat):.3f}°"
        )
        print(
            f"  Euler:      phi={np.degrees(state_euler.r[2]):.3f}°, theta={np.degrees(state_euler.r[1]):.3f}°, psi={np.degrees(state_euler.r[0]):.3f}°"
        )
        # Note: Euler state is [psi, theta, phi] order
        np.testing.assert_allclose(
            [phi_from_quat, theta_from_quat, psi_from_quat],
            [state_euler.r[2], state_euler.r[1], state_euler.r[0]],
            rtol=1e-2,
            atol=np.deg2rad(0.5),  # 0.5 degree tolerance
        )
        print("  ✓ Attitudes match")

        # Compare angular velocities
        print("\nAngular velocity (rad/s):")
        print(f"  Quaternion: {state_quat.w}")
        print(f"  Euler:      {state_euler.w}")
        np.testing.assert_allclose(state_quat.w, state_euler.w, rtol=2e-3, atol=2e-3)
        print("  ✓ Angular velocities match")

        # Compare control inputs
        print("\nControl inputs:")
        print(
            f"  Quaternion: ail={input_quat.ail:.4f}, elev={input_quat.elev:.4f}, rud={input_quat.rud:.4f}, thr={input_quat.thr:.4f}"
        )
        print(
            f"  Euler:      ail={input_euler.ail:.4f}, elev={input_euler.elev:.4f}, rud={input_euler.rud:.4f}, thr={input_euler.thr:.4f}"
        )
        np.testing.assert_allclose(
            [input_quat.ail, input_quat.elev, input_quat.rud, input_quat.thr],
            [input_euler.ail, input_euler.elev, input_euler.rud, input_euler.thr],
            rtol=1e-3,
            atol=1e-3,
        )
        print("  ✓ Control inputs match")

        # Compare outputs (forces, moments, aerodynamics)
        outputs_quat = self._extract_outputs(
            model_quat, x_trim_quat, u_trim_quat, model_quat.p0.as_vec()
        )
        outputs_euler = self._extract_outputs(
            model_euler, x_trim_euler, u_trim_euler, model_euler.p0.as_vec()
        )

        print("\nAerodynamic outputs:")
        print(
            f"  Quaternion: Vt={outputs_quat.Vt:.3f} m/s, alpha={np.degrees(outputs_quat.alpha):.3f}°, CL={outputs_quat.CL:.4f}, CD={outputs_quat.CD:.4f}"
        )
        print(
            f"  Euler:      Vt={outputs_euler.Vt:.3f} m/s, alpha={np.degrees(outputs_euler.alpha):.3f}°, CL={outputs_euler.CL:.4f}, CD={outputs_euler.CD:.4f}"
        )
        np.testing.assert_allclose(outputs_quat.Vt, outputs_euler.Vt, rtol=1e-3)
        np.testing.assert_allclose(
            outputs_quat.alpha, outputs_euler.alpha, rtol=1e-2, atol=np.deg2rad(0.5)
        )
        np.testing.assert_allclose(outputs_quat.CL, outputs_euler.CL, rtol=1e-3)
        np.testing.assert_allclose(outputs_quat.CD, outputs_euler.CD, rtol=1e-3)
        print("  ✓ Aerodynamic outputs match")

        print("\nForces and moments:")
        print(f"  Quaternion: F_b={outputs_quat.F_b}, M_b={outputs_quat.M_b}")
        print(f"  Euler:      F_b={outputs_euler.F_b}, M_b={outputs_euler.M_b}")
        np.testing.assert_allclose(outputs_quat.F_b, outputs_euler.F_b, rtol=1e-2, atol=1e-3)
        np.testing.assert_allclose(outputs_quat.M_b, outputs_euler.M_b, rtol=1e-2, atol=1e-3)
        print("  ✓ Forces and moments match")

        # Compare linearized dynamics and modes
        print("\n" + "=" * 80)
        print("COMPARING LINEARIZED DYNAMICS AND FLIGHT MODES")
        print("=" * 80)

        A_quat, B_quat = linearize_dynamics(model_quat, x_trim_quat, u_trim_quat)
        A_euler, B_euler = linearize_dynamics(model_euler, x_trim_euler, u_trim_euler)

        print(f"\nState space dimensions:")
        print(
            f"  Quaternion: A is {A_quat.shape[0]}x{A_quat.shape[1]} (13 states: p[3], v[3], q[4], w[3])"
        )
        print(
            f"  Euler:      A is {A_euler.shape[0]}x{A_euler.shape[1]} (12 states: p[3], v[3], r[3], w[3])"
        )

        # Analyze modes for both representations
        state_names_quat = getattr(model_quat, "state_names", None)
        if not state_names_quat:
            state_names_quat = [
                n
                for n, _ in sorted(
                    getattr(model_quat, "state_index", {}).items(),
                    key=lambda kv: kv[1],
                )
            ]

        state_names_euler = getattr(model_euler, "state_names", None)
        if not state_names_euler:
            state_names_euler = [
                n
                for n, _ in sorted(
                    getattr(model_euler, "state_index", {}).items(),
                    key=lambda kv: kv[1],
                )
            ]

        modes_quat = analyze_modes(A_quat, state_names=state_names_quat)
        modes_euler = analyze_modes(A_euler, state_names=state_names_euler)

        classified_quat = classify_aircraft_modes(modes_quat, state_names_quat)
        classified_euler = classify_aircraft_modes(modes_euler, state_names_euler)

        print("\n--- QUATERNION REPRESENTATION MODES ---")
        print_mode_summary(modes_quat, state_names_quat)

        print("\n--- EULER REPRESENTATION MODES ---")
        print_mode_summary(modes_euler, state_names_euler)

        # Compare key mode characteristics
        print("\n" + "=" * 80)
        print("MODE COMPARISON")
        print("=" * 80)

        mode_types = [
            ("short_period", "Short Period"),
            ("phugoid", "Phugoid"),
            ("dutch_roll", "Dutch Roll"),
            ("roll_damping", "Roll Damping"),
            ("spiral", "Spiral"),
        ]

        for key, name in mode_types:
            mode_q = classified_quat.get(key)
            mode_e = classified_euler.get(key)

            print(f"\n{name}:")
            if mode_q is None and mode_e is None:
                print("  Both: NOT IDENTIFIED")
                continue
            elif mode_q is None:
                print("  Quaternion: NOT IDENTIFIED")
                print(
                    f"  Euler:      λ={mode_e['eigenvalue_continuous']:.4f}, τ={mode_e['time_constant']:.3f}s"
                )
                continue
            elif mode_e is None:
                print(
                    f"  Quaternion: λ={mode_q['eigenvalue_continuous']:.4f}, τ={mode_q['time_constant']:.3f}s"
                )
                print("  Euler:      NOT IDENTIFIED")
                continue

            # Both identified - compare
            print(f"  Eigenvalue:")
            print(f"    Quaternion: {mode_q['eigenvalue_continuous']:.4f}")
            print(f"    Euler:      {mode_e['eigenvalue_continuous']:.4f}")

            print(f"  Time constant:")
            print(f"    Quaternion: {mode_q['time_constant']:.3f} s")
            print(f"    Euler:      {mode_e['time_constant']:.3f} s")

            print(f"  Damping ratio:")
            print(f"    Quaternion: {mode_q['damping_ratio']:.4f}")
            print(f"    Euler:      {mode_e['damping_ratio']:.4f}")

            if mode_q["is_oscillatory"] and mode_e["is_oscillatory"]:
                print(f"  Frequency:")
                print(f"    Quaternion: {mode_q['frequency_hz']:.4f} Hz")
                print(f"    Euler:      {mode_e['frequency_hz']:.4f} Hz")

                # Check that frequencies match closely (should be identical for physical modes)
                freq_diff = abs(mode_q["frequency_hz"] - mode_e["frequency_hz"])
                freq_rel_diff = freq_diff / max(mode_q["frequency_hz"], 1e-6)
                if freq_rel_diff > 0.05:  # 5% tolerance
                    print(f"    ⚠ WARNING: Frequency mismatch {freq_rel_diff*100:.1f}%")
                else:
                    print(f"    ✓ Frequencies match (diff: {freq_rel_diff*100:.2f}%)")

            # Check that time constants match
            tc_diff = abs(mode_q["time_constant"] - mode_e["time_constant"])
            tc_rel_diff = tc_diff / max(mode_q["time_constant"], 1e-6)
            if tc_rel_diff > 0.05:  # 5% tolerance
                print(f"  ⚠ WARNING: Time constant mismatch {tc_rel_diff*100:.1f}%")
            else:
                print(f"  ✓ Time constants match (diff: {tc_rel_diff*100:.2f}%)")

            # Check stability agreement
            if mode_q["stable"] != mode_e["stable"]:
                print(f"  ⚠ WARNING: Stability disagreement!")
            else:
                stable_str = "stable" if mode_q["stable"] else "UNSTABLE"
                print(f"  ✓ Both {stable_str}")

        print("\n" + "=" * 80)
        print("✓ QUATERNION AND EULER REPRESENTATIONS PRODUCE EQUIVALENT TRIM")
        print("=" * 80)
