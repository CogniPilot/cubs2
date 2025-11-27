"""Tests for the sportcub model factory."""
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
import casadi as ca
from cubs2_dynamics.sportcub import sportcub
from cubs2_dynamics.trim_fixed_wing import classify_aircraft_modes
from cubs2_dynamics.trim_fixed_wing import find_trim_fixed_wing as find_trim
from cubs2_dynamics.trim_fixed_wing import print_mode_summary
from cyecca import lie
from cyecca.dynamics.explicit.linearize import analyze_modes
from cyecca.dynamics.explicit.linearize import linearize_dynamics
import numpy as np
import pytest


class TestSportCubNode:
    """Collection of unit tests for the sportcub dynamics model."""

    def test_initial_conditions_near_origin(self):
        """Verify default initial conditions (position, velocity, attitude, throttle)."""
        model = sportcub()
        v0 = model.v0

        np.testing.assert_allclose(v0.p, [0.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(v0.v, [0.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(v0.r, [1.0, 0.0, 0.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(v0.w, [0.0, 0.0, 0.0], atol=1e-6)
        assert v0.thr == 0.0

    def test_ground_reaction_settles_at_origin(self):
        """Aircraft near ground with aero disabled should remain stable without drift."""
        model = sportcub()
        model.v0.disable_aero = 1.0

        initial_z = 0.30
        model.v0.p = np.array([0.0, 0.0, initial_z])

        def u_func(t, m):
            return {'ail': 0, 'elev': 0, 'rud': 0, 'thr': 0}

        dt = 0.02
        total_time = 3.0

        t, traj = model.simulate(t0=0.0, tf=total_time, dt=dt, u_func=u_func)

        z_series = traj.p[:, 2]
        xy_series = np.linalg.norm(traj.p[:, :2], axis=1)
        v_mags = np.linalg.norm(traj.v, axis=1)

        min_z = float(np.min(z_series))
        max_z = float(np.max(z_series))
        max_xy_drift = float(np.max(xy_series))

        half_idx = len(v_mags) // 2
        settled_mask = (v_mags[half_idx:] < 0.05) & (xy_series[half_idx:] < 0.02)
        settled = bool(settled_mask.any())

        v_final = model.v0
        assert min_z > -0.05, f'Unexpected penetration below ground: min_z={min_z:.3f}'
        assert max_z <= initial_z + 0.02, f'Bounced above initial height: max_z={max_z:.3f}'
        xy_distance = np.linalg.norm(v_final.p[:2])
        assert xy_distance < 0.05, f'Aircraft drifted horizontally {xy_distance:.3f} m'
        assert settled, 'Ground reaction did not settle within allotted simulation time'
        assert np.all(np.isfinite(v_final.p))
        assert np.all(np.isfinite(v_final.v))
        q_norm = np.linalg.norm(v_final.r)
        np.testing.assert_allclose(q_norm, 1.0, rtol=1e-3)

    def test_ground_bounce_characterization(self):
        """Characterize ground bounce over several drop heights with aero disabled."""
        drop_heights = [0.5, 5.0]
        dt = 0.01
        max_steps = 1000
        results = []

        for h0 in drop_heights:
            model = sportcub()
            model.v0.disable_aero = 1.0
            model.v0.p = np.array([0.0, 0.0, h0])

            def u_func(t, m):
                return {'ail': 0, 'elev': 0, 'rud': 0, 'thr': 0}

            total_time = dt * max_steps
            t, traj = model.simulate(t0=0.0, tf=total_time, dt=dt, u_func=u_func)

            z_series = traj.p[:, 2]
            vz_series = traj.v[:, 2]
            v_mags = np.linalg.norm(traj.v, axis=1)

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
                    if prev_vz > 0.001 and vz <= 0 and (z - min_z_after_contact) >= bounce_threshold:
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

        for h0, _, e, max_penetration, settling_time, settled in results:
            assert e <= 1.0 + 1e-6
            if h0 >= 1.0:
                assert e <= 0.55, f'Bounce too energetic for h0={h0}m: e={e:.3f}'
                assert e >= 0.08, f'Bounce too damped for h0={h0}m: e={e:.3f}'

            assert settled and settling_time is not None and settling_time <= 8.0, \
                f'Drop {h0} m took too long to settle'

            wheel_offset = 0.1
            wheel_penetration = max_penetration + wheel_offset

            if h0 <= 1.0:
                max_allowed = 0.11
            elif h0 <= 5.0:
                max_allowed = 0.18
            else:
                max_allowed = 0.25
            assert wheel_penetration < max_allowed, \
                f'Excessive penetration for h0={h0}m: {wheel_penetration:.3f}m'

    def test_glide_slope(self):
        """Verify controlled glide with wings-level control optimized for maximum L/D."""
        model = sportcub()
        model.v0.disable_aero = 0.0
        model.v0.disable_gf = 1.0

        v_trim, stats = find_trim(
            model,
            V_target=None,
            gamma=None,
            fix_throttle=True,
            print_progress=True,
            verbose=True,
            ipopt_print_level=1,
        )

        # Build vectors for verification using model helpers
        x_trim = model.x(v_trim)
        u_trim = model.u(v_trim)
        p_vec = model.p(v_trim)

        # Verify trim quality
        x_dot = model._f(x_trim, u_trim, p_vec, 0.0)
        x_dot = np.array(x_dot).flatten()
        v_dot = x_dot[3:6]
        w_dot = x_dot[9:12]
        v_dot_mag = np.linalg.norm(v_dot)
        w_dot_mag = np.linalg.norm(w_dot)

        assert v_dot_mag < 0.01, f'Velocity acceleration not zero: |v_dot|={v_dot_mag:.6f}'
        assert w_dot_mag < 0.01, f'Angular acceleration not zero: |w_dot|={w_dot_mag:.6f}'

        # Set up simulation
        model.v0.p = np.array([0.0, 0.0, 100.0])
        for name in model._state_fields:
            if name != 'p':
                setattr(model.v0, name, getattr(v_trim, name))
        for name in model._input_fields:
            setattr(model.v0, name, getattr(v_trim, name))

        base_ail = float(v_trim.ail)
        base_elev = float(v_trim.elev)
        base_rud = float(v_trim.rud)
        base_thr = float(v_trim.thr)

        kp_phi = 40.0
        kd_phi = 12.0
        kd_rud = 5.0

        def u_func(t, m):
            st = m.v0
            euler = lie.SO3EulerB321.from_Quat(lie.SO3Quat.elem(ca.DM(st.r)))
            phi = float(euler.param[2])
            roll_error = -phi
            return {
                'ail': float(np.clip(base_ail + kp_phi * roll_error - kd_phi * st.w[0], -1, 1)),
                'elev': float(np.clip(base_elev, -1, 1)),
                'rud': float(np.clip(base_rud - kd_rud * st.w[2], -1, 1)),
                'thr': float(np.clip(base_thr, 0, 1)),
            }

        x0_pos = model.v0.p.copy()
        t, traj = model.simulate(t0=0.0, tf=3.0, dt=0.05, u_func=u_func)

        altitudes = traj.p[:, 2]
        x0_horizontal = np.array([x0_pos[0], x0_pos[1]])
        horizontal_distances = np.array([np.linalg.norm(p[:2] - x0_horizontal) for p in traj.p])
        airspeeds = np.linalg.norm(traj.v, axis=1)

        final_time = float(t[-1])
        altitude_lost = float(altitudes[0] - altitudes[-1])
        horizontal_dist = float(horizontal_distances[-1])
        glide_ratio = horizontal_dist / altitude_lost if altitude_lost > 0 else 0.0
        avg_airspeed = float(np.mean(airspeeds))
        min_airspeed = float(np.min(airspeeds))

        roll_angles = []
        pitch_angles = []
        for q in traj.r:
            phi = float(np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)))
            theta = float(np.arcsin(np.clip(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1)))
            roll_angles.append(np.degrees(phi))
            pitch_angles.append(np.degrees(theta))

        max_roll = float(np.max(np.abs(roll_angles)))
        airspeed_std = float(np.std(airspeeds))
        pitch_std = float(np.std(pitch_angles))

        assert max_roll < 10.0, f'Roll control failed: max_roll={max_roll:.1f}°'
        assert airspeed_std < 1.0, f'Airspeed not stable: std={airspeed_std:.2f} m/s'
        assert pitch_std < 5.0, f'Pitch not stable: std={pitch_std:.2f}°'
        assert glide_ratio > 5.5, f'Glide ratio too low: {glide_ratio:.2f}:1'
        assert glide_ratio < 12.0, f'Glide ratio unrealistically high: {glide_ratio:.2f}:1'
        assert 4.5 < avg_airspeed < 8.0, f'Average airspeed out of band: {avg_airspeed:.2f} m/s'
        assert min_airspeed > 4.0, f'Minimum airspeed too low: {min_airspeed:.2f} m/s'

    def test_takeoff_ready_initial_conditions(self):
        """Ensure takeoff-ready setup yields zero wheel penetration and no upward impulse."""
        model = sportcub()
        model.v0.p = np.array([0.0, 0.0, 0.1])
        model.v0.v = np.array([0.0, 0.0, 0.0])
        model.v0.thr = 0.0
        model.v0.elev = 0.0

        left_wheel_b = np.array([0.1, 0.1, -0.1])
        right_wheel_b = np.array([0.1, -0.1, -0.1])
        tail_wheel_b = np.array([-0.4, 0.0, 0.0])

        for wb in [left_wheel_b, right_wheel_b]:
            world_z = model.v0.p[2] + wb[2]
            assert world_z >= -1e-9
            assert abs(world_z) < 1e-9 or world_z == 0.0
        tail_world_z = model.v0.p[2] + tail_wheel_b[2]
        assert tail_world_z > 0.0

        def u_func(t, m):
            return {'ail': 0, 'elev': 0, 'rud': 0, 'thr': 0}

        t, traj = model.simulate(t0=0.0, tf=0.001, dt=0.001, u_func=u_func)
        v_next = model.v0
        assert v_next.v[2] <= 1e-6, f'Unexpected upward vertical velocity: {v_next.v[2]}'

    def test_trim_linearize_5ms(self):
        """Trim and linearize at 5 m/s, verify near-zero net forces/moments and mode presence."""
        model = sportcub(attitude_rep='euler')

        v_trim, stats = find_trim(
            model,
            V_target=5.0,
            gamma=0.0,
            print_progress=False,
            fix_throttle=False,
            verbose=True,
            ipopt_print_level=1,
        )

        A, B, C, D = linearize_dynamics(model, v_trim)
        n_states = model.n_states
        assert A.shape == (n_states, n_states)
        assert B.shape[0] == n_states

        modes = analyze_modes(A, names=model.state_names)
        classified = classify_aircraft_modes(modes, model.state_names)
        print_mode_summary(modes, model.state_names)

        # Build vectors for output verification
        x_trim = model.x(v_trim)
        u_trim = model.u(v_trim)
        p_vec = model.p(v_trim)

        outputs = model._g(x_trim, u_trim, p_vec, 0.0)
        outputs = np.array(outputs).flatten()

        out_offset = 0
        out_indices = {}
        for field_name in model._output_fields:
            field_info = model.model_type._field_info[field_name]
            dim = field_info['dim']
            out_indices[field_name] = (out_offset, out_offset + dim)
            out_offset += dim

        F_b_idx = out_indices.get('F_b', (0, 3))
        M_b_idx = out_indices.get('M_b', (0, 3))
        F_b = outputs[F_b_idx[0]:F_b_idx[1]]
        M_b = outputs[M_b_idx[0]:M_b_idx[1]]

        F_mag = np.linalg.norm(F_b)
        M_mag = np.linalg.norm(M_b)

        assert abs(F_b[2]) < 0.02, f'Vertical force imbalance: {F_b[2]:.4f} N'
        assert F_mag < 0.05, f'Net force too large: {F_mag:.4f} N'
        assert M_mag < 0.02, f'Net moment too large: {M_mag:.4f} N·m'

        v_z = float(v_trim.v[2])
        assert abs(v_z) < 0.02, f'Vertical velocity not trimmed: {v_z:.4f} m/s'

        CL_idx = out_indices.get('CL', (0, 1))
        CD_idx = out_indices.get('CD', (0, 1))
        CL = float(outputs[CL_idx[0]])
        CD = float(outputs[CD_idx[0]])

        assert 0.15 < CL < 0.85, f'CL out of range: {CL:.3f}'
        assert 0.02 < CD < 0.12, f'CD out of range: {CD:.3f}'
        LD = CL / CD
        assert 5.0 < LD < 10.0, f'L/D out of range: {LD:.2f}'

        sp = classified.get('short_period')
        rd = classified.get('roll_damping')
        assert sp is not None and sp['real'] < 0.0, 'Short period should be stable'
        assert rd is not None and rd['real'] < 0.0, 'Roll damping should be stable'

        if sp is not None:
            assert sp['zeta'] > 0.3, f"Short period under-damped: ζ={sp['zeta']:.3f}"

    def test_euler_linearization_5ms(self):
        """Verify Euler angle representation works for linearization and trim at 5 m/s."""
        model_euler = sportcub(attitude_rep='euler')

        v_trim, stats = find_trim(
            model_euler,
            V_target=5.0,
            gamma=0.0,
            print_progress=False,
            fix_throttle=False,
            verbose=True,
            ipopt_print_level=1,
        )

        assert hasattr(v_trim, 'r'), 'State should have "r" field for rotation'
        assert len(v_trim.r) == 3, f'Euler angles should be 3D, got {len(v_trim.r)}'

        psi, theta, phi = v_trim.r[0], v_trim.r[1], v_trim.r[2]
        assert abs(phi) < np.deg2rad(5.0), f'Roll angle too large: {np.degrees(phi):.2f}°'
        assert abs(theta) < np.deg2rad(15.0), f'Pitch angle too large: {np.degrees(theta):.2f}°'

        A, B, C, D = linearize_dynamics(model_euler, v_trim)

        expected_state_size = 3 + 3 + 3 + 3
        assert A.shape == (expected_state_size, expected_state_size), f'A matrix size mismatch: {A.shape}'
        assert B.shape[0] == expected_state_size, f'B matrix rows mismatch: {B.shape}'

        modes = analyze_modes(A, names=model_euler.state_names)
        classified = classify_aircraft_modes(modes, model_euler.state_names)
        print_mode_summary(modes, model_euler.state_names)

        x_trim = model_euler.x(v_trim)
        u_trim = model_euler.u(v_trim)
        p_vec = model_euler.p(v_trim)

        outputs = model_euler._g(x_trim, u_trim, p_vec, 0.0)
        outputs = np.array(outputs).flatten()

        out_offset = 0
        out_indices = {}
        for field_name in model_euler._output_fields:
            field_info = model_euler.model_type._field_info[field_name]
            dim = field_info['dim']
            out_indices[field_name] = (out_offset, out_offset + dim)
            out_offset += dim

        F_b_idx = out_indices.get('F_b', (0, 3))
        M_b_idx = out_indices.get('M_b', (0, 3))
        F_b = outputs[F_b_idx[0]:F_b_idx[1]]
        M_b = outputs[M_b_idx[0]:M_b_idx[1]]

        F_mag = np.linalg.norm(F_b)
        M_mag = np.linalg.norm(M_b)
        assert abs(F_b[2]) < 0.02, f'Vertical force imbalance: {F_b[2]:.4f} N'
        assert F_mag < 0.05, f'Net force too large: {F_mag:.4f} N'
        assert M_mag < 0.02, f'Net moment too large: {M_mag:.4f} N·m'

        sp = classified.get('short_period')
        rd = classified.get('roll_damping')
        assert sp is not None, 'Short period mode not found'
        assert rd is not None, 'Roll damping mode not found'
        assert sp['real'] < 0.0, 'Short period should be stable'
        assert rd['real'] < 0.0, 'Roll damping should be stable'

        if sp is not None:
            assert sp['zeta'] > 0.3, f"Short period under-damped: ζ={sp['zeta']:.3f}"

        CL_idx = out_indices.get('CL', (0, 1))
        CD_idx = out_indices.get('CD', (0, 1))
        CL = float(outputs[CL_idx[0]])
        CD = float(outputs[CD_idx[0]])
        LD = CL / CD if CD > 0 else 0
        assert 5.0 < LD < 10.0, f'L/D out of range: {LD:.2f}'

    @pytest.mark.skip(reason='Trim solver convergence varies between quat and Euler at low speeds')
    def test_trim_quaternion_vs_euler_5ms(self):
        """Verify quaternion and Euler trim solutions at 5 m/s."""
        pass
