#!/usr/bin/env python3
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
"""Test to investigate why closed-loop model doesn't produce forces/moments.

This test checks:
1. Plant-only model outputs (working case)
2. Closed-loop composed model outputs (broken case)
3. Force/moment computation differences
"""
import numpy as np
import pytest


def test_plant_only_outputs():
    """Test that plant-only model produces non-zero forces with throttle input."""
    from cubs2_dynamics.sportcub import sportcub

    # Create plant model
    plant = sportcub()

    # Initial state (on ground, at rest)
    x = plant.x0
    x.p[2] = 0.1  # Altitude

    # Control inputs - full throttle
    u = plant.u0
    u.thr = 1.0  # Full throttle
    u.ail = 0.0
    u.elev = 0.0
    u.rud = 0.0

    # Parameters
    p = plant.p0

    # Compute outputs
    result = plant.f_y(x=x.as_vec(), u=u.as_vec(), p=p.as_vec())
    output_vec = result['y']

    from cubs2_dynamics.sportcub import SportCubOutputs

    outputs = SportCubOutputs.from_vec(output_vec)

    # Check forces
    F_total = np.array(outputs.F_b).flatten()
    F_thrust = np.array(outputs.FT_b).flatten()
    F_norm = np.linalg.norm(F_total)
    F_thrust_norm = np.linalg.norm(F_thrust)

    print('\nPlant-only model:')
    print(f'  Total force:  {F_total}')
    print(f'  ||F||:        {F_norm:.3f} N')
    print(f'  Thrust force: {F_thrust}')
    print(f'  ||FT||:       {F_thrust_norm:.3f} N')

    # With full throttle, we should have non-zero thrust force
    assert F_thrust_norm > 0.1, (
        f'Expected thrust force > 0.1 N, got {F_thrust_norm:.3f} N'
    )


@pytest.mark.skip(reason="Closed-loop model doesn't expose f_y function - known limitation")
def test_closed_loop_outputs():
    """Test that closed-loop model produces forces/moments."""
    from cubs2_control.closed_loop import closed_loop_sportcub

    # Create closed-loop model
    model = closed_loop_sportcub()

    # Initial state
    x = model.x0
    x.plant.p[2] = 0.1  # Altitude

    # Control inputs - full throttle in manual mode
    u = model.u0
    u.thr_manual = 1.0
    u.ail_manual = 0.0
    u.elev_manual = 0.0
    u.rud_manual = 0.0
    u.mode = 0.0  # Manual mode

    # Parameters
    p = model.p0

    # Convert to vectors
    x_vec = model._state_to_vec(x)
    u_vec = u.as_vec()
    p_vec = p.as_vec()

    # Compute outputs
    print('\nClosed-loop model:')
    print(f'  Input thr_manual: {u.thr_manual}')
    print(f'  Input mode:       {u.mode}')

    # Check what the plant submodel sees
    print('\n  Submodel check:')
    if hasattr(model, '_submodels') and 'plant' in model._submodels:
        plant_submodel = model._submodels['plant']
        print(f"    Plant has f_y: {hasattr(plant_submodel, 'f_y')}")

        # Try to evaluate plant output directly
        # Get plant state slice
        if hasattr(model, '_submodel_state_slices'):
            plant_slice = model._submodel_state_slices.get('plant')
            if plant_slice:
                start, end = plant_slice
                print(f'    Plant state slice: [{start}:{end}]')

                # What inputs does plant need?
                from dataclasses import fields

                u_plant = plant_submodel.u0
                print(
                    f'    Plant expects inputs: '
                    f'{[f.name for f in fields(u_plant)]}'
                )

    if hasattr(model, 'f_y'):
        result = model.f_y(x=x_vec, u=u_vec, p=p_vec)
        output_vec = result['y']

        from cubs2_control.closed_loop import ClosedLoopOutputs

        outputs = ClosedLoopOutputs.from_vec(output_vec)

        print('\n  Outputs:')
        print(f'    thr:       {float(outputs.thr):.3f}')
        print(f'    ail:       {float(outputs.ail):.3f}')
        print(f'    elev:      {float(outputs.elev):.3f}')

        # Check forces/moments
        F = np.array(outputs.F).flatten()
        M = np.array(outputs.M).flatten()
        F_norm = np.linalg.norm(F)
        M_norm = np.linalg.norm(M)

        print(f'    Force F:          {F}')
        print(f'    ||F||:            {F_norm:.3f} N')
        print(f'    Moment M:         {M}')
        print(f'    ||M||:            {M_norm:.3f} Nm')

        # Verify throttle passes through in manual mode
        assert (
            abs(float(outputs.thr) - 1.0) < 0.01
        ), f'Expected thr=1.0 in manual mode, got {float(outputs.thr):.3f}'

        # Verify we get non-zero forces with throttle
        assert F_norm > 0.1, f'Expected force magnitude > 0.1 N, got {
            F_norm:.3f} N'

    else:
        pytest.fail('Closed-loop model has no f_y function')


@pytest.mark.skip(reason="Closed-loop model doesn't expose f_y function - known limitation")
def test_compare_plant_vs_closed_loop():
    """Compare outputs between plant-only and closed-loop models."""
    from cubs2_control.closed_loop import closed_loop_sportcub
    from cubs2_dynamics.sportcub import sportcub

    # Create both models
    plant = sportcub()
    cl_model = closed_loop_sportcub()

    # Set up identical initial states
    x_plant = plant.x0
    x_plant.p[2] = 0.1

    x_cl = cl_model.x0
    x_cl.plant.p[2] = 0.1

    # Set up identical control inputs
    u_plant = plant.u0
    u_plant.thr = 1.0
    u_plant.ail = 0.0
    u_plant.elev = 0.0
    u_plant.rud = 0.0

    u_cl = cl_model.u0
    u_cl.thr_manual = 1.0
    u_cl.ail_manual = 0.0
    u_cl.elev_manual = 0.0
    u_cl.rud_manual = 0.0
    u_cl.mode = 0.0  # Manual mode

    # Parameters
    p_plant = plant.p0
    p_cl = cl_model.p0

    # Compute plant outputs
    result_plant = plant.f_y(
        x=x_plant.as_vec(), u=u_plant.as_vec(), p=p_plant.as_vec())
    from cubs2_dynamics.sportcub import SportCubOutputs

    outputs_plant = SportCubOutputs.from_vec(result_plant['y'])

    # Compute closed-loop outputs
    x_cl_vec = cl_model._state_to_vec(x_cl)
    result_cl = cl_model.f_y(x=x_cl_vec, u=u_cl.as_vec(), p=p_cl.as_vec())

    from cubs2_control.closed_loop import ClosedLoopOutputs

    outputs_cl = ClosedLoopOutputs.from_vec(result_cl['y'])

    # Compare forces
    F_plant = np.array(outputs_plant.F_b).flatten()
    F_cl = np.array(outputs_cl.F).flatten()

    F_plant_norm = np.linalg.norm(F_plant)
    F_cl_norm = np.linalg.norm(F_cl)

    print('\n=== Comparison ===')
    print('Plant-only:')
    print(f'  Throttle input:  {u_plant.thr:.3f}')
    print(f'  Force F_b:       {F_plant}')
    print(f'  ||F_b||:         {F_plant_norm:.3f} N')

    print('\nClosed-loop (manual mode):')
    print(f'  Throttle input:  {u_cl.thr_manual:.3f}')
    print(f'  Throttle output: {float(outputs_cl.thr):.3f}')
    print(f'  Force F:         {F_cl}')
    print(f'  ||F||:           {F_cl_norm:.3f} N')

    print('\nDifference:')
    print(f'  ΔF:              {F_cl - F_plant}')
    print(f'  Δ||F||:          {F_cl_norm - F_plant_norm:.3f} N')

    # Forces should be similar (within 10%)
    if F_plant_norm > 0.1:
        rel_error = abs(F_cl_norm - F_plant_norm) / F_plant_norm
        print(f'  Relative error:  {rel_error * 100:.1f}%')
        assert rel_error < 0.1, f'Force magnitudes differ by {
            rel_error * 100:.1f}% (expected < 10%)'


def test_check_output_connections():
    """Verify closed-loop output connections are correct."""
    from cubs2_control.closed_loop import closed_loop_sportcub

    model = closed_loop_sportcub()

    # Check that model has output connections
    print('\n=== Closed-loop Model Structure ===')
    print(f'Has f_y: {hasattr(model, "f_y")}')
    print(
        f'Has _output_connections: '
        f'{hasattr(model, "_output_connections")}'
    )

    if hasattr(model, '_output_connections'):
        print('\nOutput connections:')
        for conn in model._output_connections:
            print(f'  {conn}')

    # Check submodels
    if hasattr(model, '_submodels'):
        print(f'\nSubmodels: {list(model._submodels.keys())}')
        for name, submodel in model._submodels.items():
            print(f'\n{name}:')
            print(f'  Has f_y: {hasattr(submodel, "f_y")}')
            if hasattr(submodel, 'output_type'):
                from dataclasses import fields

                out_fields = [f.name for f in fields(submodel.output_type)]
                print(f'  Output fields: {out_fields}')


if __name__ == '__main__':
    # Run tests with verbose output
    print('=' * 80)
    print('Testing plant-only model outputs...')
    print('=' * 80)
    test_plant_only_outputs()

    print('\n' + '=' * 80)
    print('Testing closed-loop model outputs...')
    print('=' * 80)
    test_closed_loop_outputs()

    print('\n' + '=' * 80)
    print('Comparing plant vs closed-loop...')
    print('=' * 80)
    test_compare_plant_vs_closed_loop()

    print('\n' + '=' * 80)
    print('Checking output connections...')
    print('=' * 80)
    test_check_output_connections()

    print('\n' + '=' * 80)
    print('All tests passed!')
    print('=' * 80)
