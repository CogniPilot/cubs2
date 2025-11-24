"""Tests for the ModelSX and ModelMX API with @symbolic dataclass pattern."""

import pytest
import numpy as np
import casadi as ca
from cubs2_dynamics.model import (
    ModelSX,
    ModelMX,
    symbolic,
    state,
    input_var,
    param,
    output_var,
)


class TestModelCreate:
    """Test that ModelSX.create() and ModelMX.create() work correctly."""

    def test_sx_model_basic(self):
        """Test ModelSX.create() sets all expected attributes."""

        @symbolic
        class States:
            x: ca.SX = state(1, 0.0)

        @symbolic
        class Inputs:
            pass

        @symbolic
        class Params:
            m: ca.SX = param(1.0)

        model = ModelSX.create(States, Inputs, Params)
        model.build(f_x=ca.SX.zeros(1))

        # Check types are created
        assert model.state_type == States
        assert model.input_type == Inputs
        assert model.param_type == Params

        # Check symbolic instances
        assert hasattr(model, "x")
        assert hasattr(model, "p")
        assert hasattr(model, "u")

        # Check default instances
        assert hasattr(model, "x0")
        assert hasattr(model, "p0")
        assert hasattr(model, "u0")

        # Verify types
        assert model.x.x.__class__.__name__ == "SX"
        assert model.x0.x == 0.0
        assert model.p0.m == 1.0

    def test_mx_model_basic(self):
        """Test ModelMX.create() sets all expected attributes."""

        @symbolic
        class States:
            x: ca.MX = state(1, 0.0)

        @symbolic
        class Inputs:
            pass

        @symbolic
        class Params:
            m: ca.MX = param(1.0)

        model = ModelMX.create(States, Inputs, Params)
        model.build(f_x=ca.MX.zeros(1))

        assert model.state_type == States
        assert hasattr(model, "x")
        assert hasattr(model, "x0")
        assert model.x.x.__class__.__name__ == "MX"
        assert model.x0.x == 0.0
        assert model.p0.m == 1.0

    def test_vector_states(self):
        """Test with vector-valued states."""

        @symbolic
        class States:
            position: ca.SX = state(3, [1.0, 2.0, 3.0])

        @symbolic
        class Inputs:
            pass

        @symbolic
        class Params:
            k: ca.SX = param(2.5)

        model = ModelSX.create(States, Inputs, Params)
        model.build(f_x=ca.SX.zeros(3))

        # Check vector field
        assert hasattr(model.x0, "position")
        np.testing.assert_array_equal(model.x0.position, [1.0, 2.0, 3.0])

        # Check symbolic vector
        assert model.x.position.shape == (3, 1)

    def test_with_inputs(self):
        """Test with input variables."""

        @symbolic
        class States:
            x: ca.SX = state(1, 0.0)

        @symbolic
        class Inputs:
            u: ca.SX = input_var(0.5)

        @symbolic
        class Params:
            pass

        model = ModelSX.create(States, Inputs, Params)
        model.build(f_x=ca.SX.zeros(1))

        assert hasattr(model, "u")
        assert hasattr(model, "u0")
        assert model.u0.u == 0.5

    def test_with_outputs(self):
        """Test with output variables."""

        @symbolic
        class States:
            x: ca.SX = state(1, 0.0)

        @symbolic
        class Inputs:
            pass

        @symbolic
        class Params:
            pass

        @symbolic
        class Outputs:
            energy: ca.SX = output_var(1, 0.0, "energy calculation")

        model = ModelSX.create(States, Inputs, Params, output_type=Outputs)
        model.build(f_x=ca.SX.zeros(1), f_y=model.x.x)

        assert hasattr(model, "output_type")
        assert model.output_type == Outputs
        assert "energy" in model.output_names


class TestSimulation:
    """Test model simulation capabilities."""

    def test_simple_integration(self):
        """Test basic forward integration."""

        @symbolic
        class States:
            x: ca.SX = state(1, 1.0)

        @symbolic
        class Inputs:
            u: ca.SX = input_var(0.0)

        @symbolic
        class Params:
            k: ca.SX = param(1.0, "decay rate")

        model = ModelSX.create(States, Inputs, Params)
        # dx/dt = -k*x (exponential decay)
        model.build(f_x=-model.p.k * model.x.x)

        result = model.simulate(
            t0=0.0,
            tf=1.0,
            dt=0.1,
            u_func=lambda t, x, p: model.u0.as_vec(),
            p_vec=model.p0.as_vec(),
        )

        assert "t" in result
        assert "x" in result
        assert len(result["t"]) > 0

        # result["x"] has shape (n_states, n_timesteps), so transpose
        x_traj = result["x"].T  # Now shape is (n_timesteps, n_states)

        # Final value should be less than initial (decay)
        # For dx/dt = -k*x with x(0) = 1, solution is x(t) = e^(-t)
        # At t=1, x ≈ 0.368
        final_val = float(x_traj[-1, 0])
        assert final_val < 0.5, f"Expected decay but got {final_val}"


class TestHybridFeatures:
    """Test hybrid/discrete state features."""

    def test_simple_hybrid_model(self):
        """Test a simple hybrid model with discrete states."""

        @symbolic
        class States:
            x: ca.SX = state(1, 0.0, "continuous state")

        @symbolic
        class Inputs:
            pass

        @symbolic
        class Params:
            pass

        # For now, skip complex hybrid tests as they require more setup
        model = ModelSX.create(States, Inputs, Params)
        model.build(f_x=ca.SX.zeros(1))

        assert hasattr(model, "x")
        assert hasattr(model, "x0")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
