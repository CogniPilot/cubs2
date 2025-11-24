"""Tests for hierarchical model composition."""

import casadi as ca
import numpy as np
import pytest
from cubs2_control.pid_controller import pid_controller
from cubs2_dynamics.model import ModelSX, input_var, output_var, param, state, symbolic
from cubs2_dynamics.sportcub import sportcub


def test_add_submodel():
    """Test adding submodels to a parent model."""

    @symbolic
    class ParentStates:
        pass

    @symbolic
    class ParentInputs:
        cmd: ca.SX = input_var(desc="command")

    @symbolic
    class ParentParams:
        pass

    parent = ModelSX.create(ParentStates, ParentInputs, ParentParams)

    # Create submodels
    aircraft = sportcub()
    controller = pid_controller()

    # Add submodels
    parent.add_submodel("aircraft", aircraft)
    parent.add_submodel("controller", controller)

    # Verify submodels were added
    assert hasattr(parent, "_submodels")
    assert "aircraft" in parent._submodels
    assert "controller" in parent._submodels


def test_build_composed_simple():
    """Test building a composed model with simple connections."""

    @symbolic
    class ParentStates:
        pass

    @symbolic
    class ParentInputs:
        roll_cmd: ca.SX = input_var(1, 0.0, desc="roll command")
        pitch_cmd: ca.SX = input_var(1, 0.0, desc="pitch command")
        yaw_cmd: ca.SX = input_var(1, 0.0, desc="yaw command")
        speed_cmd: ca.SX = input_var(1, 3.0, desc="speed command")

    @symbolic
    class ParentParams:
        pass

    parent = ModelSX.create(ParentStates, ParentInputs, ParentParams)

    # Create simple submodels
    controller = pid_controller()

    # Add controller with parent input connections
    parent.add_submodel(
        "controller",
        controller,
        input_connections={
            "controller.roll_ref": "u.roll_cmd",
            "controller.pitch_ref": "u.pitch_cmd",
            "controller.yaw_ref": "u.yaw_cmd",
            "controller.speed_ref": "u.speed_cmd",
        },
    )

    # Build composed model
    parent.build_composed(integrator="euler")

    # Verify composed model was built
    assert hasattr(parent, "f_x")
    assert hasattr(parent, "f_step")
    assert hasattr(parent, "_composed")
    assert parent._composed == True
    assert parent._total_composed_states == 4  # Controller has 4 states


def test_simulate_composed():
    """Test simulating a composed model."""

    @symbolic
    class ParentStates:
        pass

    @symbolic
    class ParentInputs:
        roll_cmd: ca.SX = input_var(1, 0.0, desc="roll command")
        pitch_cmd: ca.SX = input_var(1, 0.0, desc="pitch command")
        yaw_cmd: ca.SX = input_var(1, 0.0, desc="yaw command")
        speed_cmd: ca.SX = input_var(1, 3.0, desc="speed command")

    @symbolic
    class ParentParams:
        pass

    parent = ModelSX.create(ParentStates, ParentInputs, ParentParams)

    controller = pid_controller()

    parent.add_submodel(
        "controller",
        controller,
        input_connections={
            "controller.roll_ref": "u.roll_cmd",
            "controller.pitch_ref": "u.pitch_cmd",
            "controller.yaw_ref": "u.yaw_cmd",
            "controller.speed_ref": "u.speed_cmd",
        },
    )

    parent.build_composed(integrator="euler")

    # Simulate
    result = parent.simulate(t0=0.0, tf=1.0, dt=0.1)

    # Verify results
    assert "t" in result
    assert "x" in result
    assert result["t"].shape[0] == result["x"].shape[1]
    assert result["x"].shape[0] == 4  # 4 controller states
    assert np.all(np.isfinite(result["x"]))


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
