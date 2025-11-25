Testing
=======

Cubs2 uses comprehensive testing at multiple levels to ensure code quality.

Test Organization
-----------------

Directory Structure
^^^^^^^^^^^^^^^^^^^

.. code-block:: text

   cubs2_dynamics/
     test/
       test_sportcub.py
       test_trim.py
   
   cubs2_control/
     test/
       test_autolevel.py
       test_pid.py
       test_closed_loop.py
   
   cubs2_planning/
     test/
       test_dubins.py

Running Tests
-------------

All Tests
^^^^^^^^^

.. code-block:: bash

   colcon test
   colcon test-result --verbose

Single Package
^^^^^^^^^^^^^^

.. code-block:: bash

   colcon test --packages-select cubs2_dynamics
   colcon test-result --verbose

Specific Test File
^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   cd src/cubs2/cubs2_dynamics
   pytest test/test_sportcub.py

   # Run specific test
   pytest test/test_sportcub.py::test_trim_computation

With Coverage
^^^^^^^^^^^^^

.. code-block:: bash

   colcon test --pytest-args --cov=cubs2_dynamics --cov-report=html
   firefox build/cubs2_dynamics/coverage_html/index.html

Unit Tests
----------

Dynamics Tests
^^^^^^^^^^^^^^

Test aircraft dynamics correctness:

.. code-block:: python

   def test_trim_computation():
       """Verify trim state for level flight."""
       from cubs2_dynamics.trim_fixed_wing import compute_trim
       
       trim_state, trim_control = compute_trim(
           velocity=20.0,
           altitude=100.0,
           gamma=0.0  # level flight
       )
       
       # Vertical velocity should be near zero
       assert abs(trim_state.v[2]) < 0.02
       
       # Angular rates should be near zero
       assert np.linalg.norm(trim_state.omega) < 2e-3

Controller Tests
^^^^^^^^^^^^^^^^

Test control algorithm behavior:

.. code-block:: python

   def test_pid_controller():
       """Verify PID response."""
       from cubs2_control.pid_controller import PIDController
       
       pid = PIDController(kp=1.0, ki=0.1, kd=0.05, dt=0.01)
       pid.set_output_limits(-1.0, 1.0)
       
       # Step response
       setpoint = 10.0
       measurement = 0.0
       
       for _ in range(100):
           output = pid.update(setpoint, measurement)
           measurement += output * 0.1  # Simple plant
       
       # Should converge
       assert abs(measurement - setpoint) < 0.5

Planning Tests
^^^^^^^^^^^^^^

Test path generation:

.. code-block:: python

   def test_dubins_path():
       """Verify Dubins path computation."""
       from cubs2_planning.dubins import dubins_path
       import numpy as np
       
       start = (0, 0, 0)
       goal = (100, 0, 0)
       radius = 50.0
       
       length, path_type, params = dubins_path(start, goal, radius)
       
       # Straight line path (LSL or RSR)
       assert path_type in ["LSL", "RSR"]
       assert abs(length - 100.0) < 1.0

Integration Tests
-----------------

Closed-Loop Tests
^^^^^^^^^^^^^^^^^

Test complete system integration:

.. code-block:: python

   def test_closed_loop_stability():
       """Verify closed-loop system is stable."""
       from cubs2_control.closed_loop import closed_loop_sportcub
       from cyecca.dynamics.linearize import linearize_dynamics
       import numpy as np
       
       # Get closed-loop model
       model = closed_loop_sportcub()
       
       # Linearize around trim
       A, B = linearize_dynamics(model, trim_state, trim_input)
       
       # Check stability (all eigenvalues negative real part)
       eigenvalues = np.linalg.eigvals(A)
       assert all(e.real < 0 for e in eigenvalues)

Simulation Tests
^^^^^^^^^^^^^^^^

Test ROS nodes:

.. code-block:: python

   def test_simulation_node():
       """Verify simulation node publishes correctly."""
       import rclpy
       from cubs2_simulation.sim import SimulationNode
       
       rclpy.init()
       node = SimulationNode()
       
       # Run for a few steps
       for _ in range(10):
           rclpy.spin_once(node, timeout_sec=0.1)
       
       # Should have published pose
       assert node.get_pose() is not None
       
       rclpy.shutdown()

Property-Based Testing
----------------------

Using Hypothesis
^^^^^^^^^^^^^^^^

Generate random test cases:

.. code-block:: python

   from hypothesis import given
   from hypothesis.strategies import floats
   
   @given(
       velocity=floats(min_value=15.0, max_value=30.0),
       altitude=floats(min_value=50.0, max_value=200.0)
   )
   def test_trim_always_converges(velocity, altitude):
       """Trim should converge for all valid inputs."""
       from cubs2_dynamics.trim_fixed_wing import compute_trim
       
       trim_state, trim_control = compute_trim(
           velocity=velocity,
           altitude=altitude,
           gamma=0.0
       )
       
       # Should have found a solution
       assert trim_state is not None
       assert trim_control is not None

Regression Tests
----------------

Baseline Comparisons
^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   def test_trim_regression():
       """Ensure trim results haven't changed."""
       from cubs2_dynamics.trim_fixed_wing import compute_trim
       
       trim_state, trim_control = compute_trim(
           velocity=20.0,
           altitude=100.0,
           gamma=0.0
       )
       
       # Compare to known good values
       expected_elevator = -0.08
       assert abs(trim_control.elevator - expected_elevator) < 0.01

Continuous Integration
----------------------

GitHub Actions
^^^^^^^^^^^^^^

Tests run automatically on:

* Every push
* Every pull request
* Scheduled (nightly)

.. code-block:: yaml

   # .github/workflows/ci.yml
   name: CI
   on: [push, pull_request]
   
   jobs:
     test:
       runs-on: ubuntu-latest
       steps:
         - uses: actions/checkout@v4
         - name: Run tests
           run: |
             colcon build
             colcon test
             colcon test-result --verbose

Coverage Requirements
^^^^^^^^^^^^^^^^^^^^^

* Minimum 70% overall coverage
* 90% for critical paths (dynamics, control)
* New code must have tests

Debugging Failed Tests
----------------------

Verbose Output
^^^^^^^^^^^^^^

.. code-block:: bash

   pytest -v -s test/test_sportcub.py

With PDB
^^^^^^^^

.. code-block:: bash

   pytest --pdb test/test_sportcub.py

Logging
^^^^^^^

.. code-block:: python

   import logging
   logging.basicConfig(level=logging.DEBUG)

Performance Testing
-------------------

Benchmarks
^^^^^^^^^^

.. code-block:: python

   import pytest
   
   def test_simulation_performance(benchmark):
       """Benchmark simulation step."""
       from cubs2_dynamics.sportcub import SportCubModel
       
       model = SportCubModel()
       x0 = model.get_initial_state()
       u = [0, 0, 0, 0.5]
       
       def step():
           return model.f_step(x0, u, dt=0.01)
       
       result = benchmark(step)
       
       # Should run faster than real-time
       assert result.stats.mean < 0.01

Best Practices
--------------

Test Structure
^^^^^^^^^^^^^^

Follow AAA pattern:

.. code-block:: python

   def test_example():
       # Arrange
       model = create_model()
       initial_state = get_initial_state()
       
       # Act
       result = model.step(initial_state)
       
       # Assert
       assert result.is_valid()

Fixtures
^^^^^^^^

Reuse setup code:

.. code-block:: python

   import pytest
   
   @pytest.fixture
   def sportcub_model():
       from cubs2_dynamics.sportcub import sportcub
       return sportcub()
   
   def test_with_fixture(sportcub_model):
       # Use the fixture
       assert sportcub_model is not None

Parametrize
^^^^^^^^^^^

Test multiple inputs:

.. code-block:: python

   @pytest.mark.parametrize("velocity,expected", [
       (15.0, -0.1),
       (20.0, -0.08),
       (25.0, -0.06),
   ])
   def test_trim_elevator(velocity, expected):
       trim_state, trim_control = compute_trim(
           velocity=velocity,
           altitude=100.0,
           gamma=0.0
       )
       assert abs(trim_control.elevator - expected) < 0.02

See Also
--------

* :doc:`contributing` - Development workflow
* :doc:`architecture` - System design
