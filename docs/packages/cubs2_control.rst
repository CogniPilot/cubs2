cubs2_control
=============

Control algorithms for Cubs2 aircraft. These controllers are designed to be reusable across both simulation and real hardware.

For detailed API documentation, see :doc:`../api/control`.

Features
--------

* **Platform-agnostic** - No ROS-specific code in controllers
* **Reusable** - Same algorithms work in simulation and on hardware
* **Well-tested** - Comprehensive unit tests

Controllers
-----------

Autolevel Controller
^^^^^^^^^^^^^^^^^^^^

``autolevel_controller.py`` - Attitude stabilization controller for fixed-wing aircraft.

**Features:**

* Roll (phi) and pitch (theta) stabilization
* Airspeed tracking
* Manual mode pass-through
* Anti-windup for integral terms
* ModelSX-based for composition with aircraft dynamics

**Usage:**

.. code-block:: python

   from cubs2_control.autolevel_controller import autolevel_controller
   from cubs2_dynamics.sportcub import sportcub
   from cyecca.dynamics import ModelSX

   # Create controller
   ctrl = autolevel_controller()

   # Or compose with aircraft for closed-loop simulation
   aircraft = sportcub()
   parent = ModelSX.compose({
       "plant": aircraft,
       "controller": ctrl
   })
   parent.connect("controller.u.q", "plant.x.r")
   parent.connect("plant.u.ail", "controller.y.ail")
   parent.build_composed(integrator="rk4")

Closed-Loop System
^^^^^^^^^^^^^^^^^^

``closed_loop.py`` - Pre-configured closed-loop system combining SportCub aircraft with autolevel controller.

**Features:**

* Single function to create integrated system
* All connections pre-configured
* Manual control inputs preserved
* Mode switching (manual/stabilized)

**Usage:**

.. code-block:: python

   from cubs2_control.closed_loop import closed_loop_sportcub

   # Get ready-to-use closed-loop model
   model = closed_loop_sportcub()

   # Access structured states
   model.x0.plant.p  # Aircraft position
   model.x0.controller.i_p  # Controller integral state

   # Simulate
   x_next = model.f_step(
       x=model._state_to_vec(model.x0),
       u=model.u0.as_vec(),
       p=model.p0.as_vec(),
       dt=0.01
   )

   # Convert back to structured form
   x_struct = model._vec_to_state(x_next["x_next"])

PID Controller
^^^^^^^^^^^^^^

``pid_controller.py`` - Classic PID control with anti-windup.

**Features:**

* Proportional, Integral, Derivative terms
* Anti-windup (integral clamping)
* Derivative filtering
* Configurable gains

**Usage:**

.. code-block:: python

   from cubs2_control.pid_controller import PIDController

   # Create controller
   pid = PIDController(kp=1.0, ki=0.1, kd=0.05, dt=0.01)

   # Set limits
   pid.set_output_limits(-1.0, 1.0)

   # Compute control
   setpoint = 10.0
   measurement = 8.5
   control_output = pid.update(setpoint, measurement)

   # Reset (e.g., on mode switch)
   pid.reset()

Testing
-------

.. code-block:: bash

   # Run controller tests
   colcon test --packages-select cubs2_control
   colcon test-result --verbose

Dependencies
------------

**Package Dependencies:**

* cubs2_dynamics - For dynamics models used in testing
* cyecca - Dynamics modeling framework

**Build Dependencies:**

* ament_cmake
* ament_cmake_python

Design Philosophy
-----------------

Control algorithms in this package:

1. **Have no ROS dependencies** - Can be used outside ROS
2. **Are pure Python** - Easy to test and verify
3. **Use composition** - Controllers can be combined and tested independently
4. **Have clear interfaces** - Simple input/output relationships

This allows the same control code to run in:

* Simulation (``cubs2_simulation``)
* Hardware platforms
* MATLAB/Simulink
* Standalone test scripts
