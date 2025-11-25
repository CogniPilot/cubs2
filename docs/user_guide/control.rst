Control
=======

Cubs2 provides attitude stabilization and manual control for fixed-wing aircraft.

Controller Architecture
------------------------

Autolevel Controller
^^^^^^^^^^^^^^^^^^^^

The primary controller stabilizes roll and pitch angles while tracking airspeed.

**Control Loops:**

* **Roll (φ)** - PID control with anti-windup
* **Pitch (θ)** - PID control with anti-windup  
* **Airspeed (V)** - Throttle control

**Mode Switching:**

* **Manual Mode** - Direct pass-through of pilot commands
* **Stabilized Mode** - Attitude stabilization with pilot rate commands
* **Auto Mode** - Full attitude hold

Usage Example
^^^^^^^^^^^^^

.. code-block:: python

   from cubs2_control.autolevel_controller import autolevel_controller
   from cubs2_dynamics.sportcub import sportcub
   from cyecca.dynamics import ModelSX

   # Create controller
   ctrl = autolevel_controller()

   # Compose with aircraft
   aircraft = sportcub()
   system = ModelSX.compose({
       "plant": aircraft,
       "controller": ctrl
   })

   # Connect signals
   system.connect("controller.u.q", "plant.x.r")
   system.connect("plant.u.ail", "controller.y.ail")
   system.build_composed(integrator="rk4")

Tuning
------

PID Gains
^^^^^^^^^

Edit ``cubs2_data/config/control.yaml``:

.. code-block:: yaml

   autolevel:
     roll:
       kp: 1.0
       ki: 0.1
       kd: 0.05
     pitch:
       kp: 0.8
       ki: 0.1
       kd: 0.04

Anti-Windup
^^^^^^^^^^^

Integral limits prevent windup:

.. code-block:: python

   pid = PIDController(kp=1.0, ki=0.1, kd=0.05, dt=0.01)
   pid.set_output_limits(-1.0, 1.0)  # Clamp output
   pid.set_integral_limits(-0.5, 0.5)  # Limit integral term

Manual Control
--------------

Gamepad
^^^^^^^

Connect a gamepad and launch:

.. code-block:: bash

   ros2 launch cubs2_bringup gamepad_control.xml

**Default Mapping:**

* Left stick: Throttle (Y), Rudder (X)
* Right stick: Pitch (Y), Roll (X)

Virtual Joystick
^^^^^^^^^^^^^^^^

Use the RViz Joy Panel:

1. Launch RViz: ``ros2 launch cubs2_bringup viz.xml``
2. Use virtual sticks in Joy Panel
3. Enable/disable to switch between manual and auto

Control Topics
--------------

Published
^^^^^^^^^

* ``/control`` (cubs2_msgs/AircraftControl) - Control commands

Subscribed
^^^^^^^^^^

* ``/sportcub/pose`` - Aircraft state (for feedback)
* ``/sportcub/velocity`` - Velocity (for airspeed control)

Advanced Usage
--------------

Custom Controllers
^^^^^^^^^^^^^^^^^^

Create new controllers inheriting from ModelSX:

.. code-block:: python

   from cyecca.dynamics import ModelSX
   from dataclasses import dataclass

   @dataclass
   class MyControllerState:
       integral_error: float = 0.0

   @dataclass
   class MyControllerInput:
       setpoint: float = 0.0
       measurement: float = 0.0

   @dataclass
   class MyControllerOutput:
       command: float = 0.0

   def my_controller() -> ModelSX:
       # Define controller dynamics
       ...
       return model

Closed-Loop Analysis
^^^^^^^^^^^^^^^^^^^^

Linearize the closed-loop system:

.. code-block:: python

   from cyecca.dynamics.linearize import linearize_dynamics
   from cubs2_control.closed_loop import closed_loop_sportcub

   # Get closed-loop model
   model = closed_loop_sportcub()

   # Linearize around trim
   A, B = linearize_dynamics(model, trim_state, trim_input)

   # Analyze eigenvalues
   import numpy as np
   eigenvalues = np.linalg.eigvals(A)

See Also
--------

* :doc:`simulation` - Simulation details
* :doc:`../packages/cubs2_control` - Control module reference
* :doc:`../api/control` - Control API
