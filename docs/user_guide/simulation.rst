Simulation
==========

Cubs2 provides physics-based simulation of fixed-wing aircraft using CasADi for automatic differentiation.

Architecture
------------

The simulation architecture separates concerns into three layers:

1. **Dynamics Layer** (``cubs2_dynamics``)
   
   * Pure Python dynamics models
   * No ROS dependencies
   * CasADi-based symbolic differentiation
   * Hierarchical model composition

2. **Control Layer** (``cubs2_control``)
   
   * Reusable control algorithms
   * Platform-agnostic implementation
   * Composition-ready with dynamics

3. **Runtime Layer** (``cubs2_simulation``)
   
   * ROS2 integration
   * Message publishing/subscribing
   * Real-time or accelerated execution

Physics Model
-------------

SportCub Dynamics
^^^^^^^^^^^^^^^^^

The SportCub model includes:

* **6-DOF rigid body dynamics** - Position, velocity, orientation, angular rates
* **Aerodynamics** - Lift, drag, side force, moments
* **Propulsion** - Propeller thrust model
* **Ground contact** - Landing gear forces

State Vector
^^^^^^^^^^^^

.. math::

   x = [p_x, p_y, p_z, v_x, v_y, v_z, q_w, q_x, q_y, q_z, \\omega_x, \\omega_y, \\omega_z]^T

Where:

* :math:`p` - Position (NED frame)
* :math:`v` - Velocity (body frame)
* :math:`q` - Quaternion orientation
* :math:`\\omega` - Angular velocity (body frame)

Control Inputs
^^^^^^^^^^^^^^

.. math::

   u = [\\delta_a, \\delta_e, \\delta_r, \\delta_t]^T

Where:

* :math:`\\delta_a` - Aileron deflection [-1, 1]
* :math:`\\delta_e` - Elevator deflection [-1, 1]
* :math:`\\delta_r` - Rudder deflection [-1, 1]
* :math:`\\delta_t` - Throttle [0, 1]

Integration
-----------

Numerical Methods
^^^^^^^^^^^^^^^^^

Cubs2 uses Runge-Kutta 4th order (RK4) integration:

.. code-block:: python

   from cyecca.dynamics.integrators import rk4

   # Simulate one timestep
   x_next = rk4(dynamics_func, x0, dt)

Timestep Selection
^^^^^^^^^^^^^^^^^^

Recommended timesteps:

* **Standard**: 0.01s (100 Hz) - Good balance
* **High accuracy**: 0.001s (1000 Hz) - For sensitive analysis
* **Fast preview**: 0.05s (20 Hz) - Quick visualization

Adjust via ROS parameter:

.. code-block:: bash

   ros2 param set /sim dt 0.001

Real-Time vs Accelerated
^^^^^^^^^^^^^^^^^^^^^^^^^

Control simulation speed:

.. code-block:: bash

   # 2x real-time (faster than reality)
   ros2 param set /sim rate 2.0

   # 0.5x real-time (slow motion)
   ros2 param set /sim rate 0.5

Configuration
-------------

Aircraft Parameters
^^^^^^^^^^^^^^^^^^^

Edit ``cubs2_data/config/sportcub.yaml``:

.. code-block:: yaml

   mass: 11.0  # kg
   inertia:
     Ixx: 0.5
     Iyy: 0.8
     Izz: 1.2
   aerodynamics:
     S_wing: 0.5  # m^2
     c_bar: 0.2   # m
     b: 2.5       # m

Initial Conditions
^^^^^^^^^^^^^^^^^^

Edit ``cubs2_data/config/sim.yaml``:

.. code-block:: yaml

   initial_state:
     position: [0, 0, -100]  # NED (meters)
     velocity: [20, 0, 0]    # Body frame (m/s)
     orientation: [1, 0, 0, 0]  # Quaternion
     angular_velocity: [0, 0, 0]  # rad/s

Visualization
-------------

RViz Topics
^^^^^^^^^^^

The simulation publishes:

* ``/sportcub/pose`` - Position and orientation
* ``/sportcub/velocity`` - Body-frame velocities
* ``/sportcub/imu`` - IMU measurements
* ``/clock`` - Simulation time

3D Model
^^^^^^^^

Aircraft mesh and URDF are in ``cubs2_description``.

Troubleshooting
---------------

Simulation Unstable
^^^^^^^^^^^^^^^^^^^

* Reduce timestep: ``ros2 param set /sim dt 0.001``
* Check initial conditions (trim flight)
* Verify control limits

Poor Performance
^^^^^^^^^^^^^^^^

* Increase timestep (within stability limits)
* Disable unnecessary visualization
* Run without GUI (headless)

See Also
--------

* :doc:`control` - Control algorithms
* :doc:`planning` - Path planning
* :doc:`../packages/cubs2_dynamics` - Dynamics module details
