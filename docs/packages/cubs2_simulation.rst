cubs2_simulation
================

Simulation runtime nodes for the Cubs2 SportCub aircraft. This package contains the ROS2-specific simulation nodes that integrate dynamics and control.

For detailed API documentation, see :doc:`../api/simulation`.

Nodes
-----

sim.py
^^^^^^

Main aircraft simulation node.

**Published Topics:**

* ``/sportcub/pose`` (geometry_msgs/PoseStamped) - Aircraft position and orientation
* ``/sportcub/velocity`` (geometry_msgs/TwistStamped) - Body-frame velocities
* ``/sportcub/imu`` (sensor_msgs/Imu) - IMU measurements
* ``/clock`` (rosgraph_msgs/Clock) - Simulation time

**Subscribed Topics:**

* ``/control`` (cubs2_msgs/AircraftControl) - Control inputs
* ``/reset`` (std_msgs/Empty) - Reset simulation
* ``/pause`` (std_msgs/Empty) - Pause/unpause simulation
* ``/set_speed`` (std_msgs/Float64) - Set simulation speed multiplier
* ``/set_dt`` (std_msgs/Float64) - Set integration time step

**Parameters:**

* ``rate`` (float, default: 1.0) - Real-time speed multiplier
* ``dt`` (float, default: 0.01) - Integration time step (seconds)

pose_to_tf.py
^^^^^^^^^^^^^

Publishes TF transforms from pose messages.

**Subscribed Topics:**

* ``/sportcub/pose`` (geometry_msgs/PoseStamped)

**Published:**

* TF transform from ``map`` to ``vehicle`` frame

gamepad_control.py
^^^^^^^^^^^^^^^^^^

Gamepad/joystick control interface.

**Subscribed Topics:**

* ``/joy`` (sensor_msgs/Joy)

**Published Topics:**

* ``/control`` (cubs2_msgs/AircraftControl)

**Features:**

* Configurable button/axis mappings
* Dead-zone handling
* Exponential stick response

Usage
-----

Launch Simulation
^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 run cubs2_simulation sim

With Launch File
^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 launch cubs2_bringup sim.xml

Gamepad Control
^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 launch cubs2_bringup gamepad_control.xml

Configuration
-------------

Simulation Parameters
^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # 2x speed simulation
   ros2 param set /sim rate 2.0

   # Finer time step for accuracy
   ros2 param set /sim dt 0.001

   # Reset to initial conditions
   ros2 topic pub /reset std_msgs/Empty

Gamepad Mapping
^^^^^^^^^^^^^^^

Edit gamepad configuration in ``cubs2_bringup/launch/gamepad_control.xml``:

.. code-block:: xml

   <param name="axis_aileron" value="0"/>
   <param name="axis_elevator" value="1"/>
   <param name="axis_throttle" value="2"/>
   <param name="axis_rudder" value="3"/>

Testing
-------

.. code-block:: bash

   colcon test --packages-select cubs2_simulation
   colcon test-result --verbose

Dependencies
------------

**ROS2 Packages:**

* rclpy
* std_msgs
* geometry_msgs
* sensor_msgs
* visualization_msgs
* tf2_ros

**Cubs2 Packages:**

* cubs2_msgs - Control message definitions
* cubs2_dynamics - Aircraft dynamics models
* cubs2_control - Control algorithms

**Python Libraries:**

* numpy

Architecture
------------

Simulation nodes use a composition-based architecture:

1. **Dynamics Layer** (``cubs2_dynamics``) - Pure dynamics, no ROS
2. **Control Layer** (``cubs2_control``) - Reusable controllers
3. **Runtime Layer** (this package) - ROS2 integration

This separation allows dynamics and control to be tested independently and reused on hardware.
