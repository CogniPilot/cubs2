Quick Start
===========

This guide will help you get started with Cubs2 simulation and visualization.

Basic Simulation
----------------

Launch a complete simulation with visualization:

.. code-block:: bash

   source install/setup.bash
   ros2 launch cubs2_bringup sim.xml

This will start:

* Physics simulation node (``cubs2_simulation``)
* Aircraft dynamics model (Sport Cub)
* RViz visualization with custom panels
* Planning and control nodes

The aircraft will spawn at the origin and you can control it using the joystick panel in RViz.

Visualization Only
------------------

To launch just the visualization (e.g., for connecting to a real aircraft):

.. code-block:: bash

   ros2 launch cubs2_bringup viz.xml

Replay Mode
-----------

Cubs2 supports replaying recorded flight data and comparing it with simulation:

.. code-block:: bash

   ros2 launch cubs2_bringup sim.xml replay_mode:=true

This is useful for:

* Debugging flight test data
* Validating simulation accuracy
* Comparing controller performance

Available Topics
----------------

Key ROS 2 topics published by Cubs2:

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Topic
     - Type
     - Description
   * - ``/state/full``
     - ``cubs2_msgs/FullState``
     - Complete aircraft state
   * - ``/odometry``
     - ``nav_msgs/Odometry``
     - Position and velocity
   * - ``/joy``
     - ``sensor_msgs/Joy``
     - Joystick input
   * - ``/cmd_vel``
     - ``geometry_msgs/Twist``
     - Velocity commands
   * - ``/path``
     - ``nav_msgs/Path``
     - Planned trajectory
   * - ``/control/actuators``
     - ``cubs2_msgs/Actuators``
     - Control surface positions

Using the Joystick Panel
-------------------------

The RViz joystick panel provides virtual stick control:

1. **Left Stick**: Throttle (vertical), Yaw (horizontal)
2. **Right Stick**: Pitch (vertical), Roll (horizontal)
3. **Trim Buttons**: Fine-tune neutral positions
4. **Enable Switch**: Activate/deactivate control

.. note::
   You can also use a physical joystick by mapping it to ``/joy`` topic.

Adjusting Simulation Parameters
--------------------------------

Edit configuration files in ``cubs2_data/config/``:

* ``sportcub.yaml``: Aircraft parameters (mass, inertia, aerodynamics)
* ``sim.yaml``: Simulation settings (timestep, initial conditions)
* ``control.yaml``: Controller gains and limits

After editing, rebuild and relaunch:

.. code-block:: bash

   colcon build --symlink-install --packages-select cubs2_data
   source install/setup.bash
   ros2 launch cubs2_bringup sim.xml

Next Steps
----------

* :doc:`simulation` - Learn about the physics simulation
* :doc:`control` - Understand the control architecture
* :doc:`planning` - Generate paths and trajectories
* :doc:`visualization` - Customize RViz panels
