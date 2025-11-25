Quick Start
===========

This guide will help you get started with Cubs2 simulation and visualization.

Basic Simulation
----------------

Launch a complete simulation with visualization in a single command:

.. code-block:: bash

   source install/setup.bash
   ros2 launch cubs2_bringup sim.xml

This will start:

* Physics simulation node (``cubs2_simulation``)
* Aircraft dynamics model (Sport Cub)
* RViz visualization with custom panels
* Planning and control nodes

The aircraft will spawn at the origin and you can control it using the joystick panel in RViz.

Launch Options
--------------

The ``sim.xml`` launch file supports several options:

.. code-block:: bash

   ros2 launch cubs2_bringup sim.xml replay:=false viz:=false gamepad:=true

Available arguments:

* ``replay`` (default: ``true``) - Enable replay of recorded flight data
* ``viz`` (default: ``true``) - Launch RViz visualization
* ``gamepad`` (default: ``false``) - Enable gamepad control
* ``bag_path`` - Path to rosbag file for replay mode

Examples:

.. code-block:: bash

   # Simulation only (no visualization)
   ros2 launch cubs2_bringup sim.xml viz:=false

   # With gamepad control
   ros2 launch cubs2_bringup sim.xml gamepad:=true

   # Disable replay
   ros2 launch cubs2_bringup sim.xml replay:=false

Visualization Only
------------------

To launch just the visualization (e.g., for connecting to a real aircraft):

.. code-block:: bash

   ros2 launch cubs2_bringup viz.xml

.. note::
   With the new defaults, you typically don't need separate launch calls.
   Use ``ros2 launch cubs2_bringup sim.xml`` to start everything together.

Replay Mode
-----------

Cubs2 supports replaying recorded flight data and comparing it with simulation.
Replay mode is **enabled by default**:

.. code-block:: bash

   ros2 launch cubs2_bringup sim.xml

To disable replay:

.. code-block:: bash

   ros2 launch cubs2_bringup sim.xml replay:=false

The replay shows a "ghost" plane following the recorded trajectory, allowing you to:

* Debug flight test data
* Validate simulation accuracy
* Compare controller performance
* Analyze flight patterns

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
