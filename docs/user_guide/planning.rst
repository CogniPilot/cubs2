Planning
========

Cubs2 uses Dubins path planning for autonomous navigation of fixed-wing aircraft.

Dubins Paths
------------

What are Dubins Paths?
^^^^^^^^^^^^^^^^^^^^^^^

Dubins paths are the shortest curves connecting two poses (position + heading) with bounded curvature. They consist of three segments:

* **C** - Circular arc (left or right turn)
* **S** - Straight line

Path Types
^^^^^^^^^^

Six canonical path types exist:

**CSC Paths** (turn-straight-turn):

* **RSR** - Right, Straight, Right
* **RSL** - Right, Straight, Left
* **LSR** - Left, Straight, Right
* **LSL** - Left, Straight, Left

**CCC Paths** (turn-turn-turn):

* **RLR** - Right, Left, Right
* **LRL** - Left, Right, Left

The planner selects the shortest feasible path.

Algorithm
^^^^^^^^^

Based on: Shkel, A. M. and Lumelsky, V. (2001). "Classification of the Dubins set"

Implementation handles:

* All 6 path types
* Singularities (collinear start/goal)
* Path length optimization
* Parameter extraction for trajectory generation

Usage
-----

Python API
^^^^^^^^^^

.. code-block:: python

   from cubs2_planning.dubins import dubins_path
   import numpy as np

   # Define start and goal poses
   start = (0, 0, 0)  # (x, y, heading)
   goal = (100, 50, np.pi/4)
   turning_radius = 50.0

   # Compute path
   path_length, path_type, params = dubins_path(start, goal, turning_radius)

   print(f"Path type: {path_type}")
   print(f"Length: {path_length:.2f} m")

ROS2 Node
^^^^^^^^^

.. code-block:: bash

   # Launch planner
   ros2 run cubs2_planning planner_dubins

   # With custom parameters
   ros2 run cubs2_planning planner_dubins \
     --ros-args -p turning_radius:=75.0 -p cruise_altitude:=20.0

Racecourse Navigation
---------------------

Gate Sequencing
^^^^^^^^^^^^^^^

The planner navigates through a sequence of gates:

.. code-block:: bash

   ros2 launch cubs2_bringup sim.xml gate_sequence:="[0,1,2,3,4,5]"

**Gate Definition:**

Gates are defined in ``racecourse_description/config/racecourse.yaml``:

.. code-block:: yaml

   gates:
     - id: 0
       position: [0, 0, -15]
       orientation: [1, 0, 0, 0]
     - id: 1
       position: [100, 50, -15]
       orientation: [0.707, 0, 0, 0.707]

Visualization
^^^^^^^^^^^^^

In RViz, the planner shows:

* **Green path** - Planned trajectory
* **Colored spheres** - Gate positions
* **Yellow highlight** - Current target gate

Parameters
----------

Turning Radius
^^^^^^^^^^^^^^

Minimum turn radius based on aircraft performance:

.. math::

   R_{min} = \\frac{V^2}{g \\tan(\\phi_{max})}

Where:

* :math:`V` - Airspeed
* :math:`g` - Gravity (9.81 m/s²)
* :math:`\\phi_{max}` - Maximum bank angle

**Example:** For V=20 m/s, φ_max=45°:

.. math::

   R_{min} = \\frac{20^2}{9.81 \\times \\tan(45°)} \\approx 40.8 \\text{ m}

Set via parameter:

.. code-block:: bash

   ros2 param set /planner_dubins turning_radius 50.0

Cruise Altitude
^^^^^^^^^^^^^^^

Target flight altitude:

.. code-block:: bash

   ros2 param set /planner_dubins cruise_altitude 20.0

Path Following
--------------

Published Topics
^^^^^^^^^^^^^^^^

* ``/planned_path`` (nav_msgs/Path) - Waypoints along trajectory
* ``/waypoint_markers`` (visualization_msgs/MarkerArray) - Gate visualization

Subscribed Topics
^^^^^^^^^^^^^^^^^

* ``/sportcub/pose`` (geometry_msgs/PoseStamped) - Aircraft position

Control Integration
^^^^^^^^^^^^^^^^^^^^

The path planner publishes waypoints. A separate guidance controller (not yet implemented) would:

1. Track the path using cross-track error
2. Compute desired heading
3. Command roll/pitch to follow

Advanced Usage
--------------

Custom Path Generation
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   from cubs2_planning.dubins import DubinsPath

   # Create path object
   path = DubinsPath(start, goal, turning_radius)

   # Sample along path
   num_points = 100
   points = [path.sample(i / num_points) for i in range(num_points)]

   # Get tangent at point
   tangent = path.tangent(0.5)  # At midpoint

Obstacle Avoidance
^^^^^^^^^^^^^^^^^^

For obstacle avoidance, combine Dubins paths with:

* RRT (Rapidly-exploring Random Trees)
* A* search on Dubins graph
* Potential fields

See Also
--------

* :doc:`simulation` - Physics simulation
* :doc:`control` - Flight control
* :doc:`../packages/cubs2_planning` - Planning module reference
