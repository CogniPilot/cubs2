Architecture
============

Cubs2 follows a layered architecture separating dynamics, control, and ROS integration.

System Overview
---------------

.. code-block:: text

   ┌─────────────────────────────────────────────┐
   │          cubs2_bringup (Launch)             │
   └──────────────┬──────────────────────────────┘
                  │
   ┌──────────────┴──────────────────────────────┐
   │                                              │
   │  cubs2_simulation (ROS2 Runtime)            │
   │  ┌───────────────────────────────────────┐  │
   │  │  cubs2_control (Controllers)          │  │
   │  │  ┌────────────────────────────────┐   │  │
   │  │  │  cubs2_dynamics (Physics)      │   │  │
   │  │  │  ┌─────────────────────────┐   │   │  │
   │  │  │  │  cyecca (Core Models)   │   │   │  │
   │  │  │  └─────────────────────────┘   │   │  │
   │  │  └────────────────────────────────┘   │  │
   │  └───────────────────────────────────────┘  │
   └──────────────────────────────────────────────┘

Layer Responsibilities
----------------------

Cyecca Layer
^^^^^^^^^^^^

**Location:** External library (``pip install cyecca``)

**Responsibilities:**

* Core ModelSX framework
* Symbolic differentiation (CasADi)
* Numerical integrators
* Linearization tools
* Generic dynamics utilities

**Dependencies:** numpy, scipy, casadi

**ROS:** None

cubs2_dynamics
^^^^^^^^^^^^^^

**Responsibilities:**

* Aircraft-specific models (SportCub)
* Trim computation
* Aerodynamic coefficients
* Ground contact dynamics

**Dependencies:** cyecca, numpy, scipy, casadi

**ROS:** None

cubs2_control
^^^^^^^^^^^^^

**Responsibilities:**

* Attitude controllers
* PID controllers
* Closed-loop compositions
* Control algorithms

**Dependencies:** cubs2_dynamics, cyecca

**ROS:** None

cubs2_simulation
^^^^^^^^^^^^^^^^

**Responsibilities:**

* ROS2 node implementation
* Message publishing/subscribing
* Real-time scheduling
* TF broadcasting
* Gamepad integration

**Dependencies:** cubs2_dynamics, cubs2_control, cubs2_msgs

**ROS:** rclpy, geometry_msgs, sensor_msgs

cubs2_planning
^^^^^^^^^^^^^^

**Responsibilities:**

* Dubins path planning
* Racecourse navigation
* Waypoint generation
* Path visualization

**Dependencies:** cubs2_msgs

**ROS:** rclpy, nav_msgs, visualization_msgs

cubs2_rviz
^^^^^^^^^^

**Responsibilities:**

* Custom RViz panels
* Video streaming
* HUD display
* Joystick widget
* Simulation controls

**Dependencies:** cubs2_msgs, Qt5, GStreamer

**ROS:** rclcpp, rviz_common, pluginlib

cubs2_bringup
^^^^^^^^^^^^^

**Responsibilities:**

* System launch files
* Parameter configuration
* Node orchestration

**Dependencies:** All other cubs2 packages

**ROS:** launch, launch_ros

Data Flow
---------

Simulation Loop
^^^^^^^^^^^^^^^

.. code-block:: text

   ┌─────────────────┐
   │  User Input     │ (/joy, /control)
   └────────┬────────┘
            │
            v
   ┌─────────────────┐
   │  Controller     │ (cubs2_control)
   └────────┬────────┘
            │
            v
   ┌─────────────────┐
   │  Dynamics       │ (cubs2_dynamics)
   └────────┬────────┘
            │
            v
   ┌─────────────────┐
   │  Integration    │ (RK4)
   └────────┬────────┘
            │
            v
   ┌─────────────────┐
   │  State Update   │
   └────────┬────────┘
            │
            v
   ┌─────────────────┐
   │  ROS Messages   │ (/pose, /velocity, /imu)
   └─────────────────┘

Message Types
-------------

Custom Messages
^^^^^^^^^^^^^^^

* ``cubs2_msgs/AircraftControl`` - Control surface commands

Standard Messages
^^^^^^^^^^^^^^^^^

* ``geometry_msgs/PoseStamped`` - Position and orientation
* ``geometry_msgs/TwistStamped`` - Velocities
* ``sensor_msgs/Imu`` - IMU measurements
* ``nav_msgs/Path`` - Planned trajectory
* ``visualization_msgs/MarkerArray`` - Visual markers

Hierarchical Composition
-------------------------

ModelSX Framework
^^^^^^^^^^^^^^^^^

Cubs2 uses Cyecca's ModelSX for hierarchical model composition:

.. code-block:: python

   # Create submodels
   aircraft = sportcub()
   controller = autolevel_controller()

   # Compose
   system = ModelSX.compose({
       "plant": aircraft,
       "controller": controller
   })

   # Connect signals
   system.connect("controller.u.q", "plant.x.r")
   system.connect("plant.u.ail", "controller.y.ail")

   # Build with single integrator
   system.build_composed(integrator="rk4")

Benefits:

* Single integration loop (numerical accuracy)
* Type-safe state access (``x.plant.p`` not ``x[0:3]``)
* Reusable subsystems
* Clear signal flow

Plugin Architecture
-------------------

RViz Panels
^^^^^^^^^^^

Panels use Qt5 and RViz2 API:

.. code-block:: cpp

   class MyPanel : public rviz_common::Panel {
     Q_OBJECT
   public:
     MyPanel(QWidget* parent = nullptr);
   private:
     rclcpp::Node::SharedPtr node_;
     rclcpp::Publisher<...>::SharedPtr pub_;
   };

   PLUGINLIB_EXPORT_CLASS(cubs2::MyPanel, rviz_common::Panel)

Design Principles
-----------------

Separation of Concerns
^^^^^^^^^^^^^^^^^^^^^^

* Dynamics ≠ ROS
* Control ≠ ROS
* Only runtime nodes use ROS

**Benefit:** Reusable in non-ROS contexts (MATLAB, hardware)

Composition over Inheritance
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Build systems from subsystems
* Connect via explicit signals
* Avoid deep inheritance hierarchies

Type Safety
^^^^^^^^^^^

* Dataclass-based state definitions
* Named fields instead of indices
* Compile-time validation

Testing
^^^^^^^

* Unit tests for each layer
* Integration tests for compositions
* Hardware-in-loop tests

Performance Considerations
--------------------------

Simulation Speed
^^^^^^^^^^^^^^^^

Factors affecting real-time performance:

* Integration timestep (smaller = slower)
* Model complexity
* Visualization overhead
* Python vs compiled code

Optimization Strategies:

* JIT compilation (CasADi)
* Vectorized operations (NumPy)
* Reduced visualization rate
* C++ critical paths

Memory
^^^^^^

* CasADi symbolic expressions cached
* ROS messages zero-copy when possible
* Preallocated state vectors

Extensibility
-------------

Adding Aircraft
^^^^^^^^^^^^^^^

1. Create new module in ``cubs2_dynamics``
2. Inherit from or use ModelSX pattern
3. Define state/input/output dataclasses
4. Implement dynamics equations

Adding Controller
^^^^^^^^^^^^^^^^^

1. Create module in ``cubs2_control``
2. Follow ModelSX pattern for composition
3. Add unit tests
4. Document tuning parameters

Adding RViz Panel
^^^^^^^^^^^^^^^^^

1. Create header in ``cubs2_rviz/include``
2. Implement in ``cubs2_rviz/src``
3. Register in ``plugin_description.xml``
4. Update RViz config

See Also
--------

* :doc:`contributing` - Development guidelines
* :doc:`testing` - Testing procedures
