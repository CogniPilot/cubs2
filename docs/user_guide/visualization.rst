Visualization
=============

Cubs2 provides custom RViz2 panels for simulation control and telemetry display.

RViz Panels
-----------

Video Panel
^^^^^^^^^^^

Low-latency video streaming using GStreamer.

**Features:**

* Direct rendering (no ROS image topic overhead)
* Multiple input sources (USB, RTSP, UDP)
* H.264 and MJPEG support
* Custom pipeline configuration

**Setup:**

1. Add panel: ``Panels → Add New Panel → cubs2::VideoPanel``
2. Select source from dropdown
3. Click "Connect"

**Predefined Sources:**

* Test Pattern - SMPTE color bars
* USB Camera - ``/dev/video0``
* UDP H.264 - Port 5600
* UDP MJPEG - Port 5600
* RTSP Stream - Network camera

**Custom Pipeline:**

Enter GStreamer pipeline string:

.. code-block:: bash

   v4l2src device=/dev/video1 ! video/x-raw,width=1280,height=720

Joy Panel
^^^^^^^^^

Virtual joystick for manual control.

**Controls:**

* **Left stick**: Throttle (Y), Rudder (X)
* **Right stick**: Pitch (Y), Roll (X)
* **Trim buttons**: Fine-tune neutral positions
* **Enable checkbox**: Activate/deactivate

**Tips:**

* Disable when using physical gamepad
* Spring-back to center on release
* Displays external control when disabled

Sim Panel
^^^^^^^^^

Simulation configuration and control.

**Controls:**

* **Reset**: Return to initial conditions
* **Pause**: Freeze simulation
* **Speed**: 0.25x to 100x real-time
* **Timestep**: 0.001s to 0.1s

**Real-time Adjustment:**

All changes take effect immediately without relaunch.

HUD Panel
^^^^^^^^^

Flight telemetry heads-up display.

**Displays:**

* Artificial horizon (pitch/roll)
* Roll indicator arc
* Altitude (meters)
* Airspeed (m/s)

**Usage:**

Automatically updates from ``/sportcub/pose`` and ``/sportcub/velocity``.

3D Visualization
----------------

Aircraft Model
^^^^^^^^^^^^^^

URDF model with visual mesh from ``cubs2_description``.

**View Options:**

* Wireframe
* Solid with texture
* Collision geometry

Force Visualization
^^^^^^^^^^^^^^^^^^^

Enable with launch argument:

.. code-block:: bash

   ros2 launch cubs2_bringup sim.xml show_forces:=true

Shows:

* Lift vector (blue)
* Drag vector (red)
* Thrust vector (green)

Path Display
^^^^^^^^^^^^

Planned path shown as:

* Green line - Future trajectory
* Red line - Past trajectory (breadcrumbs)

Racecourse
^^^^^^^^^^

Gates displayed as:

* Colored spheres at positions
* Yellow highlight for current target
* Transparent rings (if available)

Camera Control
--------------

Follow Mode
^^^^^^^^^^^

Camera automatically tracks aircraft:

1. In RViz: ``Views → Type → ThirdPersonFollower``
2. Set target frame: ``vehicle``
3. Adjust distance and angle

Fixed Views
^^^^^^^^^^^

Common viewpoints:

* **Top-down**: Good for path visualization
* **Chase**: Behind and above aircraft
* **Cockpit**: First-person view (if model supports)

Recording
^^^^^^^^^

Record camera trajectory:

.. code-block:: bash

   # Record
   ros2 bag record /camera/pose

   # Playback
   ros2 bag play camera_recording.bag

Configuration
-------------

RViz Config
^^^^^^^^^^^

Default layout: ``cubs2_rviz/config/cubs2.rviz``

**Customize:**

1. Arrange panels and displays
2. Save: ``File → Save Config As...``
3. Load: ``ros2 launch cubs2_bringup viz.xml rviz_config:=/path/to/config.rviz``

Topics to Display
^^^^^^^^^^^^^^^^^

Useful additional topics:

* ``/tf`` - Transform tree
* ``/odometry`` - Position trail
* ``/imu`` - IMU axes
* ``/path`` - Planned trajectory

Troubleshooting
---------------

Video Panel
^^^^^^^^^^^

**No video:**

* Check GStreamer: ``gst-launch-1.0 videotestsrc ! autovideosink``
* Verify camera: ``ls /dev/video*``
* Check RViz terminal for errors

**Lag/stutter:**

* Reduce resolution
* Use H.264 encoding
* Prefer wired connection

Performance
^^^^^^^^^^^

**Low FPS:**

* Disable unused displays
* Reduce point cloud density
* Lower simulation rate

**High CPU:**

* Disable shadows
* Reduce mesh complexity
* Use simpler materials

See Also
--------

* :doc:`quickstart` - Basic usage
* :doc:`../packages/cubs2_rviz` - RViz panels reference
