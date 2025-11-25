cubs2_msgs
==========

ROS2 message definitions for Cubs2 aircraft control and telemetry.

Messages
--------

AircraftControl.msg
^^^^^^^^^^^^^^^^^^^

Normalized control inputs for fixed-wing aircraft.

**Fields:**

* ``header`` (std_msgs/Header) - Timestamp and frame information
* ``aileron`` (float32) - Roll control, range [-1, 1]
* ``elevator`` (float32) - Pitch control, range [-1, 1]
* ``rudder`` (float32) - Yaw control, range [-1, 1]
* ``throttle`` (float32) - Thrust control, range [0, 1]

**Convention:**

* Positive aileron = right wing down (right roll)
* Positive elevator = nose up (pitch up)
* Positive rudder = nose right (yaw right)

Usage
-----

.. code-block:: python

   from cubs2_msgs.msg import AircraftControl

   msg = AircraftControl()
   msg.header.stamp = node.get_clock().now().to_msg()
   msg.aileron = 0.0
   msg.elevator = 0.1
   msg.rudder = 0.0
   msg.throttle = 0.5

Dependencies
------------

* std_msgs
* geometry_msgs
