#!/usr/bin/env python3
"""Gamepad control node for SportCub simulation.

Maps gamepad inputs to AircraftControl message format.

Publishes:
  /control (AircraftControl): aircraft control commands

Requires:
  - ROS 2 joy package: sudo apt install ros-${ROS_DISTRO}-joy
  - Gamepad connected via USB or Bluetooth
  - joy_node running (automatically launched with this node)

Usage:
  ros2 launch fixed_wing_purt gamepad_control.xml

Axis indices (standard gamepad layout):
  0: Left stick X (left/right)
  1: Left stick Y (up/down)
  2: Right stick X (left/right)
  3: Right stick Y (up/down)
  Note: Axis values range from -1.0 to 1.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from cubs2_msgs.msg import AircraftControl


class GamepadControlNode(Node):
    def __init__(self):
        super().__init__("gamepad_control")

        # Declare parameters for axis mapping (can be overridden)
        self.declare_parameter("axis_throttle", 1)  # Left stick Y
        self.declare_parameter("axis_rudder", 0)  # Left stick X
        self.declare_parameter("axis_elevator", 4)  # Right stick Y
        self.declare_parameter("axis_aileron", 3)  # Right stick X
        self.declare_parameter("throttle_default", 0.5)  # Default throttle
        self.declare_parameter("invert_elevator", False)  # Invert elevator axis
        self.declare_parameter("invert_aileron", False)  # Invert aileron axis
        self.declare_parameter("invert_rudder", False)  # Invert rudder axis
        self.declare_parameter("invert_throttle", False)  # Invert throttle axis
        self.declare_parameter("deadzone", 0.05)  # Deadzone for stick drift

        # Get parameters
        self.axis_throttle = self.get_parameter("axis_throttle").value
        self.axis_rudder = self.get_parameter("axis_rudder").value
        self.axis_elevator = self.get_parameter("axis_elevator").value
        self.axis_aileron = self.get_parameter("axis_aileron").value
        self.throttle_default = self.get_parameter("throttle_default").value
        self.invert_elevator = self.get_parameter("invert_elevator").value
        self.invert_aileron = self.get_parameter("invert_aileron").value
        self.invert_rudder = self.get_parameter("invert_rudder").value
        self.invert_throttle = self.get_parameter("invert_throttle").value
        self.deadzone = self.get_parameter("deadzone").value

        # Publishers
        self.pub_control = self.create_publisher(AircraftControl, "/control", 10)
        self.pub_reset = self.create_publisher(Empty, "/reset", 10)
        self.pub_pause = self.create_publisher(Empty, "/pause", 10)

        # Subscriber to joy messages
        self.sub_joy = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # Current state
        self.aileron = 0.0
        self.elevator = 0.0
        self.throttle = self.throttle_default
        self.rudder = 0.0

        # Button state tracking (for edge detection)
        self.last_buttons = []

        self.get_logger().info("Gamepad control node started")
        self.get_logger().info("Axis mapping:")
        self.get_logger().info(
            f"  Left stick Y (up/down)    -> Throttle (axis {self.axis_throttle})"
        )
        self.get_logger().info(f"  Left stick X (left/right) -> Rudder (axis {self.axis_rudder})")
        self.get_logger().info(
            f"  Right stick Y (up/down)   -> Elevator (axis {self.axis_elevator})"
        )
        self.get_logger().info(f"  Right stick X (left/right)-> Aileron (axis {self.axis_aileron})")
        self.get_logger().info("Button mapping:")
        self.get_logger().info("  Button 0 (A/X): Reset to neutral")
        self.get_logger().info("  Button 1 (B/Circle): Send /reset")
        self.get_logger().info("  Button 6 (Back): Exit")
        self.get_logger().info("  Button 7 (Start): Toggle /pause")

    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to axis value."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def joy_callback(self, msg: Joy):
        """Process joystick messages."""

        # Extract axis values with safety checks
        def get_axis(index: int) -> float:
            if index < len(msg.axes):
                return self.apply_deadzone(float(msg.axes[index]))
            return 0.0

        # Map axes to controls
        # Left stick Y -> Throttle [0, 1]
        # Joy axes: down = -1, up = 1
        # Map: down to center (-1 to 0) = idle (0), center to up (0 to 1) = throttle 0 to 1
        throttle_raw = get_axis(self.axis_throttle)
        if self.invert_throttle:
            throttle_raw = -throttle_raw
        # Only use positive half (stick pushed up)
        if throttle_raw >= 0:
            self.throttle = throttle_raw  # 0 to 1 maps directly to throttle 0 to 1
        else:
            self.throttle = 0.0  # Negative (stick down) = idle
        self.throttle = max(0.0, min(1.0, self.throttle))

        # Left stick X -> Rudder [-1, 1]
        rudder_raw = get_axis(self.axis_rudder)
        if self.invert_rudder:
            rudder_raw = -rudder_raw
        self.rudder = max(-1.0, min(1.0, rudder_raw))

        # Right stick Y -> Elevator [-1, 1]
        # Joy Y: down = -1, up = 1, elevator positive = pitch up
        # stick up (positive) = pitch up (positive)
        elevator_raw = get_axis(self.axis_elevator)
        if self.invert_elevator:
            elevator_raw = -elevator_raw
        self.elevator = max(-1.0, min(1.0, elevator_raw))

        # Right stick X -> Aileron [-1, 1]
        aileron_raw = get_axis(self.axis_aileron)
        if self.invert_aileron:
            aileron_raw = -aileron_raw
        self.aileron = max(-1.0, min(1.0, aileron_raw))

        # Handle buttons (edge detection - trigger on press, not hold)
        if len(msg.buttons) > 0:
            # Initialize last_buttons if needed
            if len(self.last_buttons) == 0:
                self.last_buttons = [0] * len(msg.buttons)

            # Button 0: Reset to neutral
            if len(msg.buttons) > 0 and msg.buttons[0] and not self.last_buttons[0]:
                self.aileron = 0.0
                self.elevator = 0.0
                self.throttle = self.throttle_default
                self.rudder = 0.0
                self.get_logger().info("Reset to neutral")

            # Button 1: Send /reset
            if len(msg.buttons) > 1 and msg.buttons[1] and not self.last_buttons[1]:
                self.pub_reset.publish(Empty())
                self.get_logger().info("Sent /reset")

            # Button 6: Exit node
            if len(msg.buttons) > 6 and msg.buttons[6] and not self.last_buttons[6]:
                self.get_logger().info("Exit button pressed, shutting down...")
                raise KeyboardInterrupt()

            # Button 7: Toggle pause
            if len(msg.buttons) > 7 and msg.buttons[7] and not self.last_buttons[7]:
                self.pub_pause.publish(Empty())
                self.get_logger().info("Sent /pause toggle")

            # Update button state
            self.last_buttons = list(msg.buttons)

        # Publish control messages
        self.publish_controls()

    def publish_controls(self):
        """Publish current control state as AircraftControl message."""
        msg = AircraftControl()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.aileron = float(self.aileron)
        msg.elevator = float(self.elevator)
        msg.throttle = float(self.throttle)
        msg.rudder = float(self.rudder)
        self.pub_control.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GamepadControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gamepad control node")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
