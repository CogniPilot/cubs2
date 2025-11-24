#!/usr/bin/env python3
"""
Node to convert PoseStamped messages to TF transforms.

Subscribes to a pose topic and publishes the corresponding transform to /tf.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseToTFNode(Node):
    """Converts PoseStamped messages to TF transforms."""

    def __init__(self):
        super().__init__("pose_to_tf")

        # Declare parameters
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("child_frame_id", "base_link")

        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to pose topic
        self.pose_sub = self.create_subscription(PoseStamped, "pose", self.pose_callback, 10)

        self.get_logger().info(
            f"Pose to TF node started. Publishing {self.frame_id} -> {self.child_frame_id}"
        )

    def pose_callback(self, msg: PoseStamped):
        """Convert PoseStamped to TF and broadcast."""
        # Create transform message
        t = TransformStamped()

        # Use current time (for replaying old bag data with current timestamps)
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        # Copy translation
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        # Copy rotation
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
