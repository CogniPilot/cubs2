#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

from racecourse_description import RacecourseLoader, MarkerFactory


class RacecourseNode(Node):
    def __init__(self):
        super().__init__("racecourse_markers_pub")
        self.get_logger().info("Racecourse Markers Publisher Node starting...")

        # Load course YAML path as parameter
        self.declare_parameter("course_yaml", "racecourse.yaml")
        yaml_path = self.get_parameter("course_yaml").value

        # Validate file path
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"Racecourse YAML not found: {yaml_path}")
            raise FileNotFoundError(f"Racecourse YAML not found: {yaml_path}")

        self.get_logger().info(f"Loading racecourse from: {yaml_path}")

        # Load course data + prepare marker factory
        self.loader = RacecourseLoader(yaml_path)
        self.factory = MarkerFactory(self.loader.frame_id)

        # Publisher + timer
        self.pub = self.create_publisher(MarkerArray, "racecourse_markers", 10)
        self.timer = self.create_timer(0.2, self.publish)

    def publish(self):
        msg = MarkerArray()
        now = self.get_clock().now().to_msg()

        id_counter = 0

        # -------------------------------------------------------
        # Publish Generic Models (pylons, props, visual dressing)
        # -------------------------------------------------------
        for obj in self.loader.generic:
            for m in obj.markers(self.factory, id_counter):
                m.header.stamp = now
                msg.markers.append(m)
            id_counter += 10

        # -------------------------------------------------------
        # Publish Gates (visual + semantic path elements)
        # -------------------------------------------------------
        for g in self.loader.gates:
            for m in g.markers(self.factory, id_counter):
                m.header.stamp = now
                msg.markers.append(m)
            id_counter += 10

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    racecourse_node = RacecourseNode()

    rclpy.spin(racecourse_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    racecourse_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
