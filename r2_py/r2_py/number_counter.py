#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")

        self.number_subscriber_ = self.create_subscription(
            Int64, "number", self.number_callback, 10)
        self.get_logger().info("Number counter has been started.")

    def number_callback(self, msg):
        self.get_logger().info(str(msg.data))
        
def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
