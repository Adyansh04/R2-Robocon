#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import os
from math import sqrt
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from math import atan2, asin, pi


class QuatToRPY(Node):
    def __init__(self):
        super().__init__("quat_to_rpy")

        self.subscription = self.create_subscription(
            Imu,
            "/imu/data",
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(Vector3, "/rpy", 10)
        self.get_logger().info("Quat to RPY node has been started")

    def listener_callback(self, msg):


        self.get_logger().info("Received a message")
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        self.get_logger().info(f"x: {x}, y: {y}, z: {z}, w: {w}")

        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = sqrt(1 + 2 * (w * y - z * x))
        cosp = sqrt(1 - 2 * (w * y - x * z))
        pitch = 2 * atan2(sinp, cosp) - pi/2

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = atan2(siny_cosp, cosy_cosp)


        # Convert to degrees
        roll = roll * 180.0 / pi
        pitch = pitch * 180.0 / pi
        yaw = yaw * 180.0 / pi

        self.get_logger().info(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

        # Create a Vector3 message
        rpy = Vector3()
        rpy.x = roll
        rpy.y = pitch
        rpy.z = yaw

        self.publisher.publish(rpy)
    

        
def main(args=None):
    rclpy.init(args=args)
    node = QuatToRPY()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
