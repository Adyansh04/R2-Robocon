#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32
from geometry_msgs.msg import Twist
import random

class Lsa08DueFakeNode(Node):
    def __init__(self):
        super().__init__('mock_due_node')
        self.get_logger().info('LSA08-Due Fake Node Started')
        self.sensor_reading = random.randint(0, 70)

        # Create a publisher for the lsa_08 topic    
        self.publisher_ = self.create_publisher(
            Int32,
            '/lsa_08', 
            10)
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        # Create a timer to publish random sensor readings
        self.timer = self.create_timer(1, self.timer_callback)
    
    def listener_callback(self, twist):
        # This method is called when a new message is received on the cmd_vel topic
        self.get_logger().info('Received cmd_vel: linear.x = "%s", linear.y = "%s", angular.z = "%s"' % (twist.linear.x, twist.linear.y, twist.angular.z))

    def timer_callback(self):
        # This method is called every 0.5 seconds
        # Generate a random sensor reading and publish it
        msg = Int32()

        # Change the sensor reading by a random amount between -5 and 5
        change = random.randint(-15,15) 

        # Ensure the sensor reading stays within the range 0-75
        self.sensor_reading = min(max(0, self.sensor_reading + change), 70)
        # self.sensor_reading = self.sensor_reading + change

        msg.data = self.sensor_reading

        self.publisher_.publish(msg)
        self.get_logger().info('Published sensor reading: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Lsa08DueFakeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()