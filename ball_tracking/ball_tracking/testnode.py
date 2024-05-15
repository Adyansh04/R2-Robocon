#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class NumberSubscriberNode(Node):
    def __init__(self):
        super().__init__("Number_counter")

        self.number_count_publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.number_subscriber_ = self.create_subscription(Twist, "/blue_ball_data", self.callback_number_counter, 10)
        self.get_logger().info("Number counter has been started")
        self.timer = self.create_timer(0.2, self.timer_callback)  # Set the timer to 2 seconds for slower rate
        self.linear_x=0.0
        self.linear_y=0.0
        self.angular_z=0.5

    def timer_callback(self):
        cmd_msg = Twist()
        # Assign desired values to the Twist message
        # cmd_msg.linear.x = 0.0
        # cmd_msg.linear.y = 0.0
        # cmd_msg.angular.z = 0.0

        cmd_msg.linear.x =self.linear_x
        cmd_msg.linear.y = self.linear_y
        cmd_msg.angular.z = self.angular_z
        if self.linear_x == 0.0 and self.angular_z == 0.0 and self.linear_y == 0.0:
             self.number_count_publisher_.publish(cmd_msg)
             self.destroy_node()
                
        self.number_count_publisher_.publish(cmd_msg)

    def callback_number_counter(self, new_msg:Twist):
        # cmd_msg = Twist()
        # # if new_msg.linear.z == 1.0:

        # cmd_msg.linear.x = new_msg.linear.x
        # cmd_msg.linear.y = new_msg.linear.y
        # cmd_msg.angular.z = new_msg.angular.z

        if new_msg.linear.x == 0.0 and new_msg.angular.z == 0.0 and new_msg.linear.y == 0.0:
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.angular_z = 0.0
                # self.number_count_publisher_.publish(cmd_msg)
                # self.destroy_node()

        # self.number_count_publisher_.publish(cmd_msg)
        self.linear_x=new_msg.linear.x
        self.linear_y=new_msg.linear.y
        self.angular_z=new_msg.angular.z

def main(args=None):
    rclpy.init(args=args)
    node = NumberSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
