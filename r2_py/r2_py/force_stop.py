#Script to publish 0 in all cmd_vel topic on a keyboard click it will constantly run on the background. 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import keyboard

class ZeroPublisher(Node):
    def __init__(self):
        super().__init__('zero_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def publish_zero(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    zero_publisher = ZeroPublisher()

    while rclpy.ok():
        rclpy.spin_once(zero_publisher)
        if keyboard.is_pressed('q'):  # if key 'q' is pressed 
            zero_publisher.publish_zero()
            print('Force Stop Activated')
    zero_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()