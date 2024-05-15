#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from example_interfaces.srv import SetBool
from std_srvs.srv import SetBool
import time

class SetBoolServer(Node):
    def __init__(self):
        super().__init__('set_bool_server')
        self.motor_claw_service = self.create_service(SetBool, 'service_claw', self.motor_claw_callback)
        self.motor_lift_service = self.create_service(SetBool, 'service_lift', self.motor_lift_callback)

    def motor_claw_callback(self, request, response):
        self.get_logger().info('Started motor_claw_callback')
        time.sleep(3)
        response.success = not request.data
        response.message = 'Successfully inverted input boolean'
        # self.get_logger().info('Incoming request\n%s' % request.data)
        self.get_logger().info('response:%s' % response.success)

        return response
    
    def motor_lift_callback(self, request, response):
        self.get_logger().info('Started motor_lift_callback')
        time.sleep(3) 
        response.success = not request.data
        response.message = 'Successfully inverted input boolean'
        # self.get_logger().info('Incoming request\n%s' % request.data)
        self.get_logger().info('response:\n%s' % response.success)

        return response

def main(args=None):
    rclpy.init(args=args)

    set_bool_server = SetBoolServer()

    rclpy.spin(set_bool_server)

    set_bool_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()