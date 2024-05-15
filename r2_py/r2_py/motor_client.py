#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import time

class SetBoolClient(Node):
    def __init__(self):
        super().__init__('set_bool_client')
        self.motor_claw_client = self.create_client(SetBool, 'motor_claw')
        self.motor_lift_client = self.create_client(SetBool, 'motor_lift')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('client_to_call', 'motor_claw'),
                ('request_data', True)
            ]
        )

        while not self.motor_claw_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('motor_claw service not available, waiting again...')
        self.get_logger().info('motor_claw service is available')

        while not self.motor_lift_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('motor_lift service not available, waiting again...')
        self.get_logger().info('motor_lift service is available')

        self.get_logger().info('SetBool client has been started')
        self.req = SetBool.Request()


    def send_request(self):
        client_to_call = self.get_parameter('client_to_call').get_parameter_value().string_value
        request_data = self.get_parameter('request_data').get_parameter_value().bool_value

        self.req.data = request_data
        self.get_logger().info('Sending request\n%s' % self.req.data)

        if client_to_call == 'motor_claw':
            future = self.motor_claw_client.call_async(self.req)
        elif client_to_call == 'motor_lift':
            future = self.motor_lift_client.call_async(self.req)
        else:
            self.get_logger().info('Invalid client_to_call parameter value')
            return

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Result received: %s' % future.result().success)
        else:
            self.get_logger().info('Service call failed')

def main(args=None):
    rclpy.init(args=args)

    set_bool_client = SetBoolClient()
    set_bool_client.send_request()
    # while rclpy.ok():
    #     set_bool_client.send_request()
    #     time.sleep(7)

    set_bool_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()