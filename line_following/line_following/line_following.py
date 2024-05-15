#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType


from std_msgs.msg import Int32

from geometry_msgs.msg import Twist


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info('Line Follower Node Started')

        # Declare and get parameters
        self.declare_parameter("desired_value", 35.0)       # The desired sensor reading
        self.declare_parameter("Kp", 2.5)                  # Proportional gain
        self.declare_parameter("Ki", 0.00)                  # Integral gain
        self.declare_parameter("Kd", 1.50)                  # Derivative gain
        self.declare_parameter("linear_x_min", 0.0)         # Minimum linear speed
        self.declare_parameter("linear_x_max", 2.0)         # Maximum linear speed
        self.declare_parameter("angular_z_min", 0.0)       # Minimum angular speed
        self.declare_parameter("angular_z_max", 2.0)        # Maximum angular speed

        self.desired_value = self.get_parameter("desired_value").value
        self.Kp = self.get_parameter("Kp").value
        self.Ki = self.get_parameter("Ki").value
        self.Kd = self.get_parameter("Kd").value
        self.linear_x_min = self.get_parameter("linear_x_min").value
        self.linear_x_max = self.get_parameter("linear_x_max").value
        self.angular_z_min = self.get_parameter("angular_z_min").value
        self.angular_z_max = self.get_parameter("angular_z_max").value


        self.error_sum = 0           # Sum of errors (for integral term)
        self.last_error = 0          # Last error (for derivative term)
        self.state = "FOLLOWING"     # Initial State


        # Create a publisher for the cmd_vel topic
        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel', 
            10)

        # Create a subscription to the lsa_08 topic
        self.subscription = self.create_subscription(
            Int32,
            '/line_lsa',
            self.listener_callback,
            10)
        
    def calculate_angular_velocity(self, current_sensor_reading):
        # Calculate the error
        error = self.desired_value - current_sensor_reading

        # Calculate the integral and derivative terms
        self.error_sum += error
        error_derivative = error - self.last_error

        # Calculate the control output
        output = self.Kp * error + self.Ki * self.error_sum + self.Kd * error_derivative
    
        # Clamp the turn rate between -0.5 and 0.5
        # angular_z = max(min(output, 0.5), -0.5)  
        angular_z = max(min(output, self.angular_z_max), self.angular_z_min)

        return angular_z, error
    
    def calculate_linear_velocity(self, error):
        # Calculate the forward speed based on the absolute error
        # linear_x = max(min(1.0 - abs(error) / 75.0, 1.0), -1.0)
        linear_x = max(min(self.linear_x_max - abs(error) / 75.0, self.linear_x_max), -self.linear_x_max)
        return linear_x

        

    def listener_callback(self, msg):
        # This method is called when a new message is received on the lsa_08 topic

        current_sensor_reading = msg.data

        self.get_logger().info('I heard: "%s"' % msg.data)


        # Create a new Twist message
        twist = Twist()

        if current_sensor_reading == 255:
            self.state = "SWEEPING"
        else:
            self.state = "FOLLOWING"
        
        if self.state == "FOLLOWING":
            # Calculate the control output
            output_angular_z, error = self.calculate_angular_velocity(current_sensor_reading)
            output_linear_x = self.calculate_linear_velocity(error)

            twist.angular.z = output_angular_z
            twist.linear.x = output_linear_x  

            # Update the last error
            self.last_error = error

        elif self.state == "SWEEPING":
            # If the sensor reading is 255, the line is lost
            # Make the robot sweep towards the last known direction of the black line
            twist.angular.z = 0.5 if self.last_error < 0 else -0.5
            twist.linear.x = 0.0  # Stop moving forward

        # Calculate the control output
        output_angular_z,error = self.calculate_angular_velocity(current_sensor_reading)
        output_linear_x = self.calculate_linear_velocity(error)

        twist.angular.z = output_angular_z
        twist.linear.x = -output_linear_x  

        # Publish the new Twist message
        self.publisher_.publish(twist)
        self.get_logger().info('Published cmd_vel: linear.x = "%s", angular.z = "%s"' % (twist.linear.x, twist.angular.z))

        # Update the last error
        self.last_error = error
    



def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollowerNode()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()