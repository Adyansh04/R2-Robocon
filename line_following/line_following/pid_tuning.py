#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
import sys


from std_msgs.msg import Int32,Int8
from geometry_msgs.msg import Twist

import tkinter as tk
from tkinter import Scale, HORIZONTAL
import threading
from queue import Queue

     
# def map_range(z, in_min, in_max, out_min, out_max):
#             return (z - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

class PidTuningNode(Node):
    def __init__(self):
        super().__init__('pid_tuning')
        self.get_logger().info('Line Follower Node Started')

        # Declare and get parameters
        self.declare_parameter("desired_value", 35)         # The desired sensor reading
        self.declare_parameter("Kp", 0.01)                  # Proportional gain
        self.declare_parameter("Ki", 0.00)                  # Integral gain
        self.declare_parameter("Kd", 0.00)                  # Derivative gain
        # self.declare_parameter("angular_z_min", -0.290196078)       # Minimum angular speed
        # self.declare_parameter("angular_z_max", 0.290196078) 
        # self.declare_parameter("angular_z_min", -0.698039216)       # Minimum angular speed
        # self.declare_parameter("angular_z_max", 0.698039216) 
        self.declare_parameter("angular_z_min", -1.0)       # Minimum angular speed
        self.declare_parameter("angular_z_max", 1.0) 
        
        self.desired_value = self.get_parameter("desired_value").value
        self.Kp = self.get_parameter("Kp").value
        self.Ki = self.get_parameter("Ki").value
        self.Kd = self.get_parameter("Kd").value
        self.angular_z_min = self.get_parameter("angular_z_min").value
        self.angular_z_max = self.get_parameter("angular_z_max").value


        self.error_sum = 0           # Sum of errors (for integral term)
        self.last_error = 0          # Last error (for derivative term)
        self.state = "FOLLOWING"     # Initial State
        self.node_counter = 0        # node counter variable (if node counter = 4 then stop => reached third zone)
        self.node_passed = False

        self.Kp_value = 0.00806100217
        self.Kd_value = 0.00806100217
        # self.Kp_value = 0.00392156863
        # self.Kd_value = 0.00392156863
        # self.Kp_value = 0.01
        # self.Kd_value = 0.005
        # self.condition_to_destroy = False

        self.window = tk.Tk()
        self.window.title("PID Tuner")

        self.kp_scale = Scale(self.window, from_=0, to=5, resolution=0.00001, orient=HORIZONTAL, length=2000, label="Kp", command=self.update_kp)
        self.kp_scale.pack()

        self.ki_scale = Scale(self.window, from_=0, to=5, resolution=0.00001, orient=HORIZONTAL, length=400, label="Ki", command=self.update_ki)
        self.ki_scale.pack()

        self.kd_scale = Scale(self.window, from_=0, to=5, resolution=0.00001, orient=HORIZONTAL, length=2000, label="Kd", command=self.update_kd)
        self.kd_scale.pack()

        self.pid_params_queue = Queue()        

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
        
        self.node_count_subscriber = self.create_subscription(
            Int8,
            "/Junction_count",
            self.junctionCallback,
            10)
        
    def run(self):
        self.window.mainloop()

    def on_slider_change(self, value):
        # Put the new PID parameters in the queue
        self.pid_params_queue.put(self.get_pid_params())

    def get_pid_params(self):
        # If there are new PID parameters in the queue, return them
        if not self.pid_params_queue.empty():
            return self.pid_params_queue.get()
        # Otherwise, return the current PID parameters
        return self.kp, self.ki, self.kd
        
    def update_kp(self, value):
        self.Kp = float(value)
        self.set_parameters([Parameter("Kp", Parameter.Type.DOUBLE, self.Kp)])
        self.pid_params_queue.put((self.Kp, self.Ki, self.Kd))

    def update_ki(self, value):
        self.Ki = float(value)
        self.set_parameters([Parameter("Ki", Parameter.Type.DOUBLE, self.Ki)])
        self.pid_params_queue.put((self.Kp, self.Ki, self.Kd))

    def update_kd(self, value):
        self.Kd = float(value)
        self.set_parameters([Parameter("Kd", Parameter.Type.DOUBLE, self.Kd)])
        self.pid_params_queue.put((self.Kp, self.Ki, self.Kd))
        
    def junctionCallback(self, msg:Int32):
        self.node_counter = msg.data
        self.get_logger().info("node count = " + str(self.node_counter))
   

    def calculate_angular_velocity(self, current_sensor_reading):
        # Calculate the error
        error = self.desired_value - current_sensor_reading
        print("Current Sensor Reading")
        print(error,"\n")
        # Calculate the integral and derivative terms
        self.error_sum += error
        error_derivative = error - self.last_error

        # Calculate the control output
        # output = (self.kp_value * error) + (self.kd_value * error_derivative)
        output = (self.Kp_value * error) + (self.Kd_value * error_derivative)
        # Clamp the turn rate between -5.0 and 5.0
        # angular_z = max(min(output, 1.0), -1.0)    # max value of z = 0.373 (max turn angle)
        # angular_z = map_range(output,0,self.Kp*75,0,1.0)

        # angular_z = ((self.angular_z_max-self.angular_z_min)/((self.Kp * 35) + self.Kd) ) * output
        angular_z = (self.angular_z_max/((self.Kp_value * 35) + self.Kd_value)) * output
        angular_z = float(max(min(output, self.angular_z_max), -self.angular_z_max))
        return angular_z,error
    
    def calculate_linear_velocity(self, error):
        # Calculate the forward speed based on the absolute error
        linear_x = max(min(2.0 - abs(error) / 75.0, 2.0), -2.0)
        return linear_x

    def listener_callback(self, msg):
        # This method is called when a new message is received on the lsa_08 topic

        current_sensor_reading = msg.data
        self.get_logger().info('LSA08 Data "%s"' % msg.data)


        # Create a new Twist message
        twist = Twist()
        
        z_vel = 0.0
        # x_vel = 1.56862745  # 1.56862745
        x_vel = 1.0  # 1.56862745
        y_vel = 0.0
        error = 0
        
        condition_to_destroy = False
            
        if(self.node_counter >= 2): # stop
            z_vel = 0.0
            x_vel = 0.0
            y_vel = 0.0
            condition_to_destroy = True

            # self.destroy_node()
            
        elif(self.node_counter == 1 and current_sensor_reading == 255):
            z_vel = 0.0
            x_vel = 0.0
            y_vel = 1.0
            # self.destroy_node()


        # Calculate the control output
        # Calculate the control output
        else:
            z_vel, error = self.calculate_angular_velocity(current_sensor_reading)
            # output_linear_x = self.calculate_linear_velocity(error)

        twist.angular.z = 0.0
        if(condition_to_destroy):
            z_vel = 0.0
        # print(z_vel)
        # twist.linear.x = -output_linear_x  
        twist.angular.z = z_vel
        twist.linear.x = x_vel  # keeing base speed 2 -> 255 PWM val
        twist.linear.y = y_vel  # for straight right motion

        # Publish the new Twist message
        self.publisher_.publish(twist)

        self.get_logger().info('Published cmd_vel: linear.x = "%s", angular.z = "%s"' % (twist.linear.x, twist.angular.z))
        print("destroy: " + str(condition_to_destroy))

        # Update the last error
        self.last_error = error
        # time.sleep(1.0)

        if(condition_to_destroy):
            print("destroy condition reached")
            self.destroy_node()
            # rclpy.shutdown()
            sys.exit()
            return

def main(args=None):
    rclpy.init(args=args)
    line_follower = PidTuningNode()

    # Use the multithreaded executor
    # executor = MultiThreadedExecutor()

    # def spin_ros():
    #     executor.add_node(line_follower)
    #     try:
    #         executor.spin()
    #     except KeyboardInterrupt:
    #         pass

    #     # Shutdown and cleanup
    #     executor.shutdown()
    #     line_follower.destroy_node()
    #     rclpy.shutdown()

    # # Start the ROS2 node in a separate thread
    # # threading.Thread(target=spin_ros).start()

    # # Start the Tkinter main loop in the main thread
    # # line_follower.run()

    # # Shutdown and cleanup
    # executor.shutdown()
    rclpy.spin(line_follower)
    # line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()