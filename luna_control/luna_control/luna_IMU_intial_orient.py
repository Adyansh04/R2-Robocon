# /usr/bin/env python3

# How to run this code:
# ros2 run luna_control luna_wall_align --ros-args -p x_goal:=40.0 -p y_goal:=50.0 -p kp_linear:=0.05 -p ki_linear:=0.00 -p kd_linear:=0.00 -p kp_angular:=0.08 -p ki_angular:=0.00 -p kd_angular:=0.0

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import sys
import os

class LunaWallAlignNode(Node):
    def __init__(self):
        super().__init__('luna_wall_align')
        self.get_logger().info('Initializing my_node')

        self.prev_ang_error = 0

        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_x_max', 2.0),
                ('linear_y_max', 2.0),
                ('linear_x_min', -2.0),
                ('linear_y_min', -2.0),
                ('angular_z_max', 1.0),
                ('angular_z_min', -1.0),
                ('kp_linear', 0.08),
                ('ki_linear', 0.0),
                ('kd_linear', 0.0),
                ('kp_angular', 0.08),
                ('ki_angular', 0.0),
                ('kd_angular', 0.0),         
                # ('x_goal', 13.0),
                # ('y_goal', 250.0),
                ('silo_number', 1),    
                ('silo_1_x', 25.0),
                ('silo_1_y', 200.0),
                ('silo_2_x', 13.0),
                ('silo_2_y', 250.0),
                ('silo_3_x', 32.0),
                ('silo_3_y', 21.0),
                ('silo_4_x', 25.0),
                ('silo_4_y', 300.0),
                ('silo_5_x', 25.0),
                ('silo_5_y', 340.0),     
                ('theta_goal', 0.0)
                ]
        )

        self. linear_x_max = self.get_parameter('linear_x_max').value
        self.linear_y_max = self.get_parameter('linear_y_max').value
        self.linear_x_min = self.get_parameter('linear_x_min').value
        self.linear_y_min = self.get_parameter('linear_y_min').value
        self.angular_z_max = self.get_parameter('angular_z_max').value
        self.angular_z_min = self.get_parameter('angular_z_min').value

        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.kd_linear = self.get_parameter('kd_linear').value
        self.ki_angular = self.get_parameter('kd_angular').value
        self.kd_angular = self.get_parameter('ki_linear').value
        self.ki_linear = self.get_parameter('ki_angular').value

        # self.x_goal = self.get_parameter('x_goal').value
        # self.y_goal = self.get_parameter('y_goal').value
        self.silo_number = self.get_parameter('silo_number').value

        self.silo_1_x = self.get_parameter('silo_1_x').value
        self.silo_1_y = self.get_parameter('silo_1_y').value
        self.silo_2_x = self.get_parameter('silo_2_x').value
        self.silo_2_y = self.get_parameter('silo_2_y').value
        self.silo_3_x = self.get_parameter('silo_3_x').value
        self.silo_3_y = self.get_parameter('silo_3_y').value
        self.silo_4_x = self.get_parameter('silo_4_x').value
        self.silo_4_y = self.get_parameter('silo_4_y').value
        self.silo_5_x = self.get_parameter('silo_5_x').value
        self.silo_5_y = self.get_parameter('silo_5_y').value
        self.theta_goal = self.get_parameter('theta_goal').value    

        
        self.luna_subscriber = self.create_subscription(
            Int64MultiArray,
            'luna_data',
            self.luna_callback,
            10,
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.imu_rpy_subscriber = self.create_subscription(
            Vector3,
            'rpy',
            self.imu_rpy_callback,
            10,
        )
            

        # Initialize the luna data
        self.luna_1 = 0.00  
        self.luna_2 = 0.00  
        self.luna_3 = 0.00  
        self.luna_4 = 0.00  


        #State Management Variable 
        self.correct_angle = False

        # Set initial integrals to zero
        self.int_error_linear_x = 0.0
        self.int_error_linear_y = 0.0
        self.int_error_angular_z = 0.0


        # self.positions = {
        #     1: {'x': 25.0, 'y': 200.0},
        #     2: {'x': 13.0, 'y': 250.0},
        #     3: {'x': 32.0, 'y': 21.0},
        #     4: {'x': 25.0, 'y': 300.0},
        #     5: {'x': 25.0, 'y': 340.0},
        # }

        self.positions = {
            1: {'x': self.silo_1_x, 'y': self.silo_1_y},
            2: {'x': self.silo_2_x, 'y': self.silo_2_y},
            3: {'x': self.silo_3_x, 'y': self.silo_3_y},
            4: {'x': self.silo_4_x, 'y': self.silo_4_y},
            5: {'x': self.silo_5_x, 'y': self.silo_5_y},
        }

        # Get the x and y goals from the selected position
        self.x_goal = self.positions[self.silo_number]['x']
        self.y_goal = self.positions[self.silo_number]['y']
        
        self.get_logger().info('x: %d' % self.x_goal)
        self.get_logger().info('y: %d' % self.y_goal)
        self.get_logger().info('theta: %d' % self.theta_goal)


        #Initialize the rpy variables, getting this from the IMU (Quat to RPY)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        

    def pid_controller(self, error, previous_error, int_error, ki, kd, dt):
        control_action = self.kp_linear * error + ki * int_error + kd * ((error - previous_error) / dt)
        return control_action


    def imu_rpy_callback(self, msg):
        self.roll = msg.x
        self.pitch = msg.y
        self.yaw = msg.z

        self.get_logger().info('IMU data received')
        self.get_logger().info('Roll: %f' % self.roll)
        self.get_logger().info('Pitch: %f' % self.pitch)
        self.get_logger().info('Yaw: %f' % self.yaw)

    def luna_callback(self, msg):
        self.luna_1 = float(msg.data[1]) - 2         #Front Left -14    -  2 is the offset (Luna sensor error correction)
        self.luna_2 = float(msg.data[3]) - 2         #Front right -12   -  2 is the offset (Luna sensor error correction)- 
        self.luna_3 = float(msg.data[0])             #Side Left - 11
        self.luna_4 = float(msg.data[2])             #Side right -13 
        self.get_logger().info('Luna data received')
        self.get_logger().info('x: %d' % self.x_goal)
        self.get_logger().info('y: %d' % self.y_goal)


        #All 4 luna readings
        self.get_logger().info('Luna 1: %d' % self.luna_1)
        self.get_logger().info('Luna 2: %d' % self.luna_2)
        self.get_logger().info('Luna 3: %d' % self.luna_3)
        self.get_logger().info('Luna 4: %d' % self.luna_4)

        #Calculate the difference between the sensor readings
        x_diff = self.luna_1 - self.luna_2
        y_diff = self.luna_3 - self.luna_4

        #create a new Twist message
        twist = Twist()

        x_avg = (self.luna_1 + self.luna_2) / 2
        y_avg = (self.luna_3 + self.luna_4) / 2

        yaw_error = self.yaw - self.theta_goal

        if self.correct_angle:
            #Calculate the time difference
            dt = 0.1  # Assuming fixed sample rate of 10 Hz

            

            # Check if angu lar z correction is required
            if abs(yaw_error) < 3:
                self.correct_angle = False
                self.get_logger().info('Correcting angle')
                
            # #Adjust the angular z velocity based on the difference between the sensor readings
            # twist.angular.z = self.kp_angular * (x_diff)
            # twist.angular.z = max(min(twist.angular.z, self.angular_z_max), self.angular_z_min)

            # Apply PID controller for angular z
            self.int_error_angular_z += yaw_error
            twist.angular.z = self.pid_controller(yaw_error, self.prev_ang_error, self.int_error_angular_z, self.ki_angular, self.kd_angular, dt)
            self.prev_ang_error = yaw_error

            if yaw_error < 0:   #Lune front left < front right -> left side is closer to the wall
                twist.angular.z = abs(twist.angular.z)    #Rotate Anti-clockwise
            else:
                twist.angular.z = -abs(twist.angular.z) 

            self.get_logger().info('Angular z: %f' % twist.angular.z)
        
        else:
            self.get_logger().info('Entered linear velocity adjustment')

            if (abs(x_avg - self.x_goal) >= 2) or (abs(y_avg - self.y_goal) >= 2):
                self.get_logger().info("x_avg - self.x_goal = " + str(abs(x_avg - self.x_goal)))
                self.get_logger().info("x_goal : " + str(self.x_goal))
                self.get_logger().info("y_goal : " + str(self.y_goal))
                dt = 0.1
                prev_lin_error_x = 0
                prev_lin_error_y = 0
                lin_error_x = self.x_goal - x_avg               
                lin_error_y = self.y_goal - y_avg
                self.int_error_linear_x += lin_error_x
                self.int_error_linear_y += lin_error_y

                self.get_logger().info("self.y_goal - y_avg = " + str(abs(self.y_goal - y_avg)))

                ang_error = self.luna_1 - self.luna_2
                self.int_error_angular_z += ang_error
                self.get_logger().info("ang_error = " + str(ang_error))

                twist.linear.x = self.pid_controller(lin_error_x, prev_lin_error_x, self.int_error_linear_x, self.ki_linear, self.kd_linear, dt)
                twist.linear.y = self.pid_controller(lin_error_y, prev_lin_error_y, self.int_error_linear_y, self.ki_linear, self.kd_linear, dt)
                twist.angular.z = -self.pid_controller(ang_error, self.prev_ang_error, self.int_error_angular_z, self.ki_angular, self.kd_angular, dt)
                self.prev_ang_error = ang_error
                prev_lin_error_x, prev_lin_error_y = lin_error_x, lin_error_y

                # Witout PID
                # twist.linear.x = self.kp_linear * (x_avg - self.x_goal) * self.linear_x_max
                # twist.linear.y = self.kp_linear * (y_avg - self.y_goal) * self.linear_y_max

                twist.linear.x = -max(min(twist.linear.x, self.linear_x_max), self.linear_x_min)
                twist.linear.y = max(min(twist.linear.y, self.linear_y_max), self.linear_y_min)
                twist.angular.z = max(min(twist.angular.z, self.angular_z_max), self.angular_z_min)

                if(abs(y_avg - self.y_goal) <= 2):
                    twist.linear.y  = 0.0
                if(abs(x_avg - self.x_goal) <= 2):
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                self.get_logger().info('Linear x: %f' % twist.linear.x)
                self.get_logger().info('Linear y: %f' % twist.linear.y)

            else:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Robot is aligned to the goal')
                self.cmd_vel_publisher.publish(twist)
                sys.exit()
                self.destroy_node()
                rclpy.shutdown()
                
            
        #Publish the Twist message
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LunaWallAlignNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()