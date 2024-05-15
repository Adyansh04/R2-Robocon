#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import os
from ament_index_python.packages import get_package_share_directory
import subprocess


package_path = get_package_share_directory('r2_bringup')

file_path = os.path.join(package_path, 'launch', 'position_name.txt')
class TestNode1(Node):
    def __init__(self):
        super().__init__("test_node_1")
        with open(file_path, 'w') as f:
            f.write('silo2')

    def run_luna_wall_align(self):
        print("Running luna wall align")
        # cmd = "ros2 run luna_control luna_wall_align --ros-args -p silo_number:=2"
        cmd = "ros2 run r2_py test_node2"
        subprocess.Popen(cmd, shell=True)

    def random_log_2sec_loop(self):
        for i in range(3):
            self.get_logger().info("Hello from random_log_2sec_loop")
            time.sleep(2)
        self.get_logger().info("Exiting after 3 iterations")
      

        


        
def main(args=None):
    rclpy.init(args=args)
    node = TestNode1()

    node.run_luna_wall_align()
    node.random_log_2sec_loop()
    print('Done')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
