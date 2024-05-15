"""This launch will be used to launch the turtlesim node and the turtle_teleop_key node with yaml file in params directory.
 This is a sample launch file to demonstrate the use of launch file with parameters."""

#Imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

#join the path of the package with the file name
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

#join the path of the package with the file name
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    turtle_params = os.path.join(get_package_share_directory('r2_bringup'),'config','turtlesim_params.yaml')
    # print(turtle_params)

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        # output='screen',
        #parameters from the yaml file
        parameters=[os.path.join(get_package_share_directory('r2_bringup'), 'config', 'turtlesim_params.yaml')],

        #paramters from the launch file
        # parameters=[{'background_r': 255, 'background_g': 255, 'background_b': 255}],

        remappings=[('/turtle1/cmd_vel', '/turtle1/cmd_vel1'),
                    ('/turtle1/pose', '/turtle1/pose1'),
                    ('/turtle1/color_sensor', '/turtle1/color_sensor1')]
        
    )

    return LaunchDescription([
        turtlesim_node,
    ])