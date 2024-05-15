from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

#join the path of the package with the file name
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ball_tracking_params = os.path.join(get_package_share_directory('r2_bringup'),'config','ball_tracking.yaml')

    ball_tracker2 = Node(
        package='ball_tracking',
        executable='ball_tracker2',
        name='ball_tracker2',
        parameters=[ball_tracking_params]
       
    )

  

    return LaunchDescription([
       ball_tracker2
    ])