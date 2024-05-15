from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import LaunchConfiguration
import launch.actions


def generate_launch_description():
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('r2_bringup'), 'config', 'localization.yaml')],
    )

    return LaunchDescription([
        robot_localization_node
    ])