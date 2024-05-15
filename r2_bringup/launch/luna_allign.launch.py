
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os
import yaml
from launch.substitutions import LaunchConfiguration
import launch.actions

silo_luna_params = os.path.join(get_package_share_directory('r2_bringup'),'config','luna_allign.yaml')


def generate_launch_description():


    luna_align = Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('r2_bringup'), 'config', 'luna_align.yaml')],
    )

    go_to_silo_1_luna = Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        parameters=[
            {'silo_number': 1},
            silo_luna_params
        ]
    )

    go_to_silo_2_luna = Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        parameters=[
            {'silo_number': 2},
            silo_luna_params
        ]
    )

    go_to_silo_3_luna = Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        parameters=[
            {'silo_number': 3},
            silo_luna_params
        ]
    )

    go_to_silo_4_luna = Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        parameters=[
            {'silo_number': 4},
            silo_luna_params
        ]
    )

    go_to_silo_5_luna = Node(
        package='luna_control',
        executable='luna_wall_align',
        name='luna_wall_align',
        parameters=[
            {'silo_number': 5},
            silo_luna_params
        ]
    )

    return launch.LaunchDescription(
        [
            luna_align,
            
        ]
    )


