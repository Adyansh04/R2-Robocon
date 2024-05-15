from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart, OnProcessIO
from launch.actions import RegisterEventHandler

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('r2_bringup'), 'config', 'joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_params],
        )
    
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('cmd_vel', 'cmd_vel_fast')],
    )

    cmd_vel_slow_pub = Node(
        package='r2_py',
        executable='cmd_vel_slow_pub',
        name='cmd_vel_slow_pub',
    )

    ps4 = Node(
        package='r2_py',
        executable='ps4',
        name='ps4',
    )


    return LaunchDescription([
        joy_node,
        teleop_node,
        # RegisterEventHandler(
        #     event_handler=OnProcessStart(
        #         target_action=teleop_node,
        #         on_start=cmd_vel_slow_pub
        #     )
        # ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=teleop_node,
                on_start=ps4
            )
        )
    ])