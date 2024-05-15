from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart, OnProcessIO
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnExecutionComplete
from launch import logging

logger = logging.get_logger('launch.user')

def generate_launch_description():
    motor_server_fake = Node(
        package='r2_py',
        executable='motor_server_fake',
        name='motor_server_fake',
        output='screen',
        shell=True,

    )

    test_node1 = Node(
        package='r2_py',
        executable='test_node1',
        name='test_node1',
        output='screen',
        shell=True,

    )

    test_node2 = Node(
        package='r2_py',
        executable='test_node2',
        name='test_node2',
        output='screen',
        shell=True,
    )

    motor_claw_open_client = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        shell=True,
        parameters=[
            {'client_to_call': 'motor_claw', 'request_data': True}
        ]
    )

    motor_claw_close_client = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        shell=True,
        parameters=[
            {'client_to_call': 'motor_claw', 'request_data': False}
        ]
    )

    motor_lift_open_client = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        shell=True,
        parameters=[
            {'client_to_call': 'motor_lift', 'request_data': True}
        ]
    )

    motor_lift_close_client = Node(
        package='r2_py',
        executable='motor_client',
        name='motor_client',
        output='screen',
        shell=True,
        parameters=[
            {'client_to_call': 'motor_lift', 'request_data': False}
        ]
    )



    return LaunchDescription([
        # motor_server_fake,
        # RegisterEventHandler(
        #     OnProcessStart(
        #         target_action=motor_server_fake,
        #         on_start=[motor_claw_open_client],
        #     )
        # ),
        # RegisterEventHandler(
        #     OnProcessIO(
        #         target_action=motor_claw_open_client,
        #         on_stdout= motor_lift_close_client
        #     )
        # ),

        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=motor_claw_open_client,
        #         on_exit=motor_lift_close_client
        #     )
        # ),
        
        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=motor_lift_close_client,
        #         on_exit=motor_claw_close_client
        #     )
        # ),

        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=motor_claw_close_client,
        #         on_exit=test_node1
            # )
        # ),

        test_node1,

        RegisterEventHandler(
            OnProcessExit(
                target_action=test_node1,
                on_exit=test_node2
            )
        )
    ])