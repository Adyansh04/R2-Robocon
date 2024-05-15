from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path, get_package_share_directory
import os
from launch.actions import RegisterEventHandler, EmitEvent, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction


def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('r2_description'),
                             'urdf', 'r2_plain.urdf')
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


    rviz_cofig_path = os.path.join(get_package_share_path('r2_description'),
                                   'rviz', 'model.rviz')
    
    world_path = os.path.join(get_package_share_path('my_robot_bringup'), 
                              'worlds', 'empty_world.world')
    
    world_path = os.path.join(get_package_share_path('my_robot_bringup'), 
                                'worlds', 'turtlebot3_dqn_stage1.world')
        
    
    gazebo_launch_path = os.path.join(get_package_share_path('gazebo_ros'), 
                                      'launch', 'gazebo.launch.py')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)



    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description' : robot_description }]
    )

    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'r2', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0'],
                output='screen'
            )
        ]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz2_node = Node(
        package="rviz2",
        executable='rviz2',
        arguments=['-d', rviz_cofig_path],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        # launch_arguments={'world': world_path}.items()
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        gazebo_launch,
        spawn_entity
    ])
