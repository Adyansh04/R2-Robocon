
'''
TO RUN THE DOCKER, YOU NEED TO RUN THE FOLLOWING COMMANDS IN THE TERMINAL:

docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble multiserial --devs /dev/ttyUSB0 /dev/ttyACM0 -v6

Replace /dev/ttyUSB0 with the port where the ESP is connected.

'''



from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
import yaml
from launch.substitutions import LaunchConfiguration
import launch.actions


def generate_launch_description():


    imu_madwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('r2_bringup'), 'config', 'imu_filter.yaml')],
    )

    imu_complimentary_node = Node(
                package='imu_complementary_filter',
                executable='complementary_filter_node',
                name='complementary_filter_gain_node',
                output='screen',
                parameters=[
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'use_mag': False},
                    {'gain_acc': 0.01},
                    {'gain_mag': 0.01},
                    {'publish_debug_topics': True},

                ]
            
    )

    # docker_command = ExecuteProcess(
    #     cmd=['docker', 'run', '--rm', '-v', '/dev:/dev', '--privileged', '--net=host', 'microros/micro-ros-agent:humble', 'serial', '--dev', '/dev/ttyUSB0', '-v6'],
    #     output='screen'
    # )

    quat_to_euler_node = Node(
        package='r2_py',
        executable='quat_to_rpy',
        name='quat_to_euler',
        output='screen',
    )



    return launch.LaunchDescription(
        [
            imu_complimentary_node,   
            quat_to_euler_node,
            # docker_command
        ]
    )


