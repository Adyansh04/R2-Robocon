from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

#join the path of the package with the file name
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    line_follow_params = os.path.join(get_package_share_directory('line_following'),'config','line_follower.yaml')

    lsa08_due_fake_node = Node(
        package='line_following',
        executable='lsa08_due_fake.py',
        name='lsa08_due_fake',
        parameters=[line_follow_params]
    )

    line_following_node = Node(
        package='line_following',
        executable='line_following.py',
        name='line_following',
        parameters=[line_follow_params]
    )

    return LaunchDescription([
        lsa08_due_fake_node,
        line_following_node
    ])