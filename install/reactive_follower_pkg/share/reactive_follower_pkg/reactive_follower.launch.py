import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('reactive_follower_pkg'),
        'config',
        'reactive_follower.yaml'
    )
    return LaunchDescription([
        Node(
            package='reactive_follower_pkg',
            executable='reactive_follower_node',
            name='reactive_follower',
            parameters=[config]
        )
    ])
