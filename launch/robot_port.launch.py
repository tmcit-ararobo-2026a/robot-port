"""Launch the standard-message UDP bridge with an explicit configuration file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return the standalone production bridge configuration."""
    default_config = os.path.join(
        get_package_share_directory('robot_port'), 'config', 'udp.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        Node(package='robot_port', executable='udp_node', name='udp_node',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
