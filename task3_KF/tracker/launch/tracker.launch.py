"""启动 tracker_node，参数从 config/tracker.yaml 加载（不用每次手敲）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tracker')
    params_file = os.path.join(pkg_share, 'config', 'tracker.yaml')

    return LaunchDescription([
        Node(
            package='tracker',
            executable='tracker_node',
            name='tracker_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
