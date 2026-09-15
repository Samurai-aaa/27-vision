"""启动 simple_tracker_node，参数从 config/simple_tracker.yaml 加载（不用每次手敲）。

跑需求①② 的单板普通 KF（CV/CA），独立话题 /simple_tracker/target + /simple_tracker/final_img，
可与整车 tracker_node 并行（两者都订阅 /image 与 /armors，输出互不冲突）。
换模型：默认读 yaml 的 model（CV/CA），可仅本次覆盖 model:=CA。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tracker')
    camera_params = os.path.join(pkg_share, 'config', 'camera.yaml')
    params_file = os.path.join(pkg_share, 'config', 'simple_tracker.yaml')
    with open(params_file) as f:
        st = yaml.safe_load(f)['simple_tracker_node']['ros__parameters']

    # 覆盖默认 = yaml 值：不传 model:= 时等价于直接读 yaml
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=str(st.get('model', 'CV')),
                              description="运动模型：'CV' 匀速 / 'CA' 匀加速（默认读 yaml，可覆盖）"),
        Node(
            package='tracker',
            executable='simple_tracker_node',
            name='simple_tracker_node',
            output='screen',
            parameters=[camera_params, params_file, {'model': LaunchConfiguration('model')}],
        ),
    ])
