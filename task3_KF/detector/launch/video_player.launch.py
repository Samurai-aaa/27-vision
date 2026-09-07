"""虚拟相机：读本地视频按 fps 发布 /image。

上真车时这个节点换成真实相机驱动即可，detector 只认 /image 不受影响。
换视频 / 换机器：改 config/video_player.yaml 的 video_path 一处即可（yaml 为默认源，
支持 ~/ 相对主目录）。也可仅本次临时覆盖：video_path:=<你的视频> fps:=<帧率>。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # 参数默认值从 config/video_player.yaml 读（与 tracker 同款惯例：config yaml 随包安装，
    # launch 用 get_package_share_directory 定位后再整文件作为 parameters 传入节点）
    pkg_share = get_package_share_directory('detector')
    params_file = os.path.join(pkg_share, 'config', 'video_player.yaml')
    with open(params_file) as f:
        vp = yaml.safe_load(f)['video_player']['ros__parameters']

    # CLI 覆盖默认 = yaml 值：不传参时和直接读 yaml 等价，传 video_path:=/fps:= 时仅本次覆盖
    return LaunchDescription([
        DeclareLaunchArgument('video_path', default_value=str(vp['video_path']),
                              description='视频/摄像头源（默认读 config/video_player.yaml，可覆盖）'),
        DeclareLaunchArgument('fps', default_value=str(vp.get('fps', 30.0)),
                              description='发布帧率；<=0 取视频自身帧率（默认读 yaml，可覆盖）'),

        Node(
            package='detector',
            executable='video_player_node',
            name='video_player',
            output='screen',
            parameters=[params_file, {
                'video_path': LaunchConfiguration('video_path'),
                'fps': PythonExpression(['float("', LaunchConfiguration('fps'), '")']),
            }],
        ),
    ])
