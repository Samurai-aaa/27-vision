"""虚拟相机：读本地视频按 fps 发布 /image。

上真车时这个节点换成真实相机驱动即可，detector 只认 /image 不受影响。
换视频 / 换机器：ros2 launch detector video_player.launch.py video_path:=<你的视频>
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # 约定：视频在工作区外一层 video_input/（本机示例路径，换机器记得改这里）
    default_video = os.path.expanduser('~/Workspaces/27-vision-dev/video_input/red.avi')

    return LaunchDescription([
        DeclareLaunchArgument('video_path', default_value=default_video,
                              description='视频/摄像头源，-s 可覆盖'),
        DeclareLaunchArgument('fps', default_value='30.0',
                              description='发布帧率；<=0 取视频自身帧率'),

        Node(
            package='detector',
            executable='video_player_node',
            name='video_player',
            output='screen',
            parameters=[{
                'video_path': LaunchConfiguration('video_path'),
                'fps': PythonExpression(['float("', LaunchConfiguration('fps'), '")']),
                'loop': True,                       # 循环播放便于持续联调
                'frame_id': 'camera_optical_frame',
            }],
        ),
    ])
