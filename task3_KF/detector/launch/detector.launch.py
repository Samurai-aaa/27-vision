"""装甲板检测节点：订阅 /image → PnP 解算 → 发布 /armors 等。

需先 source OpenVINO 环境再 launch（detector 依赖 openvino 运行库）。
参数默认值读 config/detector.yaml（颜色 / 推理设备 / debug 开关），任一项都可本次
临时覆盖：detect_color:=1 device:=GPU debug:=false。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # 与 video_player.launch.py 同款惯例：config yaml 随包安装，launch 用
    # get_package_share_directory 定位后再整文件作为 parameters 传入节点
    pkg_share = get_package_share_directory('detector')
    camera_params = os.path.join(pkg_share, 'config', 'camera.yaml')
    params_file = os.path.join(pkg_share, 'config', 'detector.yaml')
    with open(params_file) as f:
        dp = yaml.safe_load(f)['detector']['ros__parameters']

    # CLI 覆盖默认 = yaml 值：不传参时和直接读 yaml 等价，传参时仅本次覆盖
    return LaunchDescription([
        DeclareLaunchArgument('detect_color', default_value=str(dp['detect_color']),
                              description='敌方颜色：0=红(打红车) 1=蓝(打蓝车)（默认读 config/detector.yaml）'),
        DeclareLaunchArgument('device', default_value=str(dp['device']),
                              description='OpenVINO 推理设备：CPU/GPU/AUTO（默认读 config/detector.yaml）'),
        DeclareLaunchArgument('debug', default_value=str(dp['debug']).lower(),
                              description='发布检测标注图 /armor_detector/final_img（默认读 config/detector.yaml）'),

        Node(
            package='detector',
            executable='detector_node',
            name='armor_detector',
            output='screen',
            parameters=[camera_params, params_file, {
                'detect_color': PythonExpression(['int("', LaunchConfiguration('detect_color'), '")']),
                'device': LaunchConfiguration('device'),
                'confidence_threshold': dp.get('confidence_threshold', 0.35),
                'nms_threshold': dp.get('nms_threshold', 0.45),
                'debug': PythonExpression(['str("', LaunchConfiguration('debug'), '").lower()=="true"']),
            }],
        ),
    ])
