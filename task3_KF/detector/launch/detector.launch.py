"""装甲板检测节点：订阅 /image → PnP 解算 → 发布 /armors 等。

需先 source OpenVINO 环境再 launch（detector 依赖 openvino 运行库）。
颜色：0=红方视频，1=蓝方视频，可用 detect_color:=1 覆盖。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('detect_color', default_value='0',
                              description='敌方颜色：0=红(打红车) 1=蓝(打蓝车)'),
        DeclareLaunchArgument('device', default_value='CPU',
                              description='OpenVINO 推理设备'),

        Node(
            package='detector',
            executable='detector_node',
            name='armor_detector',
            output='screen',
            parameters=[{
                'detect_color': PythonExpression(['int("', LaunchConfiguration('detect_color'), '")']),
                'device': LaunchConfiguration('device'),
                'debug': True,   # 发布 /armor_detector/final_img 标注图供 rqt 查看
            }],
        ),
    ])
