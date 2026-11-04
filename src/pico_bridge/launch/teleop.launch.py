"""足回り一式（bringup.launch.py）+ Pro コントローラで手動操作。
ZR などの「押している間だけ動く」ボタン（enable_button）を押しながら、左スティックで前後・右スティックで旋回。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('pico_bridge')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bringup.launch.py'))),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{'deadzone': 0.15, 'autorepeat_rate': 20.0}],   # スティックの真ん中付近 15% は 0 とみなす
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
            parameters=[{
                # 以前 Pro コントローラで使っていた設定
                'enable_button': 7,
                'axis_linear.x': 1,
                'axis_angular.yaw': 2,
                'scale_linear.x': 0.3,        # 最大 0.3 m/s（pico_bridge の max_linear と同じ）
                'scale_angular.yaw': 1.5,     # 最大 1.5 rad/s
                'publish_stamped_twist': False,   # pico_bridge は Twist を受け取る
            }],
        ),
    ])
