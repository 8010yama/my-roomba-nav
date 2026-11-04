"""足回り一式：Pico ブリッジ + EKF（robot_localization）+ IMU の取り付け位置（静的 TF）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('pico_bridge')
    return LaunchDescription([
        Node(
            package='pico_bridge', executable='bridge', name='pico_bridge', output='screen',
            parameters=[os.path.join(share, 'config', 'pico_bridge.yaml')],
        ),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
            parameters=[os.path.join(share, 'config', 'ekf.yaml')],
        ),
        # IMU の取り付け位置：base_link（左右のタイヤの軸の真ん中）から見た位置 [m]。実物に合わせて書き換える。
        # IMU の x 軸がロボットの前、z 軸が上を向いている前提。違う向きなら --yaw などで回す。
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='imu_tf',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.05',
                       '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        ),
        # LiDAR を付けたら、同じように base_link → laser の位置を足す（例）
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', name='laser_tf',
        #     arguments=['--x', '0.0', '--y', '0.0', '--z', '0.15',
        #                '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        # ),
    ])
