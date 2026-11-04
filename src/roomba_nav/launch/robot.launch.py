"""ロボット本体を一式起動：足回り（Pico + EKF）、LD19 LiDAR、LiDAR の取り付け位置、Pro コントローラ、Foxglove 接続口。

    ros2 launch roomba_nav robot.launch.py
    ros2 launch roomba_nav robot.launch.py lidar_yaw:=3.1416     # LiDAR の向きを変えるとき
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pico_share = get_package_share_directory('pico_bridge')
    arg = LaunchConfiguration
    return LaunchDescription([
        # LiDAR の取り付け位置：base_link（左右のタイヤの軸の真ん中・床の高さ）から見た位置 [m] と向き [rad]
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.15'),
        DeclareLaunchArgument('lidar_yaw', default_value='1.5708'),   # 実機で確認済み（LD19 の正面が左を向いて付いている）
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('teleop', default_value='true', description='Pro コントローラで操作できるようにする'),
        DeclareLaunchArgument('foxglove', default_value='true', description='Foxglove から見られるようにする（ポート 8765）'),

        # 足回り：pico_bridge + EKF + IMU の TF
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pico_share, 'launch', 'bringup.launch.py'))),

        # LD19 LiDAR → /scan（frame: laser）
        Node(
            package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node', name='LD19', output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD19',
                'topic_name': 'scan',
                'frame_id': 'laser',
                'port_name': arg('lidar_port'),
                'port_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0,
            }],
        ),
        # LD19 は 1 周ごとに点の数が変わるので、いつも 500 点にそろえた /scan_fixed も出す（slam_toolbox と Nav2 用）
        # ついでにノイズも消す：min_range より近い点（ロボット自身）と、ぽつんと 1 点だけ離れた点
        Node(
            package='laser_tools', executable='scan_normalizer', name='scan_normalizer',
            parameters=[{
                'input_topic': 'scan', 'output_topic': 'scan_fixed', 'num_bins': 500,
                'min_range': 0.15,             # [m] これより近い点は捨てる
                'remove_isolated': True,       # ぽつんと離れた点を消す（False で消さない）
                'isolated_window': 2,          # 前後この数のマスに
                'isolated_tolerance': 0.08,    # この距離 [m] 以内の点がなければノイズとみなす
            }],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='laser_tf',
            arguments=['--x', arg('lidar_x'), '--y', arg('lidar_y'), '--z', arg('lidar_z'), '--yaw', arg('lidar_yaw'),
                       '--frame-id', 'base_link', '--child-frame-id', 'laser'],
        ),

        # Pro コントローラ（ZL＝ボタン 7 を押している間だけ動く）
        Node(
            package='joy', executable='joy_node', name='joy_node', condition=IfCondition(arg('teleop')),
            parameters=[{'deadzone': 0.15, 'autorepeat_rate': 20.0}],
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
            condition=IfCondition(arg('teleop')),
            parameters=[{
                'enable_button': 7,
                'axis_linear.x': 1,
                'axis_angular.yaw': 2,
                'scale_linear.x': 0.3,
                'scale_angular.yaw': 1.5,
                'publish_stamped_twist': False,
            }],
        ),

        # ブラシ：R でメイン、ZR でサイド（押しているあいだだけ回る）
        Node(
            package='roomba_tasks', executable='joy_brush', name='joy_brush', condition=IfCondition(arg('teleop')),
            parameters=[{'main_button': 6, 'side_button': 8, 'main_power': -0.8, 'side_power': 0.6}],
        ),

        # Foxglove（Windows の Foxglove アプリから ws://<ラズパイの IP>:8765 で見る）
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('foxglove_bridge'), 'launch',
                                                    'foxglove_bridge_launch.xml')),
            condition=IfCondition(arg('foxglove')),
        ),
    ])
