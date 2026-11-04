from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ekf_config = os.path.join(
        get_package_share_directory('roomba_bringup'),
        'config', 'ekf.yaml'
    )

    return LaunchDescription([
        # Picoにリセットコマンドを送る
        ExecuteProcess(
            cmd=['python3', '-c',
                 'import serial, time; s=serial.Serial("/dev/ttyACM0", 115200); time.sleep(1.0); s.write(b"R\\n"); s.flush(); print("Pico reset sent")'],
            output='screen',
        ),

        # Joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),

        # Teleop
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'enable_button': 7,
                'axis_linear.x': 1,
                'axis_angular.yaw': 2,
                'scale_linear.x': 0.5,
                'scale_angular.yaw': 3.0,
                'publish_stamped_twist': False,
            }],
        ),

        # Pico bridge（リセット後に起動）
        TimerAction(period=2.0, actions=[
            ExecuteProcess(
                cmd=['python3', '/home/alia/roomba_ws/install/pico_bridge/bin/pico_bridge_node'],
                output='screen',
            ),
        ]),

        # EKF（pico_bridge起動後に起動）
        TimerAction(period=3.0, actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                parameters=[ekf_config],
            ),
        ]),
    ])
