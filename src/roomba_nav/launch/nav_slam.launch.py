"""地図を作りながら自律走行：ロボット本体 + slam_toolbox（地図と自己位置）+ Nav2（経路計画と走行）+ tasks。

    ros2 launch roomba_nav nav_slam.launch.py
Foxglove の 3D パネルで地図の上に「ゴール」を置くと（/goal_pose）、そこまで自分で走る。
ウェイポイント・くまなく掃除（/tasks/...）も使える（地図がまだ途中なら、見えている所だけ）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nav2(context):
    from roomba_nav.nav2_config import write_nav2_params
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    params = write_nav2_params(robot_radius=float(arg('robot_radius')), inflation_radius=float(arg('inflation_radius')),
                               controller=arg('controller'))
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch',
                                                   'navigation_launch.py')),
        launch_arguments={'params_file': params, 'use_sim_time': 'false', 'autostart': 'true'}.items(),
    )]


def generate_launch_description():
    share = get_package_share_directory('roomba_nav')
    arg = LaunchConfiguration

    return LaunchDescription([
        DeclareLaunchArgument('robot_radius', default_value='0.125'),
        DeclareLaunchArgument('inflation_radius', default_value='0.18'),
        DeclareLaunchArgument('controller', default_value='rpp', description='rpp（経路に沿うのが得意）か dwb'),
        DeclareLaunchArgument('tasks', default_value='true'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'robot.launch.py'))),

        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
            parameters=[os.path.join(share, 'config', 'slam.yaml')],
        ),

        # 足回りと地図が立ち上がってから Nav2 を起動
        TimerAction(period=5.0, actions=[OpaqueFunction(function=_nav2)]),

        Node(
            package='roomba_tasks', executable='task_manager', name='tasks', output='screen',
            parameters=[os.path.join(share, 'config', 'tasks.yaml'),
                        {'robot_radius': ParameterValue(arg('robot_radius'), value_type=float)}],
            condition=IfCondition(arg('tasks')),
        ),
    ])
