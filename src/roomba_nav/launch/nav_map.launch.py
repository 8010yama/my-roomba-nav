"""保存した地図の上で自律走行：ロボット本体 + map_server（地図）+ AMCL（自分の位置）+ Nav2 + tasks（ウェイポイント・くまなく掃除）。

    ros2 launch roomba_nav nav_map.launch.py
    ros2 launch roomba_nav nav_map.launch.py map:=$HOME/roomba_ws/maps/room.yaml
    ros2 launch roomba_nav nav_map.launch.py controller:=dwb      # 前と同じ DWB で走る

地図を作り始めた場所に、同じ向きで置いてから起動すると、そのまま位置が合う。
違う場所から始めるときは、Foxglove の Publish →「Pose estimate」（トピック /initialpose）で今の位置と向きを教える。
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
    num = lambda n: ParameterValue(LaunchConfiguration(n), value_type=float)  # noqa: E731

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/roomba_ws/maps/room.yaml'),
                              description='保存した地図（map_saver_cli で作った .yaml）'),
        DeclareLaunchArgument('x', default_value='0.0', description='起動したときにいる場所 [m]（地図の座標）'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='起動したときの向き [rad]'),
        DeclareLaunchArgument('robot_radius', default_value='0.125'),
        DeclareLaunchArgument('inflation_radius', default_value='0.18'),
        DeclareLaunchArgument('controller', default_value='rpp', description='rpp（経路に沿うのが得意）か dwb'),
        DeclareLaunchArgument('tasks', default_value='true', description='ウェイポイント・くまなく掃除のノードを起動する'),
        DeclareLaunchArgument('localize', default_value='true',
                              description='起動したら、その場で回って地図全体から自分の位置を探す（/clean でも掃除の前に行う）'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'robot.launch.py'))),

        # 地図を配る
        Node(
            package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[{'use_sim_time': False, 'yaml_filename': arg('map')}],
        ),
        # 自分の位置を推定（map → odom）
        Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[os.path.join(share, 'config', 'amcl.yaml'),
                        {'initial_pose.x': num('x'), 'initial_pose.y': num('y'), 'initial_pose.yaw': num('yaw')}],
        ),
        # 上の 2 つを順番に起動する（Nav2 のノードは「ライフサイクル」で起動・停止する）
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'amcl']}],
        ),

        # 足回りと地図が立ち上がってから Nav2 を起動
        TimerAction(period=5.0, actions=[OpaqueFunction(function=_nav2)]),

        # 自己位置を地図全体から探す（起動したとき・/localize・/clean）
        Node(
            package='roomba_tasks', executable='localize', name='localize', output='screen',
            parameters=[{'run_on_start': ParameterValue(arg('localize'), value_type=bool)}],
        ),

        Node(
            package='roomba_tasks', executable='task_manager', name='tasks', output='screen',
            parameters=[os.path.join(share, 'config', 'tasks.yaml'), {'robot_radius': num('robot_radius')}],
            condition=IfCondition(arg('tasks')),
        ),
    ])
