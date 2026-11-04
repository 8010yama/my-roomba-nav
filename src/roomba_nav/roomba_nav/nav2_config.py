"""Nav2 の設定ファイルを、このロボット用に書き換えて作る（launch ファイルから使う）。

Nav2 に入っている標準の nav2_params.yaml を読み、必要な所だけ変えて /tmp に書き出す。
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory


def _set_leaf(d, key, value):
    """入れ子の dict の中にある key を全部 value にする（RewrittenYaml と同じ動き）"""
    if isinstance(d, dict):
        for k, v in d.items():
            if k == key and not isinstance(v, dict):
                d[k] = value
            else:
                _set_leaf(v, key, value)
    elif isinstance(d, list):
        for v in d:
            _set_leaf(v, key, value)


# 経路に沿って走るのが得意なコントローラ（くまなく掃除のジグザグ経路向き）
RPP = {
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'desired_linear_vel': 0.20,            # [m/s] 走る速さ
    'lookahead_dist': 0.18,                # [m] この距離だけ先の経路上の点を目指す（短いほど折り返しで角を切らない）
    'min_lookahead_dist': 0.12,
    'max_lookahead_dist': 0.40,
    'lookahead_time': 1.5,
    'use_velocity_scaled_lookahead_dist': False,
    'use_rotate_to_heading': True,         # 目指す点が 45° 以上横にあれば、その場で回ってから進む（折り返し）
    'rotate_to_heading_min_angle': 0.785,
    'rotate_to_heading_angular_vel': 1.0,
    'max_angular_accel': 3.2,
    'allow_reversing': False,
    'max_robot_pose_search_dist': 1.0,     # 経路上の今の位置を、この道のりの先までしか探さない（となりのレーンと間違えない）
    'use_interpolation': True,
    'transform_tolerance': 0.2,
    'min_approach_linear_velocity': 0.05,
    'approach_velocity_scaling_dist': 0.3,
    'use_collision_detection': True,
    'max_allowed_time_to_collision_up_to_carrot': 1.0,
    'use_regulated_linear_velocity_scaling': True,
    'regulated_linear_scaling_min_radius': 0.5,
    'regulated_linear_scaling_min_speed': 0.08,
    'use_cost_regulated_linear_velocity_scaling': False,
}


def write_nav2_params(robot_radius=0.125, inflation_radius=0.18, controller='rpp', goal_tolerance=0.08,
                      local_size=1, local_resolution=0.05, cost_scaling=10.0):
    src = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')
    with open(src) as f:
        p = yaml.safe_load(f)

    _set_leaf(p, 'use_sim_time', False)
    _set_leaf(p, 'robot_radius', float(robot_radius))           # ロボットの半径 [m]（両方のコストマップ）
    _set_leaf(p, 'inflation_radius', float(inflation_radius))   # 壁からこの距離までは「なるべく近づかない」
    _set_leaf(p, 'odom_topic', '/odometry/filtered')            # EKF の結果を使う
    _set_leaf(p, 'obstacle_min_range', 0.12)                    # LiDAR が自分の体を見ても障害物にしない
    _set_leaf(p, 'raytrace_min_range', 0.12)
    _set_leaf(p, 'topic', '/scan_fixed')                        # 障害物もノイズを消したスキャンで見る
    _set_leaf(p, 'cost_scaling_factor', float(cost_scaling))    # 大きいほど壁から離れるとすぐ安全とみなす（壁ぎわを走れる）
    # ローカルマップ：ロボットのまわり local_size [m] 四方（Humble は整数 m のみ。最小 1）を細かいマスで見る
    lc = p['local_costmap']['local_costmap']['ros__parameters']
    lc['width'] = max(1, int(local_size))
    lc['height'] = max(1, int(local_size))
    lc['resolution'] = float(local_resolution)

    cs = p['controller_server']['ros__parameters']
    if controller == 'rpp':
        cs['FollowPath'] = dict(RPP)
    # ゴールに着いたとみなす距離 [m]
    for name, v in cs.items():
        if isinstance(v, dict) and 'xy_goal_tolerance' in v:
            v['xy_goal_tolerance'] = float(goal_tolerance)

    out = os.path.join(tempfile.gettempdir(), 'roomba_nav2_params.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(p, f, sort_keys=False)
    return out
