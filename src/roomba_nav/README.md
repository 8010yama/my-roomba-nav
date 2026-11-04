# roomba_nav（自作ルンバ：ロボット一式の起動と自律走行）

## 入れるもの（コンテナの中で1回だけ）

```bash
sudo apt update
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-foxglove-bridge
```

`~/roomba_ws/src` に `pico_bridge`・`ldlidar_stl_ros2`・`laser_tools`・`roomba_tasks`・`roomba_nav` を置いてビルド：

```bash
cd ~/roomba_ws && colcon build --packages-select laser_tools roomba_tasks roomba_nav
source install/setup.bash
```

## 1. ロボットだけ起動（コントローラ操作 + LiDAR + Foxglove）

```bash
ros2 launch roomba_nav robot.launch.py
```

Windows の Foxglove で `ws://192.168.3.4:8765` に接続 → 3D パネル。表示フレーム `base_link` で `/scan` を表示。
**ロボットの真正面に物を置いて、それが画面の +x（赤い軸の向き）に出るか確かめる。**
ずれていたら LiDAR の向きを launch 引数で直す：

LiDAR の向きは実機で確認済み（`lidar_yaw` の標準値 1.5708）。付け直したときはこの表で直す：

| 正面の物が出た場所 | 引数 |
|---|---|
| 真後ろ（-x） | `lidar_yaw:=3.1416` |
| 左（+y） | `lidar_yaw:=-1.5708` |
| 右（-y） | `lidar_yaw:=1.5708` |

LiDAR の取り付け位置は `lidar_x`（前がプラス）・`lidar_y`（左がプラス）・`lidar_z`（高さ）[m]。

## 2. 地図を作りながら自律走行

```bash
ros2 launch roomba_nav nav_slam.launch.py
```

- 最初はコントローラ（ボタン 7 + スティック）で部屋をゆっくり一周すると地図（`/map`）ができる
- Foxglove の 3D パネル（表示フレーム `map`）で、上のツールバーの「Publish」→「Pose」を選び、トピックを `/goal_pose` にして地図の上でクリック＆ドラッグ → そこまで自分で走る
- 止めたいときはコントローラのボタン 7 を押す（手動が優先されるわけではないので、危ないときは持ち上げるか Ctrl+C）
- ロボットの大きさ：`robot_radius:=0.15`（半径 m）、壁からの余裕：`inflation_radius:=0.30`

## 3. 地図を保存（あとで使うとき）

```bash
mkdir -p ~/roomba_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/roomba_ws/maps/home
```

## 4. 保存した地図の上で走る（地図は作らない。AMCL で自分の位置を出す）

```bash
ros2 launch roomba_nav nav_map.launch.py
```

- 地図は `~/roomba_ws/maps/room.yaml`（別の地図なら `map:=...`）
- **地図を作り始めた場所に、同じ向きで置いてから起動** すると、そのまま位置が合う
- 違う場所から始めるとき：Foxglove の Publish →「Pose estimate」（トピック `/initialpose`）で、今いる場所と向きをクリック＆ドラッグ
- スキャン（`/scan_fixed`）が地図の壁にぴったり重なっていれば位置が合っている
- コントローラ：標準は `rpp`（経路に沿って走るのが得意）。前の DWB に戻すなら `controller:=dwb`

## 5. ウェイポイントで回る・範囲をくまなく掃除する（tasks ノード）

Foxglove の 3D パネルで Publish →「Point」（トピック `/clicked_point`）にして、地図の上をクリックすると点が置かれる（緑の点と番号）。

| サービス | すること |
|---|---|
| `/tasks/go_waypoints` | 置いた点を番号順に回る |
| `/tasks/preview_coverage` | 点で囲んだ範囲（3 点未満なら地図全体）をくまなく走る経路を作って、`/tasks/plan` に表示するだけ |
| `/tasks/go_coverage` | 経路を作って掃除を始める |
| `/tasks/cancel` | 止める |
| `/tasks/undo`・`/tasks/clear` | 最後の点を消す・全部消す |
| `/tasks/save`・`/tasks/load` | 点を `~/roomba_ws/maps/points.txt` に保存・読み込み（起動時に自動で読み込む） |

```bash
ros2 service call /tasks/go_coverage std_srvs/srv/Trigger
```

- 状態は `/tasks/status`、走り終えた所は `/tasks/done`
- くまなく掃除のあいだはブラシが自動で回る（強さは `config/tasks.yaml` の `brush_main`・`brush_side`）
- ブラシだけ回す・止める：`/tasks/brush_on`・`/tasks/brush_off`。直接なら `ros2 topic pub -r 5 /brush std_msgs/msg/Float32MultiArray "{data: [0.5, 0.5]}"`（1 秒途切れると止まる）
- レーンの間隔などは `config/tasks.yaml`
- 地図を作りながら（`nav_slam.launch.py`）でも使える
