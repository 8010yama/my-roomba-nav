# pico_bridge（足回り ↔ ROS 2、C++ 版）

Raspberry Pi 5 の ROS 2 と、足回りの Pico 2 を USB でつなぐパッケージです。ノードは C++（rclcpp）で、launch と設定ファイルは ROS 2 の標準どおり Python / YAML です。

| 向き | トピック | 型 | 中身 |
|---|---|---|---|
| 受け取る | `/cmd_vel` | geometry_msgs/Twist | 進む速さ `linear.x` [m/s]、回る速さ `angular.z` [rad/s] |
| 出す | `/odom` | nav_msgs/Odometry | タイヤのエンコーダから計算した位置と速さ |
| 出す | `/imu/data` | sensor_msgs/Imu | 角速度・加速度（姿勢は出さない） |
| 出す（EKF） | `/odometry/filtered` | nav_msgs/Odometry | `/odom` と IMU を合わせた結果。**Nav2 にはこれを使う** |
| 出す（EKF） | TF `odom → base_link` | | robot_localization が出す |
| 出す | TF `base_link → imu_link` | | IMU の取り付け位置（launch の中で設定） |

## 1. 準備（ラズパイで1回だけ）

```bash
sudo apt install ros-$ROS_DISTRO-robot-localization python3-serial
sudo usermod -aG dialout $USER     # USB シリアルを使えるようにする（このあと一度ログインし直す）
```

## 2. ビルド

```bash
cd ~/roomba_ws
rm -rf build/pico_bridge install/pico_bridge     # Python 版から入れ替えるときだけ（1回）
colcon build --packages-select pico_bridge
source install/setup.bash
```

テスト（任意）：`colcon test --packages-select pico_bridge && colcon test-result --verbose`

## 3. 寸法を書き込む

`config/pico_bridge.yaml` の3つを実物に合わせて書き換え、もう一度 `colcon build` します（整数で書いても小数で書いても OK）。

- `wheel_diameter`：タイヤの直径 [m]
- `track_width`：左右のタイヤの中心どうしの距離 [m]
- `ticks_per_rev`：タイヤ1回転のカウント数（Thonny で `Z` → タイヤを手で1回転 → `?`）

IMU の取り付け位置は `launch/bringup.launch.py` の `imu_tf` で書き換えます。

## 4. 動かす

Pico に `main.py` を書き込んで USB でラズパイにつなぎます（Thonny は閉じておく）。

```bash
ros2 launch pico_bridge bringup.launch.py
```

起動して最初の 2 秒はジャイロのずれを測るので、ロボットを動かさないでください。

確認：

```bash
ros2 topic hz /odom                 # 約 50 Hz
ros2 topic echo /imu/data --once
ros2 topic echo /odometry/filtered --once
# キーボードで動かす（タイヤを浮かせて最初に試す）
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 安全のしくみ

- `/cmd_vel` が 0.5 秒来なくなると、ブリッジが停止の指令を送ります。
- ブリッジやケーブルが止まっても、Pico 自身が 0.5 秒で止まります。
- 速さは `max_linear`・`max_angular` で上限をかけています。

## Nav2 につなぐときの設定

- オドメトリのトピック：`/odometry/filtered`（`odom_topic` を書き換える）
- ロボットの土台のフレーム：`base_link`、オドメトリのフレーム：`odom`
- 自律移動には、このほかに LiDAR の `/scan` と、`base_link → laser` の TF が必要です（launch にひな形あり）。

## コントローラで動かす（Pro コントローラ）

```bash
sudo apt install ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-teleop-twist-joy
ros2 launch pico_bridge teleop.launch.py
```

`enable_button`（7番）を押している間だけ、左スティック上下で前後、右スティック左右で旋回します。ボタン番号が違うときは `ros2 topic echo /joy` で確かめて、`launch/teleop.launch.py` を書き換えます。
