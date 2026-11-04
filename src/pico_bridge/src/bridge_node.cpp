// Pico 2（足回り）と ROS 2 をつなぐノード（C++ / rclcpp）。
//   受け取る: /cmd_vel (geometry_msgs/Twist)、/brush (std_msgs/Float32MultiArray：[メイン, サイド] の強さ -1.0〜1.0)
//   出す    : /odom (nav_msgs/Odometry)、/imu/data (sensor_msgs/Imu)、TF odom→base_link（publish_tf: true のときだけ）
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "pico_bridge/diffdrive.hpp"

using namespace std::chrono_literals;

class PicoBridge : public rclcpp::Node
{
public:
  PicoBridge()
  : Node("pico_bridge")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    wheel_diameter_ = num_param("wheel_diameter", 0.065);
    track_width_ = num_param("track_width", 0.20);
    ticks_per_rev_ = num_param("ticks_per_rev", 1000.0);
    telemetry_hz_ = num_param("telemetry_hz", 50.0);
    cmd_timeout_ = num_param("cmd_timeout", 0.5);
    max_linear_ = num_param("max_linear", 0.3);
    max_angular_ = num_param("max_angular", 2.0);
    deadband_linear_ = num_param("deadband_linear", 0.02);
    deadband_angular_ = num_param("deadband_angular", 0.1);
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
    gyro_calib_sec_ = num_param("gyro_calibration_sec", 2.0);
    brush_timeout_ = num_param("brush_timeout", 1.0);   // /brush が途切れてからブラシを止めるまで [秒]

    odom_ = std::make_unique<pico_bridge::DiffDriveOdom>(wheel_diameter_, track_width_, ticks_per_rev_);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
    if (publish_tf_) {
      tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {on_cmd_vel(*msg);});
    brush_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "brush", 10, [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {on_brush(*msg);});

    poll_timer_ = create_wall_timer(5ms, [this]() {poll_serial();});        // 受信（200Hz）
    keep_timer_ = create_wall_timer(50ms, [this]() {keepalive();});         // 指令の送り直し（20Hz）
    open_timer_ = create_wall_timer(1s, [this]() {ensure_open();});         // つながっていなければ開き直す
    ensure_open();
  }

  ~PicoBridge() override
  {
    send("B 0 0");
    send("S");
    send("T 0");
    close_port();
  }

private:
  // 整数でも小数でも受け付ける数値パラメータ（yaml に 1200 と書いても 1200.0 と書いても OK）
  double num_param(const std::string & name, double def)
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.dynamic_typing = true;
    const rclcpp::ParameterValue v = declare_parameter(name, rclcpp::ParameterValue(def), d);
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(v.get<int64_t>());
    }
    return v.get<double>();
  }

  // ---------------- シリアル ----------------
  void ensure_open()
  {
    if (fd_ >= 0) {return;}
    int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Pico に接続できません（%s）: %s",
        port_.c_str(), std::strerror(errno));
      return;
    }
    termios tio{};
    if (tcgetattr(fd, &tio) == 0) {
      cfmakeraw(&tio);
      cfsetispeed(&tio, B115200);
      cfsetospeed(&tio, B115200);
      tio.c_cflag |= (CLOCAL | CREAD);
      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 0;
      tcsetattr(fd, TCSANOW, &tio);
    }
    fd_ = fd;
    rx_.clear();
    odom_->reset();
    send("S");
    send("B 0 0");
    char buf[32];
    std::snprintf(buf, sizeof(buf), "T %.1f", telemetry_hz_);
    send(buf);
    send("?");
    calib_active_ = true;
    calib_start_ = std::chrono::steady_clock::now();
    calib_n_ = 0;
    calib_sum_[0] = calib_sum_[1] = calib_sum_[2] = 0.0;
    RCLCPP_INFO(get_logger(), "Pico に接続しました: %s（最初の %.1f 秒はジャイロの補正のため、ロボットを動かさないでください）",
      port_.c_str(), gyro_calib_sec_);
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void lost(const char * why)
  {
    RCLCPP_ERROR(get_logger(), "Pico との通信が切れました: %s", why);
    close_port();
  }

  void send(const std::string & text)
  {
    if (fd_ < 0) {return;}
    const std::string line = text + "\n";
    const ssize_t n = ::write(fd_, line.data(), line.size());
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      lost(std::strerror(errno));
    }
  }

  void poll_serial()
  {
    if (fd_ < 0) {return;}
    char buf[4096];
    for (;;) {
      const ssize_t n = ::read(fd_, buf, sizeof(buf));
      if (n > 0) {
        rx_.append(buf, static_cast<size_t>(n));
        continue;
      }
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        lost(std::strerror(errno));
        return;
      }
      break;
    }
    size_t pos;
    while ((pos = rx_.find('\n')) != std::string::npos) {
      std::string line = rx_.substr(0, pos);
      rx_.erase(0, pos + 1);
      while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {line.pop_back();}
      if (line.empty()) {continue;}
      pico_bridge::Sample s;
      if (pico_bridge::parse_line(line, s)) {
        on_data(s);
      } else if (line[0] == 'I' || line[0] == 'E') {
        RCLCPP_INFO(get_logger(), "Pico: %s", line.c_str());
      }
    }
    if (rx_.size() > 4096) {rx_.clear();}
  }

  // ---------------- 指令 ----------------
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg)
  {
    double v = std::clamp(msg.linear.x, -max_linear_, max_linear_);
    double w = std::clamp(msg.angular.z, -max_angular_, max_angular_);
    if (std::abs(v) < deadband_linear_) {v = 0.0;}
    if (std::abs(w) < deadband_angular_) {w = 0.0;}
    cmd_v_ = v;
    cmd_w_ = w;
    cmd_time_ = now();
    have_cmd_ = true;
    send_cmd();
  }

  void send_cmd()
  {
    const auto t = odom_->wheel_targets(cmd_v_, cmd_w_);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "W %.1f %.1f", t.first, t.second);
    send(buf);
    sent_stop_ = false;
  }

  // ブラシ：[メイン, サイド]。0 で止める。マイナスで逆回転
  void on_brush(const std_msgs::msg::Float32MultiArray & msg)
  {
    brush_[0] = msg.data.size() > 0 ? std::clamp<double>(msg.data[0], -1.0, 1.0) : 0.0;
    brush_[1] = msg.data.size() > 1 ? std::clamp<double>(msg.data[1], -1.0, 1.0) : 0.0;
    brush_time_ = now();
    have_brush_ = true;
    send_brush();
  }

  void send_brush()
  {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "B %.2f %.2f", brush_[0], brush_[1]);
    send(buf);
    brush_running_ = brush_[0] != 0.0 || brush_[1] != 0.0;
  }

  // Pico は 0.5 秒指令が来ないと止まるので、/cmd_vel と /brush が新しいうちは送り続ける
  void keepalive()
  {
    if (have_brush_) {
      if ((now() - brush_time_).seconds() < brush_timeout_) {
        if (brush_[0] != 0.0 || brush_[1] != 0.0) {send_brush();}
      } else if (brush_running_) {
        brush_[0] = brush_[1] = 0.0;
        send_brush();
        RCLCPP_INFO(get_logger(), "/brush が途切れたのでブラシを止めました");
      }
    }
    if (!have_cmd_) {return;}
    const double age = (now() - cmd_time_).seconds();
    if (age < cmd_timeout_) {
      send_cmd();
    } else if (!sent_stop_) {
      send("S");
      sent_stop_ = true;
    }
  }

  // ---------------- 受信データ ----------------
  void on_data(const pico_bridge::Sample & s)
  {
    const rclcpp::Time stamp = now();
    if (odom_->update(s.ms, s.tl, s.tr)) {
      publish_odom(stamp);
    }
    if (s.has_imu) {
      handle_imu(stamp, s.imu);
    }
  }

  void publish_odom(const rclcpp::Time & stamp)
  {
    const auto & o = *odom_;
    const double qz = std::sin(o.th / 2.0), qw = std::cos(o.th / 2.0);
    nav_msgs::msg::Odometry m;
    m.header.stamp = stamp;
    m.header.frame_id = odom_frame_;
    m.child_frame_id = base_frame_;
    m.pose.pose.position.x = o.x;
    m.pose.pose.position.y = o.y;
    m.pose.pose.orientation.z = qz;
    m.pose.pose.orientation.w = qw;
    m.twist.twist.linear.x = o.v;
    m.twist.twist.angular.z = o.w;
    m.pose.covariance.fill(0.0);
    m.pose.covariance[0] = m.pose.covariance[7] = 0.01;                                   // x, y
    m.pose.covariance[14] = m.pose.covariance[21] = m.pose.covariance[28] = 1e6;          // z, roll, pitch（使わない）
    m.pose.covariance[35] = 0.05;                                                         // yaw
    m.twist.covariance.fill(0.0);
    m.twist.covariance[0] = 0.005;                                                        // vx
    m.twist.covariance[7] = 1e-4;                                                         // vy（横には動かない）
    m.twist.covariance[14] = m.twist.covariance[21] = m.twist.covariance[28] = 1e6;
    m.twist.covariance[35] = 0.02;                                                        // vyaw
    odom_pub_->publish(m);

    if (tf_pub_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = odom_frame_;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = o.x;
      t.transform.translation.y = o.y;
      t.transform.rotation.z = qz;
      t.transform.rotation.w = qw;
      tf_pub_->sendTransform(t);
    }
  }

  void handle_imu(const rclcpp::Time & stamp, const double v[6])
  {
    if (calib_active_) {                 // 接続直後はジャイロのずれ（バイアス）を測る
      calib_n_++;
      for (int i = 0; i < 3; ++i) {calib_sum_[i] += v[i];}
      const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - calib_start_).count();
      if (el >= gyro_calib_sec_ && calib_n_ > 0) {
        for (int i = 0; i < 3; ++i) {gyro_bias_[i] = calib_sum_[i] / calib_n_;}
        calib_active_ = false;
        RCLCPP_INFO(get_logger(), "ジャイロ補正: x=%+.4f y=%+.4f z=%+.4f rad/s", gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
      }
      return;
    }
    sensor_msgs::msg::Imu m;
    m.header.stamp = stamp;
    m.header.frame_id = imu_frame_;
    m.orientation_covariance.fill(0.0);
    m.orientation_covariance[0] = -1.0;              // 姿勢は出さない（角速度と加速度だけ）
    m.angular_velocity.x = v[0] - gyro_bias_[0];
    m.angular_velocity.y = v[1] - gyro_bias_[1];
    m.angular_velocity.z = v[2] - gyro_bias_[2];
    m.linear_acceleration.x = v[3];
    m.linear_acceleration.y = v[4];
    m.linear_acceleration.z = v[5];
    m.angular_velocity_covariance.fill(0.0);
    m.linear_acceleration_covariance.fill(0.0);
    for (int k : {0, 4, 8}) {
      m.angular_velocity_covariance[k] = 0.0004;
      m.linear_acceleration_covariance[k] = 0.04;
    }
    imu_pub_->publish(m);
  }

  // パラメータ
  std::string port_, odom_frame_, base_frame_, imu_frame_;
  double wheel_diameter_, track_width_, ticks_per_rev_, telemetry_hz_, cmd_timeout_;
  double max_linear_, max_angular_, deadband_linear_, deadband_angular_, gyro_calib_sec_, brush_timeout_;
  bool publish_tf_;

  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr brush_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_, keep_timer_, open_timer_;

  // 状態
  int fd_ = -1;
  std::string rx_;
  std::unique_ptr<pico_bridge::DiffDriveOdom> odom_;
  double cmd_v_ = 0.0, cmd_w_ = 0.0;
  rclcpp::Time cmd_time_;
  bool have_cmd_ = false;
  bool sent_stop_ = true;
  double brush_[2] = {0.0, 0.0};
  rclcpp::Time brush_time_;
  bool have_brush_ = false, brush_running_ = false;
  bool calib_active_ = false;
  std::chrono::steady_clock::time_point calib_start_;
  int calib_n_ = 0;
  double calib_sum_[3] = {0, 0, 0};
  double gyro_bias_[3] = {0, 0, 0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PicoBridge>();
  rclcpp::spin(node);
  node.reset();                // 止まる前に Pico へ停止を送る
  rclcpp::shutdown();
  return 0;
}
