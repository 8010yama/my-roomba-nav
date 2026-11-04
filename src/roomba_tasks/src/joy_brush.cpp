// コントローラのボタンでブラシを回すノード（名前 joy_brush）。押しているあいだだけ回る。
//   受け取る: /joy (sensor_msgs/Joy)
//   出す:     /brush (std_msgs/Float32MultiArray [メイン, サイド]) → pico_bridge が Pico へ送る
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class JoyBrush : public rclcpp::Node
{
public:
  JoyBrush()
  : Node("joy_brush")
  {
    main_button_ = declare_parameter<int>("main_button", 6);       // R
    side_button_ = declare_parameter<int>("side_button", 8);       // ZR
    main_power_ = declare_parameter<double>("main_power", -0.8);   // メインブラシの強さ（-1〜1、マイナスで逆回転）
    side_power_ = declare_parameter<double>("side_power", 0.6);    // サイドブラシの強さ
    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("brush", 10);
    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr m) {on_joy(*m);});
    timer_ = create_wall_timer(100ms, [this]() {tick();});
    RCLCPP_INFO(get_logger(), "ブラシ: メイン=ボタン%d（%.2f）、サイド=ボタン%d（%.2f）",
      main_button_, main_power_, side_button_, side_power_);
  }

private:
  static bool pressed(const sensor_msgs::msg::Joy & m, int i)
  {
    return i >= 0 && i < static_cast<int>(m.buttons.size()) && m.buttons[i] == 1;
  }

  void on_joy(const sensor_msgs::msg::Joy & m)
  {
    main_ = pressed(m, main_button_);
    side_ = pressed(m, side_button_);
    last_joy_ = std::chrono::steady_clock::now();
    have_joy_ = true;
  }

  void tick()
  {
    // コントローラが切れて /joy が 1 秒来なくなったら、離したのと同じ扱いにする
    const bool fresh = have_joy_ && std::chrono::steady_clock::now() - last_joy_ < 1s;
    const bool on = fresh && (main_ || side_);
    if (on || was_on_) {
      std_msgs::msg::Float32MultiArray m;
      m.data = {on && main_ ? static_cast<float>(main_power_) : 0.0f,
        on && side_ ? static_cast<float>(side_power_) : 0.0f};
      pub_->publish(m);   // 離した瞬間は [0, 0] を 1 回送ってすぐ止める
    }
    was_on_ = on;
  }

  int main_button_, side_button_;
  double main_power_, side_power_;
  bool main_ = false, side_ = false, was_on_ = false, have_joy_ = false;
  std::chrono::steady_clock::time_point last_joy_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyBrush>());
  rclcpp::shutdown();
  return 0;
}
