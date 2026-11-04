// /scan（1周ごとに点の数が変わる LD19 など）を、いつも同じ点の数の /scan_fixed に並べ直すノード。
// slam_toolbox は点の数が変わるスキャンを受け付けないので、その前に挟む。
#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "laser_tools/resample.hpp"

class ScanNormalizer : public rclcpp::Node
{
public:
  ScanNormalizer()
  : Node("scan_normalizer")
  {
    const std::string in = declare_parameter<std::string>("input_topic", "scan");
    const std::string out = declare_parameter<std::string>("output_topic", "scan_fixed");
    const int64_t bins = declare_parameter<int64_t>("num_bins", 500);
    num_bins_ = static_cast<std::size_t>(bins > 10 ? bins : 10);
    // これより近い点は捨てる [m]。ロボット自身の体（支柱・配線）を障害物と見まちがえないため
    min_range_ = declare_parameter<double>("min_range", 0.15);
    // ぽつんと1点だけの点（ノイズ）を消すか。前後 isolated_window マスに isolated_tolerance [m] 以内の点がなければ消す
    remove_isolated_ = declare_parameter<bool>("remove_isolated", true);
    const int64_t win = declare_parameter<int64_t>("isolated_window", 2);
    isolated_window_ = static_cast<std::size_t>(win > 0 ? win : 0);
    isolated_tolerance_ = declare_parameter<double>("isolated_tolerance", 0.08);
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(out, 10);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      in, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {on_scan(*msg);});
    RCLCPP_INFO(get_logger(), "%s → %s（いつも %zu 点、%.2f m より近い点とぽつんと離れた点%sを消す）",
      in.c_str(), out.c_str(), num_bins_, min_range_, remove_isolated_ ? "" : "（オフ）");
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan & in)
  {
    const double rmin = std::max(static_cast<double>(in.range_min), min_range_);
    auto r = laser_tools::resample(
      in.angle_min, in.angle_increment, in.ranges, in.intensities, rmin, in.range_max, num_bins_);
    if (remove_isolated_) {
      laser_tools::remove_isolated(r.ranges, r.intensities, isolated_window_, isolated_tolerance_);
    }
    sensor_msgs::msg::LaserScan out;
    out.header = in.header;
    out.angle_min = static_cast<float>(r.angle_min);
    out.angle_increment = static_cast<float>(r.angle_increment);
    out.angle_max = static_cast<float>(r.angle_min + r.angle_increment * static_cast<double>(num_bins_ - 1));
    out.scan_time = in.scan_time;
    out.time_increment = in.scan_time > 0.0f ? in.scan_time / static_cast<float>(num_bins_) : 0.0f;
    out.range_min = static_cast<float>(rmin);
    out.range_max = in.range_max;
    out.ranges = r.ranges;
    out.intensities = r.intensities;
    pub_->publish(out);
  }

  std::size_t num_bins_;
  double min_range_;
  bool remove_isolated_;
  std::size_t isolated_window_;
  double isolated_tolerance_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanNormalizer>());
  rclcpp::shutdown();
  return 0;
}
