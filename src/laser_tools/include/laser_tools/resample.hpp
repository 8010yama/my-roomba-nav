// ROS に依存しない部分：1周ぶんの距離データを、決まった数（num_bins）の角度のマスに並べ直す。
// 出力の角度は -π 〜 π（angle_min = -π、angle_increment = 2π / num_bins）。
// 1つのマスに複数の点が入ったら近いほう、点がないマスは +inf（何も当たっていない）にする。
#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace laser_tools
{

constexpr double PI = 3.14159265358979323846;

struct Resampled
{
  double angle_min = -PI;
  double angle_increment = 0.0;
  std::vector<float> ranges;
  std::vector<float> intensities;
};

inline Resampled resample(
  double in_angle_min, double in_angle_increment,
  const std::vector<float> & in_ranges, const std::vector<float> & in_intensities,
  double range_min, double range_max, std::size_t num_bins)
{
  Resampled out;
  out.angle_increment = 2.0 * PI / static_cast<double>(num_bins);
  out.ranges.assign(num_bins, std::numeric_limits<float>::infinity());
  const bool has_int = in_intensities.size() == in_ranges.size();
  out.intensities.assign(has_int ? num_bins : 0, 0.0f);

  for (std::size_t i = 0; i < in_ranges.size(); ++i) {
    const float r = in_ranges[i];
    if (!std::isfinite(r) || r < range_min || r > range_max) {continue;}
    // 角度を -π 〜 π に直してマスの番号を出す
    double a = in_angle_min + in_angle_increment * static_cast<double>(i);
    a = std::fmod(a + PI, 2.0 * PI);
    if (a < 0) {a += 2.0 * PI;}
    // ranges[k] は角度 -π + k × 刻み の向き（LaserScan の決まり）なので、いちばん近い k に入れる
    const std::size_t bin = static_cast<std::size_t>(std::llround(a / out.angle_increment)) % num_bins;
    if (r < out.ranges[bin]) {
      out.ranges[bin] = r;
      if (has_int) {out.intensities[bin] = in_intensities[i];}
    }
  }
  return out;
}

// ぽつんと1点だけ離れている点（ノイズ）を消す。
// 前後 window マスの中に、距離の差が tolerance [m] 以内の点が1つもなければノイズとみなして +inf にする。
// 壁や家具は同じくらいの距離の点が並ぶので残り、空中に1点だけ出る点は消える。消した点の数を返す。
inline std::size_t remove_isolated(
  std::vector<float> & ranges, std::vector<float> & intensities, std::size_t window, double tolerance)
{
  const std::size_t n = ranges.size();
  if (n == 0 || window == 0) {return 0;}
  std::vector<bool> drop(n, false);
  for (std::size_t i = 0; i < n; ++i) {
    const float r = ranges[i];
    if (!std::isfinite(r)) {continue;}
    bool has_neighbor = false;
    for (std::size_t d = 1; d <= window && !has_neighbor; ++d) {
      const float a = ranges[(i + d) % n];           // 1周つながっているので端は反対側へ回る
      const float b = ranges[(i + n - d % n) % n];
      if ((std::isfinite(a) && std::fabs(a - r) <= tolerance) ||
        (std::isfinite(b) && std::fabs(b - r) <= tolerance))
      {
        has_neighbor = true;
      }
    }
    drop[i] = !has_neighbor;
  }
  std::size_t count = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (!drop[i]) {continue;}
    ranges[i] = std::numeric_limits<float>::infinity();
    if (i < intensities.size()) {intensities[i] = 0.0f;}
    count++;
  }
  return count;
}

}  // namespace laser_tools
