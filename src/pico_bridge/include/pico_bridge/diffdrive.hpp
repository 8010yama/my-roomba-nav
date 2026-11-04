// ROS に依存しない部分：Pico からの行の解釈と、2輪のオドメトリ計算。
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

namespace pico_bridge
{

constexpr int64_t TICKS_PERIOD = int64_t(1) << 30;  // Pico が送る ms は 2^30 で一周する
constexpr double PI = 3.14159265358979323846;

// Pico からの1行 'D <ms> <左> <右> <gx> <gy> <gz> <ax> <ay> <az>'
struct Sample
{
  int64_t ms = 0;
  int64_t tl = 0;
  int64_t tr = 0;
  bool has_imu = false;
  double imu[6] = {0, 0, 0, 0, 0, 0};   // gx gy gz [rad/s], ax ay az [m/s^2]
};

inline std::vector<std::string> split(const std::string & s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
      if (!cur.empty()) {out.push_back(cur); cur.clear();}
    } else {
      cur += c;
    }
  }
  if (!cur.empty()) {out.push_back(cur);}
  return out;
}

inline bool to_int(const std::string & s, int64_t & v)
{
  char * end = nullptr;
  v = std::strtoll(s.c_str(), &end, 10);
  return end != s.c_str() && *end == '\0';
}

inline bool to_double(const std::string & s, double & v)
{
  char * end = nullptr;
  v = std::strtod(s.c_str(), &end);
  return end != s.c_str() && *end == '\0';
}

// D 行なら true。IMU が nan のときは has_imu = false。
inline bool parse_line(const std::string & line, Sample & out)
{
  const auto p = split(line);
  if (p.size() != 10 || p[0] != "D") {return false;}
  Sample s;
  if (!to_int(p[1], s.ms) || !to_int(p[2], s.tl) || !to_int(p[3], s.tr)) {return false;}
  bool nan = false;
  for (int i = 0; i < 6; ++i) {
    if (!to_double(p[4 + i], s.imu[i])) {return false;}
    if (std::isnan(s.imu[i])) {nan = true;}
  }
  s.has_imu = !nan;
  out = s;
  return true;
}

// ticks_ms の差（一周しても正しく出す）
inline int64_t ms_diff(int64_t now, int64_t old)
{
  int64_t d = (now - old) % TICKS_PERIOD;
  if (d < 0) {d += TICKS_PERIOD;}
  if (d >= TICKS_PERIOD / 2) {d -= TICKS_PERIOD;}
  return d;
}

// 左右のエンコーダの累計カウントから、位置 (x, y, θ) と速度 (v, ω) を出す
class DiffDriveOdom
{
public:
  DiffDriveOdom(double wheel_diameter, double track_width, double ticks_per_rev)
  : m_per_tick_(PI * wheel_diameter / ticks_per_rev), track_(track_width) {}

  void reset()
  {
    x = y = th = v = w = 0.0;
    has_prev_ = false;
  }

  // 位置が更新されたら true
  bool update(int64_t ms, int64_t tl, int64_t tr)
  {
    if (!has_prev_) {
      prev_ms_ = ms; prev_tl_ = tl; prev_tr_ = tr; has_prev_ = true;
      return false;
    }
    const double dt = ms_diff(ms, prev_ms_) / 1000.0;
    const double dl = (tl - prev_tl_) * m_per_tick_;
    const double dr = (tr - prev_tr_) * m_per_tick_;
    prev_ms_ = ms; prev_tl_ = tl; prev_tr_ = tr;
    if (dt <= 0.0 || dt > 1.0) {      // 時間が飛んだ（再接続など）ときは位置だけ据え置き
      v = w = 0.0;
      return false;
    }
    const double ds = (dl + dr) / 2.0;
    const double dth = (dr - dl) / track_;
    const double mid = th + dth / 2.0;
    x += ds * std::cos(mid);
    y += ds * std::sin(mid);
    th = std::atan2(std::sin(th + dth), std::cos(th + dth));
    v = ds / dt;
    w = dth / dt;
    return true;
  }

  // 車体の速さ (v [m/s], ω [rad/s]) → 左右のタイヤの速さ [カウント/秒]
  std::pair<double, double> wheel_targets(double lin, double ang) const
  {
    const double half = ang * track_ / 2.0;
    return {(lin - half) / m_per_tick_, (lin + half) / m_per_tick_};
  }

  double x = 0.0, y = 0.0, th = 0.0, v = 0.0, w = 0.0;

private:
  double m_per_tick_;
  double track_;
  bool has_prev_ = false;
  int64_t prev_ms_ = 0, prev_tl_ = 0, prev_tr_ = 0;
};

}  // namespace pico_bridge
