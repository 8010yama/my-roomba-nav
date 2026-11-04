// オドメトリ計算とデータ解釈のテスト（ROS 不要。colcon test でも、g++ 単体でも動く）
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "pico_bridge/diffdrive.hpp"

#ifdef USE_GTEST
#include <gtest/gtest.h>
#define CHECK(c) EXPECT_TRUE(c)
#define CASE(name) TEST(diffdrive, name)
#else
static int g_fail = 0;
#define CHECK(c) do {if (!(c)) {std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++;}} while (0)
#define CASE(name) static void name()
#endif

using pico_bridge::DiffDriveOdom;
using pico_bridge::Sample;
using pico_bridge::parse_line;
using pico_bridge::ms_diff;
static const double PI = pico_bridge::PI;

CASE(parse_ok)
{
  Sample s;
  CHECK(parse_line("D 1234 10 -5 0.0100 0.0000 -0.0200 0.010 0.020 9.810", s));
  CHECK(s.ms == 1234 && s.tl == 10 && s.tr == -5 && s.has_imu);
  CHECK(std::abs(s.imu[2] + 0.02) < 1e-9 && std::abs(s.imu[5] - 9.81) < 1e-9);
}

CASE(parse_nan_and_garbage)
{
  Sample s;
  CHECK(parse_line("D 1 2 3 nan nan nan nan nan nan", s) && !s.has_imu);
  CHECK(!parse_line("I ready imu=1", s));
  CHECK(!parse_line("D 1 2", s));
  CHECK(!parse_line("D x 2 3 0 0 0 0 0 0", s));
}

CASE(ms_wrap)
{
  CHECK(ms_diff(5, (int64_t(1) << 30) - 15) == 20);
  CHECK(ms_diff(1020, 1000) == 20);
}

CASE(straight)
{
  DiffDriveOdom o(0.1, 0.2, 1000);
  o.update(0, 0, 0);
  o.update(1000, 1000, 1000);
  CHECK(std::abs(o.x - PI * 0.1) < 1e-9 && std::abs(o.y) < 1e-9 && std::abs(o.th) < 1e-9);
  CHECK(std::abs(o.v - PI * 0.1) < 1e-9 && std::abs(o.w) < 1e-9);
}

CASE(spin_in_place)
{
  DiffDriveOdom o(0.1, 0.2, 1000);
  o.update(0, 0, 0);
  o.update(20, -100, 100);
  const double dth = 2 * 100 * (PI * 0.1 / 1000) / 0.2;
  CHECK(std::abs(o.th - dth) < 1e-9 && std::abs(o.x) < 1e-9 && std::abs(o.y) < 1e-9);
  CHECK(std::abs(o.w - dth / 0.02) < 1e-6);
}

CASE(wheel_targets_roundtrip)
{
  DiffDriveOdom o(0.065, 0.2, 1200);
  const auto t = o.wheel_targets(0.2, 1.0);
  o.update(0, 0, 0);
  o.update(1000, std::llround(t.first), std::llround(t.second));
  CHECK(std::abs(o.v - 0.2) < 1e-3 && std::abs(o.w - 1.0) < 1e-2);
}

CASE(circle_closes)
{
  DiffDriveOdom o(0.065, 0.2, 1200);
  const auto t = o.wheel_targets(0.2, 0.5);     // 半径 0.4 m の円
  o.update(0, 0, 0);
  const int steps = static_cast<int>((2 * PI / 0.5) / 0.02);
  for (int k = 1; k <= steps; ++k) {
    const double tt = k * 0.02;
    o.update(static_cast<int64_t>(tt * 1000), static_cast<int64_t>(t.first * tt), static_cast<int64_t>(t.second * tt));
  }
  CHECK(std::hypot(o.x, o.y) < 0.02);
}

#ifndef USE_GTEST
int main()
{
  parse_ok(); parse_nan_and_garbage(); ms_wrap(); straight(); spin_in_place(); wheel_targets_roundtrip(); circle_closes();
  std::printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
#endif
