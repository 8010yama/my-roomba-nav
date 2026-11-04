// 並べ直しのテスト（ROS 不要。colcon test でも g++ 単体でも動く）
#include <cmath>
#include <cstdio>
#include <vector>

#include "laser_tools/resample.hpp"

#ifdef USE_GTEST
#include <gtest/gtest.h>
#define CHECK(c) EXPECT_TRUE(c)
#define CASE(name) TEST(resample, name)
#else
static int g_fail = 0;
#define CHECK(c) do {if (!(c)) {std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++;}} while (0)
#define CASE(name) static void name()
#endif

using laser_tools::resample;
static const double PI = laser_tools::PI;

static std::vector<float> ring(std::size_t n, float r)
{
  return std::vector<float>(n, r);
}

CASE(always_same_size)
{
  for (std::size_t n : {499u, 501u, 502u, 504u}) {     // LD19 の実際の点の数
    const auto o = resample(0.0, 2 * PI / n, ring(n, 1.5f), {}, 0.02, 12.0, 500);
    CHECK(o.ranges.size() == 500);
    int filled = 0;
    for (float r : o.ranges) {if (std::isfinite(r)) {filled++;}}
    CHECK(filled >= 495);                              // ほとんどのマスに点が入る
  }
}

CASE(angle_lands_in_right_bin)
{
  // 正面（0 rad）だけ 1.0 m、ほかは 3.0 m。0〜2π の並び（LD19 と同じ）で入れる
  const std::size_t n = 502;
  auto r = ring(n, 3.0f);
  r[0] = 1.0f;
  const auto o = resample(0.0, 2 * PI / n, r, {}, 0.02, 12.0, 500);
  // 出力は -π 始まりなので、0 rad はちょうど真ん中のマス
  CHECK(std::abs(o.ranges[250] - 1.0f) < 1e-6);
  CHECK(std::abs(o.angle_min + PI) < 1e-9);
  // 左（+π/2）の点
  auto r2 = ring(n, 3.0f);
  r2[n / 4] = 0.5f;
  const auto o2 = resample(0.0, 2 * PI / n, r2, {}, 0.02, 12.0, 500);
  CHECK(std::abs(o2.ranges[375] - 0.5f) < 1e-6 || std::abs(o2.ranges[374] - 0.5f) < 1e-6);
}

CASE(invalid_points_ignored_and_nearest_kept)
{
  std::vector<float> r = {0.0f, NAN, 20.0f, 2.0f, 1.0f};
  std::vector<float> it = {9, 9, 9, 7, 8};
  // 5 点とも同じマスに入るように、角度の幅をとても小さくする
  const auto o = resample(0.0, 1e-5, r, it, 0.02, 12.0, 500);
  CHECK(std::abs(o.ranges[250] - 1.0f) < 1e-6);         // 0・nan・範囲外は捨てて、近いほう
  CHECK(std::abs(o.intensities[250] - 8.0f) < 1e-6);
  CHECK(o.intensities.size() == 500);
}

CASE(isolated_points_removed_walls_kept)
{
  // 3.0 m の壁がぐるっと1周。そのうち 2 点は窓などで点が抜けている（+inf）
  auto r = ring(500, 3.0f);
  r[10] = INFINITY;
  r[11] = INFINITY;
  // 空中に 1 点だけのノイズ（1.2 m）
  r[100] = 1.2f;
  // 2 点並んだ細いもの（椅子の脚など、0.8 m）は残したい
  r[200] = 0.80f;
  r[201] = 0.82f;
  std::vector<float> it(500, 100.0f);
  const auto removed = laser_tools::remove_isolated(r, it, 2, 0.08);
  CHECK(removed == 1);
  CHECK(std::isinf(r[100]));
  CHECK(it[100] == 0.0f);
  CHECK(std::abs(r[200] - 0.80f) < 1e-6 && std::abs(r[201] - 0.82f) < 1e-6);
  CHECK(std::abs(r[12] - 3.0f) < 1e-6 && std::abs(r[9] - 3.0f) < 1e-6);   // 抜けの隣の壁は残る
  CHECK(std::abs(r[0] - 3.0f) < 1e-6 && std::abs(r[499] - 3.0f) < 1e-6);  // 1周の端もつながっている
}

CASE(isolated_point_with_gap_kept_if_neighbor_two_away)
{
  // 1 マス空いて隣に点がある（LD19 は点が足りずにマスが空くことがある）→ 壁の一部として残す
  auto r = ring(500, INFINITY);
  r[50] = 2.0f;
  r[52] = 2.03f;
  std::vector<float> it;
  CHECK(laser_tools::remove_isolated(r, it, 2, 0.08) == 0);
  // 3 マス以上離れていればノイズとして消える
  auto r2 = ring(500, INFINITY);
  r2[50] = 2.0f;
  r2[53] = 2.0f;
  CHECK(laser_tools::remove_isolated(r2, it, 2, 0.08) == 2);
}

#ifndef USE_GTEST
int main()
{
  always_same_size(); angle_lands_in_right_bin(); invalid_points_ignored_and_nearest_kept();
  isolated_points_removed_walls_kept(); isolated_point_with_gap_kept_if_neighbor_two_away();
  std::printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
#endif
