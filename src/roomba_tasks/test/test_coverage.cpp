// 掃除経路のテスト（ROS 不要。colcon test でも g++ 単体でも動く）
#include <cmath>
#include <cstdio>
#include <vector>

#include "roomba_tasks/coverage.hpp"

#ifdef USE_GTEST
#include <gtest/gtest.h>
#define CHECK(c) EXPECT_TRUE(c)
#define CASE(name) TEST(coverage, name)
#else
static int g_fail = 0;
#define CHECK(c) do {if (!(c)) {std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); g_fail++;}} while (0)
#define CASE(name) static void name()
#endif

using namespace roomba_tasks::cov;

// 外は未知、壁で囲った部屋（x0..x1, y0..y1 [m]）。res 2.5 cm
static Grid room(double W, double H, double res = 0.025)
{
  Grid g;
  g.res = res;
  g.w = static_cast<int>(std::round((W + 1.0) / res));
  g.h = static_cast<int>(std::round((H + 1.0) / res));
  g.ox = -0.5; g.oy = -0.5;
  g.data.assign(g.w * g.h, -1);
  for (int j = 0; j < g.h; ++j) {
    for (int i = 0; i < g.w; ++i) {
      const Pt c = g.center(i, j);
      const bool in = c.x > 0 && c.x < W && c.y > 0 && c.y < H;
      const bool wall = !in && c.x > -0.1 && c.x < W + 0.1 && c.y > -0.1 && c.y < H + 0.1;
      if (in) {g.data[j * g.w + i] = 0;}
      if (wall) {g.data[j * g.w + i] = 100;}
    }
  }
  return g;
}

static void box(Grid & g, double x0, double y0, double x1, double y1, int8_t v = 100)
{
  for (int j = 0; j < g.h; ++j) {
    for (int i = 0; i < g.w; ++i) {
      const Pt c = g.center(i, j);
      if (c.x > x0 && c.x < x1 && c.y > y0 && c.y < y1) {g.data[j * g.w + i] = v;}
    }
  }
}

// 障害物（100 と未知）までの最短距離
static double clearance_at(const Grid & g, double x, double y)
{
  double best = 1e9;
  for (int j = 0; j < g.h; ++j) {
    for (int i = 0; i < g.w; ++i) {
      const int v = g.data[j * g.w + i];
      if (v >= 0 && v < 25) {continue;}
      const Pt c = g.center(i, j);
      // マスの四角までの距離
      const double dx = std::max(0.0, std::abs(x - c.x) - g.res / 2), dy = std::max(0.0, std::abs(y - c.y) - g.res / 2);
      best = std::min(best, std::hypot(dx, dy));
    }
  }
  return best;
}

// cover のマスのうち、レーンの点から reach 以内にあるものの割合
static double covered_fraction(const Grid & g, const Params & p, const std::vector<Pt> & poly, const Pt * robot,
  const Result & r, double reach)
{
  Masks m;
  std::string e;
  make_masks(g, p, poly, robot, m, e);
  int tot = 0, hit = 0;
  for (int j = 0; j < g.h; ++j) {
    for (int i = 0; i < g.w; ++i) {
      if (!m.cover[j * g.w + i]) {continue;}
      ++tot;
      const Pt c = g.center(i, j);
      for (std::size_t k = 0; k < r.path.size(); ++k) {
        if (r.is_lane[k] && std::hypot(r.path[k].x - c.x, r.path[k].y - c.y) <= reach) {++hit; break;}
      }
    }
  }
  return tot ? double(hit) / tot : 0;
}

CASE(empty_room_fully_covered_and_safe)
{
  Grid g = room(3.0, 2.0);
  Params p;
  Pt robot{0.5, 0.5};
  auto r = plan(g, p, {}, &robot);
  CHECK(r.ok);
  std::printf("  room 3x2: angle %.0f lanes %zu lane %.1f m travel %.1f m points %zu area %.2f m2\n",
    r.angle_deg, r.lanes.size(), r.lane_length, r.travel_length, r.path.size(), r.area);
  // 長い辺（x）に沿ったレーンになり、本数 = ceil((2.0 - 2*0.2 - …)/0.15)+1 くらい
  CHECK(std::abs(r.angle_deg) < 1e-9);
  CHECK(r.lanes.size() >= 10 && r.lanes.size() <= 13);
  // 経路のどの点も、壁から clearance 以上（1 マス分の誤差は許す）
  double worst = 1e9;
  for (std::size_t k = 1; k < r.path.size(); k += 3) {worst = std::min(worst, clearance_at(g, r.path[k].x, r.path[k].y));}
  std::printf("  closest approach to wall %.3f m (clearance %.2f)\n", worst, p.clearance);
  CHECK(worst >= p.clearance - g.res * 0.75);
  // ロボットの中心が行ける所は、レーンから spacing/2 + 1 マス以内にすべて入る
  const double f = covered_fraction(g, p, {}, &robot, r, p.spacing / 2 + g.res);
  std::printf("  covered %.4f\n", f);
  CHECK(f > 0.999);
  // つなぎは短い（となりのレーンへ移るだけ）
  CHECK(r.travel_length < 0.25 * r.lanes.size() + 0.6);
  // 点の間隔
  for (std::size_t k = 1; k < r.path.size(); ++k) {
    CHECK(std::hypot(r.path[k].x - r.path[k - 1].x, r.path[k].y - r.path[k - 1].y) <= p.path_step + 1e-6);
  }
}

CASE(narrow_room_picks_long_direction)
{
  Grid g = room(1.2, 3.0);
  Params p;
  auto r = plan(g, p, {}, nullptr);
  CHECK(r.ok);
  std::printf("  room 1.2x3: angle %.0f lanes %zu\n", r.angle_deg, r.lanes.size());
  CHECK(std::abs(r.angle_deg - 90) < 1e-9);
  // 決めた向きも使える
  p.angle_deg = 0;
  auto r2 = plan(g, p, {}, nullptr);
  CHECK(r2.ok && r2.lanes.size() > r.lanes.size());
}

CASE(obstacle_avoided_and_around_it_covered)
{
  Grid g = room(3.0, 3.0);
  box(g, 1.2, 1.2, 1.8, 1.8);   // 真ん中に 60 cm の箱（テーブルの脚まわりなど）
  Params p;
  Pt robot{0.4, 0.4};
  auto r = plan(g, p, {}, &robot);
  CHECK(r.ok);
  double worst = 1e9;
  for (std::size_t k = 1; k < r.path.size(); ++k) {worst = std::min(worst, clearance_at(g, r.path[k].x, r.path[k].y));}
  std::printf("  box: lanes %zu travel %.1f m closest %.3f m\n", r.lanes.size(), r.travel_length, worst);
  CHECK(worst >= p.clearance - g.res * 0.75);
  CHECK(covered_fraction(g, p, {}, &robot, r, p.spacing / 2 + g.res) > 0.999);
}

CASE(polygon_limits_area)
{
  Grid g = room(4.0, 3.0);
  Params p;
  Pt robot{3.5, 2.5};
  std::vector<Pt> poly{{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.5}, {0.0, 1.5}};
  auto r = plan(g, p, poly, &robot);
  CHECK(r.ok);
  for (const auto & l : r.lanes) {
    CHECK(l.first.x < 2.0 && l.second.x < 2.0 && l.first.y < 1.5 && l.second.y < 1.5);
  }
  std::printf("  polygon: area %.2f m2 lanes %zu first lane starts (%.2f, %.2f)\n", r.area, r.lanes.size(),
    r.lanes[0].first.x, r.lanes[0].first.y);
  CHECK(r.area < 2.0 * 1.5);
  CHECK(covered_fraction(g, p, poly, &robot, r, p.spacing / 2 + g.res) > 0.999);
  // 範囲の外にいるロボットから、範囲の一番近い角へ行ってから始める
  CHECK(r.lanes[0].first.x > 1.0 && r.lanes[0].first.y > 0.7);
}

CASE(other_room_behind_wall_ignored)
{
  Grid g = room(4.0, 2.0);
  box(g, 1.95, -0.2, 2.05, 2.2);  // 部屋を壁で 2 つに分ける（行き来できない）
  Params p;
  Pt robot{0.5, 1.0};
  auto r = plan(g, p, {}, &robot);
  CHECK(r.ok);
  bool crossed = false;
  for (const auto & q : r.path) {if (q.x > 2.0) {crossed = true;}}
  CHECK(!crossed);
}

CASE(door_between_rooms_followed)
{
  Grid g = room(4.0, 2.0);
  box(g, 1.95, -0.2, 2.05, 1.2);  // 上に 80 cm のすき間（ドア）を残した壁
  Params p;
  Pt robot{0.5, 0.5};
  auto r = plan(g, p, {}, &robot);
  CHECK(r.ok);
  bool right = false;
  for (const auto & q : r.path) {if (q.x > 2.5) {right = true;}}
  CHECK(right);
  double worst = 1e9;
  for (std::size_t k = 1; k < r.path.size(); ++k) {worst = std::min(worst, clearance_at(g, r.path[k].x, r.path[k].y));}
  std::printf("  door: lanes %zu travel %.1f m closest %.3f\n", r.lanes.size(), r.travel_length, worst);
  CHECK(worst >= p.clearance - g.res * 0.75);
  CHECK(covered_fraction(g, p, {}, &robot, r, p.spacing / 2 + g.res) > 0.999);
}

CASE(too_small_reports_error)
{
  Grid g = room(0.3, 0.3);
  Params p;
  auto r = plan(g, p, {}, nullptr);
  CHECK(!r.ok);
  CHECK(!r.error.empty());
}

#ifndef USE_GTEST
int main()
{
  empty_room_fully_covered_and_safe(); narrow_room_picks_long_direction(); obstacle_avoided_and_around_it_covered();
  polygon_limits_area(); other_room_behind_wall_ignored(); door_between_rooms_followed(); too_small_reports_error();
  std::printf(g_fail ? "%d FAILED\n" : "all passed\n", g_fail);
  return g_fail ? 1 : 0;
}
#endif
