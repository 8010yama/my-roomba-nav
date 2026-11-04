// くまなく掃除する経路（往復のジグザグ＝ボストロフェドン）を、地図（占有格子）から作る。
// ROS に依存しない純粋な C++ なので、単体テストでそのまま試せる。
//
// やること
//   1. 地図の「空いている」マスのうち、壁や未知のマスから clearance 以上離れたマスだけを
//      ロボットの中心が通ってよいマス（travel）にする（ロボットの半径ぶん地図を太らせる）
//   2. ロボットのいる場所からつながっている travel だけを残す（壁の向こうの部屋などを除く）
//   3. 掃除する範囲（多角形。なければ地図全体）と重なる所を cover にする
//   4. 向き angle の平行な線（間隔 spacing）を引き、cover の上にある部分を「レーン」にする
//      angle を決めていなければ、レーンの本数（＝折り返しの回数）が一番少ない向きを選ぶ
//   5. 今いる所から一番近い（壁を回り込んだ道のりで）レーンの端へ行き、レーンを走り、
//      また一番近いレーンの端へ…をくり返して、1 本の経路にする
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace roomba_tasks
{
namespace cov
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kSqrt2 = 1.41421356237309504880;

struct Pt { double x = 0, y = 0; };
struct Pose2 { double x = 0, y = 0, yaw = 0; };

// nav_msgs/OccupancyGrid と同じ並び（data[y * w + x]、-1 = 未知、0..100 = 占有の確からしさ）
struct Grid
{
  int w = 0, h = 0;
  double res = 0.05;
  double ox = 0, oy = 0, oyaw = 0;   // 左下のマスの角の位置と、地図の回転
  std::vector<int8_t> data;

  bool inside(int i, int j) const { return i >= 0 && j >= 0 && i < w && j < h; }
  // 世界座標 → マス
  bool to_cell(double x, double y, int & i, int & j) const
  {
    const double dx = x - ox, dy = y - oy, c = std::cos(oyaw), s = std::sin(oyaw);
    const double lx = c * dx + s * dy, ly = -s * dx + c * dy;
    i = static_cast<int>(std::floor(lx / res));
    j = static_cast<int>(std::floor(ly / res));
    return inside(i, j);
  }
  // マスの中心 → 世界座標
  Pt center(int i, int j) const
  {
    const double lx = (i + 0.5) * res, ly = (j + 0.5) * res, c = std::cos(oyaw), s = std::sin(oyaw);
    return {ox + c * lx - s * ly, oy + s * lx + c * ly};
  }
};

struct Params
{
  double clearance = 0.20;        // [m] ロボットの中心と壁の最小距離（ロボットの半径 + 余裕）
  double spacing = 0.15;          // [m] レーンの間隔（ブラシの幅より少し狭くすると塗り残しが出ない）
  double angle_deg = std::numeric_limits<double>::quiet_NaN();  // レーンの向き [度]。NaN なら自動
  double angle_step_deg = 5.0;    // 自動で向きを探すときの刻み
  int free_max = 25;              // 地図の値がこれより小さい（0 以上）マスを「空き」とみなす
  bool unknown_is_obstacle = true;
  double min_segment = 0.05;      // [m] これより短いレーンは捨てる
  double path_step = 0.025;       // [m] 出力する経路の点の間隔
};

struct Result
{
  bool ok = false;
  std::string error;
  std::vector<Pose2> path;          // 走る経路（つなぎの部分も含む）
  std::vector<uint8_t> is_lane;     // path の各点がレーン（掃除中）なら 1、移動だけなら 0
  std::vector<std::pair<Pt, Pt>> lanes;  // 走る順のレーン（始点, 終点）
  double angle_deg = 0;
  double lane_length = 0;           // [m] レーンの長さの合計
  double travel_length = 0;         // [m] つなぎの長さの合計
  double area = 0;                  // [m^2] 掃除する範囲（ロボットの中心が行ける所）の広さ
};

// ---- 小道具 ----

inline bool point_in_polygon(const std::vector<Pt> & poly, double x, double y)
{
  bool in = false;
  for (std::size_t a = 0, b = poly.size() - 1; a < poly.size(); b = a++) {
    const Pt & p = poly[a];
    const Pt & q = poly[b];
    if ((p.y > y) != (q.y > y) && x < (q.x - p.x) * (y - p.y) / (q.y - p.y) + p.x) {in = !in;}
  }
  return in;
}

// 1 次元の距離変換（Felzenszwalb & Huttenlocher）：f[i] は 0（障害物）か INF
inline void edt_1d(const std::vector<double> & f, std::vector<double> & d, int n,
  std::vector<int> & v, std::vector<double> & z)
{
  const double INF = 1e20;
  int k = 0;
  v[0] = 0; z[0] = -INF; z[1] = INF;
  for (int q = 1; q < n; ++q) {
    double s;
    while (true) {
      s = ((f[q] + double(q) * q) - (f[v[k]] + double(v[k]) * v[k])) / (2.0 * q - 2.0 * v[k]);
      if (s <= z[k] && k > 0) {--k;} else {break;}
    }
    if (s <= z[k]) {  // k == 0 で、まだ左端より左
      v[0] = q; z[0] = -INF; z[1] = INF; continue;
    }
    ++k; v[k] = q; z[k] = s; z[k + 1] = INF;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {++k;}
    d[q] = double(q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

// 各マスの中心から、一番近い障害物のマスの中心までの距離（マス単位の 2 乗）
inline std::vector<double> distance_sq(const std::vector<uint8_t> & obstacle, int w, int h)
{
  const double INF = 1e20;
  std::vector<double> g(static_cast<std::size_t>(w) * h);
  const int n = std::max(w, h);
  std::vector<double> f(n), d(n), z(n + 1);
  std::vector<int> v(n);
  for (int i = 0; i < w; ++i) {
    for (int j = 0; j < h; ++j) {f[j] = obstacle[j * w + i] ? 0.0 : INF;}
    edt_1d(f, d, h, v, z);
    for (int j = 0; j < h; ++j) {g[j * w + i] = d[j];}
  }
  for (int j = 0; j < h; ++j) {
    for (int i = 0; i < w; ++i) {f[i] = g[j * w + i];}
    edt_1d(f, d, w, v, z);
    for (int i = 0; i < w; ++i) {g[j * w + i] = d[i];}
  }
  return g;
}

struct Masks
{
  std::vector<uint8_t> travel;   // ロボットの中心が通ってよい（ロボットからつながっている）
  std::vector<uint8_t> cover;    // その中で、掃除する範囲
  int start = -1;                // ロボットに一番近い travel のマス
  int cover_cells = 0;
};

inline int idx(const Grid & g, int i, int j) {return j * g.w + i;}

// travel と cover を作る。robot が nullptr なら一番広いまとまりを使う
inline bool make_masks(const Grid & g, const Params & p, const std::vector<Pt> & polygon,
  const Pt * robot, Masks & m, std::string & err)
{
  const int N = g.w * g.h;
  if (N <= 0 || static_cast<int>(g.data.size()) != N) {err = "map is empty"; return false;}
  std::vector<uint8_t> obst(N), freec(N);
  for (int k = 0; k < N; ++k) {
    const int v = g.data[k];
    freec[k] = (v >= 0 && v < p.free_max);
    obst[k] = p.unknown_is_obstacle ? !freec[k] : (v >= p.free_max);
  }
  const auto d2 = distance_sq(obst, g.w, g.h);
  // マスの中心どうしの距離から半マス引いたものが clearance 以上なら、壁に触れない
  const double need = p.clearance / g.res + 0.5;
  const double need2 = need * need;
  std::vector<uint8_t> ok(N, 0);
  for (int j = 0; j < g.h; ++j) {
    for (int i = 0; i < g.w; ++i) {
      const int k = idx(g, i, j);
      if (!freec[k] || d2[k] < need2) {continue;}
      if (p.unknown_is_obstacle) {  // 地図の外も未知とみなす
        const double e = std::min(std::min(i + 0.5, g.w - i - 0.5), std::min(j + 0.5, g.h - j - 0.5));
        if (e < p.clearance / g.res) {continue;}
      }
      ok[k] = 1;
    }
  }
  // まとまり（8 近傍）ごとに番号を付ける
  std::vector<int> comp(N, -1);
  std::vector<int> sizes;
  std::vector<int> stack;
  for (int k0 = 0; k0 < N; ++k0) {
    if (!ok[k0] || comp[k0] >= 0) {continue;}
    const int c = static_cast<int>(sizes.size());
    sizes.push_back(0);
    stack.assign(1, k0); comp[k0] = c;
    while (!stack.empty()) {
      const int k = stack.back(); stack.pop_back(); ++sizes[c];
      const int i = k % g.w, j = k / g.w;
      for (int dj = -1; dj <= 1; ++dj) {
        for (int di = -1; di <= 1; ++di) {
          const int a = i + di, b = j + dj;
          if ((di || dj) && g.inside(a, b)) {
            const int q = idx(g, a, b);
            if (ok[q] && comp[q] < 0) {comp[q] = c; stack.push_back(q);}
          }
        }
      }
    }
  }
  if (sizes.empty()) {err = "no free space wide enough for the robot (clearance too large?)"; return false;}
  int use = -1;
  if (robot) {
    // ロボットに一番近い ok のマス（直線距離）
    int ri, rj;
    g.to_cell(robot->x, robot->y, ri, rj);
    double best = 1e18;
    for (int k = 0; k < N; ++k) {
      if (!ok[k]) {continue;}
      const double di = (k % g.w) - ri, dj = (k / g.w) - rj, dd = di * di + dj * dj;
      if (dd < best) {best = dd; m.start = k;}
    }
    use = comp[m.start];
  } else {
    use = static_cast<int>(std::max_element(sizes.begin(), sizes.end()) - sizes.begin());
    for (int k = 0; k < N; ++k) {if (comp[k] == use) {m.start = k; break;}}
  }
  m.travel.assign(N, 0);
  m.cover.assign(N, 0);
  m.cover_cells = 0;
  for (int j = 0; j < g.h; ++j) {
    for (int i = 0; i < g.w; ++i) {
      const int k = idx(g, i, j);
      if (comp[k] != use) {continue;}
      m.travel[k] = 1;
      if (polygon.size() >= 3) {
        const Pt c = g.center(i, j);
        if (!point_in_polygon(polygon, c.x, c.y)) {continue;}
      }
      m.cover[k] = 1;
      ++m.cover_cells;
    }
  }
  if (m.cover_cells == 0) {err = "the area has no place the robot can reach"; return false;}
  return true;
}

inline bool cell_on(const Grid & g, const std::vector<uint8_t> & mask, double x, double y)
{
  int i, j;
  return g.to_cell(x, y, i, j) && mask[idx(g, i, j)];
}

// レーンの一部分（障害物で切れたら別の Seg になる）
struct Seg
{
  Pt a, b;          // u の小さい端と大きい端
  double u0 = 0, u1 = 0;
  int lane = 0;     // 何本目のレーンか（v の小さい順）
};

// 向き angle のレーンを作る（lanes[k] は k 本目のレーンの Seg を u の小さい順に。空のレーンもある）
inline std::vector<std::vector<Seg>> make_lanes(const Grid & g, const Params & p, const Masks & m, double angle)
{
  const double c = std::cos(angle), s = std::sin(angle);
  double umin = 1e18, umax = -1e18, vmin = 1e18, vmax = -1e18;
  for (int j = 0; j < g.h; ++j) {
    for (int i = 0; i < g.w; ++i) {
      if (!m.cover[idx(g, i, j)]) {continue;}
      const Pt q = g.center(i, j);
      const double u = c * q.x + s * q.y, v = -s * q.x + c * q.y;
      umin = std::min(umin, u); umax = std::max(umax, u);
      vmin = std::min(vmin, v); vmax = std::max(vmax, v);
    }
  }
  std::vector<std::vector<Seg>> out;
  if (umin > umax) {return out;}
  // 端から端まで均等に並べる（両端のレーンが一番外側を通るように）
  const int n = (vmax - vmin < 1e-9) ? 1 : static_cast<int>(std::ceil((vmax - vmin) / p.spacing - 1e-9)) + 1;
  const double step = (n == 1) ? 0.0 : (vmax - vmin) / (n - 1);
  const double ds = g.res * 0.5;
  const double pad = g.res;
  auto W = [&](double u, double v) {return Pt{c * u - s * v, s * u + c * v};};
  out.resize(n);
  for (int k = 0; k < n; ++k) {
    const double v = vmin + step * k;
    bool on = false;
    double u0 = 0, ulast = 0;
    const double uend = umax + pad + 1e-9;
    for (double u = umin - pad; u <= uend; u += ds) {
      const Pt q = W(u, v);
      const bool in = cell_on(g, m.cover, q.x, q.y);
      if (in && !on) {on = true; u0 = u;}
      if (in) {ulast = u;}
      if ((!in || u + ds > uend) && on) {
        on = false;
        if (ulast - u0 >= p.min_segment) {out[k].push_back({W(u0, v), W(ulast, v), u0, ulast, k});}
      }
    }
  }
  return out;
}

inline int count_segments(const std::vector<std::vector<Seg>> & lanes)
{
  int n = 0;
  for (const auto & l : lanes) {n += static_cast<int>(l.size());}
  return n;
}

// となりのレーンと 1 対 1 で重なっている Seg をつないで「ひと続きの区画」にする
// （障害物の左右や上下で区画が分かれる。区画の中はジグザグで往復する）
inline std::vector<std::vector<Seg>> make_blocks(const std::vector<std::vector<Seg>> & lanes)
{
  struct Id { int k, a; };
  auto overlap = [](const Seg & x, const Seg & y) {return x.u0 <= y.u1 && y.u0 <= x.u1;};
  const int n = static_cast<int>(lanes.size());
  std::vector<std::vector<int>> next(n), prev(n);
  for (int k = 0; k < n; ++k) {
    next[k].assign(lanes[k].size(), -1);
    prev[k].assign(lanes[k].size(), -1);
  }
  for (int k = 0; k + 1 < n; ++k) {
    for (int a = 0; a < static_cast<int>(lanes[k].size()); ++a) {
      int cnt = 0, hit = -1;
      for (int b = 0; b < static_cast<int>(lanes[k + 1].size()); ++b) {
        if (overlap(lanes[k][a], lanes[k + 1][b])) {++cnt; hit = b;}
      }
      if (cnt != 1) {continue;}
      int back = 0;
      for (int a2 = 0; a2 < static_cast<int>(lanes[k].size()); ++a2) {
        if (overlap(lanes[k][a2], lanes[k + 1][hit])) {++back;}
      }
      if (back != 1) {continue;}
      next[k][a] = hit;
      prev[k + 1][hit] = a;
    }
  }
  std::vector<std::vector<Seg>> blocks;
  for (int k = 0; k < n; ++k) {
    for (int a = 0; a < static_cast<int>(lanes[k].size()); ++a) {
      if (prev[k][a] >= 0) {continue;}
      std::vector<Seg> b;
      for (int kk = k, aa = a; aa >= 0; aa = (kk + 1 < n ? next[kk][aa] : -1), ++kk) {
        b.push_back(lanes[kk][aa]);
        if (kk + 1 >= n) {break;}
      }
      blocks.push_back(b);
    }
  }
  return blocks;
}

// 2 点の間が travel だけを通るか
inline bool visible(const Grid & g, const std::vector<uint8_t> & travel, const Pt & a, const Pt & b)
{
  const double L = std::hypot(b.x - a.x, b.y - a.y);
  const int n = std::max(1, static_cast<int>(std::ceil(L / (g.res * 0.5))));
  for (int k = 0; k <= n; ++k) {
    const double t = double(k) / n;
    if (!cell_on(g, travel, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)) {return false;}
  }
  return true;
}

// 折れ線を、travel から出ない範囲でまっすぐにする（遠くの点が見えるならそこまで直線）
inline std::vector<Pt> shortcut(const Grid & g, const std::vector<uint8_t> & travel, const std::vector<Pt> & pts)
{
  if (pts.size() <= 2) {return pts;}
  std::vector<Pt> out{pts.front()};
  std::size_t a = 0;
  while (a + 1 < pts.size()) {
    std::size_t b = pts.size() - 1;
    while (b > a + 1 && !visible(g, travel, pts[a], pts[b])) {--b;}
    out.push_back(pts[b]);
    a = b;
  }
  return out;
}

inline void densify(const Pt & a, const Pt & b, double step, bool lane, std::vector<Pt> & pts, std::vector<uint8_t> & kind)
{
  const double L = std::hypot(b.x - a.x, b.y - a.y);
  const int n = std::max(1, static_cast<int>(std::ceil(L / step)));
  for (int k = (pts.empty() ? 0 : 1); k <= n; ++k) {
    const double t = double(k) / n;
    pts.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
    kind.push_back(lane ? 1 : 0);
  }
}

// travel の上で start から、stop(k) が true になる一番近いマスを探す（ダイクストラ法）。
// 見つかったらそのマスを返し、parent に道をたどれる情報を入れる。なければ -1
template<class Stop>
int nearest_cell(const Grid & g, const std::vector<uint8_t> & travel, int start, Stop stop,
  std::vector<double> & dist, std::vector<int> & parent)
{
  using QE = std::pair<double, int>;
  std::fill(dist.begin(), dist.end(), 1e18);
  std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
  dist[start] = 0; parent[start] = -1;
  pq.push({0, start});
  while (!pq.empty()) {
    auto [d, k] = pq.top(); pq.pop();
    if (d > dist[k]) {continue;}
    if (stop(k)) {return k;}
    const int i = k % g.w, j = k / g.w;
    for (int dj = -1; dj <= 1; ++dj) {
      for (int di = -1; di <= 1; ++di) {
        if (!di && !dj) {continue;}
        const int a = i + di, b = j + dj;
        if (!g.inside(a, b)) {continue;}
        const int q = idx(g, a, b);
        if (!travel[q]) {continue;}
        const double nd = d + ((di && dj) ? kSqrt2 : 1.0);
        if (nd < dist[q]) {dist[q] = nd; parent[q] = k; pq.push({nd, q});}
      }
    }
  }
  return -1;
}

// ---- 本体 ----
inline Result plan(const Grid & g, const Params & p, const std::vector<Pt> & polygon, const Pt * robot)
{
  Result r;
  Masks m;
  if (p.spacing <= 0 || g.res <= 0) {r.error = "spacing and resolution must be > 0"; return r;}
  if (!make_masks(g, p, polygon, robot, m, r.error)) {return r;}
  r.area = m.cover_cells * g.res * g.res;

  // 1) レーンの向き（自動なら、区画の数とレーンの数が一番少ない向き）
  std::vector<std::vector<Seg>> blocks;
  {
    auto score = [&](const std::vector<std::vector<Seg>> & l, const std::vector<std::vector<Seg>> & b) {
      return 4 * static_cast<int>(b.size()) + count_segments(l);
    };
    int best = std::numeric_limits<int>::max();
    std::vector<double> angles;
    if (std::isfinite(p.angle_deg)) {
      angles.push_back(p.angle_deg);
    } else {
      for (double a = 0; a < 180.0 - 1e-9; a += std::max(1.0, p.angle_step_deg)) {angles.push_back(a);}
    }
    for (double a : angles) {
      auto l = make_lanes(g, p, m, a * kPi / 180.0);
      if (count_segments(l) == 0) {continue;}
      auto b = make_blocks(l);
      const int sc = score(l, b);
      if (sc < best) {best = sc; blocks = std::move(b); r.angle_deg = a;}
    }
  }
  if (blocks.empty()) {r.error = "no lanes (area too small?)"; return r;}

  // 2) 区画の入口（最初の Seg の両端と、最後の Seg の両端）をマスに登録
  struct Entry { int block; bool from_last; bool at_b; };
  const int N = g.w * g.h;
  std::vector<std::vector<Entry>> entries(N);
  auto cell_of = [&](const Pt & q) {
      int i, j;
      return (g.to_cell(q.x, q.y, i, j) && m.travel[idx(g, i, j)]) ? idx(g, i, j) : -1;
    };
  for (int bi = 0; bi < static_cast<int>(blocks.size()); ++bi) {
    for (int last = 0; last < 2; ++last) {
      const Seg & s = last ? blocks[bi].back() : blocks[bi].front();
      for (int e = 0; e < 2; ++e) {
        const int k = cell_of(e ? s.b : s.a);
        if (k >= 0) {entries[k].push_back({bi, last != 0, e != 0});}
      }
    }
  }

  std::vector<double> dist(N);
  std::vector<int> parent(N);
  std::vector<Pt> pts;
  std::vector<uint8_t> kind;
  Pt cur = robot ? *robot : g.center(m.start % g.w, m.start / g.w);
  int cur_cell = m.start;
  if (robot && cell_of(*robot) >= 0) {cur_cell = cell_of(*robot);}
  pts.push_back(cur);
  kind.push_back(0);

  // cur から target へ、travel の中を通って行く（見えるなら直線）
  auto go_to = [&](const Pt & target, int found) {
      std::vector<Pt> way;
      if (visible(g, m.travel, cur, target) || found < 0) {
        way = {cur, target};
      } else {
        for (int k = found; k >= 0; k = parent[k]) {way.push_back(g.center(k % g.w, k / g.w));}
        std::reverse(way.begin(), way.end());
        way.front() = cur;
        way.push_back(target);
        way = shortcut(g, m.travel, way);
      }
      for (std::size_t w = 1; w < way.size(); ++w) {
        r.travel_length += std::hypot(way[w].x - way[w - 1].x, way[w].y - way[w - 1].y);
        densify(way[w - 1], way[w], p.path_step, false, pts, kind);
      }
      cur = target;
    };
  auto run_lane = [&](const Pt & a, const Pt & b) {
      densify(a, b, p.path_step, true, pts, kind);
      r.lane_length += std::hypot(b.x - a.x, b.y - a.y);
      r.lanes.push_back({a, b});
      cur = b;
      const int k = cell_of(b);
      if (k >= 0) {cur_cell = k;}
    };

  // 3) 一番近い（壁を回り込んだ道のりで）区画の入口へ行き、区画の中をジグザグに走る。これをくり返す
  std::vector<uint8_t> done(blocks.size(), 0);
  std::size_t left = blocks.size();
  while (left > 0) {
    Entry fe{-1, false, false};
    const int found = nearest_cell(g, m.travel, cur_cell, [&](int k) {
          for (const Entry & e : entries[k]) {
            if (!done[e.block]) {fe = e; return true;}
          }
          return false;
        }, dist, parent);
    if (found < 0) {break;}   // 残りはたどり着けない（ふつうは起きない）
    const auto & blk = blocks[fe.block];
    const int nseg = static_cast<int>(blk.size());
    bool at_b = fe.at_b;
    for (int t = 0; t < nseg; ++t) {
      const Seg & s = blk[fe.from_last ? nseg - 1 - t : t];
      const Pt a = at_b ? s.b : s.a;
      const Pt b = at_b ? s.a : s.b;
      if (t == 0) {
        go_to(a, found);
      } else {
        int f2 = -1;
        const int ta = cell_of(a);
        if (!visible(g, m.travel, cur, a) && ta >= 0) {
          f2 = nearest_cell(g, m.travel, cur_cell, [&](int k) {return k == ta;}, dist, parent);
        }
        go_to(a, f2);
      }
      run_lane(a, b);
      at_b = !at_b;   // 次のレーンは逆向き
    }
    done[fe.block] = 1;
    --left;
  }

  // 4) 同じ場所に重なった点を消して、向き（次の点の方向）を付ける
  std::vector<Pt> q;
  std::vector<uint8_t> qk;
  for (std::size_t k = 0; k < pts.size(); ++k) {
    if (!q.empty() && std::hypot(pts[k].x - q.back().x, pts[k].y - q.back().y) < 1e-6) {
      qk.back() = std::max(qk.back(), kind[k]);
      continue;
    }
    q.push_back(pts[k]);
    qk.push_back(kind[k]);
  }
  r.path.resize(q.size());
  for (std::size_t k = 0; k < q.size(); ++k) {
    double yaw = 0;
    if (k + 1 < q.size()) {
      yaw = std::atan2(q[k + 1].y - q[k].y, q[k + 1].x - q[k].x);
    } else if (k > 0) {
      yaw = std::atan2(q[k].y - q[k - 1].y, q[k].x - q[k - 1].x);
    }
    r.path[k] = {q[k].x, q[k].y, yaw};
  }
  r.is_lane = qk;
  r.ok = r.path.size() >= 2;
  if (!r.ok) {r.error = "path is empty";}
  return r;
}

}  // namespace cov
}  // namespace roomba_tasks
