// 地図の上での「仕事」をまとめるノード（名前 tasks）
//
//   Foxglove で地図の上をクリック（Publish → Point → /clicked_point）して点を並べ、
//   サービスを呼ぶと動き出す。
//
//   /tasks/go_waypoints     並べた点を順番に回る（Nav2 の FollowWaypoints）
//   /tasks/preview_coverage 並べた点で囲んだ範囲（3 点未満なら地図全体）をくまなく走る経路を作って表示するだけ
//   /tasks/go_coverage      その経路を作って走る（Nav2 の controller_server の FollowPath に直接渡す）
//   /tasks/cancel           止める
//   /tasks/brush_on・brush_off  ブラシだけ回す・止める（掃除中は自動で回る）
//   /tasks/undo  /tasks/clear  /tasks/save  /tasks/load   点の取り消し・全消し・保存・読み込み
//
//   ブラシ：/brush（std_msgs/Float32MultiArray [メイン, サイド]）に出す。pico_bridge が Pico へ送る
//   表示：/tasks/markers（点と範囲）、/tasks/plan（作った経路）、/tasks/done（走り終えた所）、/tasks/status（状態）
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "roomba_tasks/coverage.hpp"

using namespace std::chrono_literals;
namespace cov = roomba_tasks::cov;

class TaskManager : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using Trigger = std_srvs::srv::Trigger;

  TaskManager()
  : Node("tasks")
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    const double radius = declare_parameter<double>("robot_radius", 0.15);        // [m] ロボットの半径
    const double margin = declare_parameter<double>("clearance_margin", 0.05);    // [m] 壁との余裕（半径に足す）
    cp_.clearance = radius + margin;
    cp_.spacing = declare_parameter<double>("lane_spacing", 0.15);                // [m] レーンの間隔
    const double ang = declare_parameter<double>("lane_angle_deg", -1.0);         // レーンの向き。負なら自動
    cp_.angle_deg = ang < 0 ? std::numeric_limits<double>::quiet_NaN() : ang;
    cp_.free_max = static_cast<int>(declare_parameter<int64_t>("free_threshold", 25));
    cp_.min_segment = declare_parameter<double>("min_lane_length", 0.05);
    points_file_ = expand(declare_parameter<std::string>("points_file", "~/roomba_ws/maps/points.txt"));
    controller_id_ = declare_parameter<std::string>("controller_id", "");
    goal_checker_id_ = declare_parameter<std::string>("goal_checker_id", "");
    planner_id_ = declare_parameter<std::string>("planner_id", "");
    skip_distance_ = declare_parameter<double>("skip_distance", 0.4);   // [m] 進めなくなったら、この距離ずつ先から再開
    max_retries_ = static_cast<int>(declare_parameter<int64_t>("max_retries", 5));
    loop_waypoints_ = declare_parameter<bool>("loop_waypoints", false);  // true：ウェイポイントを回り続ける
    brush_main_ = declare_parameter<double>("brush_main", 0.8);           // 掃除中のメインブラシの強さ（-1〜1）
    brush_side_ = declare_parameter<double>("brush_side", 0.6);           // 掃除中のサイドブラシの強さ（-1〜1）
    brush_with_coverage_ = declare_parameter<bool>("brush_with_coverage", true);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      declare_parameter<std::string>("map_topic", "/map"), rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {on_map(m);});
    click_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      declare_parameter<std::string>("clicked_point_topic", "/clicked_point"), 10,
      [this](geometry_msgs::msg::PointStamped::SharedPtr m) {on_click(*m);});

    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", rclcpp::QoS(1).transient_local());
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("~/plan", rclcpp::QoS(1).transient_local());
    done_pub_ = create_publisher<nav_msgs::msg::Path>("~/done", rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(1).transient_local());
    brush_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("brush", 10);

    follow_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
    compute_client_ = rclcpp_action::create_client<ComputePath>(this, "compute_path_to_pose");
    waypoints_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

    add_service("undo", [this](std::string & msg) {
        if (points_.empty()) {msg = "点がありません"; return false;}
        points_.pop_back(); publish_markers();
        msg = "最後の点を消しました（残り " + std::to_string(points_.size()) + " 点）"; return true;
      });
    add_service("clear", [this](std::string & msg) {
        points_.clear(); publish_markers();
        msg = "点を全部消しました"; return true;
      });
    add_service("save", [this](std::string & msg) {return save_points(msg);});
    add_service("load", [this](std::string & msg) {return load_points(msg);});
    add_service("go_waypoints", [this](std::string & msg) {return start_waypoints(msg);});
    add_service("preview_coverage", [this](std::string & msg) {
        if (mode_ != Mode::Idle) {msg = "走行中です（先に /tasks/cancel）"; return false;}
        return make_coverage(msg);
      });
    add_service("go_coverage", [this](std::string & msg) {
        stop_all("");
        return make_coverage(msg) && start_coverage(msg);
      });
    add_service("cancel", [this](std::string & msg) {stop_all("止めました"); manual_brush_ = false; msg = "止めました"; return true;});
    add_service("brush_on", [this](std::string & msg) {
        manual_brush_ = true; msg = "ブラシを回します（止めるのは /tasks/brush_off）"; return true;
      });
    add_service("brush_off", [this](std::string & msg) {
        manual_brush_ = false; msg = "ブラシを止めます（掃除中は掃除が終わるまで回ります）"; return true;
      });

    timer_ = create_wall_timer(200ms, [this]() {on_timer();});

    std::string m;
    if (declare_parameter<bool>("load_points_on_start", true)) {load_points(m);}
    publish_markers();
    set_status("待機中（Foxglove で /clicked_point に点を置いてください）");
    RCLCPP_INFO(get_logger(), "準備 OK：レーン間隔 %.2f m、壁との距離 %.2f m、点のファイル %s",
      cp_.spacing, cp_.clearance, points_file_.c_str());
  }

private:
  enum class Mode { Idle, Coverage, Detour, Waypoints };

  // ---------- 小道具 ----------
  static std::string expand(const std::string & p)
  {
    if (!p.empty() && p[0] == '~') {
      const char * h = std::getenv("HOME");
      return std::string(h ? h : "") + p.substr(1);
    }
    return p;
  }

  void add_service(const std::string & name, std::function<bool(std::string &)> fn)
  {
    services_.push_back(create_service<Trigger>("~/" + name,
      [this, fn, name](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        std::string msg;
        res->success = fn(msg);
        res->message = msg;
        RCLCPP_INFO(get_logger(), "[%s] %s", name.c_str(), msg.c_str());
      }));
  }

  void set_status(const std::string & s)
  {
    if (s == status_) {return;}
    status_ = s;
    std_msgs::msg::String m;
    m.data = s;
    status_pub_->publish(m);
  }

  static geometry_msgs::msg::Quaternion yaw_q(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw / 2);
    q.w = std::cos(yaw / 2);
    return q;
  }

  geometry_msgs::msg::PoseStamped pose_msg(double x, double y, double yaw) const
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = global_frame_;
    p.header.stamp = now();
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation = yaw_q(yaw);
    return p;
  }

  // frame から global_frame への 2 次元の変換（x, y, yaw）
  bool lookup(const std::string & frame, double & x, double & y, double & yaw)
  {
    try {
      const auto t = tf_buffer_->lookupTransform(global_frame_, frame, tf2::TimePointZero);
      x = t.transform.translation.x;
      y = t.transform.translation.y;
      const auto & q = t.transform.rotation;
      yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "%s → %s の位置がわかりません: %s", global_frame_.c_str(), frame.c_str(), e.what());
      return false;
    }
  }

  bool robot_pose(double & x, double & y, double & yaw) {return lookup(robot_frame_, x, y, yaw);}

  // ---------- 地図と点 ----------
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr & m)
  {
    grid_.w = static_cast<int>(m->info.width);
    grid_.h = static_cast<int>(m->info.height);
    grid_.res = m->info.resolution;
    grid_.ox = m->info.origin.position.x;
    grid_.oy = m->info.origin.position.y;
    const auto & q = m->info.origin.orientation;
    grid_.oyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    grid_.data.assign(m->data.begin(), m->data.end());
    if (!have_map_) {
      RCLCPP_INFO(get_logger(), "地図を受け取りました：%d x %d マス、%.3f m/マス", grid_.w, grid_.h, grid_.res);
    }
    have_map_ = true;
  }

  void on_click(const geometry_msgs::msg::PointStamped & m)
  {
    double x = m.point.x, y = m.point.y;
    const std::string frame = m.header.frame_id.empty() ? global_frame_ : m.header.frame_id;
    if (frame != global_frame_) {
      double tx, ty, tyaw;
      if (!lookup(frame, tx, ty, tyaw)) {return;}
      x = tx + std::cos(tyaw) * m.point.x - std::sin(tyaw) * m.point.y;
      y = ty + std::sin(tyaw) * m.point.x + std::cos(tyaw) * m.point.y;
    }
    points_.push_back({x, y});
    RCLCPP_INFO(get_logger(), "点 %zu: (%.2f, %.2f)", points_.size(), x, y);
    publish_markers();
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = global_frame_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    auto base = [this](int id, int type) {
        visualization_msgs::msg::Marker k;
        k.header.frame_id = global_frame_;
        k.header.stamp = now();
        k.ns = "tasks";
        k.id = id;
        k.type = type;
        k.action = visualization_msgs::msg::Marker::ADD;
        k.pose.orientation.w = 1.0;
        return k;
      };
    if (!points_.empty()) {
      auto s = base(0, visualization_msgs::msg::Marker::SPHERE_LIST);
      s.scale.x = s.scale.y = s.scale.z = 0.08;
      s.color.r = 0.1f; s.color.g = 0.9f; s.color.b = 0.3f; s.color.a = 1.0f;
      for (const auto & p : points_) {
        geometry_msgs::msg::Point q;
        q.x = p.x; q.y = p.y; q.z = 0.05;
        s.points.push_back(q);
      }
      arr.markers.push_back(s);
      // 点をつなぐ線（3 点以上なら閉じて「掃除する範囲」に見せる）
      if (points_.size() >= 2) {
        auto l = base(1, visualization_msgs::msg::Marker::LINE_STRIP);
        l.scale.x = 0.02;
        l.color.r = 1.0f; l.color.g = 0.8f; l.color.b = 0.1f; l.color.a = 0.9f;
        l.points = s.points;
        if (points_.size() >= 3) {l.points.push_back(s.points.front());}
        arr.markers.push_back(l);
      }
      for (std::size_t k = 0; k < points_.size(); ++k) {
        auto t = base(10 + static_cast<int>(k), visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        t.pose.position.x = points_[k].x;
        t.pose.position.y = points_[k].y;
        t.pose.position.z = 0.2;
        t.scale.z = 0.15;
        t.color.r = t.color.g = t.color.b = 1.0f; t.color.a = 1.0f;
        t.text = std::to_string(k + 1);
        arr.markers.push_back(t);
      }
    }
    markers_pub_->publish(arr);
  }

  bool save_points(std::string & msg)
  {
    std::ofstream f(points_file_);
    if (!f) {msg = points_file_ + " に書けません"; return false;}
    f << "# x y [m]（" << global_frame_ << " 座標）\n";
    for (const auto & p : points_) {f << p.x << " " << p.y << "\n";}
    msg = std::to_string(points_.size()) + " 点を " + points_file_ + " に保存しました";
    return true;
  }

  bool load_points(std::string & msg)
  {
    std::ifstream f(points_file_);
    if (!f) {msg = points_file_ + " がありません"; return false;}
    std::vector<cov::Pt> pts;
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') {continue;}
      std::istringstream ss(line);
      cov::Pt p;
      if (ss >> p.x >> p.y) {pts.push_back(p);}
    }
    points_ = pts;
    publish_markers();
    msg = std::to_string(points_.size()) + " 点を " + points_file_ + " から読み込みました";
    return true;
  }

  // ---------- ウェイポイント ----------
  bool start_waypoints(std::string & msg)
  {
    if (points_.empty()) {msg = "点がありません（Foxglove で /clicked_point に点を置いてください）"; return false;}
    if (!waypoints_client_->action_server_is_ready()) {msg = "Nav2 の waypoint_follower が動いていません"; return false;}
    stop_all("");
    FollowWaypoints::Goal goal;
    for (std::size_t k = 0; k < points_.size(); ++k) {
      // 向きは次の点の方向（最後は、来た方向のまま）
      const std::size_t a = (k + 1 < points_.size()) ? k : (k > 0 ? k - 1 : k);
      const std::size_t b = (k + 1 < points_.size()) ? k + 1 : k;
      const double yaw = (a == b) ? 0.0 : std::atan2(points_[b].y - points_[a].y, points_[b].x - points_[a].x);
      goal.poses.push_back(pose_msg(points_[k].x, points_[k].y, yaw));
    }
    mode_ = Mode::Waypoints;
    const int run = ++run_id_;
    rclcpp_action::Client<FollowWaypoints>::SendGoalOptions o;
    o.goal_response_callback = [this, run](rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr gh) {
        if (run != run_id_) {return;}
        if (!gh) {finish("ウェイポイント：Nav2 に断られました"); }
      };
    o.feedback_callback = [this, run](rclcpp_action::ClientGoalHandle<FollowWaypoints>::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> fb) {
        if (run != run_id_) {return;}
        set_status("ウェイポイント " + std::to_string(fb->current_waypoint + 1) + "/" + std::to_string(points_.size()) + " へ移動中");
      };
    o.result_callback = [this, run](const rclcpp_action::ClientGoalHandle<FollowWaypoints>::WrappedResult & r) {
        if (run != run_id_) {return;}
        std::string s;
        if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
          s = "ウェイポイント完了";
          if (r.result && !r.result->missed_waypoints.empty()) {
            s += "（行けなかった点:";
            for (auto k : r.result->missed_waypoints) {s += " " + std::to_string(k + 1);}
            s += "）";
          }
          if (loop_waypoints_) {
            std::string m;
            mode_ = Mode::Idle;   // この goal はもう終わっているので取り消さない
            RCLCPP_INFO(get_logger(), "%s → もう一周", s.c_str());
            start_waypoints(m);
            return;
          }
        } else {
          s = "ウェイポイント中止（" + code_name(r.code) + "）";
        }
        finish(s);
      };
    waypoints_client_->async_send_goal(goal, o);
    set_status("ウェイポイント 1/" + std::to_string(points_.size()) + " へ移動中");
    msg = std::to_string(points_.size()) + " 点を順番に回ります";
    return true;
  }

  // ---------- くまなく掃除 ----------
  bool make_coverage(std::string & msg)
  {
    if (!have_map_) {msg = "地図がまだ届いていません"; return false;}
    double x, y, yaw;
    const bool have_pose = robot_pose(x, y, yaw);
    const cov::Pt robot{x, y};
    std::vector<cov::Pt> poly;
    if (points_.size() >= 3) {poly = points_;}
    const auto t0 = now();
    auto r = cov::plan(grid_, cp_, poly, have_pose ? &robot : nullptr);
    const double ms = (now() - t0).seconds() * 1000.0;
    if (!r.ok) {msg = "経路を作れません: " + r.error; return false;}
    plan_ = r;
    // 道のりの累計（進み具合を調べるため）
    cum_.assign(r.path.size(), 0.0);
    lane_cum_.assign(r.path.size(), 0.0);
    for (std::size_t k = 1; k < r.path.size(); ++k) {
      const double d = std::hypot(r.path[k].x - r.path[k - 1].x, r.path[k].y - r.path[k - 1].y);
      cum_[k] = cum_[k - 1] + d;
      lane_cum_[k] = lane_cum_[k - 1] + (r.is_lane[k] && r.is_lane[k - 1] ? d : 0.0);
    }
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = now();
    for (const auto & p : r.path) {path.poses.push_back(pose_msg(p.x, p.y, p.yaw));}
    plan_msg_ = path;
    plan_pub_->publish(path);
    nav_msgs::msg::Path empty;
    empty.header = path.header;
    done_pub_->publish(empty);
    char buf[512];
    std::snprintf(buf, sizeof(buf), "%s：広さ %.1f m²、レーン %zu 本（向き %.0f°）、走る距離 %.1f m（移動 %.1f m）、計算 %.0f ms",
      poly.empty() ? "地図全体" : "囲んだ範囲", r.area, r.lanes.size(), r.angle_deg,
      r.lane_length + r.travel_length, r.travel_length, ms);
    msg = buf;
    if (!have_pose) {msg += "（ロボットの位置がわからないので、一番広い所で作りました）";}
    return true;
  }

  bool start_coverage(std::string & msg)
  {
    if (!follow_client_->action_server_is_ready()) {msg = "Nav2 の controller_server が動いていません"; return false;}
    stop_all("");
    mode_ = Mode::Coverage;
    ++run_id_;
    prog_ = 0;
    retries_ = 0;
    fail_s_ = 0;
    send_follow(plan_msg_.poses);
    set_status("掃除中 0%");
    msg += " → 掃除を始めます";
    return true;
  }

  void send_follow(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
  {
    FollowPath::Goal goal;
    goal.path.header.frame_id = global_frame_;
    goal.path.header.stamp = now();
    goal.path.poses = poses;
    goal.controller_id = controller_id_;
    goal.goal_checker_id = goal_checker_id_;
    mode_ = Mode::Coverage;
    const int run = ++run_id_;   // 前に送った goal の結果は無視する
    rclcpp_action::Client<FollowPath>::SendGoalOptions o;
    o.goal_response_callback = [this, run](rclcpp_action::ClientGoalHandle<FollowPath>::SharedPtr gh) {
        if (run == run_id_ && !gh) {finish("掃除：controller_server に断られました");}
      };
    o.result_callback = [this, run](const rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult & r) {
        if (run != run_id_) {return;}
        if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
          prog_ = plan_.path.size() - 1;
          publish_done();
          finish("掃除完了");
        } else if (r.code == rclcpp_action::ResultCode::ABORTED) {
          RCLCPP_WARN(get_logger(), "進めなくなりました（%.1f m 地点）。少し先から再開します", cum_[prog_]);
          fail_s_ = cum_[prog_];
          ++retries_;
          resume();
        } else {
          finish("掃除中止（" + code_name(r.code) + "）");
        }
      };
    follow_client_->async_send_goal(goal, o);
  }

  // 進めなくなったとき：今の地点から skip_distance × 回数 だけ先の点まで Nav2 の planner で回り道を作り、続きを走る
  void resume()
  {
    if (retries_ > max_retries_) {finish("掃除中止：" + std::to_string(max_retries_) + " 回続けて進めませんでした"); return;}
    const double want = cum_[prog_] + skip_distance_ * retries_;
    std::size_t target = prog_;
    while (target + 1 < cum_.size() && cum_[target] < want) {++target;}
    if (target + 1 >= cum_.size()) {finish("掃除完了（最後の部分は進めませんでした）"); return;}
    if (!compute_client_->action_server_is_ready()) {finish("掃除中止：planner_server が動いていません"); return;}
    mode_ = Mode::Detour;
    set_status("回り道を計算中（" + std::to_string(retries_) + " 回目）");
    ComputePath::Goal goal;
    goal.goal = plan_msg_.poses[target];
    goal.goal.header.stamp = now();
    goal.planner_id = planner_id_;
    goal.use_start = false;
    const int run = ++run_id_;
    rclcpp_action::Client<ComputePath>::SendGoalOptions o;
    o.result_callback = [this, run, target](const rclcpp_action::ClientGoalHandle<ComputePath>::WrappedResult & r) {
        if (run != run_id_) {return;}
        if (r.code != rclcpp_action::ResultCode::SUCCEEDED || !r.result || r.result->path.poses.empty()) {
          RCLCPP_WARN(get_logger(), "回り道が見つかりません。もっと先から試します");
          ++retries_;
          resume();
          return;
        }
        std::vector<geometry_msgs::msg::PoseStamped> poses = r.result->path.poses;
        poses.insert(poses.end(), plan_msg_.poses.begin() + static_cast<long>(target) + 1, plan_msg_.poses.end());
        for (auto & p : poses) {p.header.frame_id = global_frame_;}
        send_follow(poses);
      };
    compute_client_->async_send_goal(goal, o);
  }

  // ---------- 共通 ----------
  static std::string code_name(rclcpp_action::ResultCode c)
  {
    switch (c) {
      case rclcpp_action::ResultCode::SUCCEEDED: return "成功";
      case rclcpp_action::ResultCode::ABORTED: return "失敗";
      case rclcpp_action::ResultCode::CANCELED: return "キャンセル";
      default: return "不明";
    }
  }

  void finish(const std::string & s)
  {
    mode_ = Mode::Idle;
    ++run_id_;
    set_status(s);
    RCLCPP_INFO(get_logger(), "%s", s.c_str());
  }

  void stop_all(const std::string & s)
  {
    const bool was = mode_ != Mode::Idle;
    ++run_id_;   // 今までのコールバックは無視する
    if (was) {
      // 今より前に受け付けられた goal だけ取り消す（このあと送る新しい goal は取り消さない）
      const auto t = now();
      if (mode_ == Mode::Waypoints) {waypoints_client_->async_cancel_goals_before(t);}
      if (mode_ == Mode::Coverage) {follow_client_->async_cancel_goals_before(t);}
      if (mode_ == Mode::Detour) {compute_client_->async_cancel_goals_before(t);}
    }
    mode_ = Mode::Idle;
    if (!s.empty()) {set_status(s);}
  }

  void publish_done()
  {
    nav_msgs::msg::Path p;
    p.header.frame_id = global_frame_;
    p.header.stamp = now();
    if (!plan_msg_.poses.empty()) {
      p.poses.assign(plan_msg_.poses.begin(), plan_msg_.poses.begin() + static_cast<long>(prog_) + 1);
    }
    done_pub_->publish(p);
  }

  // ブラシ：掃除中か brush_on のあいだ、強さを送り続ける（pico_bridge は 1 秒途切れると止める）
  void update_brush()
  {
    const bool cleaning = brush_with_coverage_ && (mode_ == Mode::Coverage || mode_ == Mode::Detour);
    const bool on = manual_brush_ || cleaning;
    if (!on && !brush_was_on_) {return;}
    std_msgs::msg::Float32MultiArray m;
    m.data = {on ? static_cast<float>(brush_main_) : 0.0f, on ? static_cast<float>(brush_side_) : 0.0f};
    brush_pub_->publish(m);
    brush_was_on_ = on;
  }

  void on_timer()
  {
    update_brush();
    if (mode_ != Mode::Coverage && mode_ != Mode::Detour) {return;}
    double x, y, yaw;
    if (!robot_pose(x, y, yaw)) {return;}
    // 今の地点から 1.5 m 先までの経路の点で、ロボットに一番近い点（0.3 m 以内）まで進んだことにする
    std::size_t best = prog_;
    double bd = 0.30;
    for (std::size_t k = prog_; k < cum_.size() && cum_[k] - cum_[prog_] < 1.5; ++k) {
      const double d = std::hypot(plan_.path[k].x - x, plan_.path[k].y - y);
      if (d < bd) {bd = d; best = k;}
    }
    if (best != prog_) {
      prog_ = best;
      if (retries_ > 0 && cum_[prog_] - fail_s_ > 1.0) {retries_ = 0;}  // ちゃんと進めたら回数を戻す
    }
    if (++tick_ % 5 == 0) {
      publish_done();
      const double total = lane_cum_.back();
      const int pct = total > 0 ? static_cast<int>(100.0 * lane_cum_[prog_] / total) : 100;
      if (mode_ == Mode::Coverage) {set_status("掃除中 " + std::to_string(pct) + "%");}
    }
  }

  // ---------- メンバ ----------
  std::string global_frame_, robot_frame_, points_file_, controller_id_, goal_checker_id_, planner_id_;
  double skip_distance_ = 0.4;
  int max_retries_ = 5;
  bool loop_waypoints_ = false;
  double brush_main_ = 0.8, brush_side_ = 0.6;
  bool brush_with_coverage_ = true, manual_brush_ = false, brush_was_on_ = false;
  cov::Params cp_;
  cov::Grid grid_;
  bool have_map_ = false;
  std::vector<cov::Pt> points_;
  cov::Result plan_;
  nav_msgs::msg::Path plan_msg_;
  std::vector<double> cum_, lane_cum_;
  std::size_t prog_ = 0;
  int retries_ = 0;
  double fail_s_ = 0;
  Mode mode_ = Mode::Idle;
  int run_id_ = 0;
  unsigned tick_ = 0;
  std::string status_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_, done_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr brush_pub_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<ComputePath>::SharedPtr compute_client_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoints_client_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManager>());
  rclcpp::shutdown();
  return 0;
}
