// 自己位置推定・自動清掃・定位置に戻る、をまとめるノード（名前 localize）
//
//   /localize  その場で回って、地図全体から自分の位置を探す（起動したときも 1 回やる）
//   /clean     自己位置推定 → 保存した範囲を掃除（/tasks/go_coverage）→ 終わったら定位置へ戻る
//   /set_home  今いる場所と向きを定位置として保存する（~/roomba_ws/maps/home.txt）
//   /go_home   定位置へ戻る（保存していなければ地図の原点）
//
//   どのサービスもすぐ返事をして、続きは裏で進める（進み具合はログに出る）。
//
//   自己位置推定のやり方（AMCL のグローバル自己位置推定）
//     1. 位置の候補を地図全体にばらまく（/reinitialize_global_localization）
//     2. その場でゆっくり回りながら、AMCL に計算を頼み続ける（/request_nomotion_update）
//     3. /amcl_pose のばらつきが小さくなったら止まる
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <sys/stat.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;

class Localize : public rclcpp::Node
{
public:
  using Empty = std_srvs::srv::Empty;
  using Trigger = std_srvs::srv::Trigger;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using PoseCov = geometry_msgs::msg::PoseWithCovarianceStamped;

  Localize()
  : Node("localize")
  {
    const bool run_on_start = declare_parameter<bool>("run_on_start", true);
    const double start_delay = declare_parameter<double>("start_delay", 10.0);  // [秒] 起動してから始めるまで
    spin_speed_ = declare_parameter<double>("spin_speed", 0.5);        // [rad/s] 回る速さ
    max_time_ = declare_parameter<double>("max_time", 40.0);           // [秒] これだけ回っても絞れなければあきらめる
    min_time_ = declare_parameter<double>("min_time", 4.0);            // [秒] 少なくともこれだけ回ってから判定する
    xy_tol_ = declare_parameter<double>("xy_tolerance", 0.25);         // [m] 位置のばらつきがこれ以下なら OK
    yaw_tol_ = declare_parameter<double>("yaw_tolerance", 0.35);       // [rad] 向きのばらつきがこれ以下なら OK
    return_home_ = declare_parameter<bool>("return_home", true);       // 掃除が終わったら定位置へ戻る
    home_file_ = expand(declare_parameter<std::string>("home_file", "~/roomba_ws/maps/home.txt"));

    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pose_sub_ = create_subscription<PoseCov>(
      "amcl_pose", latched, [this](PoseCov::SharedPtr m) {pose_ = *m; have_pose_ = true; ++pose_count_;});
    status_sub_ = create_subscription<std_msgs::msg::String>(
      "tasks/status", latched, [this](std_msgs::msg::String::SharedPtr m) {on_status(m->data);});
    glob_ = create_client<Empty>("reinitialize_global_localization");
    nomo_ = create_client<Empty>("request_nomotion_update");
    coverage_ = create_client<Trigger>("tasks/go_coverage");
    nav_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    add_service("localize", [this](std::string & msg) {return start(false, msg);});
    add_service("clean", [this](std::string & msg) {return start(true, msg);});
    add_service("set_home", [this](std::string & msg) {return set_home(msg);});
    add_service("go_home", [this](std::string & msg) {return go_home(msg);});

    timer_ = create_wall_timer(50ms, [this]() {tick();});
    if (run_on_start) {
      start_timer_ = create_wall_timer(
        std::chrono::duration<double>(start_delay), [this]() {
          start_timer_->cancel();
          std::string m;
          start(false, m);
        });
    }
    RCLCPP_INFO(get_logger(), "準備完了（/localize・/clean・/set_home・/go_home）。定位置ファイル: %s", home_file_.c_str());
  }

private:
  enum class State { Idle, WaitAmcl, Spreading, Spinning };

  // ---------- 小道具 ----------
  static std::string expand(const std::string & p)
  {
    if (p.rfind("~/", 0) == 0) {
      const char * h = std::getenv("HOME");
      if (h) {return std::string(h) + p.substr(1);}
    }
    return p;
  }

  void add_service(const std::string & name, std::function<bool(std::string &)> f)
  {
    srvs_.push_back(create_service<Trigger>(
      name, [f](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        res->success = f(res->message);
      }));
  }

  double since(Clock::time_point t) const
  {
    return std::chrono::duration<double>(Clock::now() - t).count();
  }

  void spread(double & xy, double & yaw) const
  {
    if (!have_pose_) {xy = yaw = 1e9; return;}
    const auto & c = pose_.pose.covariance;
    xy = std::sqrt(std::max(c[0], 0.0) + std::max(c[7], 0.0));
    yaw = std::sqrt(std::max(c[35], 0.0));
  }

  std::string where() const
  {
    if (!have_pose_) {return "位置が出ませんでした";}
    double xy, yaw;
    spread(xy, yaw);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "x=%.2f y=%.2f、ばらつき 位置 %.0f cm・向き %.0f 度",
      pose_.pose.pose.position.x, pose_.pose.pose.position.y, xy * 100.0, yaw * 180.0 / kPi);
    return buf;
  }

  void publish_spin(double w)
  {
    geometry_msgs::msg::Twist t;
    t.angular.z = w;
    cmd_pub_->publish(t);
  }

  // ---------- 自己位置推定 ----------
  bool start(bool then_clean, std::string & msg)
  {
    if (state_ != State::Idle) {msg = "もう自己位置推定をしています"; return false;}
    then_clean_ = then_clean;
    state_ = State::WaitAmcl;
    state_since_ = Clock::now();
    msg = then_clean ? "自己位置推定を始めました。決まったら掃除を始めます（進み具合はログ）"
                     : "自己位置推定を始めました（進み具合はログ）";
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    return true;
  }

  void finish_localize(bool ok)
  {
    state_ = State::Idle;
    stop_ticks_ = 5;   // 止まる指令を何回か送る
    if (!ok) {
      RCLCPP_WARN(get_logger(), "自己位置が絞りきれませんでした（少し走らせてから /localize をもう一度）：%s%s",
        where().c_str(), then_clean_ ? "。掃除は始めません" : "");
      return;
    }
    RCLCPP_INFO(get_logger(), "自己位置が決まりました：%s", where().c_str());
    if (then_clean_) {start_coverage();}
  }

  void tick()
  {
    if (stop_ticks_ > 0) {publish_spin(0.0); --stop_ticks_;}
    switch (state_) {
      case State::Idle:
        return;
      case State::WaitAmcl:
        if (glob_->service_is_ready() && nomo_->service_is_ready()) {
          state_ = State::Spreading;
          state_since_ = Clock::now();
          glob_->async_send_request(std::make_shared<Empty::Request>(),
            [this](rclcpp::Client<Empty>::SharedFuture) {
              if (state_ != State::Spreading) {return;}
              RCLCPP_INFO(get_logger(), "位置の候補を地図全体にばらまきました。その場で回りながら探します");
              count0_ = pose_count_;
              state_ = State::Spinning;
              state_since_ = Clock::now();
              last_nomo_ = Clock::now() - 1s;
              last_log_ = -1;
            });
        } else if (since(state_since_) > 30.0) {
          state_ = State::Idle;
          RCLCPP_ERROR(get_logger(), "AMCL が見つかりません（nav_map.launch.py で起動していますか）");
        }
        return;
      case State::Spreading:
        if (since(state_since_) > 5.0) {
          state_ = State::Idle;
          RCLCPP_ERROR(get_logger(), "候補をばらまけませんでした");
        }
        return;
      case State::Spinning: {
        const double t = since(state_since_);
        publish_spin(spin_speed_);
        if (since(last_nomo_) > 0.3) {
          nomo_->async_send_request(std::make_shared<Empty::Request>(), [](rclcpp::Client<Empty>::SharedFuture) {});
          last_nomo_ = Clock::now();
        }
        double xy, yaw;
        spread(xy, yaw);
        const int k = static_cast<int>(t / 2.0);
        if (k != last_log_) {
          last_log_ = k;
          RCLCPP_INFO(get_logger(), "探しています：位置 %.0f cm・向き %.0f 度",
            std::min(xy, 9.99) * 100.0, std::min(yaw, kPi) * 180.0 / kPi);
        }
        if (t > min_time_ && pose_count_ > count0_ && xy < xy_tol_ && yaw < yaw_tol_) {
          finish_localize(true);
        } else if (t > max_time_) {
          finish_localize(false);
        }
        return;
      }
    }
  }

  // ---------- 掃除 ----------
  void start_coverage()
  {
    if (!coverage_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "/tasks/go_coverage が見つかりません。掃除は始めません");
      return;
    }
    coverage_->async_send_request(std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture f) {
        const auto r = f.get();
        cleaning_ = r->success;
        if (r->success) {
          RCLCPP_INFO(get_logger(), "掃除を始めました：%s%s", r->message.c_str(),
            return_home_ ? "（終わったら定位置へ戻ります）" : "");
        } else {
          RCLCPP_WARN(get_logger(), "掃除を始められませんでした：%s", r->message.c_str());
        }
      });
  }

  void on_status(const std::string & s)
  {
    if (!cleaning_) {return;}
    if (s.rfind("掃除完了", 0) == 0 || s.rfind("掃除中止", 0) == 0) {
      cleaning_ = false;
      if (return_home_) {
        RCLCPP_INFO(get_logger(), "%s → 定位置へ戻ります", s.c_str());
        std::string m;
        go_home(m);
      }
    } else if (s.rfind("止めました", 0) == 0 || s.rfind("待機中", 0) == 0) {
      cleaning_ = false;   // /tasks/cancel で止めたときは戻らない
    }
  }

  // ---------- 定位置 ----------
  bool load_home(double & x, double & y, double & yaw) const
  {
    std::ifstream f(home_file_);
    x = y = yaw = 0.0;
    return static_cast<bool>(f >> x >> y >> yaw);
  }

  bool set_home(std::string & msg)
  {
    if (!have_pose_) {msg = "自分の位置がまだわかりません"; return false;}
    const auto & p = pose_.pose.pose;
    const auto & q = p.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const auto slash = home_file_.find_last_of('/');
    if (slash != std::string::npos) {mkdir(home_file_.substr(0, slash).c_str(), 0755);}
    std::ofstream f(home_file_);
    if (!f) {msg = home_file_ + " に書けません"; return false;}
    f << p.position.x << " " << p.position.y << " " << yaw << "\n";
    char buf[160];
    std::snprintf(buf, sizeof(buf), "定位置を保存しました：x=%.2f y=%.2f 向き %.0f 度",
      p.position.x, p.position.y, yaw * 180.0 / kPi);
    msg = buf;
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    return true;
  }

  bool go_home(std::string & msg)
  {
    double x, y, yaw;
    const std::string name = load_home(x, y, yaw) ? "定位置" : "地図の原点（定位置は未保存）";
    if (!nav_->action_server_is_ready()) {
      msg = "Nav2（navigate_to_pose）が動いていません";
      RCLCPP_ERROR(get_logger(), "%s", msg.c_str());
      return false;
    }
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
    opt.goal_response_callback = [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr gh) {
        if (!gh) {RCLCPP_ERROR(get_logger(), "定位置へのゴールが受け付けられませんでした");}
      };
    opt.result_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r) {
        if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(get_logger(), "定位置に戻りました");
        } else {
          RCLCPP_WARN(get_logger(), "定位置に戻れませんでした");
        }
      };
    nav_->async_send_goal(goal, opt);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s（x=%.2f y=%.2f）へ戻ります", name.c_str(), x, y);
    msg = buf;
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    return true;
  }

  // パラメータ
  double spin_speed_, max_time_, min_time_, xy_tol_, yaw_tol_;
  bool return_home_;
  std::string home_file_;
  // 状態
  State state_ = State::Idle;
  Clock::time_point state_since_, last_nomo_;
  bool then_clean_ = false, cleaning_ = false, have_pose_ = false;
  long pose_count_ = 0, count0_ = 0;
  int last_log_ = -1, stop_ticks_ = 0;
  PoseCov pose_;
  // ROS
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<PoseCov>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Client<Empty>::SharedPtr glob_, nomo_;
  rclcpp::Client<Trigger>::SharedPtr coverage_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> srvs_;
  rclcpp::TimerBase::SharedPtr timer_, start_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Localize>());
  rclcpp::shutdown();
  return 0;
}
