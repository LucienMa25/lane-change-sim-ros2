/*
 * simulation_node.cpp — ROS2 Humble
 * Simulation main loop node: subscribes to planner decisions, executes controllers,
 * advances vehicle dynamics, updates scene state, and publishes visualization.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "lane_change_sim/msg/vehicle_array.hpp"
#include "lane_change_sim/msg/lane_array.hpp"
#include "lane_change_sim/msg/behavior_decision.hpp"
#include "lane_change_sim/msg/trajectory.hpp"
#include "lane_change_sim/control/common.hpp"
#include "lane_change_sim/control/idm_controller.hpp"
#include "lane_change_sim/control/kinematics.hpp"
#include "lane_change_sim/control/lat_controller.hpp"
#include "lane_change_sim/control/state_manager.hpp"
#include "lane_change_sim/ros/msg_adapter.hpp"
#include "lane_change_sim/utils/lane_utils.hpp"
#include "lane_change_sim/utils/math_utils.hpp"
#include "lane_change_sim/utils/trajectory_utils.hpp"
#include "lane_change_sim/types.hpp"

using namespace lcs;

class SimulationNode : public rclcpp::Node {
 public:
  SimulationNode()
    : Node("simulation_node"),
      tf_br_(this) {
    // Parameters
    this->declare_parameter<double>("ctrl_ts", 0.1);
    this->declare_parameter<double>("max_a", 3.0);
    this->declare_parameter<double>("min_a", -5.0);
    this->declare_parameter<double>("max_v", 10.0);
    this->declare_parameter<double>("max_steer", 0.2);
    this->declare_parameter<double>("wheel_base", 3.0);
    this->declare_parameter<double>("target_x", 200.0);
    this->declare_parameter<double>("max_sim_time", 120.0);
    this->declare_parameter<double>("max_ego_distance", 500.0);
    this->declare_parameter<bool>("publish_coarse_path", false);
    this->declare_parameter<bool>("show_ego_marker", true);
    this->declare_parameter<double>("traffic_speed_jitter", 2.0);
    this->declare_parameter<double>("traffic_ref_speed", 7.0);
    this->declare_parameter<int>("traffic_speed_seed", 0);
    this->declare_parameter<bool>("intention_recognition", true);

    cal_.ctrl_ts = this->get_parameter("ctrl_ts").as_double();
    cal_.max_a = this->get_parameter("max_a").as_double();
    cal_.min_a = this->get_parameter("min_a").as_double();
    cal_.max_v = this->get_parameter("max_v").as_double();
    cal_.max_steer = this->get_parameter("max_steer").as_double();
    cal_.wheel_base = this->get_parameter("wheel_base").as_double();
    target_x_ = this->get_parameter("target_x").as_double();
    max_sim_time_ = this->get_parameter("max_sim_time").as_double();
    max_ego_distance_ = this->get_parameter("max_ego_distance").as_double();
    publish_coarse_path_ = this->get_parameter("publish_coarse_path").as_bool();
    show_ego_marker_ = this->get_parameter("show_ego_marker").as_bool();
    traffic_speed_jitter_ = this->get_parameter("traffic_speed_jitter").as_double();
    traffic_ref_speed_ = this->get_parameter("traffic_ref_speed").as_double();
    intention_recognition_ = this->get_parameter("intention_recognition").as_bool();
    const int speed_seed = static_cast<int>(this->get_parameter("traffic_speed_seed").as_int());
    if (speed_seed != 0) {
      rng_.seed(static_cast<unsigned>(speed_seed));  // reproducible scenario
    } else {
      rng_.seed(std::random_device{}());             // fresh scenario every run
    }

    // Subscribers
    lane_sub_ = this->create_subscription<lane_change_sim::msg::LaneArray>(
        "/scene/lanes", 1,
        [this](const lane_change_sim::msg::LaneArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mutex_);
          lanes_.clear();
          for (const auto& lm : msg->lanes) {
            lanes_.push_back(ros_adapter::fromRosLane(lm));
          }
          scene_ready_ = true;
        });

    veh_sub_ = this->create_subscription<lane_change_sim::msg::VehicleArray>(
        "/scene/vehicle_states", 1,
        [this](const lane_change_sim::msg::VehicleArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mutex_);
          if (vehicles_.empty()) {
            for (const auto& vm : msg->vehicles) {
              vehicles_.push_back(ros_adapter::fromRosVehicleState(vm));
              auto& v = vehicles_.back();
              // Per-vehicle random preferred speed for traffic (ego keeps 0 =
              // lane ref). Creates genuine speed differences -> realistic
              // car-following pressure and lane-change triggers for the planner.
              if (v.id != 0 && traffic_speed_jitter_ > 0.0) {
                std::uniform_real_distribution<double> dist(
                    -traffic_speed_jitter_, traffic_speed_jitter_);
                const double base = control::LaneRefSpeedOrDefault(
                    lanes_, v.lane_id, traffic_ref_speed_);
                v.desired_speed = std::clamp(base + dist(rng_), 1.0, cal_.max_v);
              }
              RCLCPP_WARN(this->get_logger(),
                         "Init veh id=%u px=%.1f py=%.1f speed=%.2f lane=%u v_desired=%.2f",
                         v.id, v.state.px, v.state.py, v.state.speed, v.lane_id,
                         v.desired_speed);
            }
          }
        });

    dec_sub_ = this->create_subscription<lane_change_sim::msg::BehaviorDecision>(
        "/planner/decision", 1,
        [this](const lane_change_sim::msg::BehaviorDecision::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mutex_);
          win_lat_ = static_cast<LatBehavior>(msg->lat_behavior);
          win_lon_ = static_cast<LonBehavior>(msg->lon_behavior);
        });

    traj_sub_ = this->create_subscription<lane_change_sim::msg::Trajectory>(
        "/planner/trajectory", 1,
        [this](const lane_change_sim::msg::Trajectory::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mutex_);
          opt_traj_ = ros_adapter::fromRosTrajectory(*msg);
        });

    coarse_traj_sub_ = this->create_subscription<lane_change_sim::msg::Trajectory>(
        "/planner/coarse_trajectory", 1,
        [this](const lane_change_sim::msg::Trajectory::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mutex_);
          coarse_traj_ = ros_adapter::fromRosTrajectory(*msg);
        });

    // Publishers
    ctrl_veh_pub_ = this->create_publisher<lane_change_sim::msg::VehicleArray>("/sim/vehicle_states", 10);
    opt_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sim/opt_path", 10);
    coarse_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/sim/coarse_path", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/sim/visualization", 10);
    decision_text_pub_ = this->create_publisher<std_msgs::msg::String>("/sim/decision_text", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/sim/joint_states", 10);

    // Main simulation timer
    sim_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(cal_.ctrl_ts),
        std::bind(&SimulationNode::simLoop, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for scene to be ready...");
  }

 private:
  static Vehicle* findEgo(std::vector<Vehicle>& vehicles) {
    for (auto& v : vehicles) if (v.id == 0) return &v;
    return nullptr;
  }

  // ── Behavior-based traffic intention recognition ──
  // Judges the merge-relevant vehicle of ego's intended target lane from its
  // OBSERVED motion (unlike StateManagerRealtime::getIntention, which is a
  // game-theory assumption used internally by the LC state machine).
  //
  // Relevance: with a merge intention (planner window behavior or lateral
  // drift), the critical opponent is the nearest vehicle BEHIND ego in the
  // target lane. Its attitude towards ceding the gap:
  //   Aggresive (red): closing on ego's merge point fast — TTC to the entry
  //                    point < 1.5 s while faster than ego → will likely block
  //   Yield (green):   TTC > 2.5 s, or clearly slower / falling back → space
  //                    is open for ego
  //   None (blue):     nobody threatening / no merge intention / out of window
  // Without an ego merge intention, traffic IDM output is just free-flow
  // cruising (observed acc ≈ 0 here), so nothing is marked.
  void recognizeIntentions(std::vector<Vehicle>& vehicles, const Vehicle& ego,
                           LatBehavior lat, const std::vector<Lane>& lanes) {
    uint32_t target_lane_id = 0;
    if (lat == LatBehavior::LaneChangeLeft || lat == LatBehavior::AvoidLeft) {
      const Lane* cur = control::FindLaneById(lanes, ego.lane_id);
      if (cur) target_lane_id = static_cast<uint32_t>(cur->left_lane_id);
    } else if (lat == LatBehavior::LaneChangeRight ||
               lat == LatBehavior::AvoidRight) {
      const Lane* cur = control::FindLaneById(lanes, ego.lane_id);
      if (cur) target_lane_id = static_cast<uint32_t>(cur->right_lane_id);
    }
    // Mid-maneuver (drifting), use the lane we are actually heading into even
    // if the FSM already switched lane_id bookkeeping.
    if (target_lane_id == 0 && ego.lane_change_stage != LaneChangeStage::Off) {
      for (auto& v : vehicles) v.intention = Intention::None;  // clear marks
      return;
    }
    if (target_lane_id == 0) {
      for (auto& v : vehicles) v.intention = Intention::None;
      return;
    }

    Vehicle* key_v = nullptr;
    double best_dx = std::numeric_limits<double>::max();
    for (auto& v : vehicles) {
      if (v.id == ego.id || v.lane_id != target_lane_id) continue;
      if (v.state.px >= ego.state.px) continue;  // only rear-side opponents
      const double dx = ego.state.px - v.state.px;
      if (dx < best_dx) { best_dx = dx; key_v = &v; }
    }

    for (auto& v : vehicles) {
      if (&v != key_v) v.intention = Intention::None;
    }
    if (!key_v || best_dx > 45.0) return;

    // Predict where ego and the opponent will be in the next second
    const double ego_next  = ego.state.px  + ego.state.speed  * 1.0;
    const double other_next = key_v->state.px + key_v->state.speed * 1.0;
    const double closing = key_v->state.speed - ego.state.speed;
    const double ttc = (closing > 0.1)
        ? std::max(0.0, ego_next - other_next) / closing
        : std::numeric_limits<double>::infinity();

    Intention est = Intention::None;
    if (ttc < 1.5) {
      est = Intention::Aggresive;                    // sprinting to block the gap
    } else if (ttc > 2.5 || closing < -0.5) {
      est = Intention::Yield;                        // gap is opening / falling back
    }
    // Hysteresis: keep an established Aggressive until the threat clearly passes
    if (est == Intention::None && key_v->intention == Intention::Aggresive &&
        ttc < 2.2) {
      est = Intention::Aggresive;
    }
    key_v->intention = est;
  }

  nav_msgs::msg::Path makePath(const Trajectory& traj, const std::string& frame, double z = 0.1) {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    path.header.stamp = this->now();
    for (const auto& pt : traj.points) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = pt.x; ps.pose.position.y = pt.y; ps.pose.position.z = z;
      double c = std::cos(pt.h / 2.0), s = std::sin(pt.h / 2.0);
      ps.pose.orientation.w = c; ps.pose.orientation.z = s;
      path.poses.push_back(ps);
    }
    return path;
  }

  void simLoop() {
    std::lock_guard<std::mutex> lk(mutex_);

    if (!scene_ready_) return;
    if (vehicles_.empty() || lanes_.empty()) return;

    // Check termination
    if (finished_) return;

    auto& vehicles = vehicles_;

    // Update lane IDs
    for (auto& v : vehicles) utils::updateLaneId(v, lanes_);

    // Control
    Vehicle* ego = findEgo(vehicles);
    for (auto& v : vehicles) {
      const double prev_steer = v.steer;
      const auto leading = control::FindLeadingVehicle(v, vehicles, lanes_, 20.0, cal_.max_v, 1.0, true);
      const double lead_px = leading.state.px;
      const double lead_spd = leading.state.speed;
      const double desired_speed = control::VehicleDesiredSpeed(v, lanes_, cal_.max_v);

      if (v.id == 0 && ego) {
        state_mgr_.update(*ego, win_lat_, win_lon_, vehicles, lanes_, cal_.ctrl_ts);

        VehicleState lead_state;
        lead_state.px = lead_px;
        lead_state.speed = lead_spd;
        const double idm_acc = control::ComputeIidmAcceleration(*ego, lead_state, desired_speed, cal_.min_a, cal_.max_a);
        if (!opt_traj_.points.empty()) {
          const size_t nearest_idx = utils::nearestTrajectoryIndex(opt_traj_, ego->state.px, ego->state.py);
          const size_t lookahead_idx = std::min(nearest_idx + static_cast<size_t>(2), opt_traj_.points.size() - 1);
          const double planned_acc = utils::clamp(opt_traj_.points[lookahead_idx].a, cal_.min_a, cal_.max_a);
          ego->acc = std::min(planned_acc, idm_acc);
        } else {
          ego->acc = idm_acc;
        }

        if (!opt_traj_.points.empty() && ego->lat_behavior == LatBehavior::LaneKeeping) {
          ego->steer = control::CurvatureTrackingSteer(*ego, opt_traj_, cal_.max_steer);
        } else {
          ego->steer = control::LatControl(*ego, lanes_, cal_.max_steer);
        }
      } else {
        VehicleState lead_state;
        lead_state.px = lead_px;
        lead_state.speed = lead_spd;
        v.acc = control::ComputeIidmAcceleration(v, lead_state, desired_speed, cal_.min_a, cal_.max_a);
        v.steer = control::LatControl(v, lanes_, cal_.max_steer);
      }
      const double steer_cmd = utils::clamp(v.steer, -cal_.max_steer, cal_.max_steer);
      const double max_step = cal_.max_steer_rate * cal_.ctrl_ts;
      v.steer = utils::clamp(steer_cmd, prev_steer - max_step, prev_steer + max_step);
      control::UpdateBicycleKinematics(v, cal_.ctrl_ts, cal_.max_v);
    }

    // Behavior-based intention recognition for surrounding traffic (runs after
    // the dynamics update; replaces the FSM's game-theory assumption marking
    // with real observed kinematics of the merge-relevant opponent).
    // During the maneuver the FSM keeps ego.lat_behavior at LaneChange*/Avoid*
    // even after the planner window reverted to LaneKeeping, so prefer it.
    if (intention_recognition_ && ego) {
      const LatBehavior active_lat =
          (ego->lat_behavior != LatBehavior::LaneKeeping) ? ego->lat_behavior
                                                          : win_lat_;
      recognizeIntentions(vehicles, *ego, active_lat, lanes_);
    }

    // Publish updated vehicle states
    auto now = this->now();
    ctrl_veh_pub_->publish(ros_adapter::toRosVehicleArray(vehicles, now, "map"));

    // TF broadcasts
    {
      std::vector<geometry_msgs::msg::TransformStamped> tfs;
      for (const auto& v : vehicles) {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = now;
        ts.header.frame_id = "map";
        ts.child_frame_id = (v.id == 0) ? "base_footprint"
                                        : "agent_" + std::to_string(v.id) + "/base_footprint";
        ts.transform.translation.x = v.state.px;
        ts.transform.translation.y = v.state.py;
        ts.transform.translation.z = 0.0;
        const double half = v.state.heading * 0.5;
        ts.transform.rotation.w = std::cos(half);
        ts.transform.rotation.x = 0.0;
        ts.transform.rotation.y = 0.0;
        ts.transform.rotation.z = std::sin(half);
        tfs.push_back(ts);
      }
      tf_br_.sendTransform(tfs);
    }

    // Path visualization
    if (publish_coarse_path_ && !coarse_traj_.points.empty()) {
      coarse_path_pub_->publish(makePath(coarse_traj_, "map", 0.10));
    }
    if (!opt_traj_.points.empty()) opt_path_pub_->publish(makePath(opt_traj_, "map", 0.20));

    // Robot model joint states
    if (ego) {
      const double wheel_radius = 0.40;
      ego_wheel_roll_ += (ego->state.speed * cal_.ctrl_ts) / std::max(0.05, wheel_radius);

      const double track = 2.40;
      const double L = cal_.wheel_base;
      const double delta = ego->steer;
      double left_steer = delta;
      double right_steer = delta;
      if (std::abs(delta) > 1e-6) {
        const double R = L / std::tan(std::abs(delta));
        const double sign = (delta > 0.0) ? 1.0 : -1.0;
        left_steer  = sign * std::atan2(L, R - track * 0.5);
        right_steer = sign * std::atan2(L, R + track * 0.5);
      }

      sensor_msgs::msg::JointState js;
      js.header.stamp = now;
      js.name = {
        "left_front_turning", "right_front_turning",
        "left_front_rolling", "right_front_rolling",
        "left_rear_rolling", "right_rear_rolling"
      };
      js.position = {
        left_steer, right_steer,
        ego_wheel_roll_, ego_wheel_roll_,
        ego_wheel_roll_, ego_wheel_roll_
      };
      joint_state_pub_->publish(js);
    }

    // Vehicle markers
    visualization_msgs::msg::MarkerArray ma;
    int mid = 100;
    for (const auto& v : vehicles) {
      if (v.id == 0 && !show_ego_marker_) {
        continue;
      }
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map"; m.header.stamp = now;
      m.ns = "sim_vehicles"; m.id = mid++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = v.state.px; m.pose.position.y = v.state.py; m.pose.position.z = 0.5;
      double c = std::cos(v.state.heading / 2.0), s_v = std::sin(v.state.heading / 2.0);
      m.pose.orientation.w = c; m.pose.orientation.z = s_v;
      m.scale.x = v.length; m.scale.y = v.width; m.scale.z = 1.0;
      if (v.id == 0) {
        m.color.r = 0.95f; m.color.g = 0.85f; m.color.b = 0.1f;
      } else if (v.intention == Intention::Yield) {
        m.color.r = 0.0f; m.color.g = 0.9f; m.color.b = 0.0f;
      } else if (v.intention == Intention::Aggresive) {
        m.color.r = 0.9f; m.color.g = 0.0f; m.color.b = 0.0f;
      } else {
        m.color.r = 0.2f; m.color.g = 0.3f; m.color.b = 0.9f;
      }
      m.color.a = 0.9;
      ma.markers.push_back(m);
    }
    viz_pub_->publish(ma);

    // Decision text
    std_msgs::msg::String dtxt;
    auto lat_str = [](LatBehavior l) -> std::string {
      switch (l) {
        case LatBehavior::LaneChangeLeft:  return "LC_Left";
        case LatBehavior::LaneChangeRight: return "LC_Right";
        case LatBehavior::AvoidLeft:       return "Avd_Left";
        case LatBehavior::AvoidRight:      return "Avd_Right";
        default:                           return "Keep";
      }
    };
    auto lon_str = [](LonBehavior l) -> std::string {
      switch (l) {
        case LonBehavior::Accelerate: return "Accel";
        case LonBehavior::Decelerate: return "Decel";
        case LonBehavior::Stopping:   return "Stop";
        default:                      return "Maintain";
      }
    };
    dtxt.data = "Decision: " + lat_str(win_lat_) + "-" + lon_str(win_lon_);
    decision_text_pub_->publish(dtxt);

    // Track ego start position
    if (ego && !ego_start_set_) {
      ego_start_x_ = ego->state.px;
      ego_start_set_ = true;
      sim_start_ = now;
    }

    // Check termination
    const double elapsed = (now - sim_start_).seconds();
    const double traveled = ego ? (ego->state.px - ego_start_x_) : 0.0;
    if (ego && ego->state.px >= target_x_) {
      RCLCPP_INFO(this->get_logger(), "Ego reached target_x=%.1f (traveled=%.1fm, t=%.1fs), stopping.",
                  target_x_, traveled, elapsed);
      finished_ = true;
      sim_timer_->cancel();
      return;
    }
    if (elapsed >= max_sim_time_) {
      RCLCPP_INFO(this->get_logger(), "Max sim time %.1fs reached (traveled=%.1fm), stopping.",
                  max_sim_time_, traveled);
      finished_ = true;
      sim_timer_->cancel();
      return;
    }
    if (ego && traveled >= max_ego_distance_) {
      RCLCPP_INFO(this->get_logger(), "Max ego distance %.1fm reached (t=%.1fs), stopping.",
                  max_ego_distance_, elapsed);
      finished_ = true;
      sim_timer_->cancel();
      return;
    }
  }

  std::mutex mutex_;
  std::vector<Vehicle> vehicles_;
  std::vector<Lane> lanes_;
  Trajectory opt_traj_;
  Trajectory coarse_traj_;
  LatBehavior win_lat_{LatBehavior::LaneKeeping};
  LonBehavior win_lon_{LonBehavior::MainTain};
  bool scene_ready_{false};
  bool finished_{false};
  bool show_ego_marker_{true};
  bool publish_coarse_path_{false};
  double traffic_speed_jitter_{2.0};
  double traffic_ref_speed_{7.0};
  bool intention_recognition_{true};
  std::mt19937 rng_;  // seeded in constructor (traffic_speed_seed or random_device)
  double ego_wheel_roll_{0.0};

  control::StateManagerRealtime state_mgr_;
  Calibration cal_;

  double target_x_{200.0};
  double max_sim_time_{120.0};
  double max_ego_distance_{500.0};
  double ego_start_x_{0.0};
  bool ego_start_set_{false};
  rclcpp::Time sim_start_;

  tf2_ros::TransformBroadcaster tf_br_;

  rclcpp::Subscription<lane_change_sim::msg::LaneArray>::SharedPtr lane_sub_;
  rclcpp::Subscription<lane_change_sim::msg::VehicleArray>::SharedPtr veh_sub_;
  rclcpp::Subscription<lane_change_sim::msg::BehaviorDecision>::SharedPtr dec_sub_;
  rclcpp::Subscription<lane_change_sim::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<lane_change_sim::msg::Trajectory>::SharedPtr coarse_traj_sub_;

  rclcpp::Publisher<lane_change_sim::msg::VehicleArray>::SharedPtr ctrl_veh_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr coarse_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_text_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::TimerBase::SharedPtr sim_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationNode>());
  rclcpp::shutdown();
  return 0;
}
