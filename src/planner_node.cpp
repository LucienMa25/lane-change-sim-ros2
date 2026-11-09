/*
 * planner_node.cpp — ROS2 Humble
 * Planner node: subscribes to simulation vehicle states, runs MCTS + trajectory
 * optimization periodically, publishes decisions, optimized trajectories, and
 * provides RunEpisode action.
 */
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "lane_change_sim/action/run_episode.hpp"
#include "lane_change_sim/msg/behavior_decision.hpp"
#include "lane_change_sim/msg/vehicle_array.hpp"
#include "lane_change_sim/msg/lane_array.hpp"
#include "lane_change_sim/msg/trajectory.hpp"
#include "lane_change_sim/ros/msg_adapter.hpp"
#include "lane_change_sim/types.hpp"
#include "lane_change_sim/planner_core.hpp"

using namespace lcs;
using RunEpisode = lane_change_sim::action::RunEpisode;

// ─────────────── PlannerNode class ───────────────

class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode()
    : Node("planner_node"),
      plan_cycle_(0),
      vehicles_ready_(false),
      lanes_ready_(false) {
    // Load calibration from parameters
    loadCalibration();

    // Compute max search times
    calib_.max_search_times = 0;
    for (int i = 1; i <= calib_.tree_hight; ++i) {
      int k = 1;
      for (int j = 0; j < i; ++j) k *= 9;
      calib_.max_search_times += k;
    }

    planner_.setCalibration(calib_);

    // Subscribers
    veh_sub_ = this->create_subscription<lane_change_sim::msg::VehicleArray>(
        "/sim/vehicle_states", 1,
        [this](const lane_change_sim::msg::VehicleArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mutex_);
          vehicles_.clear();
          for (const auto& vm : msg->vehicles) {
            vehicles_.push_back(ros_adapter::fromRosVehicleState(vm));
          }
          vehicles_ready_ = true;
        });

    lane_sub_ = this->create_subscription<lane_change_sim::msg::LaneArray>(
        "/scene/lanes", 1,
        [this](const lane_change_sim::msg::LaneArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mutex_);
          lanes_.clear();
          for (const auto& lm : msg->lanes) {
            lanes_.push_back(ros_adapter::fromRosLane(lm));
          }
          lanes_ready_ = true;
        });

    // Publishers
    dec_pub_ = this->create_publisher<lane_change_sim::msg::BehaviorDecision>("/planner/decision", 10);
    coarse_traj_pub_ = this->create_publisher<lane_change_sim::msg::Trajectory>("/planner/coarse_trajectory", 10);
    traj_pub_ = this->create_publisher<lane_change_sim::msg::Trajectory>("/planner/trajectory", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planner/visualization", 10);
    cube_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planner/cubes", 10);
    decision_text_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planner/decision_text", 10);

    // Action server
    action_server_ = rclcpp_action::create_server<RunEpisode>(
        this, "/planner/run_episode",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const RunEpisode::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<EpisodeHandle>&) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<EpisodeHandle>& handle) {
          std::thread{std::bind(&PlannerNode::executeEpisode, this, handle)}.detach();
        });

    // Periodic planning timer
    plan_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(calib_.sim_ts),
        [this]() {
          if (vehicles_ready_ && lanes_ready_) planOnce();
        });

    RCLCPP_INFO(this->get_logger(), "Ready. Advertising on /planner/decision and /planner/trajectory.");
  }

 private:
  using EpisodeHandle = rclcpp_action::ServerGoalHandle<RunEpisode>;

  void loadCalibration() {
    this->declare_parameter<double>("sim_time", 9.0);
    this->declare_parameter<double>("sim_ts", 0.3);
    this->declare_parameter<double>("ctrl_ts", 0.1);
    this->declare_parameter<int>("tree_hight", 3);
    this->declare_parameter<bool>("mcts_delete_on_failure", true);
    this->declare_parameter<double>("max_v", 10.0);
    this->declare_parameter<double>("max_a", 3.0);
    this->declare_parameter<double>("min_a", -5.0);
    this->declare_parameter<double>("max_j", 5.0);
    this->declare_parameter<double>("max_delta", 0.5);
    this->declare_parameter<double>("max_omega", 1.0);
    this->declare_parameter<double>("max_steer", 0.2);
    this->declare_parameter<double>("max_steer_rate", 0.5);
    this->declare_parameter<double>("effic_cost", 2.0);
    this->declare_parameter<double>("safe_cost", 10.0);
    this->declare_parameter<double>("action_cost", 10.0);
    this->declare_parameter<double>("navi_cost", 1000.0);
    this->declare_parameter<double>("affect_cost", 5.0);
    this->declare_parameter<double>("fade_factor", 0.8);
    this->declare_parameter<double>("search_new_wight", 10.0);
    this->declare_parameter<double>("rss_response_time", 1.0);
    this->declare_parameter<double>("rss_longitudinal_acc_max", 2.0);
    this->declare_parameter<double>("rss_longitudinal_brake_min", 4.0);
    this->declare_parameter<double>("rss_longitudinal_brake_max", 8.0);
    this->declare_parameter<double>("rss_lateral_miu", 0.3);
    this->declare_parameter<double>("mobil_politeness", 0.3);
    this->declare_parameter<double>("mobil_gain_threshold", 0.1);
    this->declare_parameter<double>("mobil_rear_acc_threshold", -2.0);
    this->declare_parameter<double>("rss_cost", 30.0);
    this->declare_parameter<double>("traj_time", 6.0);
    this->declare_parameter<double>("traj_ts", 0.1);
    this->declare_parameter<double>("weight_penalty", 1e5);
    this->declare_parameter<double>("weight_x", 5.0);
    this->declare_parameter<double>("weight_y", 5.0);
    this->declare_parameter<double>("weight_theta", 5.0);
    this->declare_parameter<double>("weight_cube_center_x", 2.0);
    this->declare_parameter<double>("weight_cube_center_y", 6.0);
    this->declare_parameter<double>("weight_omega", 100.0);
    this->declare_parameter<double>("weight_jerk", 1.0);
    this->declare_parameter<int>("ipopt_max_iter", 80);
    this->declare_parameter<double>("ipopt_tol", 1e-4);
    this->declare_parameter<double>("ipopt_acceptable_tol", 1e-4);
    this->declare_parameter<int>("ipopt_acceptable_iter", 10);
    this->declare_parameter<std::string>("ipopt_mu_strategy", "monotone");
    this->declare_parameter<bool>("warm_start_enable", false);
    this->declare_parameter<double>("pred_horizon", 6.0);
    this->declare_parameter<bool>("pred_enable_imm", false);
    this->declare_parameter<double>("pred_process_noise", 0.5);
    this->declare_parameter<bool>("adapt_weights_enable", true);
    this->declare_parameter<double>("adapt_front_dist_near", 10.0);
    this->declare_parameter<double>("adapt_front_dist_far", 50.0);
    this->declare_parameter<double>("adapt_front_rel_v_threshold", 1.0);
    this->declare_parameter<bool>("lc_gap_tracking_enable", true);
    this->declare_parameter<double>("lc_gap_headway_time", 1.0);
    this->declare_parameter<double>("lc_gap_min_dist", 2.0);
    this->declare_parameter<double>("lc_ctx_weight_cur", 0.4);
    this->declare_parameter<double>("lc_ctx_weight_tar", 0.8);
    this->declare_parameter<double>("lc_gap_kp", 0.1);
    this->declare_parameter<double>("lc_gap_dv_lb", -3.0);
    this->declare_parameter<double>("lc_gap_dv_ub", 5.0);
    this->declare_parameter<int>("target_lane_id", 2);
    this->declare_parameter<double>("target_x", 200.0);

    calib_.sim_time = this->get_parameter("sim_time").as_double();
    calib_.sim_ts = this->get_parameter("sim_ts").as_double();
    calib_.ctrl_ts = this->get_parameter("ctrl_ts").as_double();
    calib_.tree_hight = this->get_parameter("tree_hight").as_int();
    calib_.mcts_delete_on_failure = this->get_parameter("mcts_delete_on_failure").as_bool();
    calib_.max_v = this->get_parameter("max_v").as_double();
    calib_.max_a = this->get_parameter("max_a").as_double();
    calib_.min_a = this->get_parameter("min_a").as_double();
    calib_.max_j = this->get_parameter("max_j").as_double();
    calib_.max_delta = this->get_parameter("max_delta").as_double();
    calib_.max_omega = this->get_parameter("max_omega").as_double();
    calib_.max_steer = this->get_parameter("max_steer").as_double();
    calib_.max_steer_rate = this->get_parameter("max_steer_rate").as_double();
    calib_.effic_cost = this->get_parameter("effic_cost").as_double();
    calib_.safe_cost = this->get_parameter("safe_cost").as_double();
    calib_.action_cost = this->get_parameter("action_cost").as_double();
    calib_.navi_cost = this->get_parameter("navi_cost").as_double();
    calib_.affect_cost = this->get_parameter("affect_cost").as_double();
    calib_.fade_factor = this->get_parameter("fade_factor").as_double();
    calib_.search_new_wight = this->get_parameter("search_new_wight").as_double();
    calib_.rss_response_time = this->get_parameter("rss_response_time").as_double();
    calib_.rss_longitudinal_acc_max = this->get_parameter("rss_longitudinal_acc_max").as_double();
    calib_.rss_longitudinal_brake_min = this->get_parameter("rss_longitudinal_brake_min").as_double();
    calib_.rss_longitudinal_brake_max = this->get_parameter("rss_longitudinal_brake_max").as_double();
    calib_.rss_lateral_miu = this->get_parameter("rss_lateral_miu").as_double();
    calib_.mobil_politeness = this->get_parameter("mobil_politeness").as_double();
    calib_.mobil_gain_threshold = this->get_parameter("mobil_gain_threshold").as_double();
    calib_.mobil_rear_acc_threshold = this->get_parameter("mobil_rear_acc_threshold").as_double();
    calib_.rss_cost = this->get_parameter("rss_cost").as_double();
    calib_.traj_time = this->get_parameter("traj_time").as_double();
    calib_.traj_ts = this->get_parameter("traj_ts").as_double();
    calib_.weight_penalty = this->get_parameter("weight_penalty").as_double();
    calib_.weight_x = this->get_parameter("weight_x").as_double();
    calib_.weight_y = this->get_parameter("weight_y").as_double();
    calib_.weight_theta = this->get_parameter("weight_theta").as_double();
    calib_.weight_cube_center_x = this->get_parameter("weight_cube_center_x").as_double();
    calib_.weight_cube_center_y = this->get_parameter("weight_cube_center_y").as_double();
    calib_.weight_omega = this->get_parameter("weight_omega").as_double();
    calib_.weight_jerk = this->get_parameter("weight_jerk").as_double();
    calib_.ipopt_max_iter = this->get_parameter("ipopt_max_iter").as_int();
    calib_.ipopt_tol = this->get_parameter("ipopt_tol").as_double();
    calib_.ipopt_acceptable_tol = this->get_parameter("ipopt_acceptable_tol").as_double();
    calib_.ipopt_acceptable_iter = this->get_parameter("ipopt_acceptable_iter").as_int();
    calib_.ipopt_mu_strategy = this->get_parameter("ipopt_mu_strategy").as_string();
    calib_.warm_start_enable = this->get_parameter("warm_start_enable").as_bool();
    calib_.pred_horizon = this->get_parameter("pred_horizon").as_double();
    calib_.pred_enable_imm = this->get_parameter("pred_enable_imm").as_bool();
    calib_.pred_process_noise = this->get_parameter("pred_process_noise").as_double();
    calib_.adapt_weights_enable = this->get_parameter("adapt_weights_enable").as_bool();
    calib_.adapt_front_dist_near = this->get_parameter("adapt_front_dist_near").as_double();
    calib_.adapt_front_dist_far = this->get_parameter("adapt_front_dist_far").as_double();
    calib_.adapt_front_rel_v_threshold = this->get_parameter("adapt_front_rel_v_threshold").as_double();
    calib_.lc_gap_tracking_enable = this->get_parameter("lc_gap_tracking_enable").as_bool();
    calib_.lc_gap_headway_time = this->get_parameter("lc_gap_headway_time").as_double();
    calib_.lc_gap_min_dist = this->get_parameter("lc_gap_min_dist").as_double();
    calib_.lc_ctx_weight_cur = this->get_parameter("lc_ctx_weight_cur").as_double();
    calib_.lc_ctx_weight_tar = this->get_parameter("lc_ctx_weight_tar").as_double();
    calib_.lc_gap_kp = this->get_parameter("lc_gap_kp").as_double();
    calib_.lc_gap_dv_lb = this->get_parameter("lc_gap_dv_lb").as_double();
    calib_.lc_gap_dv_ub = this->get_parameter("lc_gap_dv_ub").as_double();
    target_lane_id_ = static_cast<uint32_t>(this->get_parameter("target_lane_id").as_int());
    target_x_ = this->get_parameter("target_x").as_double();
  }

  void planOnce() {
    std::vector<Vehicle> vehicles_snap;
    std::vector<Lane> lanes_snap;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      vehicles_snap = vehicles_;
      lanes_snap = lanes_;
    }
    if (vehicles_snap.empty() || lanes_snap.empty()) return;

    auto t0 = this->now();
    SearchResult result = planner_.plan(vehicles_snap, lanes_snap, plan_cycle_, target_lane_id_, target_x_);
    ++plan_cycle_;

    double dt = (this->now() - t0).seconds();
    RCLCPP_INFO(this->get_logger(), "Planner done in %.3fs  decision=%s", dt, result.decision_type.c_str());

    // Publish decision
    {
      lane_change_sim::msg::BehaviorDecision dm;
      dm.header.stamp = this->now();
      dm.type = result.decision_type;
      dm.lat_behavior = static_cast<uint8_t>(result.lat_behavior);
      dm.lon_behavior = static_cast<uint8_t>(result.lon_behavior);
      dm.value = result.value;
      dm.score = result.score;
      dec_pub_->publish(dm);
    }

    // Publish trajectories
    {
      const auto stamp = this->now();
      coarse_traj_pub_->publish(ros_adapter::toRosTrajectory(result.coarse_ego, stamp, "map"));
      if (!result.optimized_ego.points.empty()) {
        traj_pub_->publish(ros_adapter::toRosTrajectory(result.optimized_ego, stamp, "map"));
      } else {
        traj_pub_->publish(ros_adapter::toRosTrajectory(result.coarse_ego, stamp, "map"));
      }
    }

    publishVisualization(result);
    publishCubes(planner_.getLastCubes());
  }

  void publishVisualization(const SearchResult& result) {
    visualization_msgs::msg::MarkerArray ma;
    int mid = 0;
    auto now = this->now();

    for (size_t k = 0; k < result.coarse_agents.size(); ++k) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map"; m.header.stamp = now;
      m.ns = "mcts_agent_traj"; m.id = mid++;
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.scale.x = 0.10; m.pose.orientation.w = 1.0;
      m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.7;
      for (const auto& pt : result.coarse_agents[k].points) {
        geometry_msgs::msg::Point p; p.x = pt.x; p.y = pt.y; p.z = 0.15;
        m.points.push_back(p);
      }
      if (!m.points.empty()) ma.markers.push_back(m);
    }

    // Decision text
    {
      visualization_msgs::msg::MarkerArray text_ma;
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map"; m.header.stamp = now;
      m.ns = "decision_text"; m.id = 0;
      m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;
      {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!vehicles_.empty()) {
          m.pose.position.x = vehicles_[0].state.px;
          m.pose.position.y = vehicles_[0].state.py + 3.0;
        }
      }
      m.pose.position.z = 3.0;
      m.pose.orientation.w = 1.0;
      m.scale.z = 1.2;
      m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0;
      m.text = result.decision_type + " Score=" + std::to_string(static_cast<int>(result.value));
      text_ma.markers.push_back(m);
      decision_text_pub_->publish(text_ma);
    }

    viz_pub_->publish(ma);
  }

  void publishCubes(const std::vector<DriveCube>& cubes) {
    visualization_msgs::msg::MarkerArray ma;
    auto now = this->now();

    visualization_msgs::msg::Marker del;
    del.header.frame_id = "map"; del.header.stamp = now;
    del.ns = "drive_cubes";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);

    for (size_t i = 0; i < cubes.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map"; m.header.stamp = now;
      m.ns = "drive_cubes"; m.id = static_cast<int>(i);
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;

      const double cx = 0.5 * (cubes[i].lb.x + cubes[i].ub.x);
      const double cy = 0.5 * (cubes[i].lb.y + cubes[i].ub.y);
      m.pose.position.x = cx;
      m.pose.position.y = cy;
      m.pose.position.z = 0.05;
      m.pose.orientation.w = 1.0;

      m.scale.x = std::max(0.1, cubes[i].ub.x - cubes[i].lb.x);
      m.scale.y = std::max(0.1, cubes[i].ub.y - cubes[i].lb.y);
      m.scale.z = 0.1;

      m.color.r = 0.2f;
      m.color.g = 0.6f;
      m.color.b = 1.0f;
      m.color.a = 0.35f;

      ma.markers.push_back(m);
    }

    cube_pub_->publish(ma);
  }

  void executeEpisode(const std::shared_ptr<EpisodeHandle>& handle) {
    auto goal = handle->get_goal();
    uint32_t max_cycles = goal->max_cycles;
    RCLCPP_INFO(this->get_logger(), "RunEpisode action: max_cycles=%u", max_cycles);

    auto feedback = std::make_shared<RunEpisode::Feedback>();
    auto result = std::make_shared<RunEpisode::Result>();
    rclcpp::Rate r(10);
    uint32_t cycle = 0;

    while (rclcpp::ok() && cycle < max_cycles) {
      if (handle->is_canceling()) {
        handle->canceled(result);
        return;
      }
      if (vehicles_ready_ && lanes_ready_) {
        planOnce();
        ++cycle;
      }
      feedback->cycle = cycle;
      feedback->status = "running";
      handle->publish_feedback(feedback);
      r.sleep();
    }

    result->success = true;
    result->executed_cycles = cycle;
    result->final_decision = "";
    handle->succeed(result);
  }

  std::mutex mutex_;
  std::vector<Vehicle> vehicles_;
  std::vector<Lane> lanes_;
  bool vehicles_ready_;
  bool lanes_ready_;

  PlannerCore planner_;
  Calibration calib_;
  int plan_cycle_;
  uint32_t target_lane_id_{2};
  double target_x_{200.0};

  rclcpp::Subscription<lane_change_sim::msg::VehicleArray>::SharedPtr veh_sub_;
  rclcpp::Subscription<lane_change_sim::msg::LaneArray>::SharedPtr lane_sub_;
  rclcpp::Publisher<lane_change_sim::msg::BehaviorDecision>::SharedPtr dec_pub_;
  rclcpp::Publisher<lane_change_sim::msg::Trajectory>::SharedPtr coarse_traj_pub_;
  rclcpp::Publisher<lane_change_sim::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cube_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr decision_text_pub_;
  rclcpp_action::Server<RunEpisode>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
