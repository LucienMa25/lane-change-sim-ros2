#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lcs {

enum class LatBehavior : uint8_t {
  LaneKeeping = 0,
  LaneChangeLeft = 1,
  LaneChangeRight = 2,
  AvoidLeft = 3,
  AvoidRight = 4
};

enum class LonBehavior : uint8_t {
  MainTain = 0,
  Accelerate = 1,
  Decelerate = 2,
  Stopping = 3
};

enum class Intention : uint8_t {
  None = 0,
  Yield = 1,
  Aggresive = 2
};

enum class LaneChangeStage : uint8_t {
  Off = 0,
  Stage0 = 1,
  Stage1 = 2,
  Stage2 = 3,
  Aborting = 4
};

struct VehicleState {
  double px{0.0};
  double py{0.0};
  double speed{0.0};
  double heading{0.0};
  double yaw_rate{0.0};
};

struct Vehicle {
  double length{5.0};
  double width{2.0};
  double wheel_base{3.0};
  double front_suspension{0.93};
  double rear_suspension{1.10};
  double max_steering_angle{0.78539816339};
  double max_longitudinal_acc{3.0};
  double max_lateral_acc{2.0};

  std::string subclass{"car"};
  std::string type{"VehicleWithBicycleKinematics"};

  uint32_t id{0};
  uint32_t lane_id{1};

  double acc{0.0};
  double steer{0.0};

  VehicleState state;
  Intention intention{Intention::None};
  LatBehavior lat_behavior{LatBehavior::LaneKeeping};
  LonBehavior lon_behavior{LonBehavior::MainTain};
  LaneChangeStage lane_change_stage{LaneChangeStage::Off};

  // Per-vehicle preferred speed [m/s]. 0 = follow the lane ref_speed.
  // Traffic vehicles get a random offset at scene init (traffic_speed_jitter).
  double desired_speed{0.0};
};

struct Point2d {
  double x{0.0};
  double y{0.0};
};

struct Lane {
  uint32_t id{1};
  int dir{1};
  std::vector<uint32_t> child_ids;
  std::vector<uint32_t> father_ids;
  int left_lane_id{0};
  bool left_change_available{false};
  int right_lane_id{0};
  bool right_change_available{false};
  std::string behavior{"s"};
  double length{0.0};
  double center_y{0.0};
  double left_y{0.0};
  double right_y{0.0};
  double ref_speed{7.0};
  std::vector<Point2d> points;
};

struct TrajectoryPoint {
  double x{0.0};
  double y{0.0};
  double v{0.0};
  double h{0.0};
  double a{0.0};
  double t{0.0};
};

struct Trajectory {
  std::vector<TrajectoryPoint> points;
  double infeasibility{0.0};
};

struct PredictionTrajectory {
  uint32_t vehicle_id{0};
  std::vector<TrajectoryPoint> predicted_points;
  double confidence{1.0};
};

struct Calibration {
  double sim_time{9.0};
  double sim_ts{0.3};
  double ctrl_ts{0.1};
  int tree_hight{3};
  int max_search_times{0};
  bool mcts_delete_on_failure{true};

  double wheel_base{3.0};
  double max_v{10.0};
  double max_a{3.0};
  double min_a{-5.0};
  double max_j{5.0};
  double max_delta{0.5};
  double max_omega{1.0};
  double max_steer{0.2};
  double max_steer_rate{0.5};

  double effic_cost{2.0};
  double safe_cost{10.0};
  double action_cost{10.0};
  double navi_cost{1000.0};
  double affect_cost{5.0};
  double fade_factor{0.8};
  double search_new_wight{10.0};

  double rss_response_time{0.3};
  double rss_longitudinal_acc_max{2.0};
  double rss_longitudinal_brake_min{4.0};
  double rss_longitudinal_brake_max{5.0};
  double rss_lateral_miu{0.5};

  double mobil_politeness{0.3};
  double mobil_gain_threshold{0.1};
  double mobil_rear_acc_threshold{-3.0};
  double rss_cost{30.0};

  std::vector<int> cube_inflate_steps{10, 5, 5, 5};
  double cube_inflate_dx{1.0};
  double cube_inflate_dy{0.3};

  double traj_time{6.0};
  double traj_ts{0.1};
  double weight_penalty{1e5};
  double weight_x{5.0};
  double weight_y{5.0};
  double weight_theta{5.0};
  double weight_cube_center_x{2.0};
  double weight_cube_center_y{6.0};
  double weight_omega{100.0};
  double weight_jerk{1.0};

  int    ipopt_max_iter{80};
  double ipopt_tol{1e-4};
  double ipopt_acceptable_tol{1e-4};
  int    ipopt_acceptable_iter{10};
  std::string ipopt_mu_strategy{"monotone"};
  bool   warm_start_enable{false};

  double pred_horizon{6.0};
  bool pred_enable_imm{false};
  double pred_process_noise{0.5};

  // ── Adaptive weights ──
  bool adapt_weights_enable{true};
  double adapt_front_dist_near{10.0};
  double adapt_front_dist_far{50.0};
  double adapt_front_rel_v_threshold{1.0};

  // ── Lane change gap tracking (inspired by EPSILON PropagateOnceAdvancedLC) ──
  bool lc_gap_tracking_enable{true};       // Enable gap-aware longitudinal control during LC
  double lc_gap_headway_time{1.0};         // Time headway for gap safe distance [s]
  double lc_gap_min_dist{2.0};             // Minimum static gap distance [m]
  double lc_ctx_weight_cur{0.4};           // CtxIDM weight for current-lane leading vehicle
  double lc_ctx_weight_tar{0.8};           // CtxIDM weight for target-gap tracking
  double lc_gap_kp{0.1};                   // P-gain for gap position error → velocity correction
  double lc_gap_dv_lb{-3.0};              // Lower bound for dv correction [m/s]
  double lc_gap_dv_ub{5.0};               // Upper bound for dv correction [m/s]
};

struct DriveCube {
  struct Bound {
    double x{0.0};
    double y{0.0};
    double t{0.0};
  };
  Bound lb;
  Bound ub;
};

struct SearchResult {
  std::string decision_type{"Keep-MainTain"};
  LatBehavior lat_behavior{LatBehavior::LaneKeeping};
  LonBehavior lon_behavior{LonBehavior::MainTain};
  double value{0.0};
  double score{0.0};
  Trajectory coarse_ego;
  Trajectory optimized_ego;
  std::vector<Trajectory> coarse_agents;
};

}  // namespace lcs
