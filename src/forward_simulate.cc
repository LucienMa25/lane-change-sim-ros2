/**
 * @file forward_simulate.cc
 * @brief Forward simulation functions for MCTS behavior planning.
 *
 * Contains propagateOnce, propagateOnceLC, and forwardSimulate implementations.
 * Separated from behavior_planner.cpp for better code organization.
 */

#include "lane_change_sim/behavior_planner.hpp"
#include "lane_change_sim/control/common.hpp"
#include "lane_change_sim/control/idm_controller.hpp"
#include "lane_change_sim/control/kinematics.hpp"
#include "lane_change_sim/control/lat_controller.hpp"
#include "lane_change_sim/control/rss.hpp"
#include "lane_change_sim/utils/collision_utils.hpp"
#include "lane_change_sim/utils/lane_utils.hpp"
#include "lane_change_sim/utils/math_utils.hpp"
#include "lane_change_sim/utils/trajectory_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lcs {

void BehaviorPlanner::propagateOnce(Vehicle& vehicle, const Vehicle& leading,
                                    const std::vector<Lane>& lanes) const {
  const double desired_speed = control::VehicleDesiredSpeed(
      vehicle, lanes, calibration_.max_v);
  vehicle.acc = control::ComputeIidmAcceleration(
      vehicle, leading.state, desired_speed, calibration_.min_a, calibration_.max_a);
  const double desired_steer = control::LatControl(vehicle, lanes, calibration_.max_steer);
  const double max_step = calibration_.max_steer_rate * calibration_.sim_ts;
  vehicle.steer = utils::clamp(desired_steer, vehicle.steer - max_step, vehicle.steer + max_step);
  control::UpdateBicycleKinematics(vehicle, calibration_.sim_ts, calibration_.max_v);
}

void BehaviorPlanner::propagateOnceLC(Vehicle& vehicle,
                                      const Vehicle& current_leading,
                                      const Vehicle* gap_front,
                                      const Vehicle* gap_rear,
                                      uint32_t target_lane_id,
                                      const std::vector<Lane>& lanes) const {
  // ── Step I: Lateral control (steering towards target lane) ──
  const double desired_steer = control::LatControl(vehicle, lanes, calibration_.max_steer);
  const double max_step = calibration_.max_steer_rate * calibration_.sim_ts;
  vehicle.steer = utils::clamp(desired_steer, vehicle.steer - max_step, vehicle.steer + max_step);

  // ── Step II: Longitudinal control with gap tracking ──
  const double desired_speed = control::VehicleDesiredSpeed(
      vehicle, lanes, calibration_.max_v);
  const double target_desired_speed = control::LaneRefSpeedOrDefault(lanes, target_lane_id, calibration_.max_v);

  // Current-lane IDM acceleration (baseline)
  const double acc_cur = control::ComputeIidmAcceleration(
      vehicle, current_leading.state, desired_speed, calibration_.min_a, calibration_.max_a);

  if (!calibration_.lc_gap_tracking_enable || (!gap_front && !gap_rear)) {
    // No gap tracking: fall back to normal IDM
    vehicle.acc = acc_cur;
    control::UpdateBicycleKinematics(vehicle, calibration_.sim_ts, calibration_.max_v);
    return;
  }

  // ── Compute gap target state (inspired by EPSILON GetTargetStateOnTargetLane) ──
  const double ego_v = std::max(0.0, vehicle.state.speed);
  const double headway = calibration_.lc_gap_headway_time;
  const double min_gap = calibration_.lc_gap_min_dist;
  const double kp = calibration_.lc_gap_kp;
  const double dv_lb = calibration_.lc_gap_dv_lb;
  const double dv_ub = calibration_.lc_gap_dv_ub;

  // Ego's preferred velocity (blend between current and target lane desired speed)
  const double ego_desired_vel = ego_v + (target_desired_speed - ego_v) * 0.3;

  double desired_s = vehicle.state.px;
  double desired_v = ego_desired_vel;

  // Compute safe distance thresholds for front and rear
  double s_thres_front = std::numeric_limits<double>::max();
  double s_ref_front = std::numeric_limits<double>::max();
  bool has_front = (gap_front != nullptr);

  double s_thres_rear = -std::numeric_limits<double>::max();
  double s_ref_rear = -std::numeric_limits<double>::max();
  bool has_rear = (gap_rear != nullptr);

  if (has_front) {
    // Tail of front vehicle (approximate rear bumper position)
    s_ref_front = gap_front->state.px - 0.5 * gap_front->length;
    // Safe threshold: don't get closer than min_gap + headway * ego_v
    s_thres_front = s_ref_front - min_gap - headway * ego_v;
  }

  if (has_rear) {
    // Head of rear vehicle (approximate front bumper position)
    s_ref_rear = gap_rear->state.px + 0.5 * gap_rear->length;
    // Safe threshold: don't let rear get closer than min_gap + headway * rear_v
    s_thres_rear = s_ref_rear + min_gap + headway * gap_rear->state.speed;
  }

  if (has_front && has_rear) {
    // Both front and rear: check if gap is valid
    if (s_ref_front < s_ref_rear) {
      // Gap is too small, fall back to current lane IDM
      vehicle.acc = acc_cur;
      control::UpdateBicycleKinematics(vehicle, calibration_.sim_ts, calibration_.max_v);
      return;
    }

    // Target center of gap
    const double gap_center = (s_ref_front + s_ref_rear) * 0.5;
    s_thres_front = std::max(gap_center, s_thres_front);
    s_thres_rear = std::min(gap_center, s_thres_rear);

    // Clamp desired position within safe corridor
    desired_s = utils::clamp(vehicle.state.px, s_thres_rear, s_thres_front);

    // Compute velocity references from front and rear
    const double s_err_front = s_thres_front - vehicle.state.px;
    const double v_ref_front = std::max(0.0,
        gap_front->state.speed + utils::clamp(s_err_front * kp, dv_lb, dv_ub));

    const double s_err_rear = s_thres_rear - vehicle.state.px;
    const double v_ref_rear = std::max(0.0,
        gap_rear->state.speed + utils::clamp(s_err_rear * kp, dv_lb, dv_ub));

    // Desired velocity: bounded by front and rear references
    desired_v = utils::clamp(ego_desired_vel, v_ref_rear, v_ref_front);

  } else if (has_front) {
    // Only front: don't exceed safe distance behind front
    desired_s = std::min(vehicle.state.px, s_thres_front);
    const double s_err_front = s_thres_front - vehicle.state.px;
    const double v_ref_front = std::max(0.0,
        gap_front->state.speed + utils::clamp(s_err_front * kp, dv_lb, dv_ub));
    desired_v = std::min(ego_desired_vel, v_ref_front);

  } else if (has_rear) {
    // Only rear: don't let rear catch up
    desired_s = std::max(vehicle.state.px, s_thres_rear);
    const double s_err_rear = s_thres_rear - vehicle.state.px;
    const double v_ref_rear = std::max(0.0,
        gap_rear->state.speed + utils::clamp(s_err_rear * kp, dv_lb, dv_ub));
    desired_v = std::max(ego_desired_vel, v_ref_rear);
  }

  // ── Step III: Compute gap-tracking acceleration using IDM-like control ──
  // Create a virtual leading state at the desired gap position/velocity
  VehicleState gap_target_state;
  gap_target_state.px = desired_s;
  gap_target_state.speed = std::max(0.0, desired_v);

  const double acc_gap = control::ComputeIidmAcceleration(
      vehicle, gap_target_state, target_desired_speed, calibration_.min_a, calibration_.max_a);

  // ── Step IV: Compute lateral progress and blend accelerations ──
  // alpha increases as ego moves laterally towards target lane
  const Lane* cur_lane = control::FindLaneById(lanes, vehicle.lane_id);
  const double cur_lane_y = cur_lane ? cur_lane->center_y : vehicle.state.py;
  const Lane* tar_lane = control::FindLaneById(lanes, target_lane_id);
  const double tar_lane_y = tar_lane ? tar_lane->center_y : cur_lane_y;
  const double lane_width = std::abs(tar_lane_y - cur_lane_y);

  double alpha = 0.0;
  if (lane_width > 0.1) {
    alpha = std::abs(vehicle.state.py - cur_lane_y) / lane_width;
    alpha = utils::clamp(alpha, 0.0, 1.0);
  }

  // Blend: more weight on gap tracking as lateral progress increases
  // Use EPSILON-style context weights
  const double w_cur = calibration_.lc_ctx_weight_cur * (1.0 - alpha);
  const double w_tar = calibration_.lc_ctx_weight_tar * alpha;
  const double w_sum = w_cur + w_tar;

  double acc_final;
  if (w_sum > 1e-6) {
    acc_final = (w_cur * acc_cur + w_tar * acc_gap) / w_sum;
  } else {
    acc_final = acc_cur;
  }

  vehicle.acc = utils::clamp(acc_final, calibration_.min_a, calibration_.max_a);
  control::UpdateBicycleKinematics(vehicle, calibration_.sim_ts, calibration_.max_v);
}

BehaviorPlanner::SimResult BehaviorPlanner::forwardSimulate(const std::vector<Vehicle>& vehicles,
                                                            LatBehavior lat_behavior,
                                                            LonBehavior lon_behavior,
                                                            const std::vector<Lane>& lanes,
                                                            int layer) {
  SimResult result;
  result.end_vehicles = vehicles;
  result.ego.points.clear();
  result.agents.assign(vehicles.size() - 1, Trajectory{});

  state_mgr_for_sim_.reset();

  const double sim_time = calibration_.sim_time / calibration_.tree_hight;
  const int total_steps = static_cast<int>(sim_time / calibration_.sim_ts);

  for (int step = 1; step <= total_steps; ++step) {
    for (size_t i = 0; i < result.end_vehicles.size(); ++i) {
      auto& veh = result.end_vehicles[i];
      Vehicle leading = control::FindLeadingVehicle(
          veh, result.end_vehicles, lanes, 20.0, calibration_.max_v, 1.0, true);

      if (veh.id == 0) {
        state_mgr_for_sim_.update(veh, lat_behavior, lon_behavior, lanes, layer, step);
      }

      // Use gap-aware propagation for ego during lane change (Stage0/Stage1)
      if (veh.id == 0 && calibration_.lc_gap_tracking_enable &&
          (veh.lane_change_stage == LaneChangeStage::Stage0 ||
           veh.lane_change_stage == LaneChangeStage::Stage1)) {
        // Determine target lane ID from lat_behavior
        uint32_t target_lane_id = veh.lane_id;
        const Lane* cur_lane = control::FindLaneById(lanes, veh.lane_id);
        if (veh.lat_behavior == LatBehavior::LaneChangeLeft && cur_lane && cur_lane->left_lane_id > 0) {
          target_lane_id = static_cast<uint32_t>(cur_lane->left_lane_id);
        } else if (veh.lat_behavior == LatBehavior::LaneChangeRight && cur_lane && cur_lane->right_lane_id > 0) {
          target_lane_id = static_cast<uint32_t>(cur_lane->right_lane_id);
        }

        // Find gap front/rear vehicles in target lane
        const Vehicle* gap_front = nullptr;
        const Vehicle* gap_rear = nullptr;
        control::FindNearestFrontRearInLane(veh, result.end_vehicles, target_lane_id, &gap_front, &gap_rear);

        propagateOnceLC(veh, leading, gap_front, gap_rear, target_lane_id, lanes);
      } else {
        propagateOnce(veh, leading, lanes);
      }

      utils::updateLaneId(veh, lanes);

      const double t = (layer - 1) * sim_time + calibration_.sim_ts * step;
      if (veh.id == 0) {
        result.ego.points.push_back(utils::fromState(veh, t));
      } else {
        result.agents[i - 1].points.push_back(utils::fromState(veh, t));
      }
    }

    for (size_t k = 1; k < result.end_vehicles.size(); ++k) {
      const double inflate = std::min(1.3, std::max(1.0, 0.7 + 0.03 * result.end_vehicles[0].state.speed));
      if (utils::hasVehicleOverlap(result.end_vehicles[0].state, result.end_vehicles[0],
                       result.end_vehicles[k].state, result.end_vehicles[k], inflate)) {
        result.success = false;
        return result;
      }
    }
  }

  result.success = true;
  return result;
}

}  // namespace lcs
