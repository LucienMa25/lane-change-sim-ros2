#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "lane_change_sim/control/common.hpp"
#include "lane_change_sim/types.hpp"
#include "lane_change_sim/utils/math_utils.hpp"
#include "lane_change_sim/utils/trajectory_utils.hpp"

namespace lcs::control {

inline double PurePursuitSteer(const Vehicle& ego,
                               double target_y,
                               double max_steer) {
  const double lookahead = 5.0 + ego.state.speed;
  const double lat_offset = target_y - ego.state.py;
  const double angle_diff = std::atan2(lat_offset, lookahead) - ego.state.heading;
  const double desired = std::atan2(2.0 * ego.wheel_base * std::sin(angle_diff), lookahead);
  return utils::clamp(desired, -max_steer, max_steer);
}

inline double PurePursuitSteerOnLane(const Vehicle& ego,
                                     const Lane& lane,
                                     double max_steer) {
  const double lookahead = 5.0 + ego.state.speed;
  const double target_y = InterpLaneY(ego.state.px + lookahead, lane);
  return PurePursuitSteer(ego, target_y, max_steer);
}

inline double CurvatureTrackingSteer(const Vehicle& ego,
                                     const Trajectory& traj,
                                     double max_steer,
                                     double k_lat = 0.9,
                                     double k_heading = 0.85,
                                     double preview_base = 5.0,
                                     double preview_speed_gain = 0.8) {
  if (traj.points.empty()) {
    return 0.0;
  }

  const size_t nearest_idx = utils::nearestTrajectoryIndex(traj, ego.state.px, ego.state.py);

  const double preview_dis = preview_base + preview_speed_gain * ego.state.speed;
  size_t idx = nearest_idx;
  for (size_t i = nearest_idx; i < traj.points.size(); ++i) {
    const double d = std::hypot(traj.points[i].x - ego.state.px,
                                traj.points[i].y - ego.state.py);
    if (d >= preview_dis) {
      idx = i;
      break;
    }
  }

  const auto& target = traj.points[idx];
  const double dx = target.x - ego.state.px;
  const double dy = target.y - ego.state.py;

  const double e_lat = -std::sin(ego.state.heading) * dx + std::cos(ego.state.heading) * dy;
  const double e_heading = utils::normalizeAngle(target.h - ego.state.heading);

  const size_t i0 = (idx == 0) ? 0 : idx - 1;
  const size_t i1 = std::min(idx + 1, traj.points.size() - 1);
  const double ds = std::max(0.2, std::hypot(traj.points[i1].x - traj.points[i0].x,
                                             traj.points[i1].y - traj.points[i0].y));
  const double dtheta = utils::normalizeAngle(traj.points[i1].h - traj.points[i0].h);
  const double kappa_ref = dtheta / ds;

  const double delta_ff = std::atan(ego.wheel_base * kappa_ref);
  const double delta_fb = std::atan2(k_lat * e_lat, 1.0 + ego.state.speed) + k_heading * e_heading;
  const double delta_cmd = delta_ff + delta_fb;

  return utils::clamp(delta_cmd, -max_steer, max_steer);
}

inline double LatControl(const Vehicle& ego, const std::vector<Lane>& lanes,
                         double max_steer) {
  if (lanes.empty()) return 0.0;

  const Lane* current_lane = FindLaneById(lanes, ego.lane_id);
  Lane target_lane = current_lane ? *current_lane : lanes.front();

  if (ego.lat_behavior == LatBehavior::LaneKeeping) {
    double best = std::numeric_limits<double>::max();
    for (const auto& l : lanes) {
      const double d = std::abs(l.center_y - ego.state.py);
      if (d < best) { best = d; target_lane = l; }
    }
  } else if (ego.lat_behavior == LatBehavior::LaneChangeLeft) {
    if (ego.lane_change_stage == LaneChangeStage::Stage0) {
      target_lane.center_y = target_lane.left_y - 1.0;
    } else if (ego.lane_change_stage == LaneChangeStage::Stage1) {
      if (current_lane && current_lane->left_lane_id > 0) {
        const Lane* next_lane = FindLaneById(lanes, static_cast<uint32_t>(current_lane->left_lane_id));
        if (next_lane) target_lane = *next_lane;
      }
    }
  } else if (ego.lat_behavior == LatBehavior::LaneChangeRight) {
    if (ego.lane_change_stage == LaneChangeStage::Stage0) {
      target_lane.center_y = target_lane.right_y + 1.0;
    } else if (ego.lane_change_stage == LaneChangeStage::Stage1) {
      if (current_lane && current_lane->right_lane_id > 0) {
        const Lane* next_lane = FindLaneById(lanes, static_cast<uint32_t>(current_lane->right_lane_id));
        if (next_lane) target_lane = *next_lane;
      }
    }
  } else if (ego.lat_behavior == LatBehavior::AvoidLeft) {
    target_lane.center_y = target_lane.center_y - 1.0;
  } else if (ego.lat_behavior == LatBehavior::AvoidRight) {
    target_lane.center_y = target_lane.center_y + 1.0;
  }

  return PurePursuitSteerOnLane(ego, target_lane, max_steer);
}

}  // namespace lcs::control
