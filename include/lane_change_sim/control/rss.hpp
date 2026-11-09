#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "lane_change_sim/types.hpp"

namespace lcs::control {

inline bool FindNearestFrontRearInLane(const Vehicle& ego,
                                       const std::vector<Vehicle>& vehicles,
                                       uint32_t lane_id,
                                       const Vehicle** front,
                                       const Vehicle** rear) {
  const Vehicle* best_front = nullptr;
  const Vehicle* best_rear = nullptr;
  double min_front_dx = std::numeric_limits<double>::max();
  double min_rear_dx = std::numeric_limits<double>::max();

  for (const auto& other : vehicles) {
    if (other.id == ego.id || other.lane_id != lane_id) {
      continue;
    }
    const double dx = other.state.px - ego.state.px;
    if (dx > 0.0 && dx < min_front_dx) {
      min_front_dx = dx;
      best_front = &other;
    } else if (dx < 0.0 && -dx < min_rear_dx) {
      min_rear_dx = -dx;
      best_rear = &other;
    }
  }

  *front = best_front;
  *rear = best_rear;
  return best_front || best_rear;
}

inline double CalcRssSafeDistance(double rear_speed,
                                  double front_speed,
                                  const Calibration& calibration) {
  const double t = std::max(0.0, calibration.rss_response_time);
  const double a = std::max(0.1, calibration.rss_longitudinal_acc_max);
  const double b_rear = std::max(0.1, calibration.rss_longitudinal_brake_min);
  const double b_front = std::max(0.1, calibration.rss_longitudinal_brake_max);

  const double rear_after_reaction = rear_speed * t + 0.5 * a * t * t;
  const double rear_brake_dist = std::pow(rear_speed + a * t, 2) / (2.0 * b_rear);
  const double front_brake_dist = std::pow(front_speed, 2) / (2.0 * b_front);
  return std::max(0.0, rear_after_reaction + rear_brake_dist - front_brake_dist);
}

/// Check if lane change to target_lane_id is RSS-safe.
/// Returns true if both front and rear gaps satisfy RSS safe distance.
inline bool CheckLaneChangeRssSafe(const Vehicle& ego,
                                   const std::vector<Vehicle>& vehicles,
                                   uint32_t target_lane_id,
                                   const Calibration& calibration) {
  const Vehicle* front = nullptr;
  const Vehicle* rear = nullptr;
  FindNearestFrontRearInLane(ego, vehicles, target_lane_id, &front, &rear);

  if (front) {
    const double actual_gap = front->state.px - ego.state.px - 0.5 * (ego.length + front->length);
    const double safe_gap = CalcRssSafeDistance(ego.state.speed, front->state.speed, calibration);
    if (actual_gap < safe_gap) {
      return false;
    }
  }

  if (rear) {
    const double actual_gap = ego.state.px - rear->state.px - 0.5 * (ego.length + rear->length);
    const double safe_gap = CalcRssSafeDistance(rear->state.speed, ego.state.speed, calibration);
    if (actual_gap < safe_gap) {
      return false;
    }
  }

  return true;
}

}  // namespace lcs::control
