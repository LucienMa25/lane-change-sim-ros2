#pragma once

#include <algorithm>
#include <cmath>

#include "lane_change_sim/control/common.hpp"
#include "lane_change_sim/types.hpp"

namespace lcs::control {

inline double ComputeIidmAcceleration(const Vehicle& ego,
                                      const VehicleState& leading_state,
                                      double desired_speed,
                                      double min_a,
                                      double max_a) {
  desired_speed = std::max(0.1, desired_speed);

  double min_gap = 1.5;
  double headway_time = 1.0;
  const double accel_max = std::max(0.5, std::min(ego.max_longitudinal_acc, max_a));
  const double decel_max = std::max(1.0, std::abs(min_a));
  double floor_acc = -100.0;
  double ceil_acc = 100.0;

  if (ego.lon_behavior == LonBehavior::Accelerate) {
    min_gap = 1.0;
    headway_time = 0.5;
    floor_acc = 0.2;
  } else if (ego.lon_behavior == LonBehavior::Decelerate) {
    min_gap = 2.0;
    headway_time = 1.5;
    ceil_acc = -0.2;
  } else if (ego.lon_behavior == LonBehavior::Stopping) {
    min_gap = 5.0;
    headway_time = 5.0;
    desired_speed = std::min(desired_speed, 0.5);
  }

  const double v = std::max(0.0, ego.state.speed);
  const double v_front = std::max(0.0, leading_state.speed);
  const double dv = v - v_front;
  const double s_alpha = std::max(0.1, leading_state.px - ego.state.px - ego.length);
  const double s_star = min_gap + std::max(
      0.0, v * headway_time + v * dv / (2.0 * std::sqrt(std::max(0.1, accel_max * decel_max))));

  const double exponent = 4.0;
  const double a_free = (v <= desired_speed)
                            ? accel_max * (1.0 - std::pow(v / desired_speed, exponent))
                            : -decel_max * (1.0 - std::pow(desired_speed / std::max(0.1, v),
                                                           accel_max * exponent / decel_max));
  const double z = s_star / s_alpha;

  double desired = 0.0;
  if (z >= 1.0) {
    desired = accel_max * (1.0 - z * z);
  } else {
    const double a_abs = std::max(0.1, std::abs(a_free));
    desired = a_free * (1.0 - std::pow(z, 2.0 * accel_max / a_abs));
  }

  desired = Clamp(desired, -decel_max, accel_max);
  desired = std::max(desired, floor_acc);
  desired = std::min(desired, ceil_acc);
  return Clamp(desired, min_a, max_a);
}

}  // namespace lcs::control
