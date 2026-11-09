#pragma once

#include <cmath>

#include "lane_change_sim/types.hpp"
#include "lane_change_sim/utils/math_utils.hpp"

namespace lcs::control {

inline void UpdateBicycleKinematics(Vehicle& vehicle,
                                    double ts,
                                    double max_v) {
  vehicle.state.yaw_rate =
      (vehicle.state.speed + 0.5 * vehicle.acc * ts) * std::tan(vehicle.steer) / vehicle.wheel_base;
  const double ds = vehicle.state.speed * ts + 0.5 * vehicle.acc * ts * ts;
  vehicle.state.px += ds * std::cos(vehicle.state.heading + 0.5 * vehicle.state.yaw_rate * ts);
  vehicle.state.py += ds * std::sin(vehicle.state.heading + 0.5 * vehicle.state.yaw_rate * ts);
  vehicle.state.heading = utils::normalizeAngle(vehicle.state.heading + vehicle.state.yaw_rate * ts);
  vehicle.state.speed = utils::clamp(vehicle.state.speed + vehicle.acc * ts, 0.0, max_v);
}

}  // namespace lcs::control
