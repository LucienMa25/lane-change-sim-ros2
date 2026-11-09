#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "lane_change_sim/types.hpp"

namespace lcs::utils {

inline uint32_t nearestLaneId(double py, const std::vector<Lane>& lanes) {
  if (lanes.empty()) {
    return 1;
  }

  double min_delta = std::numeric_limits<double>::max();
  uint32_t id = lanes.front().id;
  for (const auto& lane : lanes) {
    const double dy = std::abs(py - lane.center_y);
    if (dy < min_delta) {
      min_delta = dy;
      id = lane.id;
    }
  }

  return id;
}

inline void updateLaneId(Vehicle& vehicle, const std::vector<Lane>& lanes) {
  if (lanes.empty()) {
    return;
  }
  const uint32_t id = nearestLaneId(vehicle.state.py, lanes);
  vehicle.lane_id = id;
}

}  // namespace lcs::utils
