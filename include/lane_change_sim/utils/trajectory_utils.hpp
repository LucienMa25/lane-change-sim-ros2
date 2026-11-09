#pragma once

#include <cmath>
#include <limits>

#include "lane_change_sim/types.hpp"

namespace lcs::utils {

inline size_t nearestTrajectoryIndex(const Trajectory& traj, double px, double py) {
  if (traj.points.empty()) {
    return 0;
  }

  size_t nearest_idx = 0;
  double min_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < traj.points.size(); ++i) {
    const double d = std::hypot(traj.points[i].x - px,
                                traj.points[i].y - py);
    if (d < min_d) {
      min_d = d;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

/// Convert a Vehicle state to a TrajectoryPoint at time t.
inline TrajectoryPoint fromState(const Vehicle& v, double t) {
  TrajectoryPoint p;
  p.x = v.state.px;
  p.y = v.state.py;
  p.v = v.state.speed;
  p.h = v.state.heading;
  p.a = v.acc;
  p.t = t;
  return p;
}

}  // namespace lcs::utils
