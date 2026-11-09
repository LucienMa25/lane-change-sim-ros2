#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "lane_change_sim/types.hpp"

namespace lcs::control {

inline double Clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

inline const Lane* FindLaneById(const std::vector<Lane>& lanes, uint32_t lane_id) {
  for (const auto& lane : lanes) {
    if (lane.id == lane_id) {
      return &lane;
    }
  }
  return nullptr;
}

inline double LaneRefSpeedOrDefault(const std::vector<Lane>& lanes, uint32_t lane_id, double fallback) {
  const auto* lane = FindLaneById(lanes, lane_id);
  return lane ? lane->ref_speed : fallback;
}

// Preferred speed of a specific vehicle: per-vehicle desired_speed when set
// (> 0), otherwise the lane ref_speed. Traffic vehicles carry a random offset
// assigned at scene init; ego keeps 0 and follows the lane reference.
inline double VehicleDesiredSpeed(const Vehicle& vehicle,
                                  const std::vector<Lane>& lanes,
                                  double fallback) {
  return vehicle.desired_speed > 0.0
             ? vehicle.desired_speed
             : LaneRefSpeedOrDefault(lanes, vehicle.lane_id, fallback);
}

inline double InterpLaneY(double px, const Lane& lane) {
  if (lane.points.size() < 2) {
    return lane.center_y;
  }

  const auto& points = lane.points;
  if (px <= points.front().x) {
    return points.front().y;
  }
  if (px >= points.back().x) {
    return points.back().y;
  }

  auto upper = std::lower_bound(
      points.begin(), points.end(), px,
      [](const Point2d& p, double x) { return p.x < x; });
  if (upper == points.begin() || upper == points.end()) {
    return lane.center_y;
  }

  const auto& p1 = *(upper - 1);
  const auto& p2 = *upper;
  const double dx = p2.x - p1.x;
  if (std::abs(dx) < 1e-6) {
    return p1.y;
  }

  const double ratio = (px - p1.x) / dx;
  return p1.y + ratio * (p2.y - p1.y);
}

inline Vehicle FindLeadingVehicle(const Vehicle& ego,
                                  const std::vector<Vehicle>& vehicles,
                                  const std::vector<Lane>& lanes,
                                  double fallback_distance,
                                  double fallback_speed,
                                  double alpha,
                                  bool include_cutin) {
  Vehicle lead;
  lead.id = 9999;
  lead.lane_id = ego.lane_id;
  lead.state.px = ego.state.px + fallback_distance;
  lead.state.py = ego.state.py;
  lead.state.speed = LaneRefSpeedOrDefault(lanes, ego.lane_id, fallback_speed);
  lead.state.heading = 0.0;

  double best_px = std::numeric_limits<double>::max();
  for (const auto& other : vehicles) {
    if (other.id == ego.id) {
      continue;
    }
    if (other.state.px <= ego.state.px) {
      continue;
    }

    const bool same_lane = other.lane_id == ego.lane_id;
    bool candidate = same_lane;

    if (!candidate && include_cutin) {
      const bool left_cutin =
          other.lane_id > ego.lane_id && other.lat_behavior == LatBehavior::LaneChangeRight;
      const bool right_cutin =
          other.lane_id < ego.lane_id && other.lat_behavior == LatBehavior::LaneChangeLeft;
      if (left_cutin || right_cutin) {
        const double lat_v = std::abs(other.state.speed * std::sin(other.state.heading));
        const double lat_dist = std::abs(other.state.py - ego.state.py);
        if (lat_dist < 1.0 / std::max(0.2, alpha) ||
            lat_dist / std::max(0.1, lat_v) < 1.0 / std::max(0.2, alpha)) {
          candidate = true;
        }
      }
    }

    if (candidate && other.state.px < best_px) {
      best_px = other.state.px;
      lead = other;
    }
  }
  return lead;
}

}  // namespace lcs::control
