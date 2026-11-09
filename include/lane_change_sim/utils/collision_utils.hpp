#pragma once

#include "lane_change_sim/types.hpp"
#include "lane_change_sim/utils/box2d.hpp"

namespace lcs::utils {

inline Box2d makeBox(double x, double y, double heading,
                     double length, double width,
                     double inflate = 1.0) {
  return Box2d(x, y, heading, length * inflate, width * inflate);
}

inline Box2d makeVehicleBox(const VehicleState& state, const Vehicle& vehicle,
                            double inflate = 1.0) {
  return makeBox(state.px, state.py, state.heading,
                 vehicle.length, vehicle.width, inflate);
}

inline bool hasVehicleOverlap(const VehicleState& a, const Vehicle& va,
                              const VehicleState& b, const Vehicle& vb,
                              double inflate = 1.0) {
  return makeVehicleBox(a, va, inflate).hasOverlap(makeVehicleBox(b, vb, inflate));
}

}  // namespace lcs::utils
