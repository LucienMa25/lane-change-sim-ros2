#pragma once

#include <algorithm>
#include <cmath>

namespace lcs::utils {

inline double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

inline double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace lcs::utils
