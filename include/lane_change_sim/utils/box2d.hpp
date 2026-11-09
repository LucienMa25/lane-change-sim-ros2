#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace lcs::utils {

struct Vec2 {
  double x{0.0};
  double y{0.0};
};

class Box2d {
 public:
  Box2d() = default;
  Box2d(double center_x, double center_y, double heading,
        double length, double width)
      : center_{center_x, center_y},
        heading_(heading),
        length_(length),
        width_(width) {
    updateCorners();
  }

  const Vec2& center() const { return center_; }
  double heading() const { return heading_; }
  double length() const { return length_; }
  double width() const { return width_; }

  const std::array<Vec2, 4>& corners() const { return corners_; }

  bool hasOverlap(const Box2d& other) const {
    const auto& a = corners_;
    const auto& b = other.corners_;

    std::array<Vec2, 4> axes{};
    axes[0] = {a[1].x - a[0].x, a[1].y - a[0].y};
    axes[1] = {a[3].x - a[0].x, a[3].y - a[0].y};
    axes[2] = {b[1].x - b[0].x, b[1].y - b[0].y};
    axes[3] = {b[3].x - b[0].x, b[3].y - b[0].y};

    for (const auto& e : axes) {
      const Vec2 axis{-e.y, e.x};
      if (!overlapOnAxis(a, b, axis)) {
        return false;
      }
    }
    return true;
  }

 private:
  void updateCorners() {
    const double hl = 0.5 * length_;
    const double hw = 0.5 * width_;

    const double c = std::cos(heading_);
    const double s = std::sin(heading_);

    const Vec2 ex{c * hl, s * hl};
    const Vec2 ey{-s * hw, c * hw};

    corners_[0] = {center_.x + ex.x + ey.x, center_.y + ex.y + ey.y};
    corners_[1] = {center_.x + ex.x - ey.x, center_.y + ex.y - ey.y};
    corners_[2] = {center_.x - ex.x - ey.x, center_.y - ex.y - ey.y};
    corners_[3] = {center_.x - ex.x + ey.x, center_.y - ex.y + ey.y};
  }

  static bool overlapOnAxis(const std::array<Vec2, 4>& a,
                            const std::array<Vec2, 4>& b,
                            const Vec2& axis) {
    const double norm = std::hypot(axis.x, axis.y);
    if (norm < 1e-9) {
      return true;
    }
    const Vec2 n{axis.x / norm, axis.y / norm};

    auto project = [&n](const std::array<Vec2, 4>& poly, double* min_p, double* max_p) {
      *min_p = *max_p = poly[0].x * n.x + poly[0].y * n.y;
      for (size_t i = 1; i < poly.size(); ++i) {
        const double p = poly[i].x * n.x + poly[i].y * n.y;
        *min_p = std::min(*min_p, p);
        *max_p = std::max(*max_p, p);
      }
    };

    double a_min, a_max, b_min, b_max;
    project(a, &a_min, &a_max);
    project(b, &b_min, &b_max);
    return !(a_max < b_min || b_max < a_min);
  }

  Vec2 center_;
  double heading_{0.0};
  double length_{0.0};
  double width_{0.0};
  std::array<Vec2, 4> corners_{};
};

}  // namespace lcs::utils
