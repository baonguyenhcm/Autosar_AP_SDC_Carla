// av_common/math.hpp
// Small math helpers shared across modules.
#pragma once

#include <cmath>
#include <algorithm>
#include "av_common/types.hpp"

namespace av {

constexpr double kPi = 3.14159265358979323846;

// Wrap an angle to (-pi, pi].
inline double normalizeAngle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a <= -kPi) a += 2.0 * kPi;
  return a;
}

inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

inline double dist2D(double ax, double ay, double bx, double by) {
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

inline double dist2D(const Point3& a, const Point3& b) {
  return dist2D(a.x, a.y, b.x, b.y);
}

}  // namespace av
