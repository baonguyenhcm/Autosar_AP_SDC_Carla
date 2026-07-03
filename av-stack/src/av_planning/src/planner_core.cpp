// av_planning/planner_core.cpp
#include "av_planning/planner_core.hpp"
#include "av_common/math.hpp"

#include <cmath>
#include <limits>

namespace av {

void PlannerCore::setGlobalPath(const std::vector<Point3>& centerline) {
  path_.clear();
  arc_.clear();
  if (centerline.empty()) return;

  double s = 0.0;
  for (size_t i = 0; i < centerline.size(); ++i) {
    TrajectoryPoint tp;
    tp.x = centerline[i].x;
    tp.y = centerline[i].y;
    tp.v = params_.cruise_speed;
    // Yaw from forward difference (last point copies previous heading).
    if (i + 1 < centerline.size()) {
      tp.yaw = std::atan2(centerline[i + 1].y - centerline[i].y,
                          centerline[i + 1].x - centerline[i].x);
    } else if (!path_.empty()) {
      tp.yaw = path_.back().yaw;
    }
    if (i > 0) {
      s += dist2D(centerline[i].x, centerline[i].y, centerline[i - 1].x,
                  centerline[i - 1].y);
    }
    path_.push_back(tp);
    arc_.push_back(s);
  }
}

int PlannerCore::nearestIndex(double x, double y) const {
  int best = 0;
  double best_d = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(path_.size()); ++i) {
    const double d = dist2D(x, y, path_[i].x, path_[i].y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

Trajectory PlannerCore::plan(const VehicleState& ego,
                             const DetectedObjectArray& objects) const {
  Trajectory out;
  if (path_.empty()) return out;

  const int start = nearestIndex(ego.x, ego.y);
  const int end =
      std::min<int>(path_.size(), start + params_.horizon_points);
  const double s_ego = arc_[start];

  // --- Find the nearest blocking obstacle along the path ahead. ---
  // For each object, project it onto the path: nearest path index gives its
  // arc length; lateral distance to that point tells us if it is in-lane.
  double stop_s = std::numeric_limits<double>::max();  // arc length of stop line
  for (const auto& obj : objects) {
    int oi = nearestIndex(obj.centroid.x, obj.centroid.y);
    const double lateral =
        dist2D(obj.centroid.x, obj.centroid.y, path_[oi].x, path_[oi].y);
    if (lateral > params_.lane_half_width + 0.5 * obj.size_y) continue;  // not in lane
    if (arc_[oi] <= s_ego) continue;                                     // behind us
    stop_s = std::min(stop_s, arc_[oi] - params_.stop_margin);
  }

  // --- Build the local trajectory with a speed profile. ---
  for (int i = start; i < end; ++i) {
    TrajectoryPoint tp = path_[i];
    if (stop_s < std::numeric_limits<double>::max()) {
      const double dist_to_stop = stop_s - arc_[i];
      if (dist_to_stop <= 0.0) {
        tp.v = 0.0;  // at/after the stop line
      } else {
        // v = sqrt(2 a d) capped at cruise -> smooth decel to 0 at the stop line.
        const double v_lim =
            std::sqrt(2.0 * params_.comfort_decel * dist_to_stop);
        tp.v = std::min(params_.cruise_speed, v_lim);
      }
    }
    out.push_back(tp);
  }
  return out;
}

}  // namespace av
