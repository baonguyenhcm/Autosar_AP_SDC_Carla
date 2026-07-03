// av_planning/planner_core.hpp
// ROS-free planning core.
//
// Behaviour: follow a fixed global reference path (lane centerline) at a cruise
// speed, but detect obstacles lying on/near the path ahead and generate a smooth
// speed profile that brings the ego to a stop before the nearest one
// (a minimal "adaptive cruise / emergency stop" behaviour).
#pragma once

#include "av_common/types.hpp"

namespace av {

struct PlanningParams {
  double cruise_speed{8.0};      // target speed with clear road [m/s]
  int horizon_points{40};        // number of points published ahead
  double comfort_decel{1.5};     // used to shape the stopping profile [m/s^2]
  double stop_margin{6.0};       // stop this far before the obstacle centroid [m]
  double lane_half_width{1.75};  // an object within this lateral offset blocks us [m]
};

class PlannerCore {
 public:
  explicit PlannerCore(const PlanningParams& p = {}) : params_(p) {}

  // Set the global reference path as a centerline (x,y). Yaw is derived.
  void setGlobalPath(const std::vector<Point3>& centerline);

  const std::vector<TrajectoryPoint>& globalPath() const { return path_; }

  // Produce a local trajectory (subset of the path ahead of the ego with a
  // speed profile that respects obstacles).
  Trajectory plan(const VehicleState& ego, const DetectedObjectArray& objects) const;

 private:
  int nearestIndex(double x, double y) const;

  PlanningParams params_;
  std::vector<TrajectoryPoint> path_;  // global path with cumulative arc length in yaw-less form
  std::vector<double> arc_;            // cumulative arc length at each path point
};

}  // namespace av
