// Unit tests for av::PlannerCore (lane-follow + obstacle stop).
#include <gtest/gtest.h>
#include "av_planning/planner_core.hpp"

using namespace av;

namespace {
std::vector<Point3> straightLane(double len = 200.0, double step = 1.0) {
  std::vector<Point3> c;
  for (double x = 0.0; x <= len; x += step) c.push_back({x, 0.0, 0.0});
  return c;
}
VehicleState egoAt(double x) { VehicleState s; s.x = x; s.y = 0; s.yaw = 0; s.v = 8; return s; }
DetectedObject obstacleAt(double x, double y) {
  DetectedObject o; o.centroid = {x, y, 0}; o.size_x = 2; o.size_y = 2; o.size_z = 1.5; return o;
}
}  // namespace

TEST(PlannerCore, NoGlobalPathYieldsEmptyTrajectory) {
  PlannerCore core;
  EXPECT_TRUE(core.plan(egoAt(0), {}).empty());
}

TEST(PlannerCore, ClearRoadCruisesAtTarget) {
  PlanningParams p; p.cruise_speed = 8.0; p.horizon_points = 30;
  PlannerCore core(p);
  core.setGlobalPath(straightLane());
  Trajectory t = core.plan(egoAt(0), {});
  ASSERT_GT(t.size(), 0u);
  for (const auto& pt : t) EXPECT_NEAR(pt.v, 8.0, 1e-9);
}

TEST(PlannerCore, StopsBeforeObstacleInLane) {
  PlanningParams p; p.cruise_speed = 8.0; p.horizon_points = 40; p.stop_margin = 6.0;
  PlannerCore core(p);
  core.setGlobalPath(straightLane());
  // Ego at x=40, obstacle at x=60 in-lane -> stop line at ~54.
  Trajectory t = core.plan(egoAt(40), {obstacleAt(60.0, 0.0)});
  ASSERT_GT(t.size(), 0u);
  // Speeds must be monotone-ish decreasing to zero; some point reaches ~0.
  double min_v = 1e9, v_at_stop_region = 1e9;
  for (const auto& pt : t) {
    min_v = std::min(min_v, pt.v);
    if (pt.x >= 54.0) v_at_stop_region = std::min(v_at_stop_region, pt.v);
  }
  EXPECT_LT(min_v, 0.1);              // comes to a stop
  EXPECT_LT(v_at_stop_region, 0.1);   // stop occurs at/after the stop line
  EXPECT_LT(t.front().v, 8.0 + 1e-9); // start already below/at cruise
}

TEST(PlannerCore, IgnoresObstacleOutsideLane) {
  PlanningParams p; p.cruise_speed = 8.0; p.horizon_points = 40; p.lane_half_width = 1.75;
  PlannerCore core(p);
  core.setGlobalPath(straightLane());
  Trajectory t = core.plan(egoAt(40), {obstacleAt(60.0, 6.0)});  // 6m lateral -> off lane
  for (const auto& pt : t) EXPECT_NEAR(pt.v, 8.0, 1e-9);
}

TEST(PlannerCore, IgnoresObstacleBehindEgo) {
  PlanningParams p; p.cruise_speed = 8.0; p.horizon_points = 40;
  PlannerCore core(p);
  core.setGlobalPath(straightLane());
  Trajectory t = core.plan(egoAt(40), {obstacleAt(10.0, 0.0)});  // behind
  for (const auto& pt : t) EXPECT_NEAR(pt.v, 8.0, 1e-9);
}
