// Integration test: exercises the full Perception -> Localization -> Planning ->
// Control loop against an inline kinematic simulator (no ROS, no middleware).
// This validates that the four cores cooperate correctly end-to-end.
#include <gtest/gtest.h>
#include <cmath>
#include <random>

#include "av_common/types.hpp"
#include "av_common/math.hpp"
#include "av_perception/perception_core.hpp"
#include "av_localization/ekf_localizer.hpp"
#include "av_planning/planner_core.hpp"
#include "av_control/controller_core.hpp"

using namespace av;

namespace {

struct TrueEgo { double x{0}, y{0}, yaw{0}, v{0}; };

double stepVehicle(TrueEgo& e, const ControlCommand& cmd, double dt) {
  const double L = 2.7, a_max = 2.0, d_max = 4.0;
  const double accel = cmd.throttle * a_max - cmd.brake * d_max;
  e.v = std::max(0.0, e.v + accel * dt);
  const double yaw_rate = (e.v / L) * std::tan(cmd.steer);
  e.yaw = normalizeAngle(e.yaw + yaw_rate * dt);
  e.x += e.v * std::cos(e.yaw) * dt;
  e.y += e.v * std::sin(e.yaw) * dt;
  return yaw_rate;
}

PointCloud lidar(const TrueEgo& e, bool has_obstacle, const Point3& obst) {
  PointCloud c;
  if (!has_obstacle) return c;
  const double cs = std::cos(-e.yaw), sn = std::sin(-e.yaw);
  for (double ox = -1; ox <= 1; ox += 0.25)
    for (double oy = -1; oy <= 1; oy += 0.25)
      for (double oz = -1; oz <= 0.5; oz += 0.5) {
        const double rx = (obst.x + ox) - e.x, ry = (obst.y + oy) - e.y;
        c.push_back({rx * cs - ry * sn, rx * sn + ry * cs, oz});
      }
  return c;
}

struct Result {
  TrueEgo ego;
  bool detected_ever{false};
  double max_pos_err{0.0};
  bool any_nan{false};
};

// Run one closed-loop scenario for t_end seconds.
Result runScenario(bool has_obstacle, double obstacle_x, double t_end = 45.0) {
  std::vector<Point3> centerline;
  for (double x = 0; x <= 220; x += 1.0) centerline.push_back({x, 0, 0});
  const Point3 obst{obstacle_x, 0.0, 0.0};

  PerceptionCore perception;
  EkfLocalizer ekf;
  PlannerCore planner; planner.setGlobalPath(centerline);
  ControllerCore controller;

  TrueEgo ego;
  std::mt19937 rng(7);
  std::normal_distribution<double> gps_n(0.0, 0.3), spd_n(0.0, 0.1), yaw_n(0.0, 0.02);

  ekf.initialize(0, 0, 0, 0, 0.0);
  const double dt = 0.1;
  double last_gps = -1.0, imu_yaw_rate = 0.0;
  Result r;

  for (double t = 0; t <= t_end; t += dt) {
    auto cloud = lidar(ego, has_obstacle, obst);
    auto objs = perception.detect(cloud, {ego.x, ego.y, ego.yaw});
    if (!objs.empty()) r.detected_ever = true;

    ekf.predict(dt, imu_yaw_rate);
    ekf.updateSpeed(ego.v + spd_n(rng));
    ekf.updateYaw(normalizeAngle(ego.yaw + yaw_n(rng)));
    if (t - last_gps >= 0.5) {
      GpsMeasurement g; g.stamp = t; g.x = ego.x + gps_n(rng); g.y = ego.y + gps_n(rng);
      ekf.updateGps(g); last_gps = t;
    }
    ekf.setStamp(t);
    VehicleState est = ekf.state();

    if (std::isnan(est.x) || std::isnan(est.y) || std::isnan(est.yaw)) r.any_nan = true;
    r.max_pos_err = std::max(r.max_pos_err, std::hypot(est.x - ego.x, est.y - ego.y));

    auto traj = planner.plan(est, objs);
    auto cmd = controller.computeCommand(est, traj, dt);
    if (std::isnan(cmd.steer) || std::isnan(cmd.throttle)) r.any_nan = true;

    imu_yaw_rate = stepVehicle(ego, cmd, dt);
  }
  r.ego = ego;
  return r;
}

}  // namespace

TEST(Integration, StopsSafelyBeforeObstacle) {
  Result r = runScenario(/*has_obstacle=*/true, /*obstacle_x=*/100.0);
  EXPECT_TRUE(r.detected_ever) << "perception never saw the obstacle";
  EXPECT_FALSE(r.any_nan) << "NaN propagated through the pipeline";
  // Stops in a safe window before the obstacle (stop_margin default 6 m).
  EXPECT_GT(r.ego.x, 88.0) << "stopped too early, final x=" << r.ego.x;
  EXPECT_LT(r.ego.x, 99.0) << "did not stop before obstacle, final x=" << r.ego.x;
  EXPECT_LT(r.ego.v, 0.5) << "not stopped, final v=" << r.ego.v;
}

TEST(Integration, LocalizationTracksGroundTruth) {
  Result r = runScenario(true, 100.0);
  EXPECT_LT(r.max_pos_err, 3.0) << "EKF drifted, max err=" << r.max_pos_err;
}

TEST(Integration, ClearRoadKeepsDrivingAndStaysInLane) {
  Result r = runScenario(/*has_obstacle=*/false, /*obstacle_x=*/0.0, /*t_end=*/25.0);
  EXPECT_FALSE(r.detected_ever) << "false-positive detection on a clear road";
  EXPECT_GT(r.ego.x, 60.0) << "car failed to progress, final x=" << r.ego.x;
  EXPECT_LT(std::fabs(r.ego.y), 1.0) << "car left the lane, |y|=" << std::fabs(r.ego.y);
  EXPECT_FALSE(r.any_nan);
}
