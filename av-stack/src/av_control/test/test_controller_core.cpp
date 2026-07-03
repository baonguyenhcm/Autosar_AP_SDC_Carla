// Unit tests for av::ControllerCore (pure-pursuit + PID).
#include <gtest/gtest.h>
#include "av_control/controller_core.hpp"

using namespace av;

namespace {
VehicleState ego(double x, double y, double yaw, double v) {
  VehicleState s; s.x = x; s.y = y; s.yaw = yaw; s.v = v; return s;
}
// Straight trajectory ahead along +x from x0, all at speed v.
Trajectory straight(double x0, double v, int n = 30) {
  Trajectory t;
  for (int i = 0; i < n; ++i) t.push_back({x0 + i * 1.0, 0.0, 0.0, v});
  return t;
}
}  // namespace

TEST(ControllerCore, EmptyTrajectoryBrakesAsFailSafe) {
  ControllerCore c;
  ControlCommand cmd = c.computeCommand(ego(0, 0, 0, 5), {}, 0.02);
  EXPECT_GT(cmd.brake, 0.9);
  EXPECT_NEAR(cmd.throttle, 0.0, 1e-9);
}

TEST(ControllerCore, StraightAlignedPathGivesNearZeroSteer) {
  ControllerCore c;
  ControlCommand cmd = c.computeCommand(ego(0, 0, 0, 5), straight(0, 8), 0.02);
  EXPECT_NEAR(cmd.steer, 0.0, 1e-3);
}

TEST(ControllerCore, TargetToLeftSteersLeftPositive) {
  ControllerCore c;
  Trajectory t;
  for (int i = 0; i < 30; ++i) t.push_back({static_cast<double>(i), 2.0, 0.0, 6.0});  // offset +y
  ControlCommand cmd = c.computeCommand(ego(0, 0, 0, 5), t, 0.02);
  EXPECT_GT(cmd.steer, 0.05);
}

TEST(ControllerCore, TargetToRightSteersRightNegative) {
  ControllerCore c;
  Trajectory t;
  for (int i = 0; i < 30; ++i) t.push_back({static_cast<double>(i), -2.0, 0.0, 6.0});
  ControlCommand cmd = c.computeCommand(ego(0, 0, 0, 5), t, 0.02);
  EXPECT_LT(cmd.steer, -0.05);
}

TEST(ControllerCore, AcceleratesWhenBelowTargetSpeed) {
  ControllerCore c;
  ControlCommand cmd = c.computeCommand(ego(0, 0, 0, 2.0), straight(0, 8.0), 0.02);
  EXPECT_GT(cmd.throttle, 0.0);
  EXPECT_NEAR(cmd.brake, 0.0, 1e-9);
}

TEST(ControllerCore, BrakesWhenAboveTargetSpeed) {
  ControllerCore c;
  ControlCommand cmd = c.computeCommand(ego(0, 0, 0, 9.0), straight(0, 2.0), 0.02);
  EXPECT_GT(cmd.brake, 0.0);
  EXPECT_NEAR(cmd.throttle, 0.0, 1e-9);
}

TEST(ControllerCore, HoldsStopWhenTargetZeroAndStopped) {
  ControllerCore c;
  ControlCommand cmd = c.computeCommand(ego(0, 0, 0, 0.1), straight(0, 0.0), 0.02);
  EXPECT_GT(cmd.brake, 0.9);
  EXPECT_NEAR(cmd.throttle, 0.0, 1e-9);
}

TEST(ControllerCore, SteeringRespectsSaturationLimit) {
  ControlParams p; p.max_steer = 0.4;
  ControllerCore c(p);
  Trajectory t;
  for (int i = 0; i < 30; ++i) t.push_back({static_cast<double>(i) * 0.2, 10.0, 0.0, 6.0});  // hard left
  ControlCommand cmd = c.computeCommand(ego(0, 0, 0, 5), t, 0.02);
  EXPECT_LE(cmd.steer, 0.4 + 1e-9);
  EXPECT_GE(cmd.steer, -0.4 - 1e-9);
}
