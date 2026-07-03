// Unit tests for av::EkfLocalizer (2D EKF).
#include <gtest/gtest.h>
#include "av_localization/ekf_localizer.hpp"
#include "av_common/math.hpp"

using namespace av;

TEST(EkfLocalizer, InitializeSetsState) {
  EkfLocalizer ekf;
  ekf.initialize(1.0, 2.0, 0.3, 4.0, 0.0);
  ASSERT_TRUE(ekf.initialized());
  const auto s = ekf.state();
  EXPECT_NEAR(s.x, 1.0, 1e-9);
  EXPECT_NEAR(s.y, 2.0, 1e-9);
  EXPECT_NEAR(s.yaw, 0.3, 1e-9);
  EXPECT_NEAR(s.v, 4.0, 1e-9);
}

TEST(EkfLocalizer, GpsUpdateConvergesToMeasurement) {
  EkfLocalizer ekf;
  ekf.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  GpsMeasurement g; g.x = 10.0; g.y = -3.0; g.stamp = 0.0;
  for (int i = 0; i < 100; ++i) {
    ekf.predict(0.1, 0.0);   // v=0 => no motion
    ekf.updateGps(g);
    ekf.updateYaw(0.0);
  }
  const auto s = ekf.state();
  EXPECT_NEAR(s.x, 10.0, 0.3);
  EXPECT_NEAR(s.y, -3.0, 0.3);
}

TEST(EkfLocalizer, SpeedUpdateConvergesToWheelSpeed) {
  EkfLocalizer ekf;
  ekf.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  for (int i = 0; i < 50; ++i) { ekf.predict(0.1, 0.0); ekf.updateSpeed(6.0); }
  EXPECT_NEAR(ekf.state().v, 6.0, 0.2);
}

TEST(EkfLocalizer, YawUpdateConvergesToHeading) {
  EkfLocalizer ekf;
  ekf.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  for (int i = 0; i < 50; ++i) { ekf.predict(0.1, 0.0); ekf.updateYaw(1.0); }
  EXPECT_NEAR(normalizeAngle(ekf.state().yaw - 1.0), 0.0, 0.05);
}

TEST(EkfLocalizer, DeadReckoningAdvancesPosition) {
  // Constant 5 m/s heading +x, no GPS: after 1s x ~= 5m, y ~= 0.
  EkfLocalizer ekf;
  ekf.initialize(0.0, 0.0, 0.0, 5.0, 0.0);
  for (int i = 0; i < 10; ++i) ekf.predict(0.1, 0.0);
  const auto s = ekf.state();
  EXPECT_NEAR(s.x, 5.0, 0.2);
  EXPECT_NEAR(s.y, 0.0, 0.05);
}

TEST(EkfLocalizer, PredictBeforeInitIsNoOp) {
  EkfLocalizer ekf;
  ekf.predict(0.1, 0.5);  // must not crash / must stay uninitialized
  EXPECT_FALSE(ekf.initialized());
}

TEST(EkfLocalizer, TracksATurn) {
  // Yaw-rate 0.2 rad/s for 1s with v=0 -> yaw ~= 0.2 rad.
  EkfLocalizer ekf;
  ekf.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  for (int i = 0; i < 10; ++i) ekf.predict(0.1, 0.2);
  EXPECT_NEAR(ekf.state().yaw, 0.2, 1e-6);
}
