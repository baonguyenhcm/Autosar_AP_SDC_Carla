// av_localization/ekf_localizer.hpp
// ROS-free 2D Extended Kalman Filter fusing GPS position, wheel-speed odometry
// and an absolute heading measurement (e.g. from GNSS/INS or NDT scan-matching).
//
// State x = [px, py, yaw, v]^T
//   - prediction uses a constant-velocity / measured-yaw-rate motion model,
//     driven by the IMU yaw-rate as a control input;
//   - GPS provides an (px, py) correction;
//   - wheel odometry provides a v correction;
//   - GNSS/INS (or NDT) provides an absolute yaw correction.
//
// A fixed 4-state filter is small enough to hand-roll without Eigen, which keeps
// the module dependency-free and portable to embedded targets.
#pragma once

#include <array>
#include "av_common/types.hpp"

namespace av {

struct EkfParams {
  double q_pos{0.05};      // process noise std on position [m]
  double q_yaw{0.02};      // process noise std on yaw [rad]
  double q_v{0.20};        // process noise std on speed [m/s]
  double r_gps{0.80};      // GPS measurement std [m]
  double r_speed{0.15};    // wheel-speed measurement std [m/s]
  double r_yaw{0.05};      // heading measurement std [rad]
  double init_cov{1.0};    // initial state covariance
};

class EkfLocalizer {
 public:
  explicit EkfLocalizer(const EkfParams& p = {});

  // Seed the filter (e.g. from the first GPS fix).
  void initialize(double x, double y, double yaw, double v, double stamp);
  bool initialized() const { return initialized_; }

  // Prediction step: propagate the state forward by dt using the IMU yaw-rate.
  void predict(double dt, double yaw_rate);

  // Correction steps.
  void updateGps(const GpsMeasurement& z);
  void updateSpeed(double speed);
  void updateYaw(double yaw);  // absolute heading (e.g. from GNSS/INS or NDT)

  VehicleState state() const;
  void setStamp(double s) { stamp_ = s; }

 private:
  EkfParams params_;
  bool initialized_{false};
  double stamp_{0.0};
  std::array<double, 4> x_{{0, 0, 0, 0}};          // state
  std::array<std::array<double, 4>, 4> P_{};       // covariance
};

}  // namespace av
