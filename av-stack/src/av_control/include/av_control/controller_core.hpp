// av_control/controller_core.hpp
// ROS-free control core:
//   - lateral:      pure-pursuit steering toward a lookahead point;
//   - longitudinal: PID on speed error, mapped to throttle/brake.
#pragma once

#include "av_common/types.hpp"

namespace av {

struct ControlParams {
  double wheelbase{2.7};        // L [m]
  double lookahead_gain{0.6};   // Ld = gain*v + min_lookahead
  double min_lookahead{4.0};    // [m]
  double max_steer{0.6};        // [rad]
  // Longitudinal PID (on speed error, output ~ acceleration in m/s^2).
  double kp{0.8};
  double ki{0.1};
  double kd{0.05};
  double max_accel{2.0};        // maps to full throttle [m/s^2]
  double max_decel{4.0};        // maps to full brake  [m/s^2]
};

class ControllerCore {
 public:
  explicit ControllerCore(const ControlParams& p = {}) : params_(p) {}

  // dt is the time since the previous call (for the PID derivative/integral).
  ControlCommand computeCommand(const VehicleState& ego,
                                const Trajectory& traj, double dt);

  void reset() { integral_ = 0.0; prev_err_ = 0.0; have_prev_ = false; }

 private:
  ControlParams params_;
  double integral_{0.0};
  double prev_err_{0.0};
  bool have_prev_{false};
};

}  // namespace av
