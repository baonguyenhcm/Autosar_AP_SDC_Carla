// av_control/controller_core.cpp
#include "av_control/controller_core.hpp"
#include "av_common/math.hpp"

#include <cmath>
#include <limits>

namespace av {

ControlCommand ControllerCore::computeCommand(const VehicleState& ego,
                                              const Trajectory& traj,
                                              double dt) {
  ControlCommand cmd;
  cmd.stamp = ego.stamp;
  if (traj.empty()) {
    cmd.brake = 1.0;  // fail-safe: no plan -> stop
    return cmd;
  }

  // --- Lateral: pure pursuit ---
  const double Ld =
      params_.lookahead_gain * std::max(0.0, ego.v) + params_.min_lookahead;

  // Pick the first trajectory point at least Ld ahead of the ego.
  int target = static_cast<int>(traj.size()) - 1;
  for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
    if (dist2D(ego.x, ego.y, traj[i].x, traj[i].y) >= Ld) {
      target = i;
      break;
    }
  }
  const TrajectoryPoint& tp = traj[target];

  // Heading error to the lookahead point, expressed in the ego frame.
  const double dx = tp.x - ego.x;
  const double dy = tp.y - ego.y;
  const double alpha =
      normalizeAngle(std::atan2(dy, dx) - ego.yaw);
  const double ld_actual = std::max(1e-3, std::sqrt(dx * dx + dy * dy));
  // Pure-pursuit curvature -> steering angle.
  const double steer =
      std::atan2(2.0 * params_.wheelbase * std::sin(alpha), ld_actual);
  cmd.steer = clamp(steer, -params_.max_steer, params_.max_steer);

  // --- Longitudinal: PID on speed error ---
  // Target speed: the speed of the nearest point in the local trajectory.
  int nearest = 0;
  double best = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
    const double d = dist2D(ego.x, ego.y, traj[i].x, traj[i].y);
    if (d < best) {
      best = d;
      nearest = i;
    }
  }
  const double v_target = traj[nearest].v;
  const double err = v_target - ego.v;

  if (dt > 0.0) {
    integral_ += err * dt;
    integral_ = clamp(integral_, -5.0, 5.0);  // anti-windup
  }
  const double deriv = (have_prev_ && dt > 0.0) ? (err - prev_err_) / dt : 0.0;
  prev_err_ = err;
  have_prev_ = true;

  double accel = params_.kp * err + params_.ki * integral_ + params_.kd * deriv;

  // Hard stop when commanded speed is ~0 and we are nearly stopped.
  if (v_target < 0.1 && ego.v < 0.3) {
    cmd.throttle = 0.0;
    cmd.brake = 1.0;
    return cmd;
  }

  if (accel >= 0.0) {
    cmd.throttle = clamp(accel / params_.max_accel, 0.0, 1.0);
    cmd.brake = 0.0;
  } else {
    cmd.throttle = 0.0;
    cmd.brake = clamp(-accel / params_.max_decel, 0.0, 1.0);
  }
  return cmd;
}

}  // namespace av
