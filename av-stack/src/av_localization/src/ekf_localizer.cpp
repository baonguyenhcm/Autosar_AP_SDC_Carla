// av_localization/ekf_localizer.cpp
#include "av_localization/ekf_localizer.hpp"
#include "av_common/math.hpp"

#include <cmath>

namespace av {

EkfLocalizer::EkfLocalizer(const EkfParams& p) : params_(p) {
  for (auto& row : P_) row.fill(0.0);
}

void EkfLocalizer::initialize(double x, double y, double yaw, double v,
                              double stamp) {
  x_ = {x, y, yaw, v};
  for (auto& row : P_) row.fill(0.0);
  for (int i = 0; i < 4; ++i) P_[i][i] = params_.init_cov;
  stamp_ = stamp;
  initialized_ = true;
}

void EkfLocalizer::predict(double dt, double yaw_rate) {
  if (!initialized_ || dt <= 0.0) return;

  const double yaw = x_[2];
  const double v = x_[3];
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  // --- State propagation (motion model) ---
  x_[0] += v * c * dt;
  x_[1] += v * s * dt;
  x_[2] = normalizeAngle(yaw + yaw_rate * dt);
  // x_[3] (v) held constant in prediction; corrected by wheel-speed update.

  // --- Jacobian F = d f / d x ---
  std::array<std::array<double, 4>, 4> F{};
  for (int i = 0; i < 4; ++i) F[i][i] = 1.0;
  F[0][2] = -v * s * dt;
  F[0][3] = c * dt;
  F[1][2] = v * c * dt;
  F[1][3] = s * dt;

  // --- P = F P F^T + Q ---
  std::array<std::array<double, 4>, 4> FP{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) sum += F[i][k] * P_[k][j];
      FP[i][j] = sum;
    }
  std::array<std::array<double, 4>, 4> newP{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) sum += FP[i][k] * F[j][k];  // FP * F^T
      newP[i][j] = sum;
    }

  const double q0 = params_.q_pos * params_.q_pos * dt;
  const double q2 = params_.q_yaw * params_.q_yaw * dt;
  const double q3 = params_.q_v * params_.q_v * dt;
  newP[0][0] += q0;
  newP[1][1] += q0;
  newP[2][2] += q2;
  newP[3][3] += q3;
  P_ = newP;

  stamp_ += dt;
}

void EkfLocalizer::updateGps(const GpsMeasurement& z) {
  if (!initialized_) {
    initialize(z.x, z.y, 0.0, 0.0, z.stamp);
    return;
  }
  // H maps state->[px,py]; innovation y = z - H x.
  const double yx = z.x - x_[0];
  const double yy = z.y - x_[1];

  // S = H P H^T + R  (2x2, top-left block of P plus R)
  const double r = params_.r_gps * params_.r_gps;
  const double s00 = P_[0][0] + r;
  const double s01 = P_[0][1];
  const double s10 = P_[1][0];
  const double s11 = P_[1][1] + r;
  const double det = s00 * s11 - s01 * s10;
  if (std::abs(det) < 1e-12) return;
  // S^-1
  const double i00 = s11 / det;
  const double i01 = -s01 / det;
  const double i10 = -s10 / det;
  const double i11 = s00 / det;

  // K = P H^T S^-1  (4x2). P H^T = first two columns of P.
  std::array<std::array<double, 2>, 4> K{};
  for (int i = 0; i < 4; ++i) {
    const double a = P_[i][0];
    const double b = P_[i][1];
    K[i][0] = a * i00 + b * i10;
    K[i][1] = a * i01 + b * i11;
  }

  // x += K y
  for (int i = 0; i < 4; ++i) x_[i] += K[i][0] * yx + K[i][1] * yy;
  x_[2] = normalizeAngle(x_[2]);

  // P = (I - K H) P.  K H is 4x4 with nonzero first two columns = K.
  std::array<std::array<double, 4>, 4> KH{};
  for (int i = 0; i < 4; ++i) {
    KH[i][0] = K[i][0];
    KH[i][1] = K[i][1];
  }
  std::array<std::array<double, 4>, 4> newP{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double sum = P_[i][j];
      for (int k = 0; k < 4; ++k) sum -= KH[i][k] * P_[k][j];
      newP[i][j] = sum;
    }
  P_ = newP;
  stamp_ = z.stamp;
}

void EkfLocalizer::updateSpeed(double speed) {
  if (!initialized_) return;
  // Scalar update on v (state index 3).
  const double y = speed - x_[3];
  const double r = params_.r_speed * params_.r_speed;
  const double s = P_[3][3] + r;
  if (s < 1e-12) return;
  std::array<double, 4> K{};
  for (int i = 0; i < 4; ++i) K[i] = P_[i][3] / s;
  for (int i = 0; i < 4; ++i) x_[i] += K[i] * y;
  x_[2] = normalizeAngle(x_[2]);
  std::array<std::array<double, 4>, 4> newP{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) newP[i][j] = P_[i][j] - K[i] * P_[3][j];
  P_ = newP;
}

void EkfLocalizer::updateYaw(double yaw) {
  if (!initialized_) return;
  // Scalar update on yaw (state index 2). Innovation wrapped to (-pi, pi].
  const double y = normalizeAngle(yaw - x_[2]);
  const double r = params_.r_yaw * params_.r_yaw;
  const double s = P_[2][2] + r;
  if (s < 1e-12) return;
  std::array<double, 4> K{};
  for (int i = 0; i < 4; ++i) K[i] = P_[i][2] / s;
  for (int i = 0; i < 4; ++i) x_[i] += K[i] * y;
  x_[2] = normalizeAngle(x_[2]);
  std::array<std::array<double, 4>, 4> newP{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) newP[i][j] = P_[i][j] - K[i] * P_[2][j];
  P_ = newP;
}

VehicleState EkfLocalizer::state() const {
  VehicleState vs;
  vs.stamp = stamp_;
  vs.x = x_[0];
  vs.y = x_[1];
  vs.yaw = x_[2];
  vs.v = x_[3];
  return vs;
}

}  // namespace av
