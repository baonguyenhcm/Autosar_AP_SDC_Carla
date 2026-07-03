// Control Adaptive Application entry point. Periodic at ~50 Hz.
#include "av_common/ap_main.hpp"
#include "av_control/control_app.hpp"

int main() {
  av::ControlApp app("av");
  const double dt = 0.02;
  return av::ap::run_app([&] { app.step(dt); }, std::chrono::milliseconds(20));
}
