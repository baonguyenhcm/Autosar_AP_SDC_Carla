// Safe-Stop Adaptive Application entry point. Periodic at ~50 Hz.
#include "av_common/ap_main.hpp"
#include "av_control/safe_stop_app.hpp"
int main() {
  av::SafeStopApp app("av");
  return av::ap::run_app([&] { app.step(); }, std::chrono::milliseconds(20));
}
