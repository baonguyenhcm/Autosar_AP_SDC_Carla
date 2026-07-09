// Localization Adaptive Application entry point (started by Execution Management).
// Event-driven: ara::com receive handlers publish VehicleStateService on new sensors.
#include "av_common/ap_main.hpp"
#include "av_localization/localization_app.hpp"

int main() {
  // Initialize the ara runtime BEFORE constructing the app. The app ctor subscribes to
  // SensorService events whose receive handlers fire on ara::com threads immediately —
  // localization's IMU feed is already live at startup, so a handler can run during
  // construction. Without this, that handler races an uninitialized runtime (intermittent
  // registration timeout / crash). run_app() calls Initialize() again; it is idempotent.
  av::ap::Initialize();
  av::LocalizationApp app("av");                      // INIT: register + offer, no subscribe
  return av::ap::run_app(nullptr, std::chrono::milliseconds(10),
                         [&] { app.Start(); });        // RUNNING: subscribe once registered
}
