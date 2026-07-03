// Localization Adaptive Application entry point (started by Execution Management).
// Event-driven: ara::com receive handlers publish VehicleStateService on new sensors.
#include "av_common/ap_main.hpp"
#include "av_localization/localization_app.hpp"

int main() {
  av::LocalizationApp app("av");
  return av::ap::run_app(nullptr, std::chrono::milliseconds(10));
}
