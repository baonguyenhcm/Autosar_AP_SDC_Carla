// Safe-Stop Adaptive Application (conformant ara::com).
//   provides: ControlService (command = standstill brake)   [Skeleton]
// Runs ONLY in the DrivingFG "SafeStop" function-group state, which State Management
// enters (via a PHM recovery action) on a control-loop supervision failure. Execution
// Management stops control_app and starts this app for that state.
#ifndef AV_CONTROL_SAFE_STOP_APP_HPP
#define AV_CONTROL_SAFE_STOP_APP_HPP
#include <string>
#include "av_ap/ap.hpp"
#include "av_ap/av_services.hpp"
namespace av {
class SafeStopApp {
 public:
  explicit SafeStopApp(const std::string& instance)
      : out_(instance), log_("safe_stop") {
    out_.OfferService();
    log_.Info("SafeStopApp: ControlService offered (commands standstill brake)");
  }
  void step() {
    ControlCommand c; c.throttle = 0.0; c.brake = 1.0; c.steer = 0.0;
    out_.command.Send(c);
  }
 private:
  services::ControlServiceSkeleton out_;
  ap::Logger log_;
};
}  // namespace av
#endif  // AV_CONTROL_SAFE_STOP_APP_HPP
