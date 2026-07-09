// av_common/ap_main.hpp — shared main() helpers for Adaptive Applications.
// Handles ara::core init, ara::exec running/terminating, and a SIGINT-driven run loop.
#ifndef AV_COMMON_AP_MAIN_HPP
#define AV_COMMON_AP_MAIN_HPP

#include <atomic>
#include <chrono>
#include <csignal>
#include <functional>
#include <thread>

#include "av_ap/ap.hpp"

namespace av {
namespace ap {

inline std::atomic<bool>& run_flag() {
  static std::atomic<bool> g{true};
  return g;
}
inline void install_sigint() {
  std::signal(SIGINT, [](int) { run_flag() = false; });
  std::signal(SIGTERM, [](int) { run_flag() = false; });
}

// Run an Adaptive Application: init AP, report running, tick `on_period` every
// `period` (use a no-op for purely event-driven apps), until SIGINT/SIGTERM.
// on_start (optional): the app's transition into the RUNNING state — e.g. subscribing to
// SOME/IP events. It runs AFTER ReportRunning and a short settle, so the binding registration
// has completed before any receive handler can fire. Apps whose subscribed data is already
// live at startup (localization's IMU feed) MUST defer their Subscribe here; subscribing in the
// ctor lets the event flood race an incomplete registration -> SIGSEGV.
inline int run_app(const std::function<void()>& on_period,
                   std::chrono::milliseconds period,
                   const std::function<void()>& on_start = nullptr) {
  Initialize();
  install_sigint();
  ExecutionClient exec;
  exec.ReportRunning();  // ara::exec: process is up (Execution Management) -> enter RUNNING
  if (on_start) {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));  // let registration settle
    on_start();
  }
  while (run_flag()) {
    if (on_period) on_period();
    std::this_thread::sleep_for(period);
  }
  exec.ReportTerminating();
  Deinitialize();
  return 0;
}

}  // namespace ap
}  // namespace av
#endif  // AV_COMMON_AP_MAIN_HPP
