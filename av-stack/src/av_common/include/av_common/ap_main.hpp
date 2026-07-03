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
inline int run_app(const std::function<void()>& on_period,
                   std::chrono::milliseconds period) {
  Initialize();
  install_sigint();
  ExecutionClient exec;
  exec.ReportRunning();  // ara::exec: process is up (Execution Management)
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
