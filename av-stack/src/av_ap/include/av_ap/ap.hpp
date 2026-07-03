// av_ap/ap.hpp — AUTOSAR Adaptive Platform Functional-Cluster adaptation layer.
//
// Thin C++ wrappers that let the av-stack node processes use the AP Functional
// Clusters directly, matching the arc42 architecture:
//   ara::exec  → ExecutionClient  (report process lifecycle to Execution Management)
//   ara::log   → Logger           (structured Log & Trace)
//   ara::phm   → SupervisedEntity  (alive/deadline supervision by Platform Health Mgmt)
//   (system state = ara::sm Machine States + Function Groups; see the Machine/Execution Manifests)
//
// Communication itself is provided by lwrcl's `ara::com` backend, so the node code
// keeps the rclcpp-compatible pub/sub API and does not appear here.
//
// Build modes:
//   -DAP_HAVE_ARA  → calls the real ara::* APIs (Jetson / adaptive-autosar backend).
//   (default)      → a host shim (stderr + always-Driving) so the stack still builds
//                    and runs on a plain Linux host over the CycloneDDS backend.
#ifndef AV_AP_AP_HPP
#define AV_AP_AP_HPP

#include <cstdint>
#include <string>

#if defined(AP_HAVE_ARA)
#include "ara/core/initialization.h"
#include "ara/exec/execution_client.h"
#include "ara/log/logging.h"
#include "ara/phm/supervised_entity.h"
#else
#include <iostream>
#endif

namespace av {
namespace ap {

// ---------------------------------------------------------------------------
// Runtime init/deinit — ara::core::Initialize / Deinitialize on the target.
// ---------------------------------------------------------------------------
inline void Initialize() {
#if defined(AP_HAVE_ARA)
  ara::core::Initialize();
#endif
}
inline void Deinitialize() {
#if defined(AP_HAVE_ARA)
  ara::core::Deinitialize();
#endif
}

// ---------------------------------------------------------------------------
// ara::exec — Execution Management client. Report process running/terminating.
// ---------------------------------------------------------------------------
class ExecutionClient {
 public:
  ExecutionClient() = default;

  void ReportRunning() {
#if defined(AP_HAVE_ARA)
    client_.ReportExecutionState(ara::exec::ExecutionState::kRunning);
#else
    std::cerr << "[ara::exec] ReportExecutionState(kRunning)\n";
#endif
  }
  void ReportTerminating() {
#if defined(AP_HAVE_ARA)
    client_.ReportExecutionState(ara::exec::ExecutionState::kTerminating);
#else
    std::cerr << "[ara::exec] ReportExecutionState(kTerminating)\n";
#endif
  }

 private:
#if defined(AP_HAVE_ARA)
  ara::exec::ExecutionClient client_{};
#endif
};

// ---------------------------------------------------------------------------
// ara::log — Log & Trace. Minimal severity-tagged logger.
// ---------------------------------------------------------------------------
class Logger {
 public:
  explicit Logger(const std::string& ctx_id, const std::string& ctx_desc = "")
#if defined(AP_HAVE_ARA)
      : logger_(ara::log::CreateLogger(ctx_id, ctx_desc)) {}
#else
      : ctx_(ctx_id) { (void)ctx_desc; }
#endif

  void Info(const std::string& m) {
#if defined(AP_HAVE_ARA)
    logger_.LogInfo() << m;
#else
    std::cerr << "[INFO][" << ctx_ << "] " << m << "\n";
#endif
  }
  void Warn(const std::string& m) {
#if defined(AP_HAVE_ARA)
    logger_.LogWarn() << m;
#else
    std::cerr << "[WARN][" << ctx_ << "] " << m << "\n";
#endif
  }
  void Error(const std::string& m) {
#if defined(AP_HAVE_ARA)
    logger_.LogError() << m;
#else
    std::cerr << "[ERROR][" << ctx_ << "] " << m << "\n";
#endif
  }

 private:
#if defined(AP_HAVE_ARA)
  ara::log::Logger& logger_;
#else
  std::string ctx_;
#endif
};

// ---------------------------------------------------------------------------
// ara::phm — Platform Health Management. Report a checkpoint every control
// cycle so PHM alive/deadline supervision can detect a stalled/overrunning loop.
// ---------------------------------------------------------------------------
class SupervisedEntity {
 public:
  explicit SupervisedEntity(std::uint32_t se_id) : id_(se_id) {}

  void ReportCheckpoint(std::uint32_t checkpoint_id) {
#if defined(AP_HAVE_ARA)
    entity_.ReportCheckpoint(checkpoint_id);
#else
    (void)checkpoint_id;  // host: supervision is a no-op
#endif
  }
  std::uint32_t id() const { return id_; }

 private:
  std::uint32_t id_;
#if defined(AP_HAVE_ARA)
  ara::phm::SupervisedEntity<std::uint32_t> entity_{};
#endif
};

}  // namespace ap
}  // namespace av
#endif  // AV_AP_AP_HPP
