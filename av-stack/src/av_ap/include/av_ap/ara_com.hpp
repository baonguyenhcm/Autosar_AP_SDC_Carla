// av_ap/ara_com.hpp — the standardized ara::core / ara::com API used by the apps.
//
// The Adaptive Applications are written ONCE against this API (the real AP surface:
// FindService + Offer/StopOffer, SkeletonEvent::Send, ProxyEvent Subscribe /
// SetReceiveHandler / GetNewSamples with SamplePtr, and ara::core::Result).
//
//   -DAP_HAVE_ARA  -> include the real ara::core / ara::com from the AP runtime
//                     (generated proxies/skeletons come from the ARXML codegen).
//   (host default) -> a faithful in-process shim of the SAME API, so the identical app
//                     code compiles and runs without the AP runtime. The shim stands in
//                     for the SOME/IP binding (ARA_COM_EVENT_BINDING=vsomeip on target).
#ifndef AV_AP_ARA_COM_HPP
#define AV_AP_ARA_COM_HPP

#if defined(AP_HAVE_ARA)
// Real Adaptive-AUTOSAR ara::core + ara::com surface — the same headers the
// Adaptive-AUTOSAR proxy/skeleton codegen pulls in
// (tools/ara_com_codegen/generate_proxy_skeleton_header.py).
#include "ara/core/result.h"
#include "ara/core/error_code.h"
#include "ara/core/instance_specifier.h"
#include "ara/com/types.h"
#include "ara/com/sample_ptr.h"
#include "ara/com/event.h"                    // ara::com::SkeletonEvent<T> / ProxyEvent<T>
#include "ara/com/service_handle_type.h"
#include "ara/com/service_skeleton_base.h"
#include "ara/com/service_proxy_base.h"
#include "ara/com/internal/binding_factory.h"
#include "ara/com/com_error_domain.h"
//
// TARGET PROXY/SKELETON LAYER — not a drop-in include:
//   `autosar-generate-proxy-skeleton` does NOT emit grouped `<Service>Skeleton`
//   (e.g. SensorService with imu/gps/speed/lidar). It emits generic per-topic
//   `TopicEventSkeleton<T>` / `TopicEventProxy<T>` (each holding one
//   `ara::com::SkeletonEvent<T>` / `ProxyEvent<T>`) in a chosen namespace
//   (default `autosar_generated`), constructed from a topic-name string.
//   So on the target, av_services.hpp's `SkeletonEvent<T>` / `ProxyEvent<T>` must be thin
//   wrappers over real `ara::com::SkeletonEvent<T>` / `ProxyEvent<T>` + BindingFactory,
//   keyed by the same `svc_key()` topic strings (one topic binding per event, becoming the
//   manifest topic bindings). Offering is service-level (`OfferService()`), not per event.
//   The real `ara::com::SkeletonEvent<T>` has no string-key ctor / per-event Offer, so this
//   wrapper still needs to be written before AP_HAVE_ARA compiles on hardware.
#else  // ------------------------- host shim of the ara::* API -------------------------
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ara {
namespace core {

class ErrorCode {
 public:
  ErrorCode() = default;
  explicit ErrorCode(int v, const char* m = "") : value_(v), msg_(m) {}
  int Value() const { return value_; }
  const char* Message() const { return msg_; }
 private:
  int value_{0};
  const char* msg_{""};
};

template <class T>
class Result {
 public:
  Result(const T& v) : has_(true), val_(v) {}
  Result(ErrorCode e) : has_(false), err_(e) {}
  static Result FromValue(const T& v) { return Result(v); }
  static Result FromError(ErrorCode e) { return Result(e); }
  bool HasValue() const { return has_; }
  explicit operator bool() const { return has_; }
  const T& Value() const { return val_; }
  const T& operator*() const { return val_; }
  const ErrorCode& Error() const { return err_; }
  T ValueOr(const T& d) const { return has_ ? val_ : d; }
 private:
  bool has_{false};
  T val_{};
  ErrorCode err_{};
};

template <>
class Result<void> {
 public:
  Result() : has_(true) {}
  Result(ErrorCode e) : has_(false), err_(e) {}
  static Result FromValue() { return Result(); }
  bool HasValue() const { return has_; }
  explicit operator bool() const { return has_; }
  const ErrorCode& Error() const { return err_; }
 private:
  bool has_{true};
  ErrorCode err_{};
};

class InstanceSpecifier {
 public:
  explicit InstanceSpecifier(std::string s) : s_(std::move(s)) {}
  const std::string& ToString() const { return s_; }
 private:
  std::string s_;
};

}  // namespace core

namespace com {

using InstanceIdentifier = std::string;
using FindServiceHandle = std::uint32_t;

template <class T>
using SamplePtr = std::shared_ptr<const T>;

template <class T>
using ServiceHandleContainer = std::vector<T>;

enum class SubscriptionState { kSubscribed, kNotSubscribed, kSubscriptionPending };

namespace detail {
template <class T>
struct Channel {
  bool offered = false;
  std::vector<std::function<void(const T&)>> handlers;
  std::shared_ptr<const T> last;
  std::uint64_t seq = 0;
};
template <class T>
inline Channel<T>& channel(const std::string& key) {
  static std::map<std::string, Channel<T>> reg;
  return reg[key];
}
}  // namespace detail

template <class T>
class SkeletonEvent {
 public:
  explicit SkeletonEvent(std::string key) : key_(std::move(key)) {}
  void Offer() { detail::channel<T>(key_).offered = true; }
  void StopOffer() { detail::channel<T>(key_).offered = false; }
  ara::core::Result<void> Send(const T& sample) {
    auto& ch = detail::channel<T>(key_);
    if (!ch.offered) return ara::core::ErrorCode(1, "kServiceNotOffered");
    ch.last = std::make_shared<const T>(sample);
    ++ch.seq;
    for (auto& h : ch.handlers) h(sample);
    return ara::core::Result<void>::FromValue();
  }
 private:
  std::string key_;
};

template <class T>
class ProxyEvent {
 public:
  explicit ProxyEvent(std::string key) : key_(std::move(key)) {}
  ara::core::Result<void> Subscribe(std::size_t = 1) {
    subscribed_ = true;
    return ara::core::Result<void>::FromValue();
  }
  void Unsubscribe() { subscribed_ = false; }
  SubscriptionState GetSubscriptionState() const {
    return subscribed_ ? SubscriptionState::kSubscribed : SubscriptionState::kNotSubscribed;
  }
  void SetReceiveHandler(std::function<void()> h) {
    detail::channel<T>(key_).handlers.push_back(
        [this, h](const T& s) { pending_ = std::make_shared<const T>(s); if (h) h(); });
  }
  template <class F>
  ara::core::Result<std::size_t> GetNewSamples(F&& f, std::size_t = 1) {
    std::size_t n = 0;
    if (!subscribed_) return n;
    if (pending_) { f(SamplePtr<T>(pending_)); pending_.reset(); ++n; return n; }
    auto& ch = detail::channel<T>(key_);
    if (ch.last && ch.seq != seen_) { seen_ = ch.seq; f(SamplePtr<T>(ch.last)); ++n; }
    return n;
  }
 private:
  std::string key_;
  bool subscribed_ = false;
  std::shared_ptr<const T> pending_;
  std::uint64_t seen_ = 0;
};

}  // namespace com
}  // namespace ara
#endif  // AP_HAVE_ARA
#endif  // AV_AP_ARA_COM_HPP
