// av_ap/ara_com_target_bindings.hpp — SKETCH of the AP_HAVE_ARA wrapper layer.
//
// STATUS: reference sketch, NOT yet compiled on hardware. It shows how av-stack's
// SkeletonEvent<T>/ProxyEvent<T> (the API av_services.hpp + the AAs use) map onto the real
// Adaptive-AUTOSAR ara::com via the codegen's per-topic classes. Wire it in once the
// generated header exists on the target (see integration notes at the bottom).
//
// WHY a wrapper is needed:
//   * `autosar-generate-proxy-skeleton` emits generic per-topic classes
//     `autosar_generated::TopicEventSkeleton<T>` / `TopicEventProxy<T>`, each constructed
//     from a TOPIC-NAME STRING and holding one `ara::com::SkeletonEvent<T>` / `ProxyEvent<T>`
//     as a public member `Event` (it resolves the topic -> service/instance/event IDs via the
//     manifest mapping YAML, and builds the SOME/IP or DDS binding through BindingFactory).
//   * av-stack's SkeletonEvent<T>/ProxyEvent<T> are keyed by a topic string (svc_key()) and
//     expose Offer/StopOffer/Send and Subscribe/SetReceiveHandler/GetNewSamples/... — which
//     is exactly the surface `TopicEventSkeleton::{OfferService,StopOfferService}` +
//     `TopicEventSkeleton::Event.{Send}` and `TopicEventProxy::Event.{Subscribe,...}` provide.
//   So each av-stack event delegates to ONE TopicEvent{Skeleton,Proxy} (one topic binding).
#ifndef AV_AP_ARA_COM_TARGET_BINDINGS_HPP
#define AV_AP_ARA_COM_TARGET_BINDINGS_HPP
#if defined(AP_HAVE_ARA)

#include <cstddef>
#include <functional>
#include <string>
#include <utility>

// The generated per-topic proxy/skeleton header (namespace configurable; default
// `autosar_generated`). Produced by `autosar-generate-proxy-skeleton --namespace av::ap::gen
// --output .../av_gen_proxy_skeleton.hpp` from lwrcl_autosar_topic_mapping.yaml. Adjust the
// include name/namespace below to match your generation step.
#include "av_gen_proxy_skeleton.hpp"   // -> namespace av::ap::gen { TopicEventSkeleton<T>, TopicEventProxy<T> }

#include "ara/com/types.h"             // ara::com::SamplePtr<T>, SubscriptionState
#include "ara/core/result.h"

namespace av {
namespace ap {

namespace gen_ns = ::av::ap::gen;      // alias to whatever --namespace you generated into

// ---- Provider side: av-stack SkeletonEvent<T> over gen::TopicEventSkeleton<T> ----
// One topic binding per event; Offer()/StopOffer() drive the underlying service offer.
template <class T>
class SkeletonEvent {
 public:
  explicit SkeletonEvent(std::string topic) : impl_(std::move(topic)) {}
  void Offer()     { (void)impl_.OfferService(); }      // service-level offer (per topic)
  void StopOffer() { impl_.StopOfferService(); }
  ::ara::core::Result<void> Send(const T& sample) { return impl_.Event.Send(sample); }
 private:
  gen_ns::TopicEventSkeleton<T> impl_;
};

// ---- Consumer side: av-stack ProxyEvent<T> over gen::TopicEventProxy<T> ----
template <class T>
class ProxyEvent {
 public:
  explicit ProxyEvent(std::string topic) : impl_(std::move(topic)) {}
  ::ara::core::Result<void> Subscribe(std::size_t n = 1) { return impl_.Event.Subscribe(n); }
  void Unsubscribe() { impl_.Event.Unsubscribe(); }
  ::ara::com::SubscriptionState GetSubscriptionState() const {
    return impl_.Event.GetSubscriptionState();
  }
  void SetReceiveHandler(std::function<void()> h) { impl_.Event.SetReceiveHandler(std::move(h)); }
  template <class F>
  ::ara::core::Result<std::size_t> GetNewSamples(F&& f, std::size_t n = 1) {
    return impl_.Event.GetNewSamples(std::forward<F>(f), n);   // f takes ara::com::SamplePtr<T>
  }
 private:
  gen_ns::TopicEventProxy<T> impl_;
};

}  // namespace ap
}  // namespace av

// ---------------------------------------------------------------------------------------
// INTEGRATION (2 steps, done on the target where the generated header exists):
//
// 1) In av_services.hpp, under AP_HAVE_ARA, alias the event types to these wrappers instead
//    of `using ara::com::SkeletonEvent/ProxyEvent` (which would pull the REAL ara::com event
//    whose ctor takes a binding, not a topic string, and would clash by name):
//
//        #if defined(AP_HAVE_ARA)
//        #include "av_ap/ara_com_target_bindings.hpp"
//        using SkeletonEvent = av::ap::SkeletonEvent;   // (template alias per event use)
//        using ProxyEvent    = av::ap::ProxyEvent;
//        #else
//        using ara::com::SkeletonEvent;  using ara::com::ProxyEvent;
//        #endif
//
//    The grouped <Service>Skeleton/<Service>Proxy classes and svc_key() strings stay as-is;
//    each event just resolves to its own topic binding.
//
// 2) Reconcile PAYLOAD ACCESSORS. Real ara::com sample types (generated from IDL/ARXML) use
//    METHOD accessors: s->stamp(), s->x(), s->yaw_rate(). The av-stack payload structs use
//    MEMBER access: s->stamp, s->x. Either (a) generate the sample types with member fields,
//    or (b) switch the AA read sites (localization/perception/planning/control *_app.hpp,
//    and the gateway) to method accessors under AP_HAVE_ARA. The gateway already uses
//    method-style for lwrcl messages, so option (b) is consistent.
//
// NOTE: names TopicEventSkeleton/TopicEventProxy and the `.Event` member match
// Adaptive-AUTOSAR/tools/ara_com_codegen/generate_proxy_skeleton_header.py; confirm against
// your generated header and adjust `gen_ns` / the include path accordingly.
// ---------------------------------------------------------------------------------------

#endif  // AP_HAVE_ARA
#endif  // AV_AP_ARA_COM_TARGET_BINDINGS_HPP
