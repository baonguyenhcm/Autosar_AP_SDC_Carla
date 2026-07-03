// av_ap/av_services.hpp — the av-stack ara::com SERVICE INTERFACES.
//
// The grouped `<Service>Skeleton`/`<Service>Proxy` classes below (e.g. SensorService with
// imu/gps/speed/lidar events) are an av-stack CONVENIENCE layer over the
// `SkeletonEvent<T>`/`ProxyEvent<T>` from ara_com.hpp. On the host they run on the
// in-process ara::com shim.
//
// On the target (AP_HAVE_ARA): the Adaptive-AUTOSAR codegen
// (`autosar-generate-proxy-skeleton`) does NOT emit grouped <Service> classes — it emits
// generic per-topic `TopicEventSkeleton<T>`/`TopicEventProxy<T>` (namespace default
// `autosar_generated`) over the real `ara::com::SkeletonEvent`/`ProxyEvent`. So there is no
// generated header of these names to include; instead the `SkeletonEvent<T>`/`ProxyEvent<T>`
// used here must resolve to thin wrappers over real ara::com (see ara_com.hpp), one topic
// binding per event keyed by `svc_key()`. Also: real ara::com sample types use METHOD
// accessors (`s->stamp()`) whereas the host payload structs below use MEMBER access
// (`s->stamp`) — reconcile both when wiring AP_HAVE_ARA on hardware.
#ifndef AV_AP_AV_SERVICES_HPP
#define AV_AP_AV_SERVICES_HPP

#include <cstdint>
#include <string>

#include "av_ap/ara_com.hpp"
#include "av_common/types.hpp"

namespace av {
namespace services {

using ara::com::InstanceIdentifier;
using ara::com::ProxyEvent;
using ara::com::ServiceHandleContainer;
using ara::com::SkeletonEvent;

// ---- payload (event) types ----
struct ImuSample        { double stamp{0}, yaw_rate{0}, yaw{0}; };
struct GpsSample        { double stamp{0}, x{0}, y{0}, yaw{0}; };
struct SpeedSample      { double stamp{0}, v{0}; };
struct LidarSample      { double stamp{0}; av::PointCloud points; };
struct ObjectsSample    { double stamp{0}; av::DetectedObjectArray objects; };
struct TrajectorySample { double stamp{0}; av::Trajectory points; };
using  VehicleStateSample = av::VehicleState;
using  ControlSample      = av::ControlCommand;

inline std::string svc_key(const InstanceIdentifier& i, const char* s) { return i + "/" + s; }

// ---- SensorService : imu, gps, speed, lidar (provided by the gateway) ----
class SensorServiceSkeleton {
 public:
  explicit SensorServiceSkeleton(const InstanceIdentifier& i)
      : imu(svc_key(i, "SensorService/imu")), gps(svc_key(i, "SensorService/gps")),
        speed(svc_key(i, "SensorService/speed")), lidar(svc_key(i, "SensorService/lidar")) {}
  void OfferService() { imu.Offer(); gps.Offer(); speed.Offer(); lidar.Offer(); }
  void StopOfferService() { imu.StopOffer(); gps.StopOffer(); speed.StopOffer(); lidar.StopOffer(); }
  SkeletonEvent<ImuSample> imu; SkeletonEvent<GpsSample> gps;
  SkeletonEvent<SpeedSample> speed; SkeletonEvent<LidarSample> lidar;
};
class SensorServiceProxy {
 public:
  using HandleType = InstanceIdentifier;
  static ServiceHandleContainer<HandleType> FindService(const InstanceIdentifier& i) { return {i}; }
  explicit SensorServiceProxy(const HandleType& i)
      : imu(svc_key(i, "SensorService/imu")), gps(svc_key(i, "SensorService/gps")),
        speed(svc_key(i, "SensorService/speed")), lidar(svc_key(i, "SensorService/lidar")) {}
  ProxyEvent<ImuSample> imu; ProxyEvent<GpsSample> gps;
  ProxyEvent<SpeedSample> speed; ProxyEvent<LidarSample> lidar;
};

// single-event service definition
#define AV_DEFINE_SERVICE(NAME, EVT, TYPE)                                                  \
  class NAME##Skeleton {                                                                     \
   public:                                                                                   \
    explicit NAME##Skeleton(const InstanceIdentifier& i) : EVT(svc_key(i, #NAME "/" #EVT)) {}\
    void OfferService() { EVT.Offer(); }                                                     \
    void StopOfferService() { EVT.StopOffer(); }                                             \
    SkeletonEvent<TYPE> EVT;                                                                 \
  };                                                                                          \
  class NAME##Proxy {                                                                         \
   public:                                                                                    \
    using HandleType = InstanceIdentifier;                                                    \
    static ServiceHandleContainer<HandleType> FindService(const InstanceIdentifier& i) {      \
      return {i};                                                                             \
    }                                                                                         \
    explicit NAME##Proxy(const HandleType& i) : EVT(svc_key(i, #NAME "/" #EVT)) {}            \
    ProxyEvent<TYPE> EVT;                                                                      \
  };

AV_DEFINE_SERVICE(VehicleStateService, state,      VehicleStateSample)
AV_DEFINE_SERVICE(ObjectsService,      objects,    ObjectsSample)
AV_DEFINE_SERVICE(TrajectoryService,   trajectory, TrajectorySample)
AV_DEFINE_SERVICE(ControlService,      command,    ControlSample)

#undef AV_DEFINE_SERVICE

}  // namespace services
}  // namespace av
#endif  // AV_AP_AV_SERVICES_HPP
