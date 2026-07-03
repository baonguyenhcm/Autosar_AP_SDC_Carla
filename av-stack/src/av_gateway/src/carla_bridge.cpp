// av_gateway/carla_bridge.cpp
// ara::com side of the gateway (compiled WITHOUT any ROS/CycloneDDS headers). Offers
// SensorService and consumes ControlService over ara::com / SOME/IP.
#include "av_gateway/carla_bridge.hpp"

#include "av_ap/ap.hpp"
#include "av_ap/av_services.hpp"

namespace av {

struct CarlaAraBridge::Impl {
  services::SensorServiceSkeleton sensors{"av"};
  services::ControlServiceProxy   control{
      services::ControlServiceProxy::FindService("av").front()};
  ap::ExecutionClient exec;
  ap::Logger log{"gateway"};
  std::function<void(const ControlCommand&)> cb;
};

CarlaAraBridge::CarlaAraBridge() {
  ap::Initialize();
  impl_ = std::make_unique<Impl>();
  impl_->sensors.OfferService();
  impl_->control.command.Subscribe(1);
  impl_->control.command.SetReceiveHandler([this] {
    impl_->control.command.GetNewSamples(
        [this](ara::com::SamplePtr<services::ControlSample> c) {
          if (impl_->cb) impl_->cb(*c);
        });
  });
  impl_->exec.ReportRunning();
  impl_->log.Info("carla_gateway (ara side): SensorService offered, ControlService subscribed");
}

CarlaAraBridge::~CarlaAraBridge() {
  if (impl_) impl_->exec.ReportTerminating();
  impl_.reset();
  ap::Deinitialize();
}

void CarlaAraBridge::pushImu(double s, double yr, double y) {
  impl_->sensors.imu.Send({s, yr, y});
}
void CarlaAraBridge::pushGps(double s, double x, double y, double yaw) {
  impl_->sensors.gps.Send({s, x, y, yaw});
}
void CarlaAraBridge::pushSpeed(double s, double v) {
  impl_->sensors.speed.Send({s, v});
}
void CarlaAraBridge::pushLidar(double s, const PointCloud& points) {
  services::LidarSample scan;
  scan.stamp = s;
  scan.points = points;
  impl_->sensors.lidar.Send(scan);
}
void CarlaAraBridge::setControlHandler(std::function<void(const ControlCommand&)> cb) {
  impl_->cb = std::move(cb);
}

}  // namespace av
