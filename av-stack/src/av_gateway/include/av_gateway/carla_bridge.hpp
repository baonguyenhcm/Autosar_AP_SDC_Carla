// av_gateway/carla_bridge.hpp
// The ara::com side of the CARLA gateway, behind a PLAIN-C++ interface (only av:: POD
// types — no ara::com, no DDS). This firewall keeps the rclcpp/CycloneDDS translation unit
// (carla_gateway_node.cpp) from ever including the ara::com headers, whose vsomeip stub
// `dds/` tree collides with real CycloneDDS `ddscxx`. The two sides are compiled separately
// and linked together.
#ifndef AV_GATEWAY_CARLA_BRIDGE_HPP
#define AV_GATEWAY_CARLA_BRIDGE_HPP

#include <functional>
#include <memory>

#include "av_common/types.hpp"

namespace av {

class CarlaAraBridge {
 public:
  CarlaAraBridge();   // ara::core::Initialize + offer SensorService + find ControlService
  ~CarlaAraBridge();

  // CARLA -> SensorService (ara::com events)
  void pushImu(double stamp, double yaw_rate, double yaw);
  void pushGps(double stamp, double x, double y, double yaw);
  void pushSpeed(double stamp, double v);
  void pushLidar(double stamp, const PointCloud& points);

  // ControlService (ara::com) -> caller (the rclcpp side publishes it to CARLA)
  void setControlHandler(std::function<void(const ControlCommand&)> cb);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace av
#endif  // AV_GATEWAY_CARLA_BRIDGE_HPP
