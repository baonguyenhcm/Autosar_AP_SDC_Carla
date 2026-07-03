// av_common/types.hpp
// ROS-free shared data types for the whole stack.
// Everything here is plain C++17 so the algorithm cores stay middleware-agnostic.
#pragma once

#include <cstdint>
#include <vector>
#include <string>

namespace av {

// ----------------------------------------------------------------------------
// Geometry
// ----------------------------------------------------------------------------
struct Point3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Pose2D {
  double x{0.0};    // [m]
  double y{0.0};    // [m]
  double yaw{0.0};  // [rad], heading in map frame
};

// ----------------------------------------------------------------------------
// Sensor data (inputs to the stack)
// ----------------------------------------------------------------------------
using PointCloud = std::vector<Point3>;  // e.g. a LiDAR scan in the ego frame

struct GpsMeasurement {
  double stamp{0.0};  // [s]
  double x{0.0};      // [m] map frame, noisy
  double y{0.0};      // [m] map frame, noisy
};

struct OdomMeasurement {
  double stamp{0.0};     // [s]
  double speed{0.0};     // [m/s]  wheel-speed derived
  double yaw_rate{0.0};  // [rad/s] IMU gyro z
};

// ----------------------------------------------------------------------------
// Perception output
// ----------------------------------------------------------------------------
struct DetectedObject {
  std::uint32_t id{0};
  Point3 centroid;      // object centre in map frame [m]
  double size_x{0.0};   // bounding-box extents [m]
  double size_y{0.0};
  double size_z{0.0};
  double yaw{0.0};      // orientation [rad]
  int num_points{0};    // supporting LiDAR points (a crude confidence proxy)
};

using DetectedObjectArray = std::vector<DetectedObject>;

// ----------------------------------------------------------------------------
// Localization output
// ----------------------------------------------------------------------------
struct VehicleState {
  double stamp{0.0};
  double x{0.0};    // [m]
  double y{0.0};    // [m]
  double yaw{0.0};  // [rad]
  double v{0.0};    // [m/s] longitudinal speed
};

// ----------------------------------------------------------------------------
// Planning output
// ----------------------------------------------------------------------------
struct TrajectoryPoint {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};  // target speed at this point [m/s]
};

using Trajectory = std::vector<TrajectoryPoint>;

// ----------------------------------------------------------------------------
// Control output
// ----------------------------------------------------------------------------
struct ControlCommand {
  double stamp{0.0};
  double throttle{0.0};  // [0..1]
  double brake{0.0};     // [0..1]
  double steer{0.0};     // [rad] front-wheel angle, +left
};

}  // namespace av
