// av_perception/perception_core.hpp
// ROS-free perception core: grid-based Euclidean clustering of a LiDAR scan.
//
// Pipeline: ground removal (z-threshold) -> voxel grid occupancy ->
// connected-component clustering (8-neighbour) -> bounding boxes ->
// transform centroids from ego frame to map frame using the ego pose.
#pragma once

#include "av_common/types.hpp"

namespace av {

struct PerceptionParams {
  double ground_z_max{-1.2};   // points below this z (ego frame) are ground [m]
  double roi_range{60.0};      // ignore points beyond this radius [m]
  double cell_size{0.5};       // clustering grid resolution [m]
  int min_points{5};           // clusters with fewer points are discarded
  double min_z{-1.2};          // used for object height estimate
};

class PerceptionCore {
 public:
  explicit PerceptionCore(const PerceptionParams& p = {}) : params_(p) {}

  // cloud is in the ego (base_link) frame; ego is the current map-frame pose.
  DetectedObjectArray detect(const PointCloud& cloud, const Pose2D& ego) const;

 private:
  PerceptionParams params_;
};

}  // namespace av
