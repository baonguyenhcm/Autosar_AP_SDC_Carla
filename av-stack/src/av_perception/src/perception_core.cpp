// av_perception/perception_core.cpp
#include "av_perception/perception_core.hpp"
#include "av_common/math.hpp"

#include <cmath>
#include <map>
#include <queue>
#include <limits>

namespace av {

namespace {
struct Cell {
  int gx;
  int gy;
};
}  // namespace

DetectedObjectArray PerceptionCore::detect(const PointCloud& cloud,
                                           const Pose2D& ego) const {
  // 1. Filter: drop ground and out-of-ROI points, then bin into a 2D grid.
  //    key = (gx,gy) -> list of point indices.
  std::map<std::pair<int, int>, std::vector<int>> grid;
  for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {
    const Point3& p = cloud[i];
    if (p.z < params_.ground_z_max) continue;                  // ground
    const double r = std::sqrt(p.x * p.x + p.y * p.y);
    if (r > params_.roi_range) continue;                       // out of ROI
    const int gx = static_cast<int>(std::floor(p.x / params_.cell_size));
    const int gy = static_cast<int>(std::floor(p.y / params_.cell_size));
    grid[{gx, gy}].push_back(i);
  }

  // 2. Connected-component labelling over occupied cells (8-connectivity).
  std::map<std::pair<int, int>, bool> visited;
  DetectedObjectArray objects;
  std::uint32_t next_id = 0;

  for (const auto& kv : grid) {
    if (visited[kv.first]) continue;

    // BFS flood fill to collect all cells of this cluster.
    std::vector<int> member_points;
    std::queue<Cell> q;
    q.push({kv.first.first, kv.first.second});
    visited[kv.first] = true;

    while (!q.empty()) {
      const Cell c = q.front();
      q.pop();
      auto it = grid.find({c.gx, c.gy});
      if (it != grid.end()) {
        member_points.insert(member_points.end(), it->second.begin(),
                             it->second.end());
      }
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          std::pair<int, int> nb{c.gx + dx, c.gy + dy};
          if (grid.count(nb) && !visited[nb]) {
            visited[nb] = true;
            q.push({nb.first, nb.second});
          }
        }
      }
    }

    if (static_cast<int>(member_points.size()) < params_.min_points) continue;

    // 3. Bounding box from member points (ego frame).
    double minx = std::numeric_limits<double>::max(), maxx = -minx;
    double miny = std::numeric_limits<double>::max(), maxy = -miny;
    double maxz = -std::numeric_limits<double>::max();
    double sx = 0, sy = 0;
    for (int idx : member_points) {
      const Point3& p = cloud[idx];
      minx = std::min(minx, p.x);
      maxx = std::max(maxx, p.x);
      miny = std::min(miny, p.y);
      maxy = std::max(maxy, p.y);
      maxz = std::max(maxz, p.z);
      sx += p.x;
      sy += p.y;
    }
    const double cx = sx / member_points.size();  // centroid, ego frame
    const double cy = sy / member_points.size();

    // 4. Transform centroid ego->map: rotate by yaw, translate by ego position.
    const double cyaw = std::cos(ego.yaw);
    const double syaw = std::sin(ego.yaw);
    DetectedObject obj;
    obj.id = next_id++;
    obj.centroid.x = ego.x + cx * cyaw - cy * syaw;
    obj.centroid.y = ego.y + cx * syaw + cy * cyaw;
    obj.centroid.z = 0.0;
    obj.size_x = std::max(0.1, maxx - minx);
    obj.size_y = std::max(0.1, maxy - miny);
    obj.size_z = std::max(0.1, maxz - params_.min_z);
    obj.yaw = ego.yaw;
    obj.num_points = static_cast<int>(member_points.size());
    objects.push_back(obj);
  }

  return objects;
}

}  // namespace av
