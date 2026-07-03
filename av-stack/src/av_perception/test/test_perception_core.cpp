// Unit tests for av::PerceptionCore (grid Euclidean clustering).
#include <gtest/gtest.h>
#include "av_perception/perception_core.hpp"

using namespace av;

namespace {
// Build a dense box of points centred at (cx,cy) in the EGO frame, above ground.
PointCloud makeBox(double cx, double cy, double half = 0.75, double step = 0.25) {
  PointCloud c;
  for (double x = -half; x <= half; x += step)
    for (double y = -half; y <= half; y += step)
      c.push_back({cx + x, cy + y, 0.0});  // z=0 > default ground_z_max(-1.2)
  return c;
}
}  // namespace

TEST(PerceptionCore, EmptyCloudYieldsNoObjects) {
  PerceptionCore core;
  EXPECT_TRUE(core.detect({}, {0, 0, 0}).empty());
}

TEST(PerceptionCore, SingleClusterDetectedAndCounted) {
  PerceptionCore core;
  auto objs = core.detect(makeBox(10.0, 0.0), {0, 0, 0});
  ASSERT_EQ(objs.size(), 1u);
  EXPECT_GE(objs[0].num_points, 5);
  EXPECT_NEAR(objs[0].centroid.x, 10.0, 0.5);
  EXPECT_NEAR(objs[0].centroid.y, 0.0, 0.5);
}

TEST(PerceptionCore, TransformsEgoFrameToMapFrame) {
  // Ego at (5,5) rotated +90deg. A point 10m straight ahead in the ego frame
  // (+x) should map to +y in the map frame => (5, 15).
  PerceptionCore core;
  auto objs = core.detect(makeBox(10.0, 0.0), {5.0, 5.0, M_PI / 2.0});
  ASSERT_EQ(objs.size(), 1u);
  EXPECT_NEAR(objs[0].centroid.x, 5.0, 0.6);
  EXPECT_NEAR(objs[0].centroid.y, 15.0, 0.6);
}

TEST(PerceptionCore, GroundPointsAreFiltered) {
  PerceptionCore core;
  PointCloud ground;
  for (double x = 2.0; x < 20.0; x += 0.3) ground.push_back({x, 0.0, -1.5});  // below threshold
  EXPECT_TRUE(core.detect(ground, {0, 0, 0}).empty());
}

TEST(PerceptionCore, PointsBeyondRoiIgnored) {
  PerceptionParams p; p.roi_range = 30.0;
  PerceptionCore core(p);
  EXPECT_TRUE(core.detect(makeBox(100.0, 0.0), {0, 0, 0}).empty());  // 100m > ROI
}

TEST(PerceptionCore, TooFewPointsDiscarded) {
  PerceptionParams p; p.min_points = 20;
  PerceptionCore core(p);
  // A 3x3 grid box has 9 points < 20 -> discarded.
  EXPECT_TRUE(core.detect(makeBox(8.0, 0.0, 0.25, 0.25), {0, 0, 0}).empty());
}

TEST(PerceptionCore, TwoSeparatedClustersGiveTwoObjects) {
  PerceptionCore core;
  PointCloud c = makeBox(10.0, -8.0);
  PointCloud c2 = makeBox(10.0, 8.0);
  c.insert(c.end(), c2.begin(), c2.end());
  EXPECT_EQ(core.detect(c, {0, 0, 0}).size(), 2u);
}
