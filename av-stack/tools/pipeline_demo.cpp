// tools/pipeline_demo.cpp
// Standalone (no ROS) end-to-end demo of the av-stack pipeline.
//
// It wires the four algorithm cores together against a tiny kinematic-bicycle
// simulator and a synthetic LiDAR, so the whole
//   Perception -> Localization -> Planning -> Control
// data flow can be compiled and run anywhere with just a C++17 compiler.
//
// Scenario: a straight 200 m lane with a parked obstacle at x = 100 m.
// Expected behaviour: the ego accelerates to cruise speed, perceives the
// obstacle, and comes to a smooth stop a few metres before it.

#include "av_common/types.hpp"
#include "av_common/math.hpp"
#include "av_perception/perception_core.hpp"
#include "av_localization/ekf_localizer.hpp"
#include "av_planning/planner_core.hpp"
#include "av_control/controller_core.hpp"

#include <cstdio>
#include <fstream>
#include <random>
#include <cmath>

#ifdef AV_UDP
#include "udp_sender.hpp"
#include <thread>
#include <chrono>
#endif

using namespace av;

namespace {

// Ground-truth ego state used only by the simulator (the stack never sees it
// directly; it only gets noisy GPS + odometry).
struct TrueEgo {
  double x{0.0}, y{0.0}, yaw{0.0}, v{0.0};
};

// Kinematic bicycle model integration. Returns the true yaw-rate (what an
// ideal IMU gyro would report) via yaw_rate_out.
void stepVehicle(TrueEgo& e, const ControlCommand& cmd, double dt,
                 double& yaw_rate_out) {
  const double L = 2.7;
  const double sim_max_accel = 2.0;
  const double sim_max_decel = 4.0;
  const double accel = cmd.throttle * sim_max_accel - cmd.brake * sim_max_decel;
  e.v = std::max(0.0, e.v + accel * dt);
  const double yaw_rate = (e.v / L) * std::tan(cmd.steer);
  e.yaw = normalizeAngle(e.yaw + yaw_rate * dt);
  e.x += e.v * std::cos(e.yaw) * dt;
  e.y += e.v * std::sin(e.yaw) * dt;
  yaw_rate_out = yaw_rate;
}

// Synthetic LiDAR: return points (ego frame) hitting a 2x2x1.5 m box obstacle
// plus some ground points (to exercise the ground filter).
PointCloud simulateLidar(const TrueEgo& e, const Point3& obst) {
  PointCloud cloud;
  const double c = std::cos(-e.yaw), s = std::sin(-e.yaw);
  // Obstacle surface samples in world frame.
  for (double ox = -1.0; ox <= 1.0; ox += 0.25) {
    for (double oy = -1.0; oy <= 1.0; oy += 0.25) {
      for (double oz = -1.0; oz <= 0.5; oz += 0.5) {
        const double wx = obst.x + ox;
        const double wy = obst.y + oy;
        // world -> ego frame
        const double rx = wx - e.x, ry = wy - e.y;
        Point3 p;
        p.x = rx * c - ry * s;
        p.y = rx * s + ry * c;
        p.z = oz;
        cloud.push_back(p);
      }
    }
  }
  // Some ground points around the ego (z below the ground threshold).
  for (double gx = 2.0; gx <= 30.0; gx += 4.0) {
    Point3 g;
    g.x = gx;
    g.y = 0.0;
    g.z = -1.5;  // below ground_z_max, must be filtered out
    cloud.push_back(g);
  }
  return cloud;
}

}  // namespace

int main() {
  // --- Build a straight lane centerline as the global reference path. ---
  std::vector<Point3> centerline;
  for (double x = 0.0; x <= 200.0; x += 1.0) centerline.push_back({x, 0.0, 0.0});
  const Point3 obstacle{100.0, 0.0, 0.0};

  // --- Instantiate the four cores. ---
  PerceptionCore perception;
  EkfLocalizer ekf;
  PlannerCore planner;
  planner.setGlobalPath(centerline);
  ControllerCore controller;

  // --- Simulator state + noise. ---
  TrueEgo ego;
  std::mt19937 rng(42);
  std::normal_distribution<double> gps_noise(0.0, 0.3);
  std::normal_distribution<double> spd_noise(0.0, 0.1);
  std::normal_distribution<double> yaw_noise(0.0, 0.02);

  const double dt = 0.1;
  const double t_end = 40.0;
  double last_gps = -1.0;
  double imu_yaw_rate = 0.0;  // fed by the simulator each tick (IMU gyro)

  ekf.initialize(0.0, 0.0, 0.0, 0.0, 0.0);

  std::printf("  t     true_x  est_x   v(m/s)  obj_dist  thr  brk  steer  n_obj\n");
  std::printf("-----------------------------------------------------------------\n");

  // --- CSV trace for the 2D viewer (viz/av_viewer.html). ---
  // Written once per simulation step so the animation is smooth (dt = 0.1 s).
  std::ofstream csv("av_trace.csv");
  // Scenario constants as comment lines the viewer parses for the static scene.
  csv << "# lane," << centerline.front().x << ',' << centerline.front().y << ','
      << centerline.back().x << ',' << centerline.back().y << '\n';
  csv << "# obstacle," << obstacle.x << ',' << obstacle.y << ",2.0,2.0\n";
  csv << "t,true_x,true_y,true_yaw,est_x,est_y,est_yaw,v,throttle,brake,steer,"
         "n_obj,obj_x,obj_y\n";

#ifdef AV_UDP
  // Optional live stream to an Unreal viewer (build with -DAV_UDP=ON).
  UdpSender udp("127.0.0.1", 9999);
  {
    char h[128];
    std::snprintf(h, sizeof(h), "# lane,%g,%g,%g,%g\n", centerline.front().x,
                  centerline.front().y, centerline.back().x, centerline.back().y);
    udp.send(h);
    std::snprintf(h, sizeof(h), "# obstacle,%g,%g,2.0,2.0\n", obstacle.x, obstacle.y);
    udp.send(h);
  }
#endif

  for (double t = 0.0; t <= t_end + 1e-9; t += dt) {
    // 1. PERCEPTION -----------------------------------------------------------
    PointCloud cloud = simulateLidar(ego, obstacle);
    Pose2D true_pose{ego.x, ego.y, ego.yaw};
    // In a real stack perception uses the *estimated* pose; using the true pose
    // here isolates the demo from localization error for clarity.
    DetectedObjectArray objects = perception.detect(cloud, true_pose);

    // 2. LOCALIZATION ---------------------------------------------------------
    // Predict with the IMU gyro; correct with wheel speed + GNSS/INS heading
    // every tick, and GPS position at 2 Hz.
    ekf.predict(dt, imu_yaw_rate + yaw_noise(rng) * 0.1);
    ekf.updateSpeed(ego.v + spd_noise(rng));
    ekf.updateYaw(normalizeAngle(ego.yaw + yaw_noise(rng)));
    if (t - last_gps >= 0.5) {
      GpsMeasurement g;
      g.stamp = t;
      g.x = ego.x + gps_noise(rng);
      g.y = ego.y + gps_noise(rng);
      ekf.updateGps(g);
      last_gps = t;
    }
    ekf.setStamp(t);
    VehicleState est = ekf.state();

    // 3. PLANNING -------------------------------------------------------------
    Trajectory traj = planner.plan(est, objects);

    // 4. CONTROL --------------------------------------------------------------
    ControlCommand cmd = controller.computeCommand(est, traj, dt);

    // 5. SIMULATE VEHICLE -----------------------------------------------------
    stepVehicle(ego, cmd, dt, imu_yaw_rate);

    // --- Full-rate CSV row for the viewer. ---
    {
      char ob[48] = "";
      if (!objects.empty()) {
        std::snprintf(ob, sizeof(ob), "%.3f,%.3f",
                      objects.front().centroid.x, objects.front().centroid.y);
      } else {
        std::snprintf(ob, sizeof(ob), ",");  // empty obj_x,obj_y
      }
      char row[256];
      std::snprintf(row, sizeof(row),
                    "%.2f,%.3f,%.3f,%.4f,%.3f,%.3f,%.4f,%.3f,%.3f,%.3f,%.4f,%zu,%s\n",
                    t, ego.x, ego.y, ego.yaw, est.x, est.y, est.yaw, est.v,
                    cmd.throttle, cmd.brake, cmd.steer, objects.size(), ob);
      csv << row;
#ifdef AV_UDP
      udp.send(row);
      // Pace the loop at ~real time so the live view is watchable.
      std::this_thread::sleep_for(std::chrono::duration<double>(dt));
#endif
    }

    // --- Log once per second. ---
    if (std::fabs(t - std::round(t)) < 1e-6) {
      const double obj_dist =
          objects.empty()
              ? -1.0
              : dist2D(est.x, est.y, objects.front().centroid.x,
                       objects.front().centroid.y);
      std::printf("%5.1f  %6.2f  %6.2f  %6.2f  %8.2f  %.2f %.2f  %+.3f   %zu\n",
                  t, ego.x, est.x, est.v, obj_dist, cmd.throttle, cmd.brake,
                  cmd.steer, objects.size());
    }
  }

  csv.close();
  std::printf("\nWrote trace to av_trace.csv (open it in viz/av_viewer.html).\n");
  std::printf("\nFinal true ego x = %.2f m (obstacle at %.1f m, stop margin 6 m)\n",
              ego.x, obstacle.x);
  if (ego.x > 88.0 && ego.x < 96.0 && ego.v < 0.5) {
    std::printf("RESULT: PASS - ego stopped safely before the obstacle.\n");
    return 0;
  }
  std::printf("RESULT: CHECK - ego final state outside expected stop window.\n");
  return 1;
}
