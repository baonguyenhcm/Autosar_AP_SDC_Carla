// test_ap_integration.cpp — smoke test of the AP Functional-Cluster adapter layer
// (ara::exec / ara::log / ara::phm, host shim) driving the ROS-free ControllerCore.
//
// Host build:
//   g++ -std=c++17 -I src/av_ap/include -I src/av_common/include -I src/av_control/include \
//       src/av_ap/test/test_ap_integration.cpp src/av_control/src/controller_core.cpp \
//       -o /tmp/test_ap && /tmp/test_ap
#include <cassert>
#include <cstdio>

#include "av_ap/ap.hpp"
#include "av_control/controller_core.hpp"

int main() {
  av::ap::Initialize();
  av::ap::ExecutionClient exec;
  av::ap::Logger log("control", "control node");
  av::ap::SupervisedEntity se(/*se_id=*/1);

  exec.ReportRunning();                       // ara::exec
  log.Info("control FC-layer smoke test");    // ara::log

  av::ControllerCore core(av::ControlParams{});
  av::Trajectory traj;
  for (double x = 0; x <= 30; x += 1.0) traj.push_back({x, 0.0, 0.0, 8.0});

  av::VehicleState ego;   // at rest
  int cycles = 0;
  double last_throttle = 0.0;
  for (int i = 0; i < 20; ++i) {
    se.ReportCheckpoint(/*cp=*/10);           // ara::phm alive checkpoint
    av::ControlCommand c = core.computeCommand(ego, traj, 0.02);
    ego.v += (c.throttle * 2.0 - c.brake * 4.0) * 0.02;
    ego.x += ego.v * 0.02;
    last_throttle = c.throttle;
    ++cycles;
  }

  log.Info("cycles done");
  exec.ReportTerminating();
  av::ap::Deinitialize();

  assert(cycles == 20);
  assert(ego.v > 0.1);
  assert(last_throttle >= 0.0 && last_throttle <= 1.0);
  std::printf("PASS test_ap_integration: %d cycles, ego.v=%.3f m/s, throttle=%.3f\n",
              cycles, ego.v, last_throttle);
  return 0;
}
