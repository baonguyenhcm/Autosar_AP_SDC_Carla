# av-stack — Build & Run Guide

Two paths:

- **Path A — Host verification** (no hardware, ~2 min): build the ROS-free cores + the
  `ara::com` SOA wiring and run the closed-loop test. This is what proves the logic.
- **Path B — Full demo** on a Jetson Orin Nano driving CARLA (Windows) via lwrcl.

> Status: Path A is verified. Path B is the correct procedure but depends on the
> lwrcl / AUTOSAR-AP toolchain + CARLA hardware; validate each step on your machines.

---

## Path A — Host verification (Linux/WSL2, no lwrcl, no ROS)

```bash
cd git_repos/av-stack
./build_ap.sh host          # cmake build + ctest  → "100% tests passed"
# or direct g++:
g++ -std=c++17 -O2 -Isrc/av_ap/include -Isrc/av_common/include \
  -Isrc/av_perception/include -Isrc/av_localization/include \
  -Isrc/av_planning/include -Isrc/av_control/include \
  src/av_integration_tests/test/test_soa_pipeline.cpp src/av_perception/src/perception_core.cpp src/av_localization/src/ekf_localizer.cpp \
  src/av_planning/src/planner_core.cpp src/av_control/src/controller_core.cpp \
  -o test_soa && ./test_soa
```
Expected:
```
final: x=96.62 m, v=0.000 m/s, detections=331
PASS test_soa_pipeline: SOA loop over ara::com (DrivingFG=Active) stopped 3.4 m before obstacle
```
The four Adaptive Applications are wired through `ara::com` (an in-process broker stands
in for the SOME/IP binding); `control_app` represents the `DrivingFG=Active` state.

---

## Path B — Jetson Orin Nano ⇄ CARLA (Windows) via lwrcl

Topology (two transports):
```
CARLA 0.9.14 (Windows) ─RPC─ carla-ros-bridge (ROS2, WSL2) ══DDS══ [ carla_gateway ]
                                                                       │ ara::com / SOME/IP (vsomeip)
                              Jetson Orin Nano:  gateway ⇄ localization ⇄ perception ⇄ planning ⇄ control
```
- **Internal AA↔AA** = `ara::com` over **SOME/IP (vsomeip)**, `ARA_COM_EVENT_BINDING=vsomeip`.
- **Gateway ⇄ CARLA bridge** = **DDS** (the gateway's rclcpp/lwrcl side over CycloneDDS).

### Part 1 — Windows host: CARLA
1. Launch **CARLA 0.9.14**: `CarlaUE4.exe` (add `-quality-level=Low` to spare the Orin).
2. Note the Windows LAN IP (`ipconfig`), e.g. `192.168.1.10`.

### Part 2 — WSL2: carla-ros-bridge
```bash
source /opt/ros/humble/setup.bash
source ~/carla-ros-bridge/install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py \
  host:=<WINDOWS_HOST_IP> town:=Town03 \
  objects_definition_file:=<path>/av-stack/config/carla/objects.json
ros2 topic list | grep /carla/ego_vehicle    # imu / odometry / lidar / vehicle_control_cmd
```
The bundled `config/carla/objects.json` spawns the ego vehicle with the exact sensor ids
the gateway expects (`imu`, `odometry`, `lidar`).

**Bridge environment (cross-LAN DDS).** The bridge must match the Jetson gateway on
DDS: `source config/bridge-env.sh` (or add to your `env.sh`) — it sets `ROS_DOMAIN_ID`
(same as the Jetson), `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and `CYCLONEDDS_URI`
(`config/cyclonedds-peers.xml`, LAN interface + Jetson peer). Install
`ros-humble-rmw-cyclonedds-cpp` first, and open UDP 7400-7500 in the firewall.

### Part 3 — Networking: DDS discovery WSL2 ⇄ Jetson (gateway ⇄ CARLA link)
WSL2's default NAT blocks DDS multicast, so use **unicast peers** (this covers the
gateway's DDS side only; the internal SOME/IP traffic stays on the Jetson):
1. Prefer WSL **mirrored networking** — `%UserProfile%\.wslconfig`: `[wsl2]` /
   `networkingMode=mirrored`, then `wsl --shutdown`.
2. Edit `config/cyclonedds-peers.xml` → set the two `<Peer address=".."/>` to your Jetson
   and Windows/WSL2 IPs; keep `<AllowMulticast>false</AllowMulticast>`.
3. `export CYCLONEDDS_URI=file:///…/config/cyclonedds-peers.xml` on **both** sides.
4. Open the CycloneDDS UDP ports (7400–7500) in the Windows firewall.

### Part 4 — Jetson Orin Nano: build runtime + stack (aarch64 / JetPack)
1. **CycloneDDS** (gateway's DDS transport to CARLA):
   ```bash
   cd ~/lwrcl && ./scripts/install_cyclonedds.sh && source ~/.bashrc
   ```
2. **vsomeip** (SOME/IP runtime for the internal `ara::com`):
   ```bash
   cd ~/lwrcl && ./scripts/install_vsomeip.sh      # installs to /opt/vsomeip
   ```
3. **AUTOSAR-AP runtime + codegen** — build Adaptive-AUTOSAR and install the runtime +
   `ara_com_codegen` tools to `/opt/autosar-ap` (adds `autosar-generate-comm-manifest`,
   `autosar-generate-proxy-skeleton` to PATH):
   ```bash
   cd ~/Adaptive-AUTOSAR && cmake -DCMAKE_BUILD_TYPE=Release -S . -B build && cmake --build build
   # then install per the project's install step
   ```
4. **lwrcl (CycloneDDS backend — for the gateway's ROS 2 side only)**:
   ```bash
   cd ~/lwrcl
   ./build_libraries.sh  cyclonedds install
   ./build_data_types.sh cyclonedds install   # ROS 2 msg types: sensor_msgs, nav_msgs, ...
   ./build_lwrcl.sh      cyclonedds install    # -> /opt/cyclonedds-libs
   ```
   > The gateway speaks **ROS 2 DDS** to `zenoh-bridge-ros2dds`, so it needs the
   > **cyclonedds** lwrcl (ROS 2 wire-compatible). Do **not** use the `adaptive-autosar`
   > lwrcl for the gateway — its ara::com/AUTOSAR topic mapping won't interoperate with the
   > ROS 2 bridge. The four Adaptive Applications don't use lwrcl at all: they link
   > Adaptive-AUTOSAR's `ara::com` (SOME/IP) from step 3 (`AdaptiveAutosarAP::ara_*`).
5. **carla_msgs IDL** — add `carla_msgs` (+`ackermann_msgs` if used) to lwrcl's
   **cyclonedds** `data_types` and regenerate, so the gateway can (de)serialize
   `CarlaEgoVehicleControl` / `CarlaEgoVehicleStatus` (not in the stock ROS set).
6. **Build av-stack**:
   ```bash
   cd ~/av-stack && ./build_ap.sh adaptive-autosar
   ```
   Builds `localization_app perception_app planning_app control_app safe_stop_app
   carla_gateway`. The AAs use the hand-written manifests in `config/manifests/`
   (their `ara::com` layer isn't scanned by lwrcl's auto-generator).

   **Integration note (gateway transport):** the gateway is **dual-stack** — its
   CARLA-facing **rclcpp** topics ride **CycloneDDS** (set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
   `ROS_DOMAIN_ID`, and `CYCLONEDDS_URI` to match the bridge), while its ara::com service
   side and the internal apps stay on **vsomeip**. These are independent stacks in one
   binary, so do **NOT** set `ARA_COM_EVENT_BINDING=dds` for the gateway — that would break
   its service link to the AAs. See `run_ap.sh` env.

### Part 5 — Run (machine mode Driving + DrivingFG Active)
```bash
cd ~/av-stack
export ARA_COM_EVENT_BINDING=vsomeip                            # internal AAs + gateway service side
export VSOMEIP_CONFIGURATION=$PWD/config/vsomeip-av.json
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp                    # gateway rclcpp side (match bridge)
export ROS_DOMAIN_ID=0                                          # gateway rclcpp side (match bridge)
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds-peers.xml    # gateway ⇄ CARLA over LAN
./run_ap.sh
```
`run_ap.sh` starts the vsomeip **routing manager**, then the gateway + the four driving
apps — the manual stand-in for **Execution Management** in machine mode `Driving` with
`DrivingFG = Active` (on a full platform, EM starts these from the Machine/Execution
Manifests). `safe_stop_app` is **not** started here — it runs only in `DrivingFG=SafeStop`.

### Part 6 — Verify
- Ego vehicle drives the lane in CARLA and **stops before the obstacle**, then resumes
  (UC-2 / UC-3, AC-01 / AC-02).
- On WSL2: `ros2 topic echo /carla/ego_vehicle/vehicle_control_cmd`.
- **Safe-stop (UC-4)** is an EM/SM behavior: a PHM supervision failure makes State
  Management switch `DrivingFG Active → SafeStop`; Execution Management stops `control_app`
  and starts `safe_stop_app` (which commands brake). To exercise it without a real fault,
  request the `SafeStop` function-group state via the platform's SM tooling.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Gateway sees no CARLA topics | WSL2 NAT blocks DDS discovery | mirrored networking + unicast `<Peers>` + firewall (Part 3) |
| Gateway ↔ CARLA silent, apps idle | gateway's rclcpp RMW/domain not matching the bridge | set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID`, `CYCLONEDDS_URI` (Part 4 note) |
| Internal apps don't discover each other | no vsomeip routing manager | start it (run_ap.sh) / check `VSOMEIP_CONFIGURATION` |
| Gateway can't (de)serialize control | `carla_msgs` IDL missing in lwrcl | add carla_msgs to `data_types` (Part 4.5) |
| Subscriptions never match | QoS mismatch with bridge | `SensorDataQoS` for camera/lidar; reliable for control |
| Car twitches / wrong steer | steer unit mismatch | gateway converts rad→normalized via `max_steer_rad` |
| False safe-stops | clock skew WSL2↔Jetson | common time base; widen freshness threshold |

## See also
`README.md` (architecture) and the wiki pages `ap-sdc-implementation`,
`arc42-deployment-view`, `ap-sdc-constraints-assumptions`.
