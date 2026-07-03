# av-stack (AUTOSAR Adaptive Platform)

A compact self-driving software stack whose four pipeline stages run as **AUTOSAR
Adaptive Platform (AP) Adaptive Applications** that communicate **service-orientedly
over `ara::com`**. It targets a **Jetson Orin Nano** driving **CARLA** (Windows host)
through the **lwrcl** bridge; `rclcpp`/lwrcl is used only at the CARLA boundary.

```
CARLA (Windows) ─ros2─ carla-ros-bridge (WSL2) ─DDS─▶ [ carla_gateway ]
                                                          │ ara::com (SOME/IP)
        SensorService ─▶ Localization ─▶ VehicleStateService ─┐
        SensorService.lidar + VehicleStateService ─▶ Perception ─▶ ObjectsService
                                          VehicleStateService + ObjectsService ─▶ Planning ─▶ TrajectoryService
                                          VehicleStateService + TrajectoryService ─▶ Control ─▶ ControlService
                                                          │
                                    ControlService ─▶ [ carla_gateway ] ─DDS─▶ CARLA vehicle_control_cmd
```

## Design

Each stage keeps a **ROS/middleware-free algorithm core** (`*_core.hpp/.cpp`) and adds
a thin AP layer:

- **Adaptive Application** (`*_app.hpp`) — a `ara::com` **Skeleton** for the service it
  provides and a **Proxy** for each service it consumes, delegating to the core.
- **`av_ap`** — Functional-Cluster adapters: `ara::exec` (lifecycle), `ara::log`,
  `ara::phm` (control-loop supervision), and the `ara::com` SOA primitives + service
  interfaces. System state is handled by the ara::sm FC (Machine States + Function Groups).
- **`carla_gateway`** — the single `rclcpp`/lwrcl node; bridges CARLA ROS 2 topics ⇄
  `ara::com` services ("extending `ara::com` with `rclcpp`").

**Bindings.** Internal AA↔AA exchange uses the **SOME/IP** binding (vsomeip),
selected by `ARA_COM_EVENT_BINDING=vsomeip` (service/event/eventgroup IDs + SD in the
Service Instance Manifest). The gateway uses the **DDS** side of lwrcl to reach the
CARLA bridge.

## Packages (`src/`)

| Package | Role |
|---|---|
| `av_common` | Shared plain types + `ap_main.hpp` run-loop helper |
| `av_ap` | `ara::com` SOA primitives, service interfaces, and FC adapters (exec/log/phm/sm) |
| `av_perception` / `av_localization` / `av_planning` / `av_control` | cores **+** Adaptive Applications |
| `av_gateway` | `rclcpp` ⇄ `ara::com` CARLA gateway |
| `av_integration_tests` | end-to-end SOA test |

## Manifests (`config/manifests/`)

- `av_service_interfaces.arxml` — the five service interfaces + payload types.
- `av_service_instances.arxml` — **Service Instance Manifest**: provided/required
  instances per app, bound to the SOME/IP binding (service IDs 0x1001–0x1006).
- `av_execution_manifest.arxml` — one Process per app; started by Execution Management
  in the `AvStack` function-group `Driving` state, in pipeline dependency order.
- `service_instance_mapping.yaml` — lwrcl-style binding/topic mapping (incl. the
  gateway's CARLA DDS topics).

## Build & test (host — no lwrcl/ROS needed)

The algorithm cores and the whole SOA wiring are dependency-free; a process-local
broker stands in for the SOME/IP binding so the pipeline is runnable on a plain host.

```bash
./build_ap.sh host          # cmake build + ctest
# or directly, without cmake:
g++ -std=c++17 -Isrc/av_ap/include -Isrc/av_common/include -Isrc/av_perception/include \
    -Isrc/av_localization/include -Isrc/av_planning/include -Isrc/av_control/include \
    src/av_integration_tests/test/test_soa_pipeline.cpp \
    src/av_perception/src/perception_core.cpp src/av_localization/src/ekf_localizer.cpp \
  src/av_planning/src/planner_core.cpp src/av_control/src/controller_core.cpp -o test_soa && ./test_soa
```

`test_soa_pipeline` wires all four Adaptive Applications through `ara::com` only and
checks the closed loop cruises, detects the obstacle, and **stops before it**.

## Build & run on the AP target (Jetson + lwrcl)

```bash
./build_ap.sh adaptive-autosar     # AAs on ara::com/SOME/IP; gateway also on lwrcl/CycloneDDS
# then, with CARLA + zenoh bridge running on the Windows/WSL2 host:
./run_ap.sh                         # routing manager + gateway + 4 Adaptive Applications
```

Requires the **AUTOSAR AP runtime** (`/opt/autosar-ap`, exporting `AdaptiveAutosarAP::ara_*`)
for the AAs, and the **lwrcl CycloneDDS backend** (`/opt/cyclonedds-libs`) plus `carla_msgs`
generated into lwrcl's cyclonedds `data_types` for the gateway's ROS 2 side. The gateway
talks ROS 2 DDS to `zenoh-bridge-ros2dds` (loopback, domain 0); see `config/cyclonedds-local.xml`.
The internal AAs use SOME/IP (vsomeip); they do not use lwrcl.

## Conformance status

The Adaptive Applications are written against the **standardized `ara::com` API**
(`FindService`, `Offer`/`StopOffer`, `SkeletonEvent::Send`, `ProxyEvent`
`Subscribe`/`SetReceiveHandler`/`GetNewSamples` with `SamplePtr`) and use
`ara::core::Result`, `ara::exec` (state reporting), `ara::phm` (checkpoints),
`ara::log`. System state (vehicle modes Startup/Driving/Parking/IgnitionOff + the
DrivingFG function group) is owned by the **ara::sm** Functional Cluster and enforced
by Execution Management via each app's ModeDependentStartupConfig — no invented SM API
and no custom state service. Under `AP_HAVE_ARA`
these resolve to the AP runtime's generated proxies/skeletons; on the host, a shim
in `ara_com.hpp` provides the identical API so the code compiles and runs (the SOA
loop is host-verified). Manifests (service interfaces, SOME/IP Service Instance,
Machine, Execution) are R24-11-aligned; process startup is EM/manifest-driven on the
target (`run_ap.sh` is only a stand-in for EM). This is conformant *usage* on the
non-certified lwrcl/Adaptive-AUTOSAR runtime — not a certified conformance claim.

## License

MIT — reference/teaching scaffold. Simplified algorithms; not for deployment on a real
vehicle.
