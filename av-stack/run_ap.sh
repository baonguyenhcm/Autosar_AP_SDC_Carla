#!/usr/bin/env bash
# Launch the av-stack on the Jetson and bridge to CARLA.
# Prereqs: ./build_ap.sh adaptive-autosar ; CARLA (Windows) + autoware_carla_launch
# (zenoh_carla_bridge on WSL2, and zenoh-bridge-ros2dds running on THIS Jetson, domain 0).
#
# Transports:
#   * internal AA<->AA and gateway<->AA  = ara::com over SOME/IP (vsomeip).
#   * gateway<->CARLA (rclcpp/lwrcl side) = CycloneDDS on ROS_DOMAIN_ID=0 (loopback), which
#     zenoh-bridge-ros2dds republishes to/from the WSL zenoh bridge.
#
# NOTE: this script is a manual stand-in for Execution Management + State Management. On a
# full AP platform, EM starts these Processes from the Machine/Execution manifests as SM
# drives the DrivingFG function group. This script reproduces the mode-gated two-phase
# startup from config/manifests/av_execution_manifest.arxml:
#
#   DrivingFG=Init    -> launch the five self-terminating *_init Processes (--phase=init);
#                        each runs its ctor / one-time init, then exits (~2 s).
#   [barrier]         -> wait until ALL five *_init Processes have Terminated (this is the
#                        Init->Running transition SM performs; the manifest has no
#                        ExecutionDependency because constr_1689 forbids one across states).
#   DrivingFG=Running -> launch the five *_app Processes (--phase=run), supervised.
#
# safe_stop_app is NOT started here — it belongs to the DrivingFG=SafeStop state.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
BIN="${HERE}/build_ap"

APPS=(carla_gateway localization_app perception_app planning_app control_app)
# Short name per app -> used for the per-process vsomeip application names below.
SHORT=(gateway localization perception planning control)

# --- ara::com internal binding = SOME/IP (vsomeip) ---
export ARA_COM_EVENT_BINDING="${ARA_COM_EVENT_BINDING:-vsomeip}"
export VSOMEIP_CONFIGURATION="${VSOMEIP_CONFIGURATION:-${HERE}/config/vsomeip-av.json}"

# --- gateway's rclcpp/CARLA side = CycloneDDS, domain 0, loopback (see cyclonedds-local.xml) ---
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="file://${HERE}/config/cyclonedds-local.xml"
# NOTE: /opt/cyclonedds-libs must come BEFORE /opt/autosar-ap-libs: both ship a
# liblwrcl.so, and the gateway needs the CycloneDDS-backend one (the autosar-ap-libs
# copy is a different lwrcl backend with an incompatible ABI — loading it segfaults
# carla_gateway inside dds::pub::TPublisher).
export LD_LIBRARY_PATH="/opt/vsomeip/lib:/opt/autosar-ap/lib:/opt/cyclonedds-libs/lib:/opt/cyclonedds/lib:/opt/autosar-ap-libs/lib:/opt/iceoryx/lib:${LD_LIBRARY_PATH:-}"

# vsomeip routing manager (SOME/IP) for the local ara::com services — needed by both phases
# (the *_init Processes already register + offer their services).
if [ -x /opt/autosar-ap/bin/autosar_vsomeip_routing_manager ]; then
  /opt/autosar-ap/bin/autosar_vsomeip_routing_manager & RM=$!; sleep 1
fi

# Per-process vsomeip application naming. The Adaptive-AUTOSAR runtime reads
# ADAPTIVE_AUTOSAR_VSOMEIP_SERVER_APP / ADAPTIVE_AUTOSAR_VSOMEIP_CLIENT_APP (NOT
# VSOMEIP_APPLICATION_NAME) to name its skeleton/proxy vsomeip applications. Setting them to
# <short>_server / <short>_client gives each process the fixed client ids in vsomeip-av.json
# (servers 0x110x, clients 0x111x), instead of every process sharing the generic
# adaptive_autosar_{server,client} names -> dynamic-id collisions + ghost-client reconnect
# loops across the init->run transition.
vsomeip_env() {  # $1 = short app name; exports the two per-process app-name vars
  export ADAPTIVE_AUTOSAR_VSOMEIP_SERVER_APP="${1}_server"
  export ADAPTIVE_AUTOSAR_VSOMEIP_CLIENT_APP="${1}_client"
}

# ---- Phase 1: DrivingFG=Init — run the five self-terminating *_init Processes ----
# Run one --phase=init Process per app, each retried up to 3x (the stand-in for EM's
# numberOfRestartAttempts, covering localization's known vsomeip registration race).
run_init() {
  local short="$1" bin="$2" tries=0
  while true; do
    ( vsomeip_env "$short"; exec "$bin" --phase=init ) && return 0
    tries=$((tries+1)); [ "$tries" -ge 3 ] && return 1
    echo "[run_ap] $short --phase=init failed, retry $tries" >&2; sleep 0.5
  done
}

echo "[run_ap] DrivingFG=Init: launching init wave (${APPS[*]})"
declare -a IPIDS
for i in "${!APPS[@]}"; do
  run_init "${SHORT[$i]}" "${BIN}/${APPS[$i]}" & IPIDS+=($!)
done
trap 'kill "${IPIDS[@]}" ${RM:-} 2>/dev/null || true; pkill -x "${APPS[@]}" 2>/dev/null || true' INT TERM

# ---- Barrier: DrivingFG stays in Init until every init Process has Terminated ----
fail=0
for i in "${!IPIDS[@]}"; do
  if ! wait "${IPIDS[$i]}"; then echo "[run_ap] init Process failed: ${APPS[$i]}" >&2; fail=1; fi
done
if [ "$fail" -ne 0 ]; then
  echo "[run_ap] init barrier NOT satisfied — staying in DrivingFG=Init, not entering Running" >&2
  kill "${RM:-}" 2>/dev/null || true
  exit 1
fi
echo "[run_ap] all init Processes Terminated -> DrivingFG: Init -> Running"

# ---- Phase 2: DrivingFG=Running — run the five *_app Processes (supervised) ----
# supervise() restarts an app if it exits non-zero — the stand-in for Execution Management,
# which restarts failed processes. Once localization wins the vsomeip race it stays up.
supervise() {
  local short="$1"; local bin="$2"
  ( while true; do
      ( vsomeip_env "$short"; exec "$bin" --phase=run ); rc=$?
      [ $rc -eq 0 ] && break                       # clean exit (SIGINT/SIGTERM) -> stop
      echo "[run_ap] $short exited ($rc), restarting" >&2
      sleep 0.5
    done ) &
}

# Gateway first (offers SensorService), then the four AAs.
supervise gateway      "${BIN}/carla_gateway";    G=$!; sleep 0.5
supervise localization "${BIN}/localization_app"; L=$!; sleep 0.2
supervise perception   "${BIN}/perception_app";   P=$!; sleep 0.2
supervise planning     "${BIN}/planning_app";     N=$!; sleep 0.2
supervise control      "${BIN}/control_app";      C=$!

trap 'kill $G $L $P $N $C ${RM:-} 2>/dev/null; pkill -x "${APPS[@]}" 2>/dev/null || true' INT TERM
wait
