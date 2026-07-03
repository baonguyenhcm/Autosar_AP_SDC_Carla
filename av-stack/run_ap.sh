#!/usr/bin/env bash
# Launch the av-stack Adaptive Applications on the Jetson and bridge to CARLA.
# Prereqs: ./build_ap.sh adaptive-autosar ; CARLA (Windows) + autoware_carla_launch
# (zenoh_carla_bridge on WSL2, and zenoh-bridge-ros2dds running on THIS Jetson, domain 0).
#
# NOTE: this script is a STAND-IN for Execution Management. On a full platform,
# EM starts these processes from config/manifests/av_execution_manifest.arxml when the
# AvStack function group enters "Driving" — no shell launching. Use this only for a
# manual bring-up on a machine without a running EM daemon.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
BIN="${HERE}/build_ap"

# The carla_gateway is a DUAL-STACK process:
#   * ara::com service side (SensorService/ControlService <-> the AAs) = SOME/IP (vsomeip)
#   * rclcpp/lwrcl side (CARLA topics <-> the LOCAL zenoh-bridge-ros2dds)  = CycloneDDS
# They are independent stacks in one binary, so there is NO per-process binding switch:
# keep ARA_COM_EVENT_BINDING=vsomeip for everyone (setting the gateway to 'dds' would
# break its service link to the AAs). The gateway's CARLA/DDS side is governed by the
# ROS 2 RMW vars below, NOT by ARA_COM_EVENT_BINDING.

# --- internal ara::com (all AAs + the gateway's service side) = SOME/IP ---
export ARA_COM_EVENT_BINDING="${ARA_COM_EVENT_BINDING:-vsomeip}"
export VSOMEIP_CONFIGURATION="${VSOMEIP_CONFIGURATION:-${HERE}/config/vsomeip-av.json}"

# --- gateway's rclcpp side = LOCAL CycloneDDS to zenoh-bridge-ros2dds (domain 0) ---
# ZENOH topology: the machine-to-machine hop is carried by Zenoh (WSL zenoh_carla_bridge
# <-> Jetson zenoh-bridge-ros2dds). DDS stays LOCAL on the Jetson — zenoh-bridge-ros2dds
# republishes onto CycloneDDS domain 0 on loopback, and the gateway subscribes there.
# cyclonedds-local.xml binds the loopback interface (localhost-only, no LAN peers). Do NOT
# also set ROS_LOCALHOST_ONLY — it can conflict with an explicit CYCLONEDDS_URI.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"  # same RMW as zenoh-bridge-ros2dds
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"                            # same domain as `-d 0`
export CYCLONEDDS_URI="file://${HERE}/config/cyclonedds-local.xml"     # loopback discovery, domain 0
# ALTERNATIVE pure-DDS-over-LAN topology (no Zenoh): point CYCLONEDDS_URI at
# cyclonedds-peers.xml (real IPs) and set ROS_LOCALHOST_ONLY=0 instead.
export LD_LIBRARY_PATH="/opt/vsomeip/lib:/opt/cyclonedds/lib:/opt/autosar-ap-libs/lib:${LD_LIBRARY_PATH:-}"

# vsomeip routing manager (SOME/IP) for the local services.
if [ -x /opt/autosar-ap/bin/autosar_vsomeip_routing_manager ]; then
  /opt/autosar-ap/bin/autosar_vsomeip_routing_manager & RM=$!; sleep 1
fi

# Manual bring-up = machine mode 'Driving' + DrivingFG 'Active' (EM does this from
# the Machine/Execution Manifests on the target). safe_stop_app is NOT started here: it
# runs only in DrivingFG 'SafeStop', which State Management enters on a PHM fault.
"${BIN}/carla_gateway"    & G=$!;  sleep 0.5
"${BIN}/localization_app" & L=$!;  sleep 0.2
"${BIN}/perception_app"   & P=$!;  sleep 0.2
"${BIN}/planning_app"     & N=$!;  sleep 0.2
"${BIN}/control_app"      & C=$!

trap 'kill $G $L $P $N $C ${RM:-} 2>/dev/null || true' INT TERM
wait
