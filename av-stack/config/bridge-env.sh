# config/bridge-env.sh — environment for the carla-ros-bridge side so its DDS
# interoperates with the Jetson Orin Nano over the LAN.
#   source this INSTEAD OF / IN ADDITION TO your usual env.sh, then launch the bridge.
#
# The three things that must match the Jetson gateway: ROS_DOMAIN_ID, the DDS vendor
# (CycloneDDS), and a CYCLONEDDS_URI that binds to the LAN interface + lists the peer.
#
# This host runs WSL2 in MIRRORED networking mode (default route `via 192.168.1.1
# dev eth2` on the real LAN 192.168.1.0/24). Consequences:
#   - Do NOT set ROS_LOCALHOST_ONLY=1 — it confines DDS to loopback and hides the
#     bridge topics from the Jetson. Leave it unset (or =0).
#   - CARLA runs on the Windows host and is reachable from WSL at 127.0.0.1 (mirrored),
#     so the carla-ros-bridge host arg can stay 127.0.0.1 (else use the Windows LAN IP).

# 1) ROS 2 + the bridge overlay (adjust paths to your install)
source /opt/ros/humble/setup.bash
source "$HOME/carla-ros-bridge/install/setup.bash"

# 2) SAME domain id as the Jetson (default 0). Do NOT enable ROS_LOCALHOST_ONLY.
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export CARLA_SIMULATOR_IP=127.0.0.1   # mirrored WSL2 → Windows host; else set Windows LAN IP

# 3) Use CycloneDDS (match the Jetson gateway). Needs ros-humble-rmw-cyclonedds-cpp.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 4) DDS config: LAN interface + explicit peer to the Jetson (reuse the repo file, or
#    the inline one below). Set BOTH ends to the same peers.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CYCLONEDDS_URI="file://$HERE/cyclonedds-peers.xml"

echo "[bridge-env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION"
echo "[bridge-env] CYCLONEDDS_URI=$CYCLONEDDS_URI"
echo "[bridge-env] edit cyclonedds-peers.xml so the <Peer> entries are the Jetson IP"
echo "             and this host's LAN IP, and open UDP 7400-7500 in the firewall."
