#!/usr/bin/env bash
# Build the av-stack for the AP target or for host testing.
#   ./build_ap.sh host              # cores + av_ap + host tests (no lwrcl)
#   ./build_ap.sh adaptive-autosar  # also build the AAs + gateway on lwrcl
set -euo pipefail

BACKEND="${1:-host}"
BUILD_DIR="build_ap"

case "$BACKEND" in
  host)
    cmake -S . -B "$BUILD_DIR" -DAP_BACKEND=off ;;
  adaptive-autosar)
    # /opt/autosar-ap = Adaptive-AUTOSAR runtime (exports AdaptiveAutosarAP:: ara_* targets);
    # /opt/cyclonedds-libs = lwrcl CycloneDDS backend (gateway's ROS 2 side). See lwrcl README.
    export CMAKE_PREFIX_PATH="/opt/autosar-ap:/opt/autosar-ap-libs:/opt/vsomeip:/opt/cyclonedds-libs:/opt/cyclonedds:${CMAKE_PREFIX_PATH:-}"
    cmake -S . -B "$BUILD_DIR" -DAP_BACKEND=adaptive-autosar ;;
  *)
    echo "usage: $0 [host|adaptive-autosar]"; exit 1 ;;
esac

cmake --build "$BUILD_DIR" -j"$(nproc)"
echo "--- ctest ---"
ctest --test-dir "$BUILD_DIR" --output-on-failure
