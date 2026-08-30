#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! source "$ROOT/scripts/timing_profile.bash"; then
    exit 1
fi

LIDAR_IP="${LIDAR_IP:-192.168.1.201}"
LIDAR_PORT="${LIDAR_PORT:-2368}"
LIDAR_RPM="${LIDAR_RPM:-600.0}"
LIDAR_SCAN_MODE="${LIDAR_SCAN_MODE:-all_rings}"
LIDAR_SCAN_RING="${LIDAR_SCAN_RING:-7}"
PUBLISH_SCAN="${PUBLISH_SCAN:-true}"

source "$ROOT/scripts/env.bash"

set -euo pipefail

echo "Starting Velodyne VLP-16"
echo "Sensor IP: $LIDAR_IP"
echo "UDP port:  $LIDAR_PORT"
echo "Scan mode: $LIDAR_SCAN_MODE"
if [[ "$LIDAR_SCAN_MODE" == "single_ring" ]]; then
    echo "Scan ring: $LIDAR_SCAN_RING"
fi
echo "Timestamp source: host clock"
echo
echo "Edit config/vlp16.yaml for base_link -> velodyne extrinsics."

ros2 launch "$ROOT/launch/vlp16.launch.py" \
    device_ip:="$LIDAR_IP" \
    port:="$LIDAR_PORT" \
    rpm:="$LIDAR_RPM" \
    time_offset:="$VLP16_TIME_OFFSET" \
    timestamp_first_packet:="$VLP16_TIMESTAMP_FIRST_PACKET" \
    scan_mode:="$LIDAR_SCAN_MODE" \
    scan_ring:="$LIDAR_SCAN_RING" \
    publish_scan:="$PUBLISH_SCAN"
