#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! source "$ROOT/scripts/timing_profile.bash"; then
    exit 1
fi

DB_DIR="$ROOT/data/database"
DB_PATH="$DB_DIR/rtabmap.db"
FPS="$CAMERA_FPS"
CAMERA_SERIAL="${CAMERA_SERIAL:-}"
START_SENSORS="${START_SENSORS:-1}"
START_LIDAR="${START_LIDAR:-1}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"

source "$ROOT/scripts/env.bash"

TORCH_LIB_DIR="$HOME/.local/lib/python3.12/site-packages/torch/lib"
if [[ -d "$TORCH_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$TORCH_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi

set -euo pipefail

TIMING_PROFILE="$TIMING_PROFILE" \
    bash "$ROOT/scripts/check_host_clock.bash"

if [[ "$FPS" != "30" && "$FPS" != "60" ]]; then
    echo "CAMERA_FPS must be 30 or 60."
    exit 1
fi

if [[ -n "$CAMERA_SERIAL" && ! "$CAMERA_SERIAL" =~ ^_?[0-9]+$ ]]; then
    echo "CAMERA_SERIAL must be empty or a numeric RealSense serial."
    exit 1
fi

if [[ "$START_SENSORS" != "0" && "$START_SENSORS" != "1" ]]; then
    echo "START_SENSORS must be 0 or 1."
    exit 1
fi

if [[ "$START_LIDAR" != "0" && "$START_LIDAR" != "1" ]]; then
    echo "START_LIDAR must be 0 or 1."
    exit 1
fi

if [[ "$XSENS_TIME_OPTION" != "0" && "$XSENS_TIME_OPTION" != "1" && "$XSENS_TIME_OPTION" != "2" ]]; then
    echo "XSENS_TIME_OPTION must be 0, 1 or 2."
    exit 1
fi

if [[ "$VLP16_GPS_TIME" != "false" ]]; then
    echo "All supported timing profiles require VLP16_GPS_TIME=false."
    exit 1
fi

if [[ ! "$REALSENSE_INTER_CAM_SYNC_MODE" =~ ^[0-3]$ ]]; then
    echo "REALSENSE_INTER_CAM_SYNC_MODE must be 0, 1, 2 or 3."
    exit 1
fi

if [[ "$REALSENSE_INTER_CAM_SYNC_MODE" != "0" && "$FPS" != "30" ]]; then
    echo "Dual-camera master/slave modes require CAMERA_FPS=30."
    exit 1
fi

if [[ "$REALSENSE_GLOBAL_TIME_ENABLED" != "true" &&
      "$REALSENSE_GLOBAL_TIME_ENABLED" != "false" ]]; then
    echo "REALSENSE_GLOBAL_TIME_ENABLED must be true or false."
    exit 1
fi

if [[ "$VLP16_TIMESTAMP_FIRST_PACKET" != "true" &&
      "$VLP16_TIMESTAMP_FIRST_PACKET" != "false" ]]; then
    echo "VLP16_TIMESTAMP_FIRST_PACKET must be true or false."
    exit 1
fi

# Preserve the legacy behavior: this stack owns odom -> base_link unless the
# caller explicitly selects an already verified external EKF.
CUSTOM_EKF="${USE_CUSTOM_EKF:-1}"

if [[ "$CUSTOM_EKF" != "0" && "$CUSTOM_EKF" != "1" ]]; then
    echo "USE_CUSTOM_EKF must be 0 or 1."
    exit 1
fi

if [[ "$CUSTOM_EKF" == "1" ]]; then
    CUSTOM_EKF_ARG=true
else
    CUSTOM_EKF_ARG=false
fi

if [[ "$START_SENSORS" == "1" ]]; then
    START_SENSORS_ARG=true
else
    START_SENSORS_ARG=false
fi

if [[ "$START_LIDAR" == "1" ]]; then
    START_LIDAR_ARG=true
else
    START_LIDAR_ARG=false
fi

mkdir -p "$DB_DIR" "$ROOT/data/maps"

if [[ -f "$DB_PATH" ]]; then
    BACKUP="$DB_DIR/rtabmap_backup_$(date +%Y%m%d_%H%M%S).db"
    echo "Backing up existing database to: $BACKUP"
    mv "$DB_PATH" "$BACKUP"
fi

echo "Starting mapping at matched RGB/depth ${FPS} Hz"
echo "Database: $DB_PATH"
echo "Triple sensor: camera + Xsens + VLP-16 ($SCAN_TOPIC)"
echo "Timing profile: $TIMING_PROFILE ($PROFILE_ROLE)"
echo "Timestamp sources: D455f mode $REALSENSE_INTER_CAM_SYNC_MODE, Xsens host time, VLP-16 host time"
echo "Check config/extrinsics.yaml before driving."

MAPPING_LAUNCH_ARGS=(
    "$ROOT/launch/mapping.launch.py"
    timing_profile:="$TIMING_PROFILE"
    camera_fps:="$FPS"
    use_custom_ekf:="$CUSTOM_EKF_ARG"
    start_sensors:="$START_SENSORS_ARG"
    xsens_time_option:="$XSENS_TIME_OPTION"
    realsense_inter_cam_sync_mode:="$REALSENSE_INTER_CAM_SYNC_MODE"
    realsense_global_time_enabled:="$REALSENSE_GLOBAL_TIME_ENABLED"
    start_lidar:="$START_LIDAR_ARG"
    scan_topic:="$SCAN_TOPIC"
    vlp16_time_offset:="$VLP16_TIME_OFFSET"
    vlp16_timestamp_first_packet:="$VLP16_TIMESTAMP_FIRST_PACKET"
    database_path:="$DB_PATH"
)
if [[ -n "$CAMERA_SERIAL" ]]; then
    MAPPING_LAUNCH_ARGS+=(serial_no:="$CAMERA_SERIAL")
fi

ROS_LOG_DIR=/tmp/roslogs ros2 launch "${MAPPING_LAUNCH_ARGS[@]}"
