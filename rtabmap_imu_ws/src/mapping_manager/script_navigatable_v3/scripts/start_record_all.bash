#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! source "$ROOT/scripts/timing_profile.bash"; then
    exit 1
fi

BAG_DIR="${BAG_DIR:-$ROOT/data/rosbags}"
TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"
SESSION_ID="${SESSION_ID:-session_$TIMESTAMP}"
CAMERA_NAME="${CAMERA_NAME:-camera_a}"
CAMERA_SERIAL="${CAMERA_SERIAL:-unknown}"
BAG_NAME="${BAG_NAME:-${SESSION_ID}_laptop_a}"
OUTPUT="$BAG_DIR/$BAG_NAME"
RECORD_ALL="${RECORD_ALL_TOPICS:-0}"
STATE_DIR="$BAG_DIR/.v3_recorder"
PID_FILE="$STATE_DIR/pid"
OUTPUT_FILE="$STATE_DIR/output"

source "$ROOT/scripts/env.bash"


set -euo pipefail

TIMING_PROFILE="$TIMING_PROFILE" \
    bash "$ROOT/scripts/check_host_clock.bash"

camera_parameter_value() {
    local parameter="$1"
    local output
    output="$(
        timeout 5 ros2 param get /camera/camera "$parameter" 2>/dev/null
    )" || {
        echo "ERROR: Camera A parameter is unavailable: $parameter" >&2
        echo "Start the validated camera pipeline before recording." >&2
        return 1
    }
    printf '%s\n' "${output##*: }"
}

require_camera_parameter() {
    local parameter="$1"
    local expected="$2"
    local actual
    actual="$(camera_parameter_value "$parameter")" || return 1
    if [[ "$actual" != "$expected" ]]; then
        echo "ERROR: Camera A parameter mismatch: $parameter" >&2
        echo "  expected=$expected actual=$actual" >&2
        return 1
    fi
}

require_camera_parameter \
    depth_module.inter_cam_sync_mode "$REALSENSE_INTER_CAM_SYNC_MODE"
require_camera_parameter depth_module.global_time_enabled True
require_camera_parameter rgb_camera.global_time_enabled True

mkdir -p "$BAG_DIR" "$STATE_DIR"

if [[ "$RECORD_ALL" != "0" && "$RECORD_ALL" != "1" ]]; then
    echo "RECORD_ALL_TOPICS must be 0 or 1."
    exit 1
fi

if [[ ! "$SESSION_ID" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "SESSION_ID may contain only letters, numbers, dot, underscore and dash."
    exit 1
fi

if [[ -f "$PID_FILE" ]]; then
    OLD_PID="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [[ "$OLD_PID" =~ ^[0-9]+$ ]] && kill -0 "$OLD_PID" 2>/dev/null; then
        echo "A V3 rosbag recorder is already running with PID $OLD_PID."
        exit 1
    fi
    rm -f "$PID_FILE" "$OUTPUT_FILE"
fi

if [[ -e "$OUTPUT" ]]; then
    echo "Rosbag output already exists: $OUTPUT"
    exit 1
fi

COMMON_ARGS=(
    --storage sqlite3
    --output "$OUTPUT"
    --max-bag-size 4294967296
    --max-cache-size 536870912
    --polling-interval 100
    --node-name v3_rosbag_recorder
    --disable-keyboard-controls
)

CUSTOM_DATA=(
    --custom-data
    "pipeline=script_navigatable_v3"
    "session_id=$SESSION_ID"
    "host_role=laptop_a"
    "camera_name=$CAMERA_NAME"
    "camera_role=$PROFILE_ROLE"
    "camera_serial=$CAMERA_SERIAL"
    "timing_profile=$TIMING_PROFILE"
    "timing_role=$PROFILE_ROLE"
    "camera_fps=$CAMERA_FPS"
    "realsense_inter_cam_sync_mode=$REALSENSE_INTER_CAM_SYNC_MODE"
    "realsense_global_time_enabled=$REALSENSE_GLOBAL_TIME_ENABLED"
    "xsens_time_option=$XSENS_TIME_OPTION"
    "vlp16_gps_time=$VLP16_GPS_TIME"
    "recorded_camera_images=color_jpeg95,depth_png_lossless"
)

TOPICS=(
    /tf
    /tf_static
    /odom
    /a200_0881/platform/odom
    /a200_0881/platform/odom/filtered
    /a200_0881/odometry/filtered
    /odometry/filtered
    /imu/data
    /imu/time_ref
    /imu/utctime
    /status
    /camera/camera/color/image_raw/compressed
    /camera/camera/color/camera_info
    /camera/camera/color/metadata
    /camera/camera/depth/camera_info
    /camera/camera/depth/metadata
    /camera/camera/aligned_depth_to_color/image_raw/compressedDepth
    /camera/camera/extrinsics/depth_to_color
    /diagnostics
    /camera/depth_scan
    /velodyne_packets
    /velodyne_points
    /scan
    /cmd_vel
    /cmd_vel_nav
    /cmd_vel_smoothed
    /a200_0881/cmd_vel
    /rtabmap/info
    /rtabmap/odom
    /rtabmap/localization_pose
    /navigation/localization_ready
    /goal_pose
    /plan
    /local_plan
    /navigate_to_pose/_action/status
    /navigate_to_pose/_action/feedback
    /collision_monitor_state
    /forward_stop_polygon
)

SERVICES=(
    /navigate_to_pose/_action/send_goal
    /navigate_to_pose/_action/get_result
    /navigate_to_pose/_action/cancel_goal
)

echo "=========================================="
echo " Recording ROS 2 mapping data"
echo "=========================================="
echo "Session: $SESSION_ID"
echo "Output: $OUTPUT"
echo "Storage: SQLite3"
echo "File splitting: 4 GiB"
echo

if [[ "$RECORD_ALL" == "1" ]]; then
    echo "Mode: all topics (high disk and DDS load)"
    ros2 bag record \
        "${COMMON_ARGS[@]}" \
        --all-topics \
        --services "${SERVICES[@]}" \
        --include-hidden-topics \
        --exclude-regex '(/ffmpeg($|/)|/theora($|/)|/zstd($|/))' \
        "${CUSTOM_DATA[@]}" &
else
    echo "Mode: mapping essentials"
    echo "Set RECORD_ALL_TOPICS=1 only when a full-system trace is required."
    ros2 bag record \
        "${COMMON_ARGS[@]}" \
        --topics "${TOPICS[@]}" \
        --services "${SERVICES[@]}" \
        "${CUSTOM_DATA[@]}" &
fi

RECORDER_PID=$!
STOP_REQUESTED=0
printf '%s\n' "$RECORDER_PID" >"$PID_FILE"
printf '%s\n' "$OUTPUT" >"$OUTPUT_FILE"

cleanup_state() {
    if [[ "$(cat "$PID_FILE" 2>/dev/null || true)" == "$RECORDER_PID" ]]; then
        rm -f "$PID_FILE" "$OUTPUT_FILE"
    fi
}

forward_signal() {
    trap - INT TERM
    STOP_REQUESTED=1
    timeout 15 ros2 service call \
        /v3_rosbag_recorder/stop \
        rosbag2_interfaces/srv/Stop "{}" >/dev/null 2>&1 || true
    sleep 1
    kill -TERM "$RECORDER_PID" 2>/dev/null || true
    wait "$RECORDER_PID" 2>/dev/null || true
}

trap cleanup_state EXIT
trap forward_signal INT TERM

set +e
wait "$RECORDER_PID"
RECORDER_STATUS=$?
set -e

if [[ ! -s "$OUTPUT/metadata.yaml" ]] &&
   find "$OUTPUT" -maxdepth 1 -type f -name '*.db3' -print -quit | grep -q .
then
    echo "metadata.yaml is missing; rebuilding it from SQLite3 files..."
    ros2 bag reindex --storage sqlite3 "$OUTPUT"
fi

if [[ -s "$OUTPUT/metadata.yaml" ]]; then
    echo "Rosbag finalized: $OUTPUT/metadata.yaml"
else
    echo "ERROR: rosbag stopped without a valid metadata.yaml."
    exit 1
fi

set +e
python3 "$ROOT/scripts/check_bag_integrity.py" "$OUTPUT"
INTEGRITY_STATUS=$?
set -e
if (( INTEGRITY_STATUS != 0 )); then
    echo "ERROR: rosbag camera integrity check failed (exit $INTEGRITY_STATUS)." >&2
    echo "The bag was retained for diagnosis: $OUTPUT" >&2
    exit "$INTEGRITY_STATUS"
fi

if (( STOP_REQUESTED )); then
    exit 0
fi
exit "$RECORDER_STATUS"
