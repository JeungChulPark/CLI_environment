#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! source "$ROOT/scripts/timing_profile.bash"; then
    exit 1
fi

DB_PATH="$ROOT/data/database/rtabmap.db"
MAP_PATH="${MAP_PATH:-$ROOT/data/maps/map.yaml}"
FPS="$CAMERA_FPS"
CAMERA_SERIAL="${CAMERA_SERIAL:-}"
START_SENSORS="${START_SENSORS:-1}"
START_LIDAR="${START_LIDAR:-1}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
# Diagnostic escape hatch only. Accurate map-frame navigation should reject a
# graph that has no global constraints instead of pretending it is metric.
ALLOW_UNOPTIMIZED_MAP="${ALLOW_UNOPTIMIZED_MAP:-0}"

source "$ROOT/scripts/env.bash"

TORCH_LIB_DIR="$HOME/.local/lib/python3.12/site-packages/torch/lib"
if [[ -d "$TORCH_LIB_DIR" ]]; then
    export LD_LIBRARY_PATH="$TORCH_LIB_DIR:${LD_LIBRARY_PATH:-}"
fi

set -euo pipefail

# Fast DDS service replies share a bounded reader history by default. If a
# lifecycle change_state reply is evicted, Nav2's lifecycle manager waits
# forever and leaves planner/controller inactive even though configuration
# succeeded. Apply KEEP_ALL only to ROS 2 service-client endpoints. Do not
# silently replace an operator-provided DDS transport/discovery profile.
FASTRTPS_NAV2_PROFILE="$ROOT/config/fastdds_nav2_client.xml"
case "${RMW_IMPLEMENTATION:-}" in
    rmw_fastrtps_cpp|rmw_fastrtps_dynamic_cpp)
        if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" &&
              "$FASTRTPS_DEFAULT_PROFILES_FILE" != "$FASTRTPS_NAV2_PROFILE" ]]; then
            echo "ERROR: FASTRTPS_DEFAULT_PROFILES_FILE is already set:"
            echo "  $FASTRTPS_DEFAULT_PROFILES_FILE"
            echo "Merge config/fastdds_nav2_client.xml into that profile before starting Nav2."
            exit 1
        fi
        export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTRTPS_NAV2_PROFILE"
        ;;
esac

TIMING_PROFILE="$TIMING_PROFILE" \
    bash "$ROOT/scripts/check_host_clock.bash"

if [[ ! -f "$DB_PATH" ]]; then
    echo "RTAB-Map database not found: $DB_PATH"
    exit 1
fi

if [[ ! -f "$MAP_PATH" ]]; then
    echo "Static map not found: $MAP_PATH"
    echo "Run scripts/save_2d_map.bash while RTAB-Map mapping is active."
    exit 1
fi

if [[ "$ALLOW_UNOPTIMIZED_MAP" != "0" &&
      "$ALLOW_UNOPTIMIZED_MAP" != "1" ]]; then
    echo "ALLOW_UNOPTIMIZED_MAP must be 0 or 1."
    exit 1
fi

ASSET_CHECK_ARGS=(
    --database "$DB_PATH"
    --map "$MAP_PATH"
)
if [[ "$ALLOW_UNOPTIMIZED_MAP" == "1" ]]; then
    ASSET_CHECK_ARGS+=(--allow-unoptimized-map)
fi

echo "Checking that the static map and RTAB-Map database form a usable pair..."
if ! python3 "$ROOT/scripts/check_navigation_assets.py" \
    "${ASSET_CHECK_ARGS[@]}"; then
    echo
    echo "Navigation was not started because map-frame accuracy cannot be trusted."
    echo "Create a new closed-loop map and confirm an accepted long-baseline loop closure."
    echo "Use ALLOW_UNOPTIMIZED_MAP=1 only for diagnosis, never for accuracy testing."
    exit 1
fi

if [[ "$FPS" != "30" && "$FPS" != "60" ]]; then
    echo "CAMERA_FPS must be 30 or 60."
    exit 1
fi

if [[ -n "$CAMERA_SERIAL" && ! "$CAMERA_SERIAL" =~ ^_?[0-9]+$ ]]; then
    echo "CAMERA_SERIAL must be empty or a numeric RealSense serial."
    exit 1
fi

CAMERA_SERIAL_LAUNCH_ARG=""
if [[ -n "$CAMERA_SERIAL" ]]; then
    CAMERA_SERIAL_LAUNCH_ARG=" serial_no:='$CAMERA_SERIAL'"
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

if [[ "$VLP16_GPS_TIME" != "false" ]]; then
    echo "All supported timing profiles require VLP16_GPS_TIME=false."
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

topic_has_data() {
    local topic="$1"
    local timeout_seconds="${2:-2}"
    local qos_profile="${3:-}"
    local qos_args=()

    if [[ -n "$qos_profile" ]]; then
        qos_args=(--qos-profile "$qos_profile")
    fi

    ros2 topic echo \
        "$topic" \
        --once \
        --timeout "$timeout_seconds" \
        "${qos_args[@]}" \
        --no-arr >/dev/null 2>&1
}

camera_parameter_value() {
    local parameter="$1"
    local output
    output="$(
        timeout 5 ros2 param get /camera/camera "$parameter" 2>/dev/null
    )" || return 1
    printf '%s\n' "${output##*: }"
}

require_running_camera_profile() {
    local mode global_depth global_color auto_depth auto_color expected_auto
    mode="$(camera_parameter_value depth_module.inter_cam_sync_mode)" || return 1
    global_depth="$(camera_parameter_value depth_module.global_time_enabled)" || return 1
    global_color="$(camera_parameter_value rgb_camera.global_time_enabled)" || return 1
    auto_depth="$(camera_parameter_value depth_module.enable_auto_exposure)" || return 1
    auto_color="$(camera_parameter_value rgb_camera.enable_auto_exposure)" || return 1
    if [[ "$REALSENSE_AUTO_EXPOSURE" == "true" ]]; then
        expected_auto=True
    else
        expected_auto=False
    fi

    if [[ "$mode" != "$REALSENSE_INTER_CAM_SYNC_MODE" ||
          "$global_depth" != "True" ||
          "$global_color" != "True" ||
          "$auto_depth" != "$expected_auto" ||
          "$auto_color" != "$expected_auto" ]]; then
        echo "ERROR: Existing RealSense node does not match $TIMING_PROFILE." >&2
        echo "  mode: expected=$REALSENSE_INTER_CAM_SYNC_MODE actual=$mode" >&2
        echo "  depth global_time: expected=True actual=$global_depth" >&2
        echo "  color global_time: expected=True actual=$global_color" >&2
        echo "  depth auto exposure: expected=$expected_auto actual=$auto_depth" >&2
        echo "  color auto exposure: expected=$expected_auto actual=$auto_color" >&2
        return 1
    fi
}

running_realsense_pids() {
    # Match only the V3 RealSense launch command and its namespaced camera
    # process. Do not use a broad pkill because other cameras may be running
    # on the same host.
    ps -eo pid=,comm=,args= | awk '
        $2 !~ /^(awk|mawk|gawk|bash|sh|dash|timeout)$/ &&
        (index($0, "ros2 launch launch/camera.launch.py") ||
         (index($0, "realsense2_camera_node") &&
          index($0, "__node:=camera") &&
          index($0, "__ns:=/camera"))) {
            print $1
        }
    '
}

realsense_stack_present() {
    if timeout 5 ros2 node list 2>/dev/null |
        grep -Fxq "/camera/camera"; then
        return 0
    fi

    [[ -n "$(running_realsense_pids)" ]]
}

stop_stale_realsense() {
    local pids=()
    local alive=()
    local deadline
    local pid

    mapfile -t pids < <(running_realsense_pids)
    if (( ${#pids[@]} > 0 )); then
        echo "Stopping stale RealSense stack (PID: ${pids[*]})..."
        kill -INT "${pids[@]}" 2>/dev/null || true

        deadline=$((SECONDS + 8))
        while (( SECONDS < deadline )); do
            alive=()
            for pid in "${pids[@]}"; do
                if kill -0 "$pid" 2>/dev/null; then
                    alive+=("$pid")
                fi
            done
            (( ${#alive[@]} == 0 )) && break
            sleep 0.5
        done

        if (( ${#alive[@]} > 0 )); then
            echo "RealSense did not stop after SIGINT; sending SIGTERM to PID: ${alive[*]}"
            kill -TERM "${alive[@]}" 2>/dev/null || true
        fi
    fi

    # Let the ROS graph remove the old node before launching the replacement.
    for _ in $(seq 1 10); do
        if ! timeout 2 ros2 node list 2>/dev/null |
            grep -Fxq "/camera/camera"; then
            return 0
        fi
        sleep 0.5
    done

    echo "ERROR: stale /camera/camera is still present after shutdown." >&2
    return 1
}

wait_for_topic_data() {
    local topic="$1"
    local attempts="$2"
    local qos_profile="${3:-}"

    for _ in $(seq 1 "$attempts"); do
        if topic_has_data "$topic" 1 "$qos_profile"; then
            return 0
        fi
        sleep 1
    done

    return 1
}

transform_ready() {
    local parent="$1"
    local child="$2"
    local timeout_seconds="${3:-5}"
    local pipefail_state
    local status

    pipefail_state="$(set -o | awk '$1 == "pipefail" {print $2}')"
    set +o pipefail
    timeout "$timeout_seconds" ros2 run tf2_ros tf2_echo \
        "$parent" "$child" 2>/dev/null | grep -Fq -m 1 "Translation:"
    status=$?
    if [[ "$pipefail_state" == "on" ]]; then
        set -o pipefail
    fi
    return "$status"
}

start_terminal() {
    local title="$1"
    local command="$2"

    gnome-terminal \
        --title="$title" \
        -- bash -lc \
        "cd '$ROOT'; $command; echo; read -rp '$title stopped. Press Enter to close...'" &
}

check_ekf_covariance() {
    timeout 10 python3 - <<'PY'
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

rclpy.init()
node = Node("navigation_covariance_check")
sample = None
sample_sequence = 0


def callback(message):
    global sample, sample_sequence
    covariance = message.pose.covariance
    sample = (float(covariance[0]), float(covariance[7]), float(covariance[35]))
    sample_sequence += 1


def is_bounded(values):
    return (
        all(math.isfinite(value) and value >= 0.0 for value in values)
        and values[0] <= 0.5
        and values[1] <= 0.5
        and values[2] <= 0.5
    )


node.create_subscription(Odometry, "/odometry/filtered", callback, 10)
deadline = time.monotonic() + 8.0
last_sequence = -1
bounded_count = 0
bounded_sample = None
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    if sample_sequence == last_sequence or sample is None:
        continue
    last_sequence = sample_sequence
    if is_bounded(sample):
        bounded_count += 1
        if bounded_count >= 5:
            bounded_sample = sample
            break
    else:
        bounded_count = 0

node.destroy_node()
rclpy.shutdown()

if sample is None:
    raise SystemExit("no /odometry/filtered message received")

x, y, yaw = bounded_sample or sample
print(f"x={x:.6g} y={y:.6g} yaw={yaw:.6g}")
if bounded_sample is None:
    raise SystemExit(2)
PY
}

start_navigation_sensors() {
    if ! command -v gnome-terminal >/dev/null 2>&1; then
        echo "gnome-terminal is required to auto-start sensors for option 6."
        echo "Start sensors manually, then run START_SENSORS=0 bash scripts/start_navigation.bash."
        exit 1
    fi

    echo "Step 1: Starting static sensor TF..."
    if transform_ready base_link camera_link 2 &&
       transform_ready base_link imu_link 2; then
        echo "Static sensor TF is already available."
    else
        start_terminal "V3 Static Sensor TF" \
            "ros2 launch launch/static_tf.launch.py"
        sleep 1
    fi

    echo "Step 2: Starting RealSense camera..."
    local reuse_camera=false
    if topic_has_data /camera/camera/color/image_raw 2 &&
       topic_has_data /camera/camera/aligned_depth_to_color/image_raw 2; then
        if require_running_camera_profile; then
            echo "RealSense camera is already publishing with the requested profile."
            reuse_camera=true
        else
            echo "Existing RealSense profile is stale; restarting only the camera stack."
        fi
    elif realsense_stack_present; then
        echo "Existing RealSense stack has an incomplete RGB-D stream; restarting it."
    fi

    if [[ "$reuse_camera" != "true" ]]; then
        if realsense_stack_present; then
            stop_stale_realsense
        fi
        start_terminal "V3 RealSense Camera" \
            "ros2 launch launch/camera.launch.py timing_profile:='$TIMING_PROFILE'${CAMERA_SERIAL_LAUNCH_ARG} camera_fps:='$FPS' inter_cam_sync_mode:='$REALSENSE_INTER_CAM_SYNC_MODE' global_time_enabled:='$REALSENSE_GLOBAL_TIME_ENABLED'"
    fi

    echo "Waiting for RealSense RGB-D data..."
    for topic in \
        /camera/camera/color/image_raw \
        /camera/camera/aligned_depth_to_color/image_raw
    do
        if wait_for_topic_data "$topic" 45; then
            echo "OK   $topic"
        else
            echo "ERROR: no message received from $topic"
            echo "Localization cannot run without a live synchronized RGB-D stream."
            exit 1
        fi
    done

    echo "Checking color camera calibration..."
    timeout 10 ros2 topic echo \
        /camera/camera/color/image_raw \
        --timeout 9 \
        --no-arr >/dev/null 2>&1 &
    RGB_SUBSCRIBER_PID=$!
    sleep 1

    if topic_has_data /camera/camera/color/camera_info 8 best_available; then
        echo "OK   /camera/camera/color/camera_info"
    else
        kill "$RGB_SUBSCRIBER_PID" 2>/dev/null || true
        wait "$RGB_SUBSCRIBER_PID" 2>/dev/null || true
        echo "ERROR: no color camera calibration received."
        exit 1
    fi

    kill "$RGB_SUBSCRIBER_PID" 2>/dev/null || true
    wait "$RGB_SUBSCRIBER_PID" 2>/dev/null || true

    echo "Step 3: Starting Xsens IMU..."
    if topic_has_data /imu/data 2; then
        echo "Xsens IMU is already publishing."
    else
        start_terminal "V3 Xsens IMU" \
            "ros2 launch launch/xsens.launch.py time_option:='$XSENS_TIME_OPTION'"
    fi

    echo "Waiting for Xsens IMU data..."
    if wait_for_topic_data /imu/data 30; then
        echo "OK   /imu/data"
    else
        echo "ERROR: no message received from /imu/data"
        exit 1
    fi

    if [[ "$CUSTOM_EKF" == "1" ]]; then
        echo "Checking wheel odometry..."
        if topic_has_data /a200_0881/platform/odom 8 sensor_data; then
            echo "OK   /a200_0881/platform/odom"
        else
            echo "ERROR: no wheel odometry received from /a200_0881/platform/odom"
            echo "Restart clearpath-platform.service or check the base/controller manager before starting EKF."
            exit 1
        fi
    fi

    echo "Step 4: Starting EKF..."
    if topic_has_data /odometry/filtered 2; then
        if EKF_COVARIANCE="$(check_ekf_covariance 2>/dev/null)"; then
            echo "Filtered odometry is already ready: $EKF_COVARIANCE"
        else
            echo "ERROR: /odometry/filtered is publishing, but covariance is not usable."
            echo "Close the existing EKF terminal or restart clearpath-platform.service, then retry."
            exit 1
        fi
    else
        start_terminal "V3 EKF Fusion" \
            "ros2 launch launch/ekf.launch.py use_custom_ekf:='$CUSTOM_EKF_ARG'"
    fi

    echo "Waiting for filtered odometry..."
    if wait_for_topic_data /odometry/filtered 30; then
        echo "OK   /odometry/filtered"
    else
        echo "ERROR: /odometry/filtered was not ready after 30 seconds."
        exit 1
    fi

    echo "Waiting for EKF pose covariance to converge..."
    if EKF_COVARIANCE="$(check_ekf_covariance)"; then
        echo "EKF covariance is bounded: $EKF_COVARIANCE"
    else
        echo "ERROR: EKF pose covariance did not converge: ${EKF_COVARIANCE:-no sample}"
        exit 1
    fi

    echo "Checking EKF transform odom -> base_link..."
    if transform_ready odom base_link 8; then
        echo "EKF transform odom -> base_link is ready."
    else
        echo "ERROR: odom -> base_link is missing."
        exit 1
    fi

    echo "Checking static sensor transforms..."
    if transform_ready base_link camera_link 5 &&
       transform_ready base_link imu_link 5 &&
       transform_ready base_link camera_color_optical_frame 8 &&
       transform_ready base_link camera_depth_optical_frame 8; then
        echo "Sensor TF is connected."
    else
        echo "ERROR: sensor TF tree is disconnected."
        exit 1
    fi
}

if [[ "$START_SENSORS" == "1" ]]; then
    start_navigation_sensors
    START_SENSORS_ARG=false
fi

LIDAR_STREAM_READY=false
if topic_has_data /velodyne_points 2 sensor_data &&
   topic_has_data "$SCAN_TOPIC" 2 sensor_data; then
    LIDAR_STREAM_READY=true
fi

if [[ "$LIDAR_STREAM_READY" == "true" ]]; then
    if [[ "$SCAN_TOPIC" == "/scan" ]]; then
        echo "A LiDAR stack is already running; validating and reusing it..."
        if ! bash "$ROOT/scripts/check_lidar.bash"; then
            echo "ERROR: Existing LiDAR data does not match the production scan."
            echo "Stop the stale LiDAR process, then retry."
            exit 1
        fi
    fi
    # Never launch a second /scan or /velodyne_points publisher.
    START_LIDAR_ARG=false
elif [[ "$START_LIDAR" == "0" ]]; then
    echo "ERROR: START_LIDAR=0 but /velodyne_points or $SCAN_TOPIC has no data."
    echo "Nav2 costmaps, collision monitor and RTAB-Map all require the VLP-16."
    exit 1
fi

echo "Starting RTAB-Map localization + Nav2 static map navigation"
echo "Map: $MAP_PATH"
echo "Timing profile: $TIMING_PROFILE ($PROFILE_ROLE)"
echo "Safety output: /cmd_vel -> /a200_0881/cmd_vel"
echo "Motion remains locked until /navigation/localization_ready is true."
echo "If the robot is not at the last mapped pose, use RViz '2D Pose Estimate'"
echo "before sending a Nav2 goal. This initializes RTAB-Map through /initialpose."

NAVIGATION_LAUNCH_ARGS=(
    "$ROOT/launch/navigation.launch.py"
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
    map:="$MAP_PATH"
)
if [[ -n "$CAMERA_SERIAL" ]]; then
    NAVIGATION_LAUNCH_ARGS+=(serial_no:="$CAMERA_SERIAL")
fi

ROS_LOG_DIR=/tmp/roslogs ros2 launch "${NAVIGATION_LAUNCH_ARGS[@]}"
