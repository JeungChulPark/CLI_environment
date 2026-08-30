#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if ! source "$ROOT/scripts/timing_profile.bash"; then
    exit 1
fi

FPS="$CAMERA_FPS"
CAMERA_SERIAL="${CAMERA_SERIAL:-}"
SESSION_ID="${SESSION_ID:-session_$(date -u +%Y%m%dT%H%M%SZ)}"

source "$ROOT/scripts/env.bash"


set -euo pipefail

TIMING_PROFILE="$TIMING_PROFILE" \
    bash "$ROOT/scripts/check_host_clock.bash"

if [[ ! "$SESSION_ID" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "SESSION_ID may contain only letters, numbers, dot, underscore and dash."
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

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is required for the automatic desktop workflow."
    echo "Run scripts/start_mapping.bash and scripts/teleop.bash manually."
    exit 1
fi

echo "=========================================="
echo " Husky A200 automatic mapping workflow V3"
echo "=========================================="
echo "Camera: matched RGB/depth ${FPS} Hz"
echo "Camera serial: ${CAMERA_SERIAL:-automatic selection}"
echo "Timing profile: $TIMING_PROFILE ($PROFILE_ROLE)"
echo "Recording session: $SESSION_ID"
echo "Sensors: D455f + MTi-630 + VLP-16"
echo

cleanup_recorder() {
    bash "$ROOT/scripts/stop_record.bash" >/dev/null 2>&1 || true
}
trap cleanup_recorder EXIT
trap 'cleanup_recorder; exit 130' INT TERM

if [[ "$FPS" != "30" && "$FPS" != "60" ]]; then
    echo "CAMERA_FPS must be 30 or 60."
    exit 1
fi

if [[ "$REALSENSE_INTER_CAM_SYNC_MODE" != "0" && "$FPS" != "30" ]]; then
    echo "Dual-camera master/slave modes require CAMERA_FPS=30."
    exit 1
fi

if [[ "$VLP16_GPS_TIME" != "false" ]]; then
    echo "All supported timing profiles require VLP16_GPS_TIME=false."
    exit 1
fi

# Match the legacy workflow: start this pipeline's EKF by default so it owns
# odom -> base_link. Set USE_CUSTOM_EKF=0 explicitly only when another node
# is confirmed to publish that exact transform.
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

echo "Step 1.1: Starting static sensor TF..."
gnome-terminal \
    --title="V3 Static Sensor TF" \
    -- bash -lc \
    "cd '$ROOT'; ros2 launch launch/static_tf.launch.py; echo; read -rp 'Static TF stopped. Press Enter to close...'" &

sleep 1

echo "Step 1.2: Starting RealSense camera..."
gnome-terminal \
    --title="V3 RealSense Camera" \
    -- bash -lc \
    "cd '$ROOT'; ros2 launch launch/camera.launch.py timing_profile:='$TIMING_PROFILE'${CAMERA_SERIAL_LAUNCH_ARG} camera_fps:='$FPS' inter_cam_sync_mode:='$REALSENSE_INTER_CAM_SYNC_MODE' global_time_enabled:='$REALSENSE_GLOBAL_TIME_ENABLED'; echo; read -rp 'Camera stopped. Press Enter to close...'" &

echo "Waiting for RealSense topics..."
CAMERA_READY=false
for _ in $(seq 1 45); do
    TOPICS="$(ros2 topic list 2>/dev/null || true)"
    if grep -Fxq "/camera/camera/color/image_raw" <<<"$TOPICS" &&
       grep -Fxq "/camera/camera/aligned_depth_to_color/image_raw" <<<"$TOPICS"; then
        CAMERA_READY=true
        break
    fi
    sleep 1
done

if [[ "$CAMERA_READY" == "true" ]]; then
    echo "RealSense camera is ready."
else
    echo "WARNING: RealSense topics were not ready after 45 seconds."
fi

echo "Checking that RGB and aligned depth contain data..."
for topic in \
    /camera/camera/color/image_raw \
    /camera/camera/aligned_depth_to_color/image_raw
do
    if ros2 topic echo "$topic" --once --timeout 8 --no-arr >/dev/null 2>&1; then
        echo "OK   $topic"
    else
        echo "ERROR: no message received from $topic"
        echo "RTAB-Map cannot map without a live synchronized RGB-D stream."
        exit 1
    fi
done

# RealSense publishes CameraInfo lazily through its camera publisher. Keep an
# RGB subscriber alive while checking CameraInfo, matching how RTAB-Map
# subscribes to both topics at the same time.
echo "Checking color camera calibration..."
timeout 10 ros2 topic echo \
    /camera/camera/color/image_raw \
    --timeout 9 \
    --no-arr >/dev/null 2>&1 &
RGB_SUBSCRIBER_PID=$!
sleep 1

if ros2 topic echo \
    /camera/camera/color/camera_info \
    --once \
    --timeout 8 \
    --qos-profile best_available \
    --no-arr >/dev/null 2>&1; then
    echo "OK   /camera/camera/color/camera_info"
else
    kill "$RGB_SUBSCRIBER_PID" 2>/dev/null || true
    wait "$RGB_SUBSCRIBER_PID" 2>/dev/null || true
    echo "ERROR: no color camera calibration received."
    echo "RTAB-Map cannot map without CameraInfo."
    exit 1
fi

kill "$RGB_SUBSCRIBER_PID" 2>/dev/null || true
wait "$RGB_SUBSCRIBER_PID" 2>/dev/null || true

echo "Step 1.3: Starting Xsens IMU..."
gnome-terminal \
    --title="V3 Xsens IMU" \
    -- bash -lc \
    "cd '$ROOT'; ros2 launch launch/xsens.launch.py time_option:='$XSENS_TIME_OPTION'; echo; read -rp 'Xsens stopped. Press Enter to close...'" &

echo "Waiting for Xsens IMU..."
IMU_READY=false
for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -Fxq "/imu/data"; then
        IMU_READY=true
        break
    fi
    sleep 1
done

if [[ "$IMU_READY" == "true" ]]; then
    echo "Xsens IMU is ready."
else
    echo "WARNING: /imu/data was not ready after 30 seconds."
fi

if [[ "$CUSTOM_EKF" == "1" ]]; then
    echo "Checking wheel odometry..."
    if ros2 topic echo \
        /a200_0881/platform/odom \
        --once \
        --timeout 8 \
        --qos-profile sensor_data \
        --no-arr >/dev/null 2>&1; then
        echo "OK   /a200_0881/platform/odom"
    else
        echo "ERROR: no wheel odometry received from /a200_0881/platform/odom"
        echo "Clearpath platform service is running, but the odom publisher is not producing data."
        echo "Restart clearpath-platform.service or check the base/controller manager before starting EKF."
        exit 1
    fi
fi

echo "Step 1.4: Starting EKF..."
gnome-terminal \
    --title="V3 EKF Fusion" \
    -- bash -lc \
    "cd '$ROOT'; ros2 launch launch/ekf.launch.py use_custom_ekf:='$CUSTOM_EKF_ARG'; echo; read -rp 'EKF stopped. Press Enter to close...'" &

echo "Waiting for filtered odometry..."
ODOM_READY=false
for _ in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -Fxq "/odometry/filtered"; then
        ODOM_READY=true
        break
    fi
    sleep 1
done

if [[ "$ODOM_READY" == "true" ]]; then
    echo "Filtered odometry is ready."
else
    echo "ERROR: /odometry/filtered was not ready after 30 seconds."
    exit 1
fi

echo "Waiting for EKF pose covariance to converge..."
timeout 10 ros2 launch launch/ekf_covariance_check.launch.py

echo "Checking EKF transform odom -> base_link..."
ODOM_TF="$(timeout 8 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null || true)"
if grep -Fq "Translation:" <<<"$ODOM_TF"; then
    echo "EKF transform odom -> base_link is ready."
else
    echo "ERROR: odom -> base_link is missing."
    echo "The mapping TF tree cannot be connected without this transform."
    exit 1
fi

echo "Checking static sensor transforms..."
CAMERA_TF="$(timeout 5 ros2 run tf2_ros tf2_echo base_link camera_link 2>/dev/null || true)"
IMU_TF="$(timeout 5 ros2 run tf2_ros tf2_echo base_link imu_link 2>/dev/null || true)"
COLOR_TF="$(
    timeout 8 ros2 run tf2_ros tf2_echo \
        base_link camera_color_optical_frame 2>/dev/null || true
)"
DEPTH_TF="$(
    timeout 8 ros2 run tf2_ros tf2_echo \
        base_link camera_depth_optical_frame 2>/dev/null || true
)"
if grep -Fq "Translation:" <<<"$CAMERA_TF" &&
   grep -Fq "Translation:" <<<"$IMU_TF" &&
   grep -Fq "Translation:" <<<"$COLOR_TF" &&
   grep -Fq "Translation:" <<<"$DEPTH_TF"; then
    echo "Sensor TF is connected:"
    echo "  base_link -> camera_link -> camera_color_optical_frame"
    echo "  base_link -> camera_link -> camera_depth_optical_frame"
    echo "  base_link -> imu_link"
else
    echo "ERROR: sensor TF tree is disconnected."
    echo "Expected camera color/depth optical frames below camera_link,"
    echo "and imu_link below base_link."
    exit 1
fi

echo "Step 2: Starting RTAB-Map in mapping mode..."
gnome-terminal \
    --wait \
    --title="V3 RTAB-Map Mapping" \
    -- bash -lc \
    "cd '$ROOT'; TIMING_PROFILE='$TIMING_PROFILE' CAMERA_SERIAL='$CAMERA_SERIAL' CAMERA_FPS='$FPS' USE_CUSTOM_EKF='$CUSTOM_EKF' START_SENSORS=0 START_LIDAR=1 SCAN_TOPIC=/scan XSENS_TIME_OPTION='$XSENS_TIME_OPTION' VLP16_TIMESTAMP_FIRST_PACKET='$VLP16_TIMESTAMP_FIRST_PACKET' bash scripts/start_mapping.bash" &
MAPPING_TERMINAL_PID=$!

echo "Waiting for RTAB-Map and depth-obstacle topics..."
MAPPING_READY=false
for _ in $(seq 1 60); do
    TOPICS="$(ros2 topic list 2>/dev/null || true)"
    if grep -Fxq "/camera/obstacles" <<<"$TOPICS" &&
       grep -Fxq "/scan" <<<"$TOPICS" &&
       grep -Fxq "/rtabmap/map" <<<"$TOPICS"; then
        MAPPING_READY=true
        break
    fi
    sleep 1
done

if [[ "$MAPPING_READY" == "true" ]]; then
    echo "RTAB-Map and depth-obstacle topics are ready."
else
    echo "WARNING: mapping topics were not ready after 60 seconds."
    echo "Check the mapping terminal before driving."
fi

echo "Checking the host-timestamped VLP-16 planar scan..."
if SCAN_RESULT="$(
    timeout 12 python3 "$ROOT/scripts/check_lidar_geometry.py" \
        --config "$ROOT/config/vlp16.yaml" \
        --samples 5 \
        --timeout 10
)"; then
    echo "VLP-16 scan is ready: $SCAN_RESULT"
else
    echo "ERROR: VLP-16 frame/range validation failed: ${SCAN_RESULT:-no sample}"
    echo "Do not drive: RTAB-Map would record distorted ICP constraints."
    exit 1
fi

echo "Checking synchronized RGB-D output..."
timeout 10 ros2 launch launch/rgbd_rate_check.launch.py

echo "Checking active RTAB-Map frame insertion parameters..."
ros2 param get /rtabmap/rtabmap Rtabmap/DetectionRate 2>/dev/null || true
ros2 param get /rtabmap/rtabmap RGBD/LinearUpdate 2>/dev/null || true
ros2 param get /rtabmap/rtabmap RGBD/AngularUpdate 2>/dev/null || true
ros2 param get /rtabmap/rtabmap Rtabmap/LoopThr 2>/dev/null || true
ros2 param get /rtabmap/rtabmap Vis/FeatureType 2>/dev/null || true
ros2 param get /rtabmap/rtabmap Reg/Strategy 2>/dev/null || true
ros2 param get /rtabmap/rtabmap RGBD/ProximityAngle 2>/dev/null || true
ros2 param get /rtabmap/rtabmap RGBD/ProximityOdomGuess 2>/dev/null || true
ros2 param get /rtabmap/rtabmap Icp/CorrespondenceRatio 2>/dev/null || true
ros2 param get /rtabmap/rtabmap RGBD/OptimizeMaxError 2>/dev/null || true
ros2 param get /rtabmap/rtabmap Optimizer/Robust 2>/dev/null || true

echo "Checking the complete map -> odom -> base_link sensor tree..."
FULL_TF="$(
    timeout 8 ros2 run tf2_ros tf2_echo \
        map camera_color_optical_frame 2>/dev/null || true
)"
IMU_MAP_TF="$(timeout 8 ros2 run tf2_ros tf2_echo map imu_link 2>/dev/null || true)"
if grep -Fq "Translation:" <<<"$FULL_TF" &&
   grep -Fq "Translation:" <<<"$IMU_MAP_TF"; then
    echo "TF tree map -> odom -> base_link -> camera/imu is connected."
else
    echo "WARNING: the full map-to-sensor TF tree is not connected."
    echo "Check RTAB-Map (map -> odom), EKF (odom -> base_link), and sensor TF."
fi

echo "Step 3: Checking the full TF/topic pipeline..."
gnome-terminal \
    --title="V3 Pipeline Check" \
    -- bash -lc \
    "cd '$ROOT'; bash scripts/check_pipeline.bash; echo; read -rp 'Press Enter to close...'" &

sleep 5

echo "Step 4: Starting mapping-data recorder..."
gnome-terminal \
    --title="V3 Mapping Recorder" \
    -- bash -lc \
    "cd '$ROOT'; SESSION_ID='$SESSION_ID' CAMERA_SERIAL='$CAMERA_SERIAL' CAMERA_FPS='$FPS' bash scripts/start_record_all.bash; echo; read -rp 'Recorder stopped. Press Enter to close...'" &

sleep 5

echo "Step 5: Starting teleop..."
gnome-terminal \
    --title="Husky Teleop" \
    -- bash -lc \
    "cd '$ROOT'; bash scripts/teleop.bash; echo; read -rp 'Teleop stopped. Press Enter to close...'" &

echo
echo "Drive the Husky slowly to collect the map."
echo "Recommended mapping speed: <= 0.4 m/s (0.2m/s returned) and <= 0.45 rad/s."
echo
read -rp "Press Enter when the 2D/3D map has enough coverage..."

echo "Saving the static 2D occupancy map while RTAB-Map is active..."
if bash "$ROOT/scripts/save_2d_map.bash"; then
    echo "2D map saved."
else
    echo "ERROR: Failed to save the 2D map."
    exit 1
fi

echo "Stopping rosbag recording and finalizing metadata..."
bash "$ROOT/scripts/stop_record.bash" || {
    echo "Recorder may still be flushing data. Check its terminal before export."
}

echo
read -rp "Stop mapping and export the 3D point cloud now? [Y/n]: " STOP_MAPPING
STOP_MAPPING="${STOP_MAPPING:-Y}"

if [[ "$STOP_MAPPING" =~ ^[Yy]$ ]]; then
    pkill -INT -f "script_navigatable_v3/launch/mapping.launch.py" 2>/dev/null || true
    echo "Waiting for RTAB-Map to stop and closing its terminal..."
    wait "$MAPPING_TERMINAL_PID" 2>/dev/null || true
    sleep 5

    echo "Validating the finalized map/database pair and graph constraints..."
    MAP_QUALITY_OK=false
    if python3 "$ROOT/scripts/check_navigation_assets.py" \
        --database "$ROOT/data/database/rtabmap.db" \
        --map "$ROOT/data/maps/map.yaml"; then
        MAP_QUALITY_OK=true
    else
        echo "ERROR: The new map is not ready for accurate navigation."
        echo "Repeat mapping with a closed route and an accepted long-baseline loop closure."
    fi

    echo "Exporting 3D point cloud..."
    bash "$ROOT/scripts/export_maps.bash"
else
    echo "Mapping remains active. Export later with scripts/export_maps.bash."
    exit 0
fi

echo
read -rp "Start localization and navigation now? [y/N]: " START_NAV
if [[ "$START_NAV" =~ ^[Yy]$ ]]; then
    if [[ "$MAP_QUALITY_OK" == "true" ]]; then
        gnome-terminal \
            --title="V3 Localization + Nav2" \
            -- bash -lc \
            "cd '$ROOT'; TIMING_PROFILE='$TIMING_PROFILE' CAMERA_SERIAL='$CAMERA_SERIAL' CAMERA_FPS='$FPS' USE_CUSTOM_EKF='$CUSTOM_EKF' START_SENSORS=0 bash scripts/start_navigation.bash; echo; read -rp 'Navigation stopped. Press Enter to close...'" &

        echo "Waiting 15 seconds before opening RViz..."
        sleep 15
        gnome-terminal \
            --title="Nav2 RViz" \
            -- bash -lc \
            "source '$ROOT/scripts/env.bash'; ros2 launch nav2_bringup rviz_launch.py" &
    else
        echo "Navigation was not started because the map quality check failed."
    fi
fi

echo
echo "Workflow complete."
echo "2D map: $ROOT/data/maps/map.yaml"
echo "3D cloud: $ROOT/data/maps/rtabmap_cloud.ply"
