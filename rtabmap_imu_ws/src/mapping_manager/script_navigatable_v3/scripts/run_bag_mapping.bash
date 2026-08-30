#!/usr/bin/env bash
# Replay a recorded RGB-D + Xsens bag through the V3 mapping stack.
#
# The stack normally takes odometry from the Husky wheel encoders
# (ekf.yaml odom0). Recorded bags carry no wheel odometry, so RTAB-Map's
# rgbd_odometry stands in for it and the EKF still fuses the Xsens yaw rate
# on top. Nothing here edits the operational launch/config files: the
# substitutions are topic remaps and two static transforms the RealSense
# driver would otherwise publish live.
#
# Usage: run_bag_mapping.bash <bag_dir> [rate]

set -euo pipefail

THIS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$THIS_DIR/.." && pwd)"
source "$THIS_DIR/env.bash"

BAG="${1:?usage: run_bag_mapping.bash <bag_dir> [rate]}"
RATE="${2:-1.0}"

RGB_TOPIC="${RGB_TOPIC:-/camera/camera/color/image_raw}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/camera/aligned_depth_to_color/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/camera/color/camera_info}"
IMU_TOPIC="${IMU_TOPIC:-/xsens/imu/data}"
IMU_FRAME="${IMU_FRAME:-xsens_link}"
CAMERA_OPTICAL_FRAME="${CAMERA_OPTICAL_FRAME:-camera_color_optical_frame}"

DB_PATH="${DB_PATH:-$ROOT/data/database/rtabmap_rosbag.db}"
LOG_DIR="${LOG_DIR:-$ROOT/data/bag_mapping_logs}"
mkdir -p "$(dirname "$DB_PATH")" "$LOG_DIR" "$ROOT/data/maps"
rm -f "$DB_PATH" "$DB_PATH"-*

PIDS=()
cleanup() {
    for pid in "${PIDS[@]:-}"; do
        kill -INT "-$pid" 2>/dev/null || true
    done
    sleep 3
    for pid in "${PIDS[@]:-}"; do
        kill -KILL "-$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT

spawn() {  # spawn <logfile> <command...>
    local log="$1"; shift
    setsid "$@" >"$LOG_DIR/$log" 2>&1 &
    PIDS+=("$!")
}

echo "bag:      $BAG"
echo "rate:     $RATE"
echo "database: $DB_PATH"
echo "logs:     $LOG_DIR"

# base_link -> camera_link and base_link -> imu_link, from config/extrinsics.yaml
spawn static_tf.log ros2 launch "$ROOT/launch/static_tf.launch.py"

# Published live by the RealSense driver; the bag only carries the images.
spawn tf_optical.log ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll -1.5707963 --pitch 0 --yaw -1.5707963 \
    --frame-id camera_link --child-frame-id "$CAMERA_OPTICAL_FRAME"

# The bag stamps IMU messages with its own frame name; reuse the measured
# base_link -> imu_link pose from extrinsics.yaml rather than duplicating it.
spawn tf_imu.log ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id imu_link --child-frame-id "$IMU_FRAME"

# Reg/Strategy in rtabmap_mapping.yaml refines loop closures with ICP, which
# needs the scan. Without it every visual loop candidate is rejected.
USE_SCAN=false
if [[ -n "${LIDAR_BAG:-}" ]]; then
    USE_SCAN=true
    read -r VX VY VZ VR VP VYAW VPARENT VCHILD < <(python3 -c "
import yaml
e = yaml.safe_load(open('$ROOT/config/vlp16.yaml'))['extrinsics']
print(e['x'], e['y'], e['z'], e['roll'], e['pitch'], e['yaw'],
      e['parent_frame'], e['child_frame'])")
    spawn tf_velodyne.log ros2 run tf2_ros static_transform_publisher \
        --x "$VX" --y "$VY" --z "$VZ" --roll "$VR" --pitch "$VP" --yaw "$VYAW" \
        --frame-id "$VPARENT" --child-frame-id "$VCHILD"
fi

sleep 3

# rgbd_sync + rtabmap, reading the bag topics.
spawn bag_mapping.log ros2 launch "$ROOT/launch/bag_mapping.launch.py" \
    rviz:=false rtabmap_viz:=false \
    rgb_topic:="$RGB_TOPIC" depth_topic:="$DEPTH_TOPIC" \
    camera_info_topic:="$CAMERA_INFO_TOPIC" \
    use_scan:="$USE_SCAN" scan_topic:="${SCAN_TOPIC:-/scan}" \
    database_path:="$DB_PATH"

sleep 5

# Wheel-odometry stand-in. publish_tf is off so the EKF owns odom -> base_link.
spawn rgbd_odometry.log ros2 run rtabmap_odom rgbd_odometry --ros-args \
    -p use_sim_time:=true -p frame_id:=base_link -p subscribe_rgbd:=true \
    -p publish_tf:=false -p wait_for_transform:=0.5 \
    -p qos:=2 -p approx_sync:=true \
    -p topic_queue_size:=30 -p sync_queue_size:=30 \
    -r rgbd_image:=/rtabmap/rgbd_image -r odom:=/odom

# config/ekf.yaml unchanged; only its input topics are remapped onto the bag.
spawn ekf.log ros2 run robot_localization ekf_node --ros-args \
    --params-file "$ROOT/config/ekf.yaml" \
    -p use_sim_time:=true \
    -r __node:=ekf_filter_node \
    -r /a200_0881/platform/odom:=/odom \
    -r /imu/data:="$IMU_TOPIC" \
    -r odometry/filtered:=/odometry/filtered

sleep 5

echo "--- playing bag ---"
# Some rigs record each sensor into its own bag alongside the camera bag. They
# share one epoch clock, so extra players are enough; only the camera bag drives
# /clock. Each secondary player waits out the gap between its own first message
# and the camera's, so wall-clock playback stays aligned with the stamps.
bag_start_s() {
    python3 -c "
import yaml, sys
m = yaml.safe_load(open(sys.argv[1] + '/metadata.yaml'))['rosbag2_bagfile_information']
print(int(m['starting_time']['nanoseconds_since_epoch']) / 1e9)" "$1"
}
PRIMARY_START="$(bag_start_s "$BAG")"
play_secondary() {  # play_secondary <label> <bag>
    local extra
    extra="$(python3 -c "print(max(0.0, ($(bag_start_s "$2")) - $PRIMARY_START))")"
    echo "$1 bag:  $2  (+${extra}s)"
    spawn "$1_bag_play.log" ros2 bag play "$2" --rate "$RATE" \
        --delay "$(python3 -c "print(5 + $extra)")"
}
[[ -n "${IMU_BAG:-}" ]]   && play_secondary imu "$IMU_BAG"
[[ -n "${LIDAR_BAG:-}" ]] && play_secondary lidar "$LIDAR_BAG"

ros2 bag play "$BAG" --clock --rate "$RATE" --delay 5 2>&1 | tee "$LOG_DIR/bag_play.log"
echo "--- playback finished, letting the graph settle ---"
sleep 15

cleanup
trap - EXIT
sleep 2

echo "--- exporting ---"
rtabmap-export --cloud --poses --poses_format 11 --ascii \
    --output rtabmap_bag_map --output_dir "$ROOT/data/maps" "$DB_PATH" \
    2>&1 | tee "$LOG_DIR/export.log" | tail -5

echo
echo "database: $DB_PATH"
echo "maps:     $ROOT/data/maps"
