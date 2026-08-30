#!/bin/bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/env.bash"


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

publisher_count() {
    local topic="$1"

    ros2 topic info "$topic" 2>/dev/null |
        awk -F': ' '/Publisher count/ {print $2; found=1} END {if (!found) print 0}'
}

echo "=== Process sanity ==="
LEGACY_PROCESSES="$(
    pgrep -af "/script_navigatable/script/(start_nav2|cmd_vel_bridge|start_rtabmap|start_ekf)" || true
)"
if [[ -n "$LEGACY_PROCESSES" ]]; then
    echo "WARNING: legacy script_navigatable processes are running:"
    echo "$LEGACY_PROCESSES"
    echo "They can leave Nav2 subscribers active without V3 sensors/EKF."
else
    echo "OK   no legacy script_navigatable navigation process detected"
fi

echo
echo "=== Required topics ==="
declare -a TOPIC_CHECKS=(
    "/a200_0881/platform/odom sensor_data"
    "/imu/data"
    "/odometry/filtered"
    "/camera/camera/color/image_raw"
    "/camera/camera/aligned_depth_to_color/image_raw"
    "/camera/depth_cloud sensor_data"
    "/rtabmap/rgbd_image"
    "/rtabmap/info"
    "/velodyne_points"
    "/scan"
)

for check in "${TOPIC_CHECKS[@]}"; do
    read -r topic qos_profile <<<"$check"
    publishers="$(publisher_count "$topic")"
    if [[ "$publishers" == "0" ]]; then
        echo "MISS $topic (publishers=0)"
    elif topic_has_data "$topic" 2 "${qos_profile:-}"; then
        echo "OK   $topic (publishers=$publishers, data=yes)"
    else
        echo "WARN $topic (publishers=$publishers, data=no within 2s)"
    fi
done

echo
echo "=== Wheel odometry data ==="
if ros2 topic echo \
    /a200_0881/platform/odom \
    --once \
    --timeout 5 \
    --qos-profile sensor_data \
    --no-arr >/dev/null 2>&1; then
    echo "OK   /a200_0881/platform/odom has data"
else
    echo "ERROR /a200_0881/platform/odom has no data"
fi

echo
echo "=== RGB-D synchronization throughput ==="
if [[ "$(publisher_count /rtabmap/rgbd_image)" == "0" ]]; then
    echo "SKIP /rtabmap/rgbd_image: publishers=0"
else
timeout 10 python3 - <<'PY' || true
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import RGBDImage

rclpy.init()
node = Node("pipeline_rgbd_rate_check")
count = 0


def callback(_message):
    global count
    count += 1


qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
node.create_subscription(RGBDImage, "/rtabmap/rgbd_image", callback, qos)
start = time.monotonic()
deadline = start + 5.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

elapsed = time.monotonic() - start
rate = count / elapsed
state = "OK" if rate >= 2.0 else "ERROR"
print(f"{state} /rtabmap/rgbd_image: {rate:.2f} Hz ({count} frames/{elapsed:.1f}s)")

node.destroy_node()
rclpy.shutdown()
PY
fi

echo
echo "=== EKF pose covariance ==="
if [[ "$(publisher_count /odometry/filtered)" == "0" ]]; then
    echo "SKIP /odometry/filtered covariance: publishers=0"
else
timeout 10 python3 - <<'PY' || true
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

rclpy.init()
node = Node("pipeline_covariance_check")
sample = None


def callback(message):
    global sample
    covariance = message.pose.covariance
    sample = (float(covariance[0]), float(covariance[7]), float(covariance[35]))


node.create_subscription(Odometry, "/odometry/filtered", callback, 10)
deadline = time.monotonic() + 8.0
while sample is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)

if sample is None:
    print("MISS /odometry/filtered covariance")
else:
    x, y, yaw = sample
    state = "OK" if max(sample) <= 0.5 else "ERROR"
    print(f"{state} x={x:.6g} y={y:.6g} yaw={yaw:.6g}")

node.destroy_node()
rclpy.shutdown()
PY
fi

echo
echo "=== TF ==="
if [[ "$(publisher_count /odometry/filtered)" == "0" &&
      "$(publisher_count /camera/camera/color/image_raw)" == "0" &&
      "$(publisher_count /imu/data)" == "0" ]]; then
    echo "SKIP TF: V3 sensors/EKF are not publishing."
else
for transform in \
    "map odom" \
    "odom base_link" \
    "base_link camera_link" \
    "camera_link camera_depth_frame" \
    "camera_depth_frame camera_depth_optical_frame" \
    "camera_link camera_color_frame" \
    "camera_color_frame camera_color_optical_frame" \
    "base_link imu_link"
do
    read -r parent child <<<"$transform"
    echo "--- $parent -> $child"
    timeout 3 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>/dev/null | head -8 || true
done
fi

echo
echo "=== Safety command chain ==="
ros2 topic info /cmd_vel_nav || true
ros2 topic info /cmd_vel_smoothed || true
ros2 topic info /cmd_vel || true
ros2 topic info /a200_0881/cmd_vel || true


echo
echo "=== Velodyne VLP-16 detail ==="
for topic in /velodyne_packets; do
    publishers="$(publisher_count "$topic")"
    if [[ "$publishers" == "0" ]]; then
        echo "SKIP $topic (publishers=0)"
    elif topic_has_data "$topic" 2; then
        echo "OK   $topic (publishers=$publishers, data=yes)"
    else
        echo "WARN $topic (publishers=$publishers, data=no within 2s)"
    fi
done

echo
echo "=== Optional D455 derived obstacle topics ==="
for topic in /camera/depth_scan /camera/obstacles; do
    publishers="$(publisher_count "$topic")"
    if [[ "$publishers" == "0" ]]; then
        echo "SKIP $topic (publishers=0)"
    elif topic_has_data "$topic" 2; then
        echo "OK   $topic (publishers=$publishers, data=yes)"
    else
        echo "WARN $topic (publishers=$publishers, data=no within 2s)"
    fi
done
