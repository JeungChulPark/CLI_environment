#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$SCRIPT_DIR/check_stack.bash" \
    rtabmap_slam \
    rtabmap_sync \
    rtabmap_util \
    realsense2_camera \
    compressed_image_transport \
    compressed_depth_image_transport \
    robot_localization \
    xsens_mti_ros2_driver \
    teleop_twist_keyboard \
    nav2_bringup \
    pointcloud_to_laserscan \
    velodyne_driver \
    velodyne_pointcloud \
    velodyne_laserscan \
    topic_tools \
    rosbag2_transport \
    rosbag2_storage_default_plugins
