#!/bin/bash
# Nếu chạy 2 máy: TIMING_PROFILE=dual_camera_master bash run.bash

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

while true; do
    echo
    echo "=========================================="
    echo " Husky A200 Mapping & Navigation V3"
    echo "=========================================="
    echo "1) Automatic mapping workflow"
    echo "2) Start mapping"
    echo "3) Start teleop"
    echo "4) Save static 2D map"
    echo "5) Export 3D point cloud"
    echo "6) Start sensors + localization + navigation"
    echo "7) Check topics, TF and safety chain"
    echo "8) Open Nav2 RViz"
    echo "9) Edit/check sensor extrinsics"
    echo "10) Record mapping topics"
    echo "11) Stop rosbag recording"
    echo "12) Start Velodyne VLP-16 LiDAR"
    echo "13) Check Velodyne VLP-16 LiDAR"
    echo "q) Quit"
    echo
    read -rp "Select: " CHOICE

    case "$CHOICE" in
        1)
            bash "$ROOT/scripts/auto_mapping.bash"
            ;;
        2)
            bash "$ROOT/scripts/start_mapping.bash"
            ;;
        3)
            bash "$ROOT/scripts/teleop.bash"
            ;;
        4)
            bash "$ROOT/scripts/save_2d_map.bash"
            ;;
        5)
            bash "$ROOT/scripts/export_maps.bash"
            ;;
        6)
            bash "$ROOT/scripts/start_navigation.bash"
            ;;
        7)
            bash "$ROOT/scripts/check_pipeline.bash"
            ;;
        8)
            source "$ROOT/scripts/env.bash"
            ros2 launch nav2_bringup rviz_launch.py
            ;;
        9)
            echo
            echo "Extrinsics file:"
            echo "  $ROOT/config/extrinsics.yaml"
            echo
            sed -n '1,160p' "$ROOT/config/extrinsics.yaml"
            echo
            read -rp "Press Enter to return..."
            ;;
        10)
            bash "$ROOT/scripts/start_record_all.bash"
            ;;
        11)
            bash "$ROOT/scripts/stop_record.bash"
            ;;
        12)
            bash "$ROOT/scripts/start_lidar.bash"
            ;;
        13)
            bash "$ROOT/scripts/check_lidar.bash"
            ;;
        q|Q)
            exit 0
            ;;
        *)
            echo "Invalid choice."
            ;;
    esac
done
