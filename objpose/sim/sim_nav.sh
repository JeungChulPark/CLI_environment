#!/usr/bin/env bash
# Start slam_toolbox (live mapping) + Nav2 against the running simulation. Run sim_up.sh first.
# The 2D scan is the VLP-16 ring slice Gazebo publishes as sensors/lidar3d_0/scan.
# Nav2 uses nav2_sim.yaml (see make_nav2_sim_params.py for what differs from the Clearpath demo params).
set -e
here=$(cd "$(dirname "$0")" && pwd)
"$here/lidar_bridge.sh"   # 1-deep-queue lidar bridge (see lidar_bridge.yaml)
source "$here/sim_env.sh"
scan=/a200_0881/sensors/lidar3d_0/scan
common=(use_sim_time:=true setup_path:=$SIM_DIR/ scan_topic:=$scan)
setsid nohup ros2 launch clearpath_nav2_demos slam.launch.py "${common[@]}" > "$here/logs/slam.log" 2>&1 &
echo $! > "$here/logs/slam.pid"
sleep 5
python3 "$here/make_nav2_sim_params.py" "$here/nav2_sim.yaml" > /dev/null
setsid nohup ros2 launch "$here/nav2_sim.launch.py" > "$here/logs/nav2.log" 2>&1 &
echo $! > "$here/logs/nav2.pid"
echo "slam_toolbox + Nav2 started (logs: $here/logs/slam.log, nav2.log)"
