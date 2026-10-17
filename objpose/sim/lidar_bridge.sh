#!/usr/bin/env bash
# Swap the Clearpath lidar bridge for ours (lidar_bridge.yaml, 1-deep queues). Safe to re-run.
here=$(cd "$(dirname "$0")" && pwd)
source "$here/sim_env.sh"
for p in $(pgrep -f "[_]_node:=lidar3d_0_gz_bridge"); do kill "$p"; done
[ -f "$here/logs/lidar_bridge.pid" ] && kill -- -"$(cat "$here/logs/lidar_bridge.pid")" 2>/dev/null
sleep 1
setsid nohup ros2 run ros_gz_bridge parameter_bridge --ros-args -r __node:=sim_lidar_bridge -r __ns:=/a200_0881/sensors \
  -p config_file:="$here/lidar_bridge.yaml" -p use_sim_time:=true > "$here/logs/lidar_bridge.log" 2>&1 < /dev/null &
echo $! > "$here/logs/lidar_bridge.pid"
