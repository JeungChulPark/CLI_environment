#!/usr/bin/env bash
# Freeze the current hdl_graph_slam map into a ray-cast 2D grid and serve it to Nav2 instead of the live grid:
# dump_graph -> pcd2grid.py (floor = base_link - 0.13 m) -> map_server. hdl keeps localizing.
#   freeze_map.sh [name]     (default room_<date>)
set -e
here=$(cd "$(dirname "$0")" && pwd); maps=$here/maps; mkdir -p "$maps"
name=${1:-room_$(date +%y%m%d_%H%M)}
export FASTRTPS_DEFAULT_PROFILES_FILE=$here/fastdds_big_shm.xml
( source "$HOME/objpose/nav/hdl_env.sh"
  ros2 service call /hdl_graph_slam/dump_graph hdl_graph_slam/srv/DumpGraph "{destination: $maps/${name}_dump}" | tail -1
  ros2 service call /hdl_graph_slam/save_map hdl_graph_slam/srv/SaveMap "{utm: false, resolution: 0.05, destination: $maps/$name.pcd}" | tail -1 )
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0
python3 "$HOME/objpose/nav/pcd2grid.py" "$maps/${name}_dump" "$maps/$name" --floor -0.13 --obst-max 1.5
[ -f "$here/logs/grid.pid" ] && { kill -- -"$(cat "$here/logs/grid.pid")" 2>/dev/null; rm -f "$here/logs/grid.pid"; }
[ -f "$here/logs/map.pid" ] && { kill -INT -- -"$(cat "$here/logs/map.pid")" 2>/dev/null; sleep 2; }
# the lifecycle bring-up sometimes loses the change_state reply ("failed to send response ... timeout"); retry once
for attempt in 1 2; do
  setsid nohup ros2 launch "$here/real_map.launch.py" map:="$maps/$name.yaml" > "$here/logs/map.log" 2>&1 < /dev/null & echo $! > "$here/logs/map.pid"
  for i in $(seq 1 15); do grep -q "Managed nodes are active" "$here/logs/map.log" && break 2; sleep 2; done
  echo "map_server bring-up stalled, retrying"; kill -INT -- -"$(cat "$here/logs/map.pid")" 2>/dev/null; sleep 3
done
echo "serving $maps/$name.yaml on /a200_0881/map (live hdl_grid stopped)"
