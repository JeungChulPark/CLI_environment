#!/usr/bin/env bash
# Navigation mode on the fixed global map (no SLAM): VLP-16 -> map_localizer.py (GICP against the saved hdl map)
# -> map->odom; map_server serves the 2D grid; Nav2 + stuck_guard as in real_up.sh. Needs husky-up.
#   real_loc_up.sh [map name]      (default globalmap_261002; files in maps/<name>.pcd / .yaml)
#   env INIT=global|origin|"x,y,yaw_deg" (start pose; default global search), LIDAR_X/Y/Z/YAW as in real_up.sh
set -e
here=$(cd "$(dirname "$0")" && pwd); logs=$here/logs; mkdir -p "$logs"
name=${1:-globalmap_261002}
export FASTRTPS_DEFAULT_PROFILES_FILE=$here/fastdds_big_shm.xml
# reach the PC (RViz) over the wired link even if Wi-Fi multicast fails
export ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-192.168.2.2}
# mounting yaw: the caller's LIDAR_YAW, else mount.env (written by objpose/new_place.sh finish), else the 2026-10-13 value
[ -n "${LIDAR_YAW:-}" ] || LIDAR_YAW=$(sed -n 's/^LIDAR_YAW=//p' "$here/mount.env" 2>/dev/null)
export LIDAR_X=${LIDAR_X:-0} LIDAR_Y=${LIDAR_Y:-0} LIDAR_Z=${LIDAR_Z:-1.215} LIDAR_YAW=${LIDAR_YAW:-1.2171}
export MAP_PCD=$here/maps/$name.pcd MAP_GRID=$here/maps/$name.yaml LIDAR="$LIDAR_X,$LIDAR_Y,$LIDAR_Z,$LIDAR_YAW" INIT=${INIT:-global}
# the path driven by hand while this map was made: goals near it are driven along it (nav_bridge.py, road.py); ROAD=0 = off
[ "${ROAD:-1}" = 1 ] && [ -f "$here/maps/${name}_track.json" ] && export ROAD_TRACK=$here/maps/${name}_track.json
( source "$HOME/objpose/nav/hdl_env.sh"
  setsid nohup python3 -u "$HOME/objpose/lidar/vlp16_publisher.py" > "$logs/vlp16.log" 2>&1 < /dev/null & echo $! > "$logs/vlp16.pid"
  setsid nohup python3 -u "$here/map_localizer.py" > "$logs/localizer.log" 2>&1 < /dev/null & echo $! > "$logs/localizer.pid" )
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp USE_SIM_TIME=false
setsid nohup ros2 run tf2_ros static_transform_publisher --x $LIDAR_X --y $LIDAR_Y --z $LIDAR_Z --yaw $LIDAR_YAW \
  --frame-id base_link --child-frame-id velodyne --ros-args -r __node:=robot_lidar_tf -r /tf_static:=/a200_0881/tf_static \
  > "$logs/lidar_tf.log" 2>&1 < /dev/null & echo $! > "$logs/lidar_tf.pid"
setsid nohup ros2 launch "$here/real_map.launch.py" map:="$MAP_GRID" > "$logs/map.log" 2>&1 < /dev/null & echo $! > "$logs/map.pid"
OBSTACLES_IN_PLAN=${OBSTACLES_IN_PLAN:-1} python3 "$here/make_nav2_real_params.py" "$here/nav2_real.yaml" > /dev/null
sleep 5
setsid nohup ros2 launch "$here/real_nav2.launch.py" > "$logs/nav2.log" 2>&1 < /dev/null & echo $! > "$logs/nav2.pid"
# Nav2 for the web viewer (map, local costmap, goals): the object-pose hub proxies /nav/* to this (hub.py --nav-bridge)
setsid nohup python3 -u "$here/nav_bridge.py" > "$logs/nav_bridge.log" 2>&1 < /dev/null & echo $! > "$logs/nav_bridge.pid"
[ "${STUCK_GUARD:-1}" = 1 ] && { setsid nohup python3 -u "$HOME/objpose/nav/stuck_guard.py" > "$logs/stuck_guard.log" 2>&1 < /dev/null & echo $! > "$logs/guard.pid"; }
echo "started (localization mode, map $name, INIT=$INIT): vlp16, map_localizer, map_server, Nav2, stuck_guard(${STUCK_GUARD:-1}); logs $logs"
