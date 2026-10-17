#!/usr/bin/env bash
# Real Husky A200 on the laptop: VLP-16 -> hdl_graph_slam -> map->odom relay -> live 2D grid -> Nav2.
# Needs the Clearpath driver running (husky-up). PC side: objpose/nav/real/rviz_real.sh.
# env LIDAR_X/Y/Z/YAW: base_link -> velodyne mounting. Defaults (2026-10-01): z = top plate 0.245 m + 0.97 m (user);
# 2026-10-13: yaw 1.2171 rad (69.7 deg): the lidar turned again (+144.5 deg vs 10/12). 2026-10-12 it was -2.5447 rad (-145.8 deg). The lidar turned on its mount (+16 deg, then +41 deg after it was re-fixed, vs
# 10/06); forward was 104.8 deg in the lidar frame on 10/06 (straight driving; the front camera said 103.3) + 41.0 (both
# cameras, rt/261012_husky/scan_turn.py). Check after any re-mount: course minus heading on a straight drive must be ~0.
# Before: yaw -1.89 rad (-108 deg) = 260915 rig calibration (camera looks along lidar +y at ~109 deg, assuming the camera faces
# forward) and the first drive (hdl moved at 107 deg while the wheels went straight ahead); x/y unmeasured (0 0).
set -e
here=$(cd "$(dirname "$0")" && pwd); sim=$HOME/objpose/sim; logs=$here/logs; mkdir -p "$logs"
# 8 MB shared-memory segment so 0.63 MB VLP-16 clouds stay off fragmented UDP (see fastdds_big_shm.xml)
export FASTRTPS_DEFAULT_PROFILES_FILE=$here/fastdds_big_shm.xml
# reach the PC (RViz) over the wired link even if Wi-Fi multicast fails
export ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-192.168.2.2}
# mounting yaw: the caller's LIDAR_YAW, else mount.env (written by objpose/new_place.sh finish), else the 2026-10-13 value
[ -n "${LIDAR_YAW:-}" ] || LIDAR_YAW=$(sed -n 's/^LIDAR_YAW=//p' "$here/mount.env" 2>/dev/null)
export LIDAR_X=${LIDAR_X:-0} LIDAR_Y=${LIDAR_Y:-0} LIDAR_Z=${LIDAR_Z:-1.215} LIDAR_YAW=${LIDAR_YAW:-1.2171}
( source "$HOME/objpose/nav/hdl_env.sh"
  # hdl first, the lidar 5 s later: a scan that arrives before prefiltering has base_link -> velodyne goes through
  # untransformed and becomes keyframe 1, so every later pose is off by the mounting (seen as a 1.9 m "jump" at
  # standstill, robot drawn rotated, 2026-10-06)
  setsid nohup ros2 launch "$here/real_hdl.launch.py" > "$logs/hdl.log" 2>&1 < /dev/null & echo $! > "$logs/hdl.pid"
  sleep 5
  setsid nohup python3 -u "$HOME/objpose/lidar/vlp16_publisher.py" > "$logs/vlp16.log" 2>&1 < /dev/null & echo $! > "$logs/vlp16.pid" )
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp USE_SIM_TIME=false
# the same mounting in the robot tree, for the scan / RViz
setsid nohup ros2 run tf2_ros static_transform_publisher --x $LIDAR_X --y $LIDAR_Y --z $LIDAR_Z --yaw $LIDAR_YAW \
  --frame-id base_link --child-frame-id velodyne --ros-args -r __node:=robot_lidar_tf -r /tf_static:=/a200_0881/tf_static \
  > "$logs/lidar_tf.log" 2>&1 < /dev/null & echo $! > "$logs/lidar_tf.pid"
setsid nohup python3 "$sim/hdl_map_relay.py" > "$logs/relay.log" 2>&1 < /dev/null & echo $! > "$logs/relay.pid"
setsid nohup python3 "$sim/hdl_grid.py" > "$logs/grid.log" 2>&1 < /dev/null & echo $! > "$logs/grid.pid"
python3 "$here/make_nav2_real_params.py" "$here/nav2_real.yaml" > /dev/null
sleep 5
setsid nohup ros2 launch "$here/real_nav2.launch.py" > "$logs/nav2.log" 2>&1 < /dev/null & echo $! > "$logs/nav2.pid"
# Nav2 for the web viewer (map, local costmap, goals): the object-pose hub proxies /nav/* to this (hub.py --nav-bridge)
setsid nohup python3 -u "$here/nav_bridge.py" > "$logs/nav_bridge.log" 2>&1 < /dev/null & echo $! > "$logs/nav_bridge.pid"
# last-resort escape when Nav2 stalls next to an obstacle (objpose/nav/stuck_guard.py; twist_mux priority 8 < joystick 10)
[ "${STUCK_GUARD:-1}" = 1 ] && { setsid nohup python3 -u "$HOME/objpose/nav/stuck_guard.py" > "$logs/stuck_guard.log" 2>&1 < /dev/null & echo $! > "$logs/guard.pid"; }
echo "started: vlp16 publisher, hdl_graph_slam, relay, grid, pointcloud_to_laserscan + Nav2 + stuck_guard(${STUCK_GUARD:-1}) (logs $logs); lidar mount $LIDAR_X $LIDAR_Y $LIDAR_Z yaw $LIDAR_YAW"
