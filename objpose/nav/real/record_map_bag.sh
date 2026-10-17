#!/usr/bin/env bash
# Record the raw data of a mapping run (VLP-16 points, wheel/EKF odom, robot TF) so the map can be rebuilt offline.
#   record_map_bag.sh <name>   -> maps/<name>_bag ; stop with: kill -TERM $(pgrep -f "ros2 bag recor[d]")
#   (a backgrounded recorder ignores SIGINT; SIGTERM still closes the bag and writes metadata.yaml)
here=$(cd "$(dirname "$0")" && pwd); name=${1:?name}; mkdir -p "$here/maps"
export FASTRTPS_DEFAULT_PROFILES_FILE=$here/fastdds_big_shm.xml
source "$HOME/objpose/nav/hdl_env.sh"
setsid nohup ros2 bag record -o "$here/maps/${name}_bag" /velodyne_points /a200_0881/platform/odom/filtered \
  /a200_0881/platform/odom /a200_0881/tf /a200_0881/tf_static /odom /hdl/tf > "$here/logs/bag.log" 2>&1 < /dev/null &
echo $! > "$here/logs/bag.pid"; echo "recording -> $here/maps/${name}_bag (pid $(cat "$here/logs/bag.pid"))"
