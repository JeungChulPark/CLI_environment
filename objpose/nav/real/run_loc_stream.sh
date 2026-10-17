#!/usr/bin/env bash
# Pose streamer of the fixed-map navigation mode (real_loc_up.sh) for the object-pose hub; same DDS settings as that stack.
#   ~/objpose/nav/real/run_loc_stream.sh --connect <hub ip>:17007      (the hub adds --session/--mode/--rate; ignored)
here=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=$here/fastdds_big_shm.xml
exec python3 -u "$here/loc_stream.py" "$@"
