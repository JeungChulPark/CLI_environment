#!/usr/bin/env bash
# Step 1: build a 3D map with hdl_graph_slam from a recorded VLP-16 bag and save it as PCD.
#   step1_map.sh <session> [rate]     e.g. step1_map.sh 260915_inlong_middle 1.0
set -eo pipefail
S=${1:?session}; RATE=${2:-1.0}
BAG=$HOME/DeepLearning/Dataset/$S/lidar
OUT=$HOME/objpose/nav/maps/$S.pcd; LOG=$HOME/objpose/nav/logs/step1_$S
source "$HOME/objpose/nav/hdl_env.sh"
setsid ros2 launch hdl_graph_slam hdl_graph_slam_501.launch.py use_sim_time:=true raw_points_qos:=reliable graph_keyframe_delta_trans:=${KF_TRANS:-0.5} graph_keyframe_delta_angle:=${KF_ANGLE:-0.3} > $LOG.hdl.log 2>&1 &
HDL=$!
cleanup(){ kill -TERM -$HDL 2>/dev/null || true; sleep 2; kill -KILL -$HDL 2>/dev/null || true; }
trap cleanup EXIT
sleep 6
/usr/bin/time -v ros2 bag play "$BAG" --clock 100 --rate $RATE --topics /velodyne_points > $LOG.bag.log 2>&1
sleep 5   # let the last optimization run
ros2 service call /hdl_graph_slam/save_map hdl_graph_slam/srv/SaveMap "{utm: false, resolution: 0.05, destination: $OUT}" | tail -2
grep -E "loop|stage8 map_points" $LOG.hdl.log | tail -3
ros2 service call /hdl_graph_slam/dump_graph hdl_graph_slam/srv/DumpGraph "{destination: $HOME/objpose/nav/maps/${S}_dump}" | tail -1
ls -l $OUT; ls $HOME/objpose/nav/maps/${S}_dump | wc -l
