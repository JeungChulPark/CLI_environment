#!/usr/bin/env bash
# Steps 2+4: hdl_graph_slam live pose + Nav2 (on the step-3 grid) while a recorded VLP-16 bag replays.
#   step2_nav_bag.sh <session> [rate] [extra ros2 bag play args]
# Goals: RViz "2D Goal Pose" (/goal_pose) from the PC, or ros2 action send_goal /navigate_to_pose.
set -eo pipefail
S=${1:?session}; RATE=${2:-0.5}; shift 2 || true
N=$HOME/objpose/nav; BAG=$HOME/DeepLearning/Dataset/$S/lidar; LOG=$N/logs/step2_$S
[ -f $N/maps/$S.yaml ] || { echo "no grid $N/maps/$S.yaml (run step1_map.sh + pcd2grid.py)"; exit 2; }
PIDS=()
cleanup(){ for p in "${PIDS[@]}"; do kill -TERM -$p 2>/dev/null || true; done; sleep 2
           for p in "${PIDS[@]}"; do kill -KILL -$p 2>/dev/null || true; done; }
trap cleanup EXIT INT TERM
( source $N/hdl_env.sh; exec setsid ros2 launch $N/hdl_nav.launch.py use_sim_time:=true \
    raw_points_qos:=reliable graph_keyframe_delta_trans:=0.5 graph_keyframe_delta_angle:=0.3 ) > $LOG.hdl.log 2>&1 &
PIDS+=($!)
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
sleep 6
echo "bag play $S rate=$RATE"
setsid ros2 bag play "$BAG" --clock 100 --rate $RATE --topics /velodyne_points /scan "$@" > $LOG.bag.log 2>&1 &
BAGP=$!; PIDS+=($BAGP)
setsid python3 $N/map_odom_relay.py --ros-args -p use_sim_time:=true > $LOG.relay.log 2>&1 &
PIDS+=($!)
# Nav2 needs map->base_link at activation (hdl publishes map->odom after its first optimization)
python3 $N/wait_tf.py map base_link 120
setsid ros2 launch $N/nav_hdl.launch.py map:=$N/maps/$S.yaml params:=$N/nav2_husky_hdl.yaml > $LOG.nav2.log 2>&1 &
PIDS+=($!)
wait $BAGP || true      # wait (unlike a foreground command) lets the TERM/INT trap run immediately
echo "bag finished"
