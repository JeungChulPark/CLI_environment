#!/usr/bin/env bash
# PC side of Husky navigation: RViz2 (ROS 2 Jazzy, conda env ros_jazzy) on the same DDS domain as the laptop
# (192.168.2.1) that runs hdl_graph_slam + Nav2. "2D Goal Pose" sends /goal_pose to Nav2's bt_navigator.
#   objpose/nav/rviz_nav.sh          live robot (wall clock)
#   objpose/nav/rviz_nav.sh --sim    recorded bag on the laptop (follow its /clock)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${CONDA_SH:-$HOME/anaconda3/etc/profile.d/conda.sh}"
conda activate ros_jazzy
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ARGS=()
[ "${1:-}" = "--sim" ] && { shift; ARGS=(--ros-args -p use_sim_time:=true); }
exec rviz2 -d "$HERE/husky_nav.rviz" "$@" "${ARGS[@]}"
