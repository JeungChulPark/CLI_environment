#!/usr/bin/env bash
# PC side of the Gazebo simulation: RViz2 (conda env ros_jazzy) in the robot namespace /a200_0881, following the sim /clock.
# Raw VLP-16 points are off by default: RViz pulling 0.43 MB x 20 Hz over the LAN stalled the laptop lidar bridge
# (scans reached Nav2/hdl 3-46 s late). Enable "VLP-16 (live)" only briefly.
# "Nav2 Goal" / "2D Goal Pose" goes to /a200_0881/goal_pose, the Navigation 2 panel to /a200_0881/navigate_to_pose.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${CONDA_SH:-$HOME/anaconda3/etc/profile.d/conda.sh}"
conda activate ros_jazzy
# simulation domain 5 (sim_env.sh), apart from the real robot on domain 0
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-5} RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# discover the laptop over the wired link: multicast discovery rode on the shared Wi-Fi and broke when the hotspot
# renumbered (2026-10-02); a static unicast peer does not depend on Wi-Fi
export ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-192.168.2.1}
NS=/a200_0881
exec rviz2 -d "$HERE/husky_sim.rviz" "$@" --ros-args -r __ns:=$NS -r /tf:=$NS/tf -r /tf_static:=$NS/tf_static -p use_sim_time:=true
