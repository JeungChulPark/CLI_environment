#!/bin/bash
# build_hdl_linux.sh — build hdl_graph_slam (ROS 2 Jazzy, robostack in conda) on a Linux SLAM host without system ROS,
# e.g. the Ubuntu box 192.168.20.3. Run ON that host after `conda create -n hdl_graph_slam_jazzy ...` (same package
# list as this PC's env; conda-forge linux-64 has g2o — cholmod solver is absent, the sources use eigen/csparse).
#   ssh slam3 'bash ~/objpose/lidar/build_hdl_linux.sh'        (idempotent; log in ~/objpose/hdlgraphslam_ws/colcon_build.log)
set -euo pipefail
WS="${HDL_WS:-$HOME/objpose/hdlgraphslam_ws}"
CONDA_SH="${CONDA_SH:-$HOME/anaconda3/etc/profile.d/conda.sh}"
[ -f "$CONDA_SH" ] || CONDA_SH="$HOME/miniconda3/etc/profile.d/conda.sh"
set +u; source "$CONDA_SH"; conda activate "${HDL_ENV:-hdl_graph_slam_jazzy}"; set -u
cd "$WS"
# the system compiler (as on the .5 workstation): conda's ctng gcc crashed (ICE) on graph_slam_node.cpp on .3
CC_="${CC_:-/usr/bin/gcc}"; CXX_="${CXX_:-/usr/bin/g++}"
rm -rf "$WS/build/hdl_graph_slam"        # a package configured with the other compiler must be reconfigured
colcon build --symlink-install --parallel-workers "${J:-8}" --cmake-args -DCMAKE_C_COMPILER="$CC_" -DCMAKE_CXX_COMPILER="$CXX_" -DCMAKE_BUILD_TYPE=Release -DBUILD_VGICP_CUDA=OFF -DBUILD_apps=OFF \
  > "$WS/colcon_build.log" 2>&1 || { tail -40 "$WS/colcon_build.log"; exit 1; }
tail -4 "$WS/colcon_build.log"
set +u; source "$WS/install/setup.bash"; set -u
ros2 pkg executables hdl_graph_slam
