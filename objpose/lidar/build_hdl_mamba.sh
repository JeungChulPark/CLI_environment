#!/bin/bash
# build_hdl_mamba.sh — build hdl_graph_slam (ROS 2 Jazzy, robostack) on a Linux SLAM host without root and without a
# system compiler, e.g. the Samsung laptop (Ubuntu 24.04, 2026-09-28). Everything (ROS, PCL, g2o, gcc) comes from the
# micromamba env hdl_graph_slam_jazzy, which run_hdl_stream.sh activates at run time:
#   micromamba create -n hdl_graph_slam_jazzy --override-channels -c robostack-jazzy -c conda-forge python=3.12 \
#     ros-jazzy-ros-base colcon-common-extensions cmake ninja make pkg-config cxx-compiler c-compiler pcl eigen g2o \
#     suitesparse scipy pyyaml numpy ros-jazzy-{pcl-ros,pcl-conversions,geodesy,geographic-msgs,nmea-msgs} \
#     ros-jazzy-{interactive-markers,message-filters,tf2-ros,tf2-geometry-msgs,tf2-eigen,rosbag2} nlohmann_json libsqlite libopencv zlib
#   ~/objpose/lidar/build_hdl_mamba.sh        (sources in ~/objpose/hdlgraphslam_ws/src; log colcon_build.log)
set -euo pipefail
WS="${HDL_WS:-$HOME/objpose/hdlgraphslam_ws}"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"
set +u
eval "$("$MAMBA_ROOT_PREFIX/bin/micromamba" shell hook -s bash)"
micromamba activate "${HDL_ENV:-hdl_graph_slam_jazzy}"
set -u
cd "$WS"
colcon build --symlink-install --parallel-workers "${J:-4}" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_VGICP_CUDA=OFF -DBUILD_apps=OFF \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DCMAKE_INSTALL_RPATH="$CONDA_PREFIX/lib" > "$WS/colcon_build.log" 2>&1 || { tail -40 "$WS/colcon_build.log"; exit 1; }
tail -4 "$WS/colcon_build.log"
set +u; source "$WS/install/setup.bash"; set -u
ros2 pkg executables hdl_graph_slam
