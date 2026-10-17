# source me: micromamba ROS 2 Jazzy env with hdl_graph_slam overlay (same as lidar/run_hdl_stream.sh)
set +u
export MAMBA_ROOT_PREFIX="$HOME/micromamba"
eval "$("$HOME/micromamba/bin/micromamba" shell hook -s bash)"
micromamba activate hdl_graph_slam_jazzy
source "$HOME/objpose/hdlgraphslam_ws/install/setup.bash"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
