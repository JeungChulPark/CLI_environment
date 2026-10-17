#!/usr/bin/env bash
# hdl_graph_slam (VLP-16 points) + map->odom relay + live 2D grid + Nav2 against the running simulation.
# Run sim_up.sh first. hdl runs in the micromamba env (~/objpose/nav/hdl_env.sh); the rest uses /opt/ros/jazzy.
set -e
here=$(cd "$(dirname "$0")" && pwd)
"$here/lidar_bridge.sh"   # 1-deep-queue lidar bridge (see lidar_bridge.yaml)
mkdir -p "$here/logs"
( source "$HOME/objpose/nav/hdl_env.sh"; export GZ_IP= ;
  setsid nohup ros2 launch "$here/hdl_sim.launch.py" > "$here/logs/hdl.log" 2>&1 < /dev/null & echo $! > "$here/logs/hdl.pid" )
source "$here/sim_env.sh"
setsid nohup python3 "$here/hdl_map_relay.py" > "$here/logs/relay.log" 2>&1 < /dev/null & echo $! > "$here/logs/relay.pid"
setsid nohup python3 "$here/hdl_grid.py" > "$here/logs/grid.log" 2>&1 < /dev/null & echo $! > "$here/logs/grid.pid"
sleep 8
python3 "$here/make_nav2_sim_params.py" "$here/nav2_sim.yaml" > /dev/null
setsid nohup ros2 launch "$here/nav2_sim.launch.py" > "$here/logs/nav2.log" 2>&1 < /dev/null & echo $! > "$here/logs/nav2.pid"
echo "hdl_graph_slam + relay + grid + Nav2 started (logs in $here/logs)"
