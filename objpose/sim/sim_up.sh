#!/usr/bin/env bash
# Start Gazebo + simulated Husky A200 on the laptop.
# usage: sim_up.sh [world] [--headless]   (world default warehouse; --headless = no Gazebo window)
set -e
here=$(cd "$(dirname "$0")" && pwd)
source "$here/sim_env.sh"
world=warehouse; headless=0
for a in "$@"; do case $a in --headless) headless=1;; *) world=$a;; esac; done
mkdir -p "$SIM_DIR" "$here/logs"
cp "$here/robot.yaml" "$SIM_DIR/robot.yaml"
gen=()
if [ $headless = 1 ]; then
  launch=("$here/sim_headless.launch.py")
  # Clearpath's generator hard-codes the VLP-16 minimum range to 0.9 m (real driver: 0.4 m), which hides anything
  # beside the robot. Generate the description here, set SIM_LIDAR_MIN_RANGE (default 0.4), launch without regenerating.
  ros2 run clearpath_generator_common generate_description -s "$SIM_DIR/" > /dev/null
  ros2 run clearpath_generator_common generate_semantic_description -s "$SIM_DIR/" > /dev/null
  ros2 run clearpath_generator_gz generate_launch -s "$SIM_DIR/" > /dev/null
  ros2 run clearpath_generator_gz generate_param -s "$SIM_DIR/" > /dev/null
  sed -i -E "s/(velodyne_lidar [^>]*min_range=\")[0-9.]+/\1${SIM_LIDAR_MIN_RANGE:-0.4}/" "$SIM_DIR/robot.urdf.xacro"
  gen=(generate:=false)
else
  launch=(clearpath_gz simulation.launch.py)
fi
setsid nohup ros2 launch "${launch[@]}" world:=$world setup_path:=$SIM_DIR/ "${gen[@]}" > "$here/logs/sim.log" 2>&1 &
echo $! > "$here/logs/sim.pid"
echo "Gazebo ($world, headless=$headless) started, pid $(cat "$here/logs/sim.pid"), log $here/logs/sim.log"
