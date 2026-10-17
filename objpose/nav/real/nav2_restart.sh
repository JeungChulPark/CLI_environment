#!/usr/bin/env bash
# (Re)start Nav2 for the real robot with nav2_real.yaml; retries the bring-up when a lifecycle reply is lost
# ("failed to send response to .../change_state (timeout)" leaves the lifecycle manager waiting).
here=$(cd "$(dirname "$0")" && pwd); logs=$here/logs
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=$here/fastdds_big_shm.xml ROS_STATIC_PEERS=${ROS_STATIC_PEERS:-192.168.2.2}
# saved-map mode (map_localizer running) keeps live obstacles in the planner, as real_loc_up.sh does
[ -z "${OBSTACLES_IN_PLAN:-}" ] && [ -f "$logs/localizer.pid" ] && export OBSTACLES_IN_PLAN=1
python3 "$here/make_nav2_real_params.py" "$here/nav2_real.yaml" > /dev/null
for attempt in 1 2 3; do
  [ -f "$logs/nav2.pid" ] && { kill -INT -- -"$(cat "$logs/nav2.pid")" 2>/dev/null; sleep 6; }
  # a stalled bring-up can leave a lifecycle manager behind outside the process group; two managers fight over the nodes
  for p in $(pgrep -f "__node:=lifecycle_manager_navigatio[n]"); do kill -9 "$p" 2>/dev/null; done
  setsid nohup ros2 launch "$here/real_nav2.launch.py" > "$logs/nav2.log" 2>&1 < /dev/null & echo $! > "$logs/nav2.pid"
  for i in $(seq 1 25); do grep -q "Managed nodes are active" "$logs/nav2.log" && { echo "nav2 active (attempt $attempt)"; exit 0; }; sleep 2; done
  echo "nav2 bring-up stalled (attempt $attempt), retrying"
done
exit 1
