#!/usr/bin/env bash
# Start mount_yaw.py next to a running real_up.sh / real_loc_up.sh: lidar mounting yaw from straight manual driving.
# Result: logs/mount_yaw.json (real_down.sh stops it with the rest).
here=$(cd "$(dirname "$0")" && pwd); logs=$here/logs; mkdir -p "$logs"
[ -f "$logs/mount_yaw.pid" ] && kill -INT -- -"$(cat "$logs/mount_yaw.pid")" 2>/dev/null
rm -f "$logs/mount_yaw.json"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=$here/fastdds_big_shm.xml
setsid nohup python3 -u "$here/mount_yaw.py" > "$logs/mount_yaw.log" 2>&1 < /dev/null & echo $! > "$logs/mount_yaw.pid"
echo "mount_yaw started (pid $(cat "$logs/mount_yaw.pid")), result $logs/mount_yaw.json"
