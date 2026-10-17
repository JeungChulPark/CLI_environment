#!/bin/bash
# stop_live.sh — stop everything the live pipeline started: launcher (optional), hub (+ SAM-6D + ssh streamer), this PC's
# RealSense driver, and any LiDAR streamer / velodyne driver / hdl nodes left on the laptop.
#   bash objpose/stop_live.sh            # hub + camera + laptop leftovers (the launcher on :8765 stays up)
#   bash objpose/stop_live.sh --all      # also the launcher
# Patterns are matched from inside this script file, so they never hit the caller's own shell.
# the laptop answers as `laptop` (192.168.2.1) or `laptop240` (192.168.240.1) depending on its cable profile
if [ -z "${LAPTOP_SSH:-}" ]; then
  for a in laptop laptop240; do ssh -o ConnectTimeout=3 "$a" true 2>/dev/null && LAPTOP_SSH="$a" && break; done
fi
LAPTOP_SSH="${LAPTOP_SSH:-laptop}"
stop() {  # $1 label, $2 pgrep args...
  local label="$1"; shift
  local pids; pids=$(pgrep "$@" | tr '\n' ' ')
  [ -n "$pids" ] || { echo "[stop] $label: none"; return; }
  echo "[stop] $label: $pids"
  kill -INT $pids 2>/dev/null; for i in $(seq 1 20); do pgrep "$@" >/dev/null || break; sleep 1; done
  pgrep "$@" >/dev/null && { kill -KILL $(pgrep "$@") 2>/dev/null; sleep 1; }
}
# a navigation run (hub started with --nav-bridge): keep the laptop's Nav2 / localizer logs with the run, they are
# overwritten by the next start and replay_run.py reads the goals from them
HUB_OUT=$(pgrep -af 'python -u [^ ]*objpose/pc/hub\.py' | grep -e '--nav-bridge' | sed -n 's/.* --out \([^ ]*\).*/\1/p' | head -1)
if [ -n "$HUB_OUT" ] && [ -d "$HUB_OUT" ]; then
  mkdir -p "$HUB_OUT/nav_laptop" && scp -q -o ConnectTimeout=5 "$LAPTOP_SSH:objpose/nav/real/logs/*.log" "$HUB_OUT/nav_laptop/" 2>/dev/null \
    && echo "[stop] laptop Nav logs -> $HUB_OUT/nav_laptop"
fi
[ "${1:-}" = "--all" ] && stop "launcher" -f 'python -u [^ ]*objpose/pc/launcher\.py'
stop "hub (SAM-6D, ssh streamer)" -f 'python -u [^ ]*objpose/pc/hub\.py'
stop "sam6d_infer" -f '[^ ]*realtime/sam6d_infer\.py'
stop "realsense launch" -f 'ros2 launch realsense2_camera'
stop "realsense node" -x realsense2_camera_node
for f in /dev/shm/sam6d_*; do [ -e "$f" ] && rm -f "$f"; done
# The streamers always go. The driver / hdl nodes go only when they are ours: the Nav2 stack (nav/real/real_up.sh) runs
# the same node names on ROS_DOMAIN_ID 0 and `run_live.sh hdl-nav` shares them, so those are left alone.
ssh -o ConnectTimeout=5 "$LAPTOP_SSH" 'p=$(pgrep -f "lidar/hdl_stream\.py|lidar/lidar_stream\.py|nav/real/loc_stream\.py" | tr "\n" " ");
  for q in $(pgrep -f "lidar/vlp16_publisher\.py|velodyne_driver_node|velodyne_transform_node|velodyne_laserscan_node|graph_slam_node|scan_matching_odometry_node|prefiltering_node|floor_detection_node|map2odom_publisher"); do
    d=$(tr "\0" "\n" < /proc/$q/environ 2>/dev/null | sed -n "s/^ROS_DOMAIN_ID=//p"); [ -n "$d" ] && [ "$d" != 0 ] && p="$p$q "; done;
  if [ -n "$p" ]; then echo "[stop] laptop: $p"; kill -INT $p 2>/dev/null; sleep 5; kill -KILL $p 2>/dev/null; else echo "[stop] laptop: none"; fi' 2>/dev/null
echo "[stop] ports: $(ss -lnt | grep -E ':(8765|8766|17005|17007) ' | awk '{print $4}' | tr '\n' ' ')"
