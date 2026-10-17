#!/bin/bash
# run_live.sh — LIVE sensors, no bags (2026-09-17). One LiDAR backend at a time:
#   bash objpose/run_live.sh hdl   [hub.py options]   laptop: VLP-16 -> hdl_graph_slam (graph + loop closure), pose port 17005
#   LAPTOP_SSH=samsung bash objpose/run_live.sh hdl-nav [hub.py options]
#                                     laptop: the hdl_graph_slam of the Nav2 stack (nav/real/real_up.sh, already running) is
#                                     shared; nothing is launched there but the pose streamer (--attach), pose port 17005
#   LAPTOP_SSH=samsung bash objpose/run_live.sh loc-nav [hub.py options]
#                                     laptop: fixed-map navigation (nav/real/real_loc_up.sh, already running); the pose of
#                                     its localizer is forwarded (nav/real/loc_stream.py), world = the saved map, port 17007
#   bash objpose/run_live.sh kiss  [hub.py options]   laptop: VLP-16 -> KISS-ICP odometry (no keyframes),      pose port 17007
#   bash objpose/run_live.sh orb   [hub.py options]   laptop: its own D455f -> ORB-SLAM3 RGB-D (camera rig RT),  pose port 17001
#   LAPTOP_SSH=samsung bash objpose/run_live.sh orb-direct [hub.py options]
#                                     Samsung laptop (no ROS/root): its D455f -> rs_capture.py | slam_stream, pose port 17001
#   bash objpose/run_live.sh rtab  [hub.py options]   laptop: its own D455f -> RTAB-Map RGB-D (Humble),          pose port 17008
#   laptop etri@192.168.2.1 (ssh alias `laptop`, port 10022): ~/objpose/lidar/{run_hdl_stream.sh,run_lidar_stream.sh} --session live
#   this PC 192.168.2.2: D455f -> realsense2_camera (ROS_DOMAIN_ID 72) -> hub.py --sam-session live -> SAM-6D -> object memory
#                        -> viewer http://192.168.2.2:${HTTP_PORT:-8765}
#   Ctrl+C stops the hub (SAM-6D, viewer, the laptop streamer over ssh). The RealSense driver stays up across runs so that
#   switching the LiDAR backend from the launcher takes ~25 s instead of ~45 s; `bash objpose/stop_live.sh` stops it all.
#   hdl-nav / loc-nav: the viewer opens in a new browser window (OPEN_VIEWER=0 to skip) and the run is recorded for
#   objpose/pc/replay_run.py <out>: events.jsonl.gz (viewer frames, RECORD_HZ) + nav.jsonl.gz (map, obstacles, scan, goals);
#   stop_live.sh adds the laptop's Nav logs as <out>/nav_laptop/.
#   env: DURATION_S (0 = until Ctrl+C), TAU_S (skip the clock measurement), OUT, HTTP_PORT, X_LIDAR, LAPTOP_SSH, LAPTOP_HOST, PC_HOST
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
RS_ENV=/home/jucpark/anaconda3/envs/realsense
BACKEND="${1:?usage: run_live.sh hdl|hdl-nav|loc-nav|kiss|orb|orb-direct|rtab [hub.py options]}"; shift
HUB_EXTRA=("$@")
# The laptop's cable profile has come up as 192.168.2.1/24 ("laptop-link") or 192.168.240.1/30 ("sam-ptp-ipv6-slave")
# depending on how it was re-plugged (2026-09-17). This PC carries both subnets on the cable, so pick whichever answers.
if [ -z "${LAPTOP_HOST:-}" ]; then
  for cand in "192.168.2.1 laptop 192.168.2.2" "192.168.240.1 laptop240 192.168.240.2"; do
    set -- $cand
    if ping -c 1 -W 1 "$1" >/dev/null 2>&1; then LAPTOP_HOST="$1"; LAPTOP_SSH="${LAPTOP_SSH:-$2}"; PC_HOST="${PC_HOST:-$3}"; break; fi
  done
  set --
  [ -n "${LAPTOP_HOST:-}" ] || { echo "노트북이 192.168.2.1 / 192.168.240.1 어느 쪽에서도 응답하지 않습니다 (케이블·프로파일 확인)" >&2; exit 1; }
fi
LAPTOP_SSH="${LAPTOP_SSH:-laptop}"
PC_HOST="${PC_HOST:-192.168.2.2}"
HTTP_PORT="${HTTP_PORT:-8765}"
X_LIDAR_ENV="${X_LIDAR:-}"                                               # set by the caller: wins for every backend
# Husky mounting in use (nav/real/mount.env, rewritten by new_place.sh finish): lidar yaw and the lidar -> camera file
MOUNT_ENV="$REPO/objpose/nav/real/mount.env"
[ -n "${LIDAR_YAW:-}" ] || LIDAR_YAW="$(sed -n 's/^LIDAR_YAW=//p' "$MOUNT_ENV" 2>/dev/null)"
X_MOUNT="$(sed -n 's/^X_LIDAR_SAM=//p' "$MOUNT_ENV" 2>/dev/null)"; X_MOUNT="$REPO/objpose/${X_MOUNT:-rt/261013_husky/X_lidar_sam_261013_husky.json}"
X_LIDAR="${X_LIDAR:-$REPO/objpose/rt/260915/X_lidar_sam_260915.json}"   # VLP-16 -> SAM camera on the 260915 rig
X_CAM="${X_CAM:-$REPO/objpose/rt/260915/X_slam_sam_260915.json}"         # SLAM camera (laptop D455f) -> SAM camera, 260915 rig
OUT="${OUT:-$REPO/objpose/output/live_${BACKEND}_$(date +%y%m%d_%H%M%S)}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
mkdir -p "$OUT"

case "$BACKEND" in
  hdl)  LANE=(--slam hdl --slam-port 17005); REMOTE_NEEDS=('~/objpose/lidar/run_hdl_stream.sh' '~/objpose/hdlgraphslam_ws/install/setup.bash') ;;
  hdl-nav) LANE=(--slam hdl --slam-port 17005 --slam-param attach=1 --slam-param domain=0 --slam-param tf_topic=/hdl/tf
                 --slam-param graph_odom_frame=hdl_odom)
        # camera as mounted on the Husky (static LiDAR-to-depth registration, 2026-10-06); the 260915 file is the old rig
        X_LIDAR="${X_LIDAR_ENV:-$X_MOUNT}"
        REMOTE_NEEDS=('~/objpose/lidar/run_hdl_stream.sh' '~/objpose/hdlgraphslam_ws/install/setup.bash' '~/objpose/nav/real/fastdds_big_shm.xml') ;;
  loc-nav) LANE=(--slam lidar --slam-port 17007 --lidar-runner "~/objpose/nav/real/run_loc_stream.sh"
                 --slam-name hdl --slam-label "hdl_graph_slam")
        X_LIDAR="${X_LIDAR_ENV:-$X_MOUNT}"
        REMOTE_NEEDS=('~/objpose/nav/real/run_loc_stream.sh' '~/objpose/nav/real/loc_stream.py') ;;
  kiss) LANE=(--slam lidar --slam-port 17007 --lidar-runner "~/objpose/lidar/run_lidar_stream.sh"); REMOTE_NEEDS=('~/objpose/lidar/run_lidar_stream.sh' '~/objpose/lidar/lidar_stream.py') ;;
  # FEATURES = ORBextractor.nFeatures written into the laptop's ORB-SLAM3 settings (launcher buttons f2000/f3000/f4000)
  orb)  LANE=(--slam orbslam3 --slam-port 17001 --mac-runner "~/objpose/live_orb/run_orb_live.sh" --features "${FEATURES:-2000}")
        REMOTE_NEEDS=('~/objpose/live_orb/run_orb_live.sh' '~/objpose/live_orb/orb_live_stream.py' '~/CLI_environment/orbslam_ws/install/setup.bash') ;;
  # ROS-free ORB-SLAM3 of linux_slam/ (the Samsung laptop): run_slam.sh --session live pipes rs_capture.py into slam_stream
  orb-direct) LANE=(--slam orbslam3 --slam-port 17001 --mac-runner "~/objpose/run_slam.sh" --features "${FEATURES:-2000}")
        REMOTE_NEEDS=('~/objpose/run_slam.sh' '~/objpose/build/slam_stream' '~/objpose/linux_slam/rs_capture.py' '~/objpose/deps/pyrs/pyrealsense2') ;;
  rtab) LANE=(--slam rtabmap --slam-port 17008 --mac-runner "~/objpose/live_orb/run_rtab_live.sh")
        REMOTE_NEEDS=('~/objpose/live_orb/run_rtab_live.sh' '~/objpose/live_orb/rtab_live_stream.py' '~/CLI_environment/rtabmap_ws/install/setup.bash') ;;
  *) echo "backend: hdl | hdl-nav | loc-nav | kiss | orb | orb-direct | rtab (got $BACKEND)" >&2; exit 2 ;;
esac

if pgrep -f '^[^ ]*python[^ ]* -u [^ ]*objpose/pc/hub\.py' >/dev/null; then
  echo "다른 허브가 이미 실행 중입니다 (한 번에 하나만): $(pgrep -af 'objpose/pc/hub.py' | cut -c1-120)" >&2; exit 3
fi
# the paths are quoted above so that `~` expands on the LAPTOP (inside the remote shell), not here
ssh "$LAPTOP_SSH" "for f in ${REMOTE_NEEDS[*]}; do eval f=\$f; test -e \"\$f\" || { echo \"missing on laptop: \$f\" >&2; exit 1; }; done" \
  || { echo "노트북($LAPTOP_SSH)에 $BACKEND 스트리머 구성 파일이 없습니다" >&2; exit 1; }

if [ "$BACKEND" = loc-nav ]; then
  ssh "$LAPTOP_SSH" 'pgrep -f "nav/real/map_localizer\.py" >/dev/null' \
    || { echo "노트북($LAPTOP_SSH)에 고정 지도 위치 추정이 없습니다. 먼저 ~/objpose/nav/real/real_loc_up.sh 를 실행하세요" >&2; exit 1; }
fi
if [ "$BACKEND" = hdl-nav ] || [ "$BACKEND" = loc-nav ]; then
  # Both report base_link, so the hub's pose is the robot (x = forward) and the viewer can draw its heading.
  # base_link -> camera = base_link -> velodyne (LIDAR_X/Y/Z/YAW, same defaults as nav/real/real_up.sh) . velodyne -> camera
  "$PY" - "$X_LIDAR" "$OUT/X_base_sam.json" "${LIDAR_X:-0}" "${LIDAR_Y:-0}" "${LIDAR_Z:-1.215}" "${LIDAR_YAW:-1.2171}" <<'PYEOF'
import json, sys, numpy as np
src, dst, x, y, z, yaw = sys.argv[1], sys.argv[2], *map(float, sys.argv[3:7])
d = json.load(open(src)); c, s = np.cos(yaw), np.sin(yaw)
B = np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
d["T_slam_sam"] = (B @ np.array(d["T_slam_sam"])).tolist()
d["source"] = f"base_link -> SAM camera = mounting ({x}, {y}, {z}, yaw {yaw}) . [{d.get('source', src)}]"
json.dump(d, open(dst, "w"), indent=1)
PYEOF
  X_LIDAR="$OUT/X_base_sam.json"
  # keep the viewer events (camera image with overlays) so the run can be played again: objpose/pc/replay_run.py <out>
  HUB_EXTRA=(--map-min-obs "${MAP_MIN_OBS:-3}" --record-events-hz "${RECORD_HZ:-5}" --nav-bridge "http://$LAPTOP_HOST:${NAV_BRIDGE_PORT:-8770}" "${HUB_EXTRA[@]}")
  [ "$BACKEND" != hdl-nav ] || ssh "$LAPTOP_SSH" 'pgrep -x graph_slam_node >/dev/null' \
    || { echo "노트북($LAPTOP_SSH)에 Nav 쪽 hdl_graph_slam이 없습니다. 먼저 ~/objpose/nav/real/real_up.sh 를 실행하세요" >&2; exit 1; }
fi

# 1. RealSense driver on this PC (only if nothing publishes the aligned depth yet)
CAM_PID=""
# "have a camera" = colour+depth FRAMES arrive (a driver killed mid-stream keeps its topics advertised while its UVC
# stream is stuck, so a topic-list check is not enough)
have_cam() { ROS_DOMAIN_ID="$ROS_DOMAIN_ID" timeout 8 "$PY" - <<'PYEOF' 2>/dev/null
import sys, time
sys.path.insert(0, "/home/jucpark/DeepLearning/CLI_environment_server/objpose/pc")
from live_session import LiveSamSession
try:
    s = LiveSamSession(timeout_s=5)
except Exception:
    sys.exit(1)
s.close()
PYEOF
}
if ! have_cam; then
  # --no-daemon: the ros2 daemon can keep a dead driver's topics in its graph cache for a while
  if "$RS_ENV/bin/ros2" topic list --no-daemon --spin-time 3 2>/dev/null | grep -q '^/camera/camera/aligned_depth_to_color/image_raw$'; then
    echo "RealSense 토픽은 있는데 프레임이 오지 않습니다: 멈춘 드라이버가 남아 있습니다. 먼저 정리하세요:" >&2
    echo "  for p in \$(pgrep -f '[r]s_launch.py'); do kill -INT \$p; done; sleep 3; pkill -INT -x realsense2_camera_node" >&2
    exit 1
  fi
  echo "[live] starting realsense2_camera (domain $ROS_DOMAIN_ID) -> $OUT/realsense.log" >&2
  # No initial_reset by default: opening the stream right after the hardware reset failed on both D455f cameras with
  # "uvc streamer watchdog triggered / Depth stream start failure" (2026-09-17, the 12:07 and 12:39 runs here). The reset
  # is kept as a one-time fallback for a camera wedged by a killed driver.
  # set +u: conda's activate/deactivate hooks reference unset variables and would kill this subshell under -u.
  start_cam() {
    ( set +u; source /home/jucpark/anaconda3/etc/profile.d/conda.sh && conda activate realsense &&
      exec setsid ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_sync:=true initial_reset:="$1" \
           rgb_camera.color_profile:=640x480x30 depth_module.depth_profile:=640x480x30 ) > "$OUT/realsense_reset_$1.log" 2>&1 &
    CAM_PID=$!
  }
  wait_cam() {   # $1 = attempts x 5 s
    for i in $(seq 1 "$1"); do grep -q "RealSense Node Is Up" "$OUT/realsense_reset_$2.log" 2>/dev/null && have_cam && return 0; sleep 5; done
    return 1
  }
  stop_cam() { kill -INT -- -"$(ps -o pgid= "$CAM_PID" 2>/dev/null | tr -d ' ')" 2>/dev/null; sleep 6; kill -KILL -- -"$(ps -o pgid= "$CAM_PID" 2>/dev/null | tr -d ' ')" 2>/dev/null; }
  start_cam false
  if ! wait_cam 8 false; then
    echo "[live] no camera frames in 40 s without reset: retrying once with initial_reset" >&2
    stop_cam; sleep 2
    start_cam true
    wait_cam 24 true || { echo "RealSense 프레임이 오지 않았습니다 ($OUT/realsense_reset_*.log)" >&2; stop_cam; exit 1; }
  fi
fi
# the driver runs in its own session (setsid) and is NOT tied to this script (which exec's the hub below): it survives
# backend switches and is stopped by objpose/stop_live.sh

# 2. clock offset laptop - PC  (LiDAR stamps are on the laptop clock, camera stamps on this PC's)
if [ -z "${TAU_S:-}" ]; then
  TAU_S="$(python3 "$REPO/objpose/tools/clock_offset.py" --host "$LAPTOP_HOST" --ssh "$LAPTOP_SSH" | sed -n 's/^offset_s=//p')"
  [ -n "$TAU_S" ] || { echo "시계차 측정 실패" >&2; exit 1; }
fi
echo "[live] backend=$BACKEND  SAM tau (laptop - PC) = $TAU_S s  out=$OUT  viewer http://$PC_HOST:$HTTP_PORT" >&2

# Navigation backends: open the viewer in a new browser window once the hub answers (OPEN_VIEWER=0 to skip)
if { [ "$BACKEND" = hdl-nav ] || [ "$BACKEND" = loc-nav ]; } && [ "${OPEN_VIEWER:-1}" = 1 ]; then
  ( for i in $(seq 1 120); do curl -s -o /dev/null -m 1 "http://127.0.0.1:$HTTP_PORT/" && break; sleep 1; done
    export DISPLAY="${DISPLAY:-:1}"
    if command -v google-chrome >/dev/null; then setsid nohup google-chrome --new-window "http://localhost:$HTTP_PORT/" >/dev/null 2>&1 &
    else setsid nohup xdg-open "http://localhost:$HTTP_PORT/" >/dev/null 2>&1 & fi ) &
fi

# 3. hub: SAM-6D + object memory + viewer here, the LiDAR streamer on the laptop over ssh (--link direct)
case "$BACKEND" in orb|orb-direct|rtab) XFILE="$X_CAM" ;; *) XFILE="$X_LIDAR" ;; esac
exec "$PY" -u "$REPO/objpose/pc/hub.py" "${LANE[@]}" --link direct --extrinsic "$XFILE" \
  --mac-slam-session live --slam-host "$LAPTOP_HOST" --mac-host "$LAPTOP_SSH" --pc-host "$PC_HOST" \
  --http-host 0.0.0.0 --http-port "$HTTP_PORT" --sam-session live --sam-tau-s "$TAU_S" \
  --rate 1.0 --duration-s "${DURATION_S:-0}" --out "$OUT" "${HUB_EXTRA[@]}"
