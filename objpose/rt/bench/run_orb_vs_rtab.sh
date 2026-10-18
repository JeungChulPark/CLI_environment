#!/bin/bash
# run_orb_vs_rtab.sh — ORB-SLAM3 vs RTAB-Map, each without / with the xsens IMU, on ONE session, REPS times, then
# map_eval.py (localisation + map quality against the LiDAR pseudo-GT).
#   bash objpose/rt/bench/run_orb_vs_rtab.sh [session=260901_cbnu_bigeightcircle] [REPS=3]
# Env: SLAM_STREAM = slam_stream built with the .mappoints.xyz dump (objpose/src/slam_stream.cc), default ~/objpose/build
#   orb       ORB-SLAM3 NF2000, real-time paced                     orb_gyro  + gyro aid (rig_<name>.json, our patch)
#   rtab      RTAB-Map, own RGB-D visual odometry, offline unpaced  rtab_imu  + xsens orientation (stock RTAB-Map:
#             OdometryF2M bundle adjustment + per-node gravity links), same rig
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; REPO="$(cd "$HERE/../../.." && pwd)"
S="${1:-260901_cbnu_bigeightcircle}"; REPS="${2:-3}"
d="$HOME/Documents/DefenseMeta/Dataset/$S"; name=${S#260901_cbnu_}; rig="$REPO/objpose/rt/260901/rtab_odom/rigs/rig_$name.json"
SS="${SLAM_STREAM:-$HOME/objpose/build/slam_stream}"
out="$HERE/out_cmp/$S"; mkdir -p "$out"
settings=$(ls "$HERE"/out/$S/orbslam3_*.yaml | head -1)
orb() {  # $1 dir, rest extra args
  local r=$1; shift
  DYLD_LIBRARY_PATH="$(dirname "$SS"):$HOME/objpose/deps/install/lib:/opt/homebrew/opt/opencv@4/lib:/opt/homebrew/opt/libomp/lib" \
    "$SS" --session "$d/SLAM" --settings "$settings" --vocab "$HOME/objpose/Vocabulary/ORBvoc.txt" --features 2000 \
    --time-source frame --mode offline --pace 1.0 "$@" --out "$r/live_tum.txt" > "$r/run.log" 2>&1
}
for rep in $(seq 1 "$REPS"); do
  for sys in orb orb_gyro rtab rtab_imu; do
    r="$out/$sys/rep$rep"; [ -s "$r/final_tum.txt" ] && continue; mkdir -p "$r"
    echo "[$(date +%T)] $sys rep$rep"
    case $sys in
      orb)      orb "$r" ;;
      orb_gyro) orb "$r" --gyro "$rig" --imu "$d/imu" ;;
      rtab)     "$HOME/objpose/run_slam.sh" --slam rtabmap --session "$d/SLAM" --time-source frame --mode offline \
                  --out "$r/live_tum.txt" > "$r/run.log" 2>&1 ;;
      rtab_imu) "$HOME/objpose/run_slam.sh" --slam rtabmap --session "$d/SLAM" --time-source frame --mode offline \
                  --gyro "$rig" --imu "$d/imu" --out "$r/live_tum.txt" > "$r/run.log" 2>&1 ;;
    esac
    [ -f "$r/live_tum.txt.optimized.txt" ] && mv "$r/live_tum.txt.optimized.txt" "$r/final_tum.txt"
    echo "   $(wc -l < "$r/live_tum.txt") live, $(wc -l < "$r/final_tum.txt" 2>/dev/null || echo 0) final, $(wc -l < "$r/live_tum.txt.mappoints.xyz" 2>/dev/null || echo -) map points"
  done
done
args=(); for r in "$out"/*/rep*; do args+=("$(basename "$(dirname "$r")")_$(basename "$r")=$r"); done
"$HOME/objpose/lidar/venv/bin/python" "$HERE/map_eval.py" --session "$S" --settings "$settings" --out "$out/map_eval.json" "${args[@]}"
