#!/bin/bash
# run_nf4000_vs_gyro.sh — modified ORB-SLAM3 at NF4000 (no gyro) vs NF2000 + gyro aid (주력), on the 260901 sessions,
# REPS times each, same build, real-time paced; then map_eval.py against the LiDAR pseudo-GT.
#   bash objpose/rt/bench/run_nf4000_vs_gyro.sh [REPS=3] [sessions...]
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; REPO="$(cd "$HERE/../../.." && pwd)"
REPS="${1:-3}"; shift || true
SESSIONS=("${@:-}"); [ -z "${SESSIONS[0]}" ] && SESSIONS=(260901_cbnu_eightcircle 260901_cbnu_digut 260901_cbnu_longcircle 260901_cbnu_bigeightcircle)
SS="${SLAM_STREAM:-$HOME/objpose/build/slam_stream}"
for S in "${SESSIONS[@]}"; do
  d="$HOME/Documents/DefenseMeta/Dataset/$S"; name=${S#260901_cbnu_}; rig="$REPO/objpose/rt/260901/rtab_odom/rigs/rig_$name.json"
  out="$HERE/out_nf/$S"; mkdir -p "$out"
  settings=$(ls "$HERE"/out/$S/orbslam3_*.yaml | head -1)
  orb() {  # $1 dir, $2 features, rest extra args
    local r=$1 nf=$2; shift 2
    DYLD_LIBRARY_PATH="$(dirname "$SS"):$HOME/objpose/deps/install/lib:/opt/homebrew/opt/opencv@4/lib:/opt/homebrew/opt/libomp/lib" \
      "$SS" --session "$d/SLAM" --settings "$settings" --vocab "$HOME/objpose/Vocabulary/ORBvoc.txt" --features "$nf" \
      --time-source frame --mode offline --pace 1.0 "$@" --out "$r/live_tum.txt" > "$r/run.log" 2>&1
  }
  for rep in $(seq 1 "$REPS"); do
    for sys in nf4000 nf2000_gyro; do
      r="$out/$sys/rep$rep"; [ -s "$r/final_tum.txt" ] && continue; mkdir -p "$r"
      echo "[$(date +%T)] $S $sys rep$rep"
      case $sys in
        nf4000)      orb "$r" 4000 ;;
        nf2000_gyro) orb "$r" 2000 --gyro "$rig" --imu "$d/imu" ;;
      esac
      [ -f "$r/live_tum.txt.optimized.txt" ] && mv "$r/live_tum.txt.optimized.txt" "$r/final_tum.txt"
      echo "   $(wc -l < "$r/live_tum.txt") live, $(wc -l < "$r/final_tum.txt" 2>/dev/null || echo 0) final, $(grep -o 'track mean [0-9.]* ms' "$r/run.log" | tail -1)"
    done
  done
  args=(); for r in "$out"/*/rep*; do args+=("$(basename "$(dirname "$r")")_$(basename "$r")=$r"); done
  "$HOME/objpose/lidar/venv/bin/python" "$HERE/map_eval.py" --session "$S" --settings "$settings" --out "$out/map_eval.json" "${args[@]}"
done
echo "[$(date +%T)] all done"
