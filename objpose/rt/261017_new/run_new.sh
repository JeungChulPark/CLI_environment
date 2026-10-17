#!/bin/bash
# run_new.sh — ORB-SLAM3 (real-time paced, f3000, same as rt/260901/bigeight/run_host_eval.sh) on the sessions
# copied from .100 on 2026-10-17, plus the KISS-ICP reference where the session has a LiDAR bag.
# None of these sessions has an IMU bag, so there is no gyro-aided run.
#   bash objpose/rt/261017_new/run_new.sh [dataset root] [sessions...]
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIG="$HERE/../260901/bigeight"
ROOT="${1:-$HOME/Documents/DefenseMeta/Dataset}"; shift $(( $# > 0 ? 1 : 0 ))
SESS=("$@"); [ ${#SESS[@]} -gt 0 ] || SESS=(260915_eight_fast 260915_eight_middle 260826_etri_longcircle_dark 260826_etri_differentObject 260826_etri_similarObject)
PY="${PY:-$HOME/objpose/lidar/venv/bin/python}"
RUN="$HOME/objpose/run_slam.sh"
FEAT=3000

orb() {   # orb <session dir> <out dir>
  local D="$1" O="$2" set
  case "$(basename "$D")" in
    260915_*) set="$HOME/objpose/settings/orbslam3_260915_slam_nf2000.yaml" ;;
    *)        set="$HOME/objpose/settings/orbslam3_260826_dark_nf2000.yaml" ;;
  esac
  [ -f "$O/base_run1_tum.txt" ] && { echo "   orb: exists, skip"; return; }
  # intrinsics always from the bag itself (three different D455F units across the recordings)
  local P=($(python3 "$HERE/cam_params.py" "$D/SLAM"))   # macOS bash 3.2: no mapfile; values have no spaces
  "$RUN" --slam orbslam3 --session "$D/SLAM" --settings "$set" --features $FEAT "${P[@]}" --time-source frame \
    --mode offline --pace 1.0 --out "$O/base_run1_tum.txt" > "$O/base_run1.log" 2>&1
  grep -h "done:" "$O/base_run1.log" | cut -c1-200
}

kiss() {  # kiss <session dir> <out dir>
  local D="$1" O="$2"
  [ -d "$D/lidar" ] || return 0
  [ -f "$O/kiss/kiss_tum.txt" ] && { echo "   kiss: exists, skip"; return; }
  "$PY" "$BIG/kiss_gt.py" --session "$D/lidar" --out "$O/kiss" > "$O/kiss.log" 2>&1
  tail -1 "$O/kiss.log" | cut -c1-200
}

for s in "${SESS[@]}"; do
  D="$ROOT/$s"; O="$HERE/$s"; mkdir -p "$O"
  echo "######## [$(date +%T)] $s"
  kiss "$D" "$O" &
  orb "$D" "$O"
  wait
done
echo "######## [$(date +%T)] all done"
