#!/bin/bash
# run_map.sh — 261022 sessions for html/260918_slam_3d_map: copy the 2026-10-24 re-calibration ORB-SLAM3 runs
# (SLAM camera, f2000, offline, ~/objpose/rt/261022_recal; depth-corrected ones from 261022_recal_depth) and add the KISS-ICP reference with its map preview.
#   bash objpose/rt/261024_map/run_map.sh [sessions...]
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIG="$HERE/../260901/bigeight"
D=~/Documents/DefenseMeta/Dataset; R=~/objpose/rt/261022_recal; RD=~/objpose/rt/261022_recal_depth
PY="${PY:-$HOME/objpose/lidar/venv/bin/python}"
SESS=("$@"); [ ${#SESS[@]} -gt 0 ] || SESS=(digut eight kiuk kiuk_obchange long long_obchange s)
for s in "${SESS[@]}"; do
  n=261022_cbnu_$s; O="$HERE/$n"; mkdir -p "$O"
  echo "######## [$(date +%T)] $n"
  for x in "" .optimized.txt .frames.csv .json; do cp "$R/${s}_slam_orb_tum.txt$x" "$O/base_run1_tum.txt$x"; done
  cp "$R/${s}_slam.log" "$O/base_run1.log"
  # depth-corrected run of the same camera (RGBD.DepthMapFactor=925, depth x1.081), when 219.100 made one
  if [ -f "$RD/${s}_slam_orb_tum.txt.optimized.txt" ]; then
    for x in "" .optimized.txt .frames.csv .json; do cp "$RD/${s}_slam_orb_tum.txt$x" "$O/depth_run1_tum.txt$x"; done
    cp "$RD/${s}_slam.log" "$O/depth_run1.log"
  fi
  [ -f "$O/kiss/kiss_tum.txt" ] || "$PY" "$BIG/kiss_gt.py" --session "$D/$n/lidar" --out "$O/kiss" > "$O/kiss.log" 2>&1
  tail -1 "$O/kiss.log" | cut -c1-200
done
echo "######## [$(date +%T)] all done"
