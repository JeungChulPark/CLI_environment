#!/bin/bash
# sweep_features.sh — does the camera lose to ICP because it was not given enough features?
#
# Every conclusion so far about visual loop links being "the noisier party" was measured at
# Vis/MaxFeatures = 1000, the library default, which the runs actually hit: the frames report ~880
# usable features and ~320 PnP inliers, so the cap binds. The inlier count is 13x the MinInliers
# threshold, so registration is not failing for want of points — but PnP precision still improves
# with more of them, and that is the quantity the loop link's accuracy rests on.
#
# Two lanes are swept, because Vis/MaxFeatures enters them differently:
#   base    visual odometry AND visual loop geometry  -> both get better or worse together
#   lidar   KISS odometry, visual loop geometry only  -> isolates the loop constraint
#
# The lidar lane is the one that matters: if a richer visual constraint closes the gap to the ICP
# lane, then "visual loops degrade an accurate odometry" was a statement about the default settings
# rather than about cameras.
#
#   bash sweep_features.sh [session ...]
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REF="$HERE/../bigeight/Userui-MacBookPro"
DATA=/Users/user/Documents/DefenseMeta/Dataset
RUN="${RUN:-$HOME/objpose/run_slam.sh}"
SESSIONS=${*:-"bigeightcircle eightcircle longcircle digut"}
FEATS="${FEATS:-2000 3000}"          # 1000 is the default, already run in sessions/

for s in $SESSIONS; do
  d="$DATA/260901_cbnu_$s"
  rig="$HERE/rigs/rig_$s.json"
  kiss="$REF/260901_cbnu_$s/kiss/kiss_tum.txt"
  out="$HERE/feats/$s"; mkdir -p "$out"
  [ -f "$rig" ] && [ -f "$kiss" ] || { echo "== $s: 입력 없음"; continue; }
  echo "== [$(date +%T)] $s"
  for f in $FEATS; do
    ( $RUN --slam rtabmap --session "$d/SLAM" --time-source frame --mode offline --features "$f" \
           --out "$out/base_f${f}_tum.txt" > "$out/base_f${f}.log" 2>&1 ) &
    ( $RUN --slam rtabmap --session "$d/SLAM" --time-source frame --mode offline --features "$f" \
           --rig "$rig" --ext-odom "$kiss" \
           --out "$out/lidar_f${f}_tum.txt" > "$out/lidar_f${f}.log" 2>&1 ) &
  done
  wait
  for f in $FEATS; do
    for v in base lidar; do
      printf "   %-6s f%-5s " $v "$f"; grep -h "done:" "$out/${v}_f${f}.log" | cut -c15-115
    done
  done
  rm -f "$out"/*.rtabmap.db "$out"/*.frames.csv
done
echo "== [$(date +%T)] 완료"
