#!/bin/bash
# sweep_icp.sh — how much ICP correction to allow on a loop link.
#
# Icp/MaxTranslation caps how far ICP may move a candidate link away from the guess the graph gives
# it. The default 0.2 m turned out to be the reason the lidar-loop lane lost on longcircle: it took
# only 21 of the 77 loops the visual lane accepted, because that session's odometry has drifted
# 44.9 cm by the time the robot comes back, so the guess is already outside the cap and the link is
# thrown away rather than corrected. eightcircle, which drifts 12.8 cm, lost loops too (69 -> 8) but
# still came out ahead, so the cap is not simply "too small" — it is coupled to how far each session
# drifts, which is what this sweep measures.
#
#   bash sweep_icp.sh [session ...]
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REF="$HERE/../bigeight/Userui-MacBookPro"
DATA=/Users/user/Documents/DefenseMeta/Dataset
RUN="${RUN:-$HOME/objpose/run_slam.sh}"
SESSIONS=${*:-"bigeightcircle eightcircle longcircle digut"}
VALUES="${VALUES:-0.5 1.0 2.0}"          # 0.2 is the default, already run as the lidicp lane

for s in $SESSIONS; do
  d="$DATA/260901_cbnu_$s"
  rig="$HERE/rigs/rig_$s.json"
  kiss="$REF/260901_cbnu_$s/kiss/kiss_tum.txt"
  out="$HERE/sweep/$s"; mkdir -p "$out"
  [ -f "$rig" ] && [ -f "$kiss" ] || { echo "== $s: 입력 없음"; continue; }
  echo "== [$(date +%T)] $s"
  for v in $VALUES; do
    ( $RUN --slam rtabmap --session "$d/SLAM" --time-source frame --mode offline \
           --rig "$rig" --ext-odom "$kiss" --lidar "$d/lidar" --lidar-z -0.45 \
           --param "Icp/MaxTranslation=$v" \
           --out "$out/mt${v}_tum.txt" > "$out/mt${v}.log" 2>&1 ) &
  done
  wait
  for v in $VALUES; do
    printf "   MaxTranslation %-4s " "$v"; grep -h "done:" "$out/mt${v}.log" | cut -c15-115
  done
  rm -f "$out"/*.rtabmap.db
done
echo "== [$(date +%T)] 완료"
