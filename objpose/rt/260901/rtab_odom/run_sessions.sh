#!/bin/bash
# run_sessions.sh — the four odometry/loop configurations on every 260901_cbnu session.
#
#   bash run_sessions.sh [session ...]        (default: all four)
#
# One mapper configuration throughout; only where the odometry comes from and how loop links are
# closed changes. Unpaced offline, so the mapper is synchronous on the data clock and every run is
# deterministic.
#
#   base    RTAB-Map's visual odometry            camera loops (Reg/Strategy 0 = Vis)
#   imu     the same VO + xsens orientation       camera loops
#   lidar   KISS-ICP poses, no visual odometry    camera loops
#   lidicp  KISS-ICP poses, no visual odometry    LIDAR loops (Reg/Strategy 1 = Icp)
#
# Each session uses its OWN rig json (rigs/rig_<session>.json from calib_sessions.sh) because the
# camera/lidar/imu time offsets differ per recording. The vertical lever arm is NOT taken from the
# rig — calib_rig.py regularises it to zero (unobservable in planar motion), and it comes out
# different in every session, so --lidar-z pins it to the rig's measured geometry instead:
# the lidar sits 0.59 m above ground and the SLAM camera 1.04 m, hence -0.45 m.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REF="$HERE/../bigeight/Userui-MacBookPro"
DATA=/Users/user/Documents/DefenseMeta/Dataset
RUN="${RUN:-$HOME/objpose/run_slam.sh}"
PY="${PY:-$HOME/objpose/lidar/venv/bin/python}"
LIDAR_Z="${LIDAR_Z:--0.45}"
SESSIONS=${*:-"bigeightcircle eightcircle longcircle digut"}

for s in $SESSIONS; do
  d="$DATA/260901_cbnu_$s"
  rig="$HERE/rigs/rig_$s.json"
  kiss="$REF/260901_cbnu_$s/kiss/kiss_tum.txt"
  out="$HERE/sessions/$s"; mkdir -p "$out"
  [ -f "$rig" ] || { echo "== $s: rig 없음 — calib_sessions.sh 먼저"; continue; }
  [ -f "$kiss" ] || { echo "== $s: kiss 없음"; continue; }
  echo "== [$(date +%T)] $s"

  ( $RUN --slam rtabmap --session "$d/SLAM" --time-source frame --mode offline \
         --out "$out/base_tum.txt" > "$out/base.log" 2>&1 ) &
  ( $RUN --slam rtabmap --session "$d/SLAM" --time-source frame --mode offline \
         --gyro "$rig" --imu "$d/imu" --out "$out/imu_tum.txt" > "$out/imu.log" 2>&1 ) &
  ( $RUN --slam rtabmap --session "$d/SLAM" --time-source frame --mode offline \
         --rig "$rig" --ext-odom "$kiss" --out "$out/lidar_tum.txt" > "$out/lidar.log" 2>&1 ) &
  ( $RUN --slam rtabmap --session "$d/SLAM" --time-source frame --mode offline \
         --rig "$rig" --ext-odom "$kiss" --lidar "$d/lidar" --lidar-z "$LIDAR_Z" \
         --out "$out/lidicp_tum.txt" > "$out/lidicp.log" 2>&1 ) &
  wait
  for v in base imu lidar lidicp; do
    printf "   %-7s " $v; grep -h "done:" "$out/$v.log" | cut -c15-125
  done
  rm -f "$out"/*.rtabmap.db          # ~130 MB each, not part of the record
done
echo "== [$(date +%T)] 완료"
