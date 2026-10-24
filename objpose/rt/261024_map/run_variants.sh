#!/bin/bash
# run_variants.sh — the other two ORB-SLAM3 variants for the 261022 depth before/after comparison (2026-10-24):
#   up  = upstream ORB-SLAM3 (~/objpose/run_slam_upstream.sh)      gy = modified + gyro aid (--gyro rig_261022_gyro.json)
#   suffix d = camera A depth corrected (RGBD.DepthMapFactor=925.0, x1.081), same as ~/objpose/rt/261022_recal_depth
# The modified-without-gyro runs are the existing 261022_recal / 261022_recal_depth ones (run_map.sh copies them).
# All f2000, offline, default 260826 settings, same as those runs.   bash run_variants.sh [sessions...]
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
D=~/Documents/DefenseMeta/Dataset
SESS=("$@"); [ ${#SESS[@]} -gt 0 ] || SESS=(digut eight kiuk kiuk_obchange long long_obchange)
one() {  # one <session> <out name> <wrapper> [args...]
  local s=$1 n=$2 w=$3; shift 3
  local O="$HERE/261022_cbnu_$s"
  [ -f "$O/$n.txt.optimized.txt" ] && return
  ~/objpose/$w --session $D/261022_cbnu_$s/SLAM --features 2000 --mode offline "$@" --out "$O/$n.txt" > "$O/$n.log" 2>&1 \
    && echo "ok   $s $n $(date +%T)" || echo "FAIL $s $n rc=$? $(date +%T)"
}
for s in "${SESS[@]}"; do
  mkdir -p "$HERE/261022_cbnu_$s"
  one $s up_run1  run_slam_upstream.sh &
  one $s upd_run1 run_slam_upstream.sh --param RGBD.DepthMapFactor=925.0
  wait
  one $s gy_run1  run_slam.sh --gyro rig_261022_gyro.json &
  one $s gyd_run1 run_slam.sh --gyro rig_261022_gyro.json --param RGBD.DepthMapFactor=925.0
  wait
done
echo ALLDONE
