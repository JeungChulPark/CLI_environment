#!/bin/bash
# run_reps.sh — repeat runs for the unstable 261022 sessions (digut, kiuk), 2026-10-25: run1 of each variant gave
# 4..128 cm ATE depending on the variant, so this adds run2..run4 of all six variants to tell noise from trend.
#   base = modified, dep = modified + depth fix, up/upd = upstream (+ depth fix), gy/gyd = modified + gyro (+ depth fix)
# Same settings as run_map.sh / run_variants.sh (f2000, offline, 260826 settings; depth fix RGBD.DepthMapFactor=925.0).
#   [OUT=dir] [VARS="base dep up upd gy gyd"] bash run_reps.sh [first_rep] [last_rep] [sessions...]
# OUT (default: this directory) gets <OUT>/261022_cbnu_<s>/<variant>_run<r>.*; 261025_fix used it for the race-fixed build.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; HERE="${OUT:-$HERE}"
VARS=" ${VARS:-base dep up upd gy gyd} "
D=~/Documents/DefenseMeta/Dataset
A=${1:-2}; B=${2:-4}; shift 2 2>/dev/null
SESS=("$@"); [ ${#SESS[@]} -gt 0 ] || SESS=(digut kiuk)
DEP=(--param RGBD.DepthMapFactor=925.0); GY=(--gyro rig_261022_gyro.json)
one() {  # one <session> <out name> <wrapper> [args...]
  local s=$1 n=$2 w=$3; shift 3
  local O="$HERE/261022_cbnu_$s"
  [[ "$VARS" == *" ${n%_run*} "* ]] || return
  mkdir -p "$O"; [ -f "$O/$n.txt.optimized.txt" ] && return
  ~/objpose/$w --session $D/261022_cbnu_$s/SLAM --features 2000 --mode offline "$@" --out "$O/$n.txt" > "$O/$n.log" 2>&1 \
    && echo "ok   $s $n $(date +%T)" || echo "FAIL $s $n rc=$? $(date +%T)"
}
for r in $(seq $A $B); do
  for s in "${SESS[@]}"; do
    one $s base_run$r run_slam.sh &
    one $s dep_run$r  run_slam.sh "${DEP[@]}" &
    one $s up_run$r   run_slam_upstream.sh &
    wait
    one $s upd_run$r  run_slam_upstream.sh "${DEP[@]}" &
    one $s gy_run$r   run_slam.sh "${GY[@]}" &
    one $s gyd_run$r  run_slam.sh "${GY[@]}" "${DEP[@]}" &
    wait
  done
done
echo ALLDONE
