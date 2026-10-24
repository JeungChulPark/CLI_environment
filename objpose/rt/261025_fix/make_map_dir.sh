#!/bin/bash
# make_map_dir.sh — lay out map/<session>/ the way build_map_data.py --runs expects, from the race-fixed run1s here
# (modified, +depth, +gyro, +gyro+depth) and the upstream run1s of ../261024_map (upstream build was not changed).
#   bash make_map_dir.sh [sessions...];  then  build_map_data.py --runs ../261025_fix/map <sessions as 261022_cbnu_*>
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESS=("$@"); [ ${#SESS[@]} -gt 0 ] || SESS=(digut eight kiuk kiuk_obchange long long_obchange)
for s in "${SESS[@]}"; do
  n=261022_cbnu_$s; M="$HERE/map/$n"; mkdir -p "$M"
  ln -sfn ../../../261024_map/$n/kiss "$M/kiss"
  lk() {  # lk <src dir rel to $M> <src stem> <dst stem>
    for x in "" .optimized.txt .frames.csv .json; do
      [ -e "$M/$1/$2$x" ] && ln -sfn "$1/$2$x" "$M/$3$x"; done; }
  lk ../.. "$n/base_run1.txt" base_run1_tum.txt
  lk ../.. "$n/dep_run1.txt"  depth_run1_tum.txt
  lk ../.. "$n/gy_run1.txt"   gy_run1.txt
  lk ../.. "$n/gyd_run1.txt"  gyd_run1.txt
  lk ../../../261024_map "$n/up_run1.txt"  up_run1.txt
  lk ../../../261024_map "$n/upd_run1.txt" upd_run1.txt
done
