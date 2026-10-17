#!/bin/bash
# run_all.sh — pseudo-GT for every session with a LiDAR bag, one session at a time at low priority.
#   bash objpose/rt/gt/run_all.sh [session ...]      (default: the four 260901_cbnu sessions)
# Env: DATA (default ~/Documents/DefenseMeta/Dataset), PY (python with kiss_icp), THREADS (default 3)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA="${DATA:-$HOME/Documents/DefenseMeta/Dataset}"
PY="${PY:-$HOME/objpose/lidar/venv/bin/python}"
[ $# -gt 0 ] && S=("$@") || S=(260901_cbnu_bigeightcircle 260901_cbnu_eightcircle 260901_cbnu_longcircle 260901_cbnu_digut)
for s in "${S[@]}"; do
  echo "== [$(date +%T)] $s"
  nice -n 10 "$PY" "$HERE/pseudo_gt.py" --session "$DATA/$s" --out "$HERE/out/$s" --threads "${THREADS:-3}" \
    > "$HERE/out/$s.log" 2>&1 || echo "   FAILED (see out/$s.log)"
  grep -A3 '"loops"' "$HERE/out/$s/summary.json" | head -4 || true
done
