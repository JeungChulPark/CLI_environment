#!/bin/bash
# run_host_all.sh — ON a SLAM host: run_host_eval.sh over all four 260901_cbnu sessions (KISS-ICP reference + ORB / ORB+IMU, N runs each).
#   bash objpose/rt/260901/bigeight/run_host_all.sh <dataset root> [N=2] [features=3000] [sessions...]
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${1:?dataset root (contains 260901_cbnu_*)}"; N="${2:-2}"; FEAT="${3:-3000}"; shift 3 2>/dev/null || shift $#
SESS=("$@"); [ ${#SESS[@]} -gt 0 ] || SESS=(eightcircle longcircle digut bigeightcircle)
for s in "${SESS[@]}"; do
  D="$ROOT/260901_cbnu_$s"
  [ -d "$D/SLAM" ] || { echo "skip $s: no $D/SLAM" >&2; continue; }
  echo "######## [$(date +%T)] $s"
  bash "$HERE/run_host_eval.sh" "$D" "$N" "$FEAT"
done
echo "######## [$(date +%T)] all done"
