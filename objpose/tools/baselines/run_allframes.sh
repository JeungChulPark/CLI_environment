#!/usr/bin/env bash
# every frame of YCB-Video and YCB-M, original SAM-6D vs the proposed estimator, one at a time on the GPU
# (still mode: each frame on its own, per-module timers of run_profiled.py). Finished scenes are skipped on restart.
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_profiled.py
O=${1:-objpose/output/allframes_261006}
mkdir -p $O/logs
set -a; . objpose/tools/pgt/final.env; set +a          # proposed: the settings picked on the validation frames
for job in "proposed ycbv" "proposed ycbm" "orig ycbm" "orig ycbv"; do
  set -- $job
  echo "=== $(date '+%m-%d %T') start $1 $2"
  $PY $R --method $1 --dataset $2 --mode still --out $O/$2_$1 > $O/logs/$2_$1.log 2>&1
  echo "=== $(date '+%m-%d %T') end $1 $2 exit=$?"
done
touch $O/DONE
