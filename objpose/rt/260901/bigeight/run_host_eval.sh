#!/bin/bash
# run_host_eval.sh — ON a SLAM host (192.168.20.3 or the Mac), from its own CLI_environment checkout:
# KISS-ICP reference + N real-time-paced ORB-SLAM3 runs without and with the gyro aid + scoring, all on the
# host's local copy of the session (any 260901_cbnu_* session; same rig json).
#   bash objpose/rt/260901/bigeight/run_host_eval.sh <dataset dir> [N=2] [features=3000]
# Env: PY (python with kiss_icp/scipy; default ~/objpose/lidar/venv/bin/python), OUT (default <this dir>/<hostname>/<dataset>),
#      PIN_A / PIN_B (Linux: taskset cpu lists for the two concurrent runs), GYRO_PARAMS (extra --param for the gyro runs)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA="${1:?dataset dir (with SLAM/ lidar/ imu/)}"; N="${2:-2}"; FEAT="${3:-3000}"
PY="${PY:-$HOME/objpose/lidar/venv/bin/python}"
OUT="${OUT:-$HERE/$(hostname -s)/$(basename "$DATA")}"; mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"    # absolute: the scoring step cd's elsewhere
RUN="$HOME/objpose/run_slam.sh"
RIG="$HERE/rig_260901_bigeight.json"
[ -d "$DATA/SLAM" ] && [ -d "$DATA/imu" ] && [ -d "$DATA/lidar" ] || { echo "need $DATA/{SLAM,imu,lidar}" >&2; exit 1; }
pin() { if command -v taskset >/dev/null 2>&1 && [ -n "${1:-}" ]; then echo "taskset -c $1"; fi; }

echo "== [$(date +%T)] KISS-ICP reference -> $OUT/kiss"
[ -f "$OUT/kiss/kiss_tum.txt" ] || "$PY" "$HERE/kiss_gt.py" --session "$DATA/lidar" --out "$OUT/kiss" | tail -1

for i in $(seq 1 "$N"); do
  echo "== [$(date +%T)] run $i/$N: base + gyro (real-time paced, f$FEAT)"
  ( $(pin "${PIN_A:-}") "$RUN" --slam orbslam3 --session "$DATA/SLAM" --features "$FEAT" --time-source frame --mode offline --pace 1.0 \
      --out "$OUT/base_run${i}_tum.txt" > "$OUT/base_run${i}.log" 2>&1 ) &
  ( $(pin "${PIN_B:-}") "$RUN" --slam orbslam3 --session "$DATA/SLAM" --features "$FEAT" --time-source frame --mode offline --pace 1.0 \
      --gyro "$RIG" --imu "$DATA/imu" ${GYRO_PARAMS:-} \
      --out "$OUT/gyro_run${i}_tum.txt" > "$OUT/gyro_run${i}.log" 2>&1 ) &
  wait
  grep -h "done:\|gyro aid: [0-9]\|still busy" "$OUT/base_run${i}.log" "$OUT/gyro_run${i}.log" | cut -c1-160
  echo "   loops: base $(grep -c LOOP_GAP "$OUT/base_run${i}.log")  gyro $(grep -c LOOP_GAP "$OUT/gyro_run${i}.log")"
done

echo "== [$(date +%T)] scoring against KISS-ICP"
args=()
for i in $(seq 1 "$N"); do
  for v in base gyro; do
    [ -f "$OUT/${v}_run${i}_tum.txt" ] && args+=("${v}_live${i}=$OUT/${v}_run${i}_tum.txt")
    [ -f "$OUT/${v}_run${i}_tum.txt.optimized.txt" ] && args+=("${v}_opt${i}=$OUT/${v}_run${i}_tum.txt.optimized.txt")
  done
done
cd "$HERE" && "$PY" eval_traj.py --rig "$RIG" --gt "$OUT/kiss/kiss_tum.txt" --out "$OUT/eval.json" --series "$OUT/series.json" "${args[@]}" | tee "$OUT/eval.txt"
