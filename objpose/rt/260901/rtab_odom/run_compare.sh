#!/bin/bash
# run_compare.sh — RTAB-Map with three odometry sources x two link-covariance settings, on one session.
#
#   bash objpose/rt/260901/rtab_odom/run_compare.sh <dataset dir> [KISS tum]
#
# The mapper settings are identical in all six runs; only the odometry source and the covariance
# handed to Rtabmap::process change. Runs are unpaced offline, so the mapper is synchronous on the
# data clock (1 Hz) and every run is deterministic — this is a comparison of odometry quality, not
# of real-time behaviour.
#
#   base  RTAB-Map's own visual odometry (Odom/Strategy 0 = F2M)
#   imu   the same VO + xsens orientation through SensorData::setIMU   (src/imu_source.h)
#   ext   no visual odometry at all; KISS-ICP poses fed straight in    (src/rtab_aux.h)
#
#   covdefault/  Rtabmap::process's default odometry covariance (identity: sigma 1 m / 1 rad)
#   covreal/     the odometry's own covariance, collected as rtabmap_ros CoreWrapper does
#
# Env: PY (python with numpy/scipy; default ~/objpose/lidar/venv/bin/python), RUN (run_slam.sh),
#      OUT (default: this directory).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIG="$HERE/../bigeight"
DATA="${1:?dataset dir (with SLAM/ lidar/ imu/)}"
RIG="$BIG/rig_260901_bigeight.json"
PY="${PY:-$HOME/objpose/lidar/venv/bin/python}"
RUN="${RUN:-$HOME/objpose/run_slam.sh}"
OUT="${OUT:-$HERE}"
KISS="${2:-$OUT/kiss/kiss_tum.txt}"

[ -d "$DATA/SLAM" ] && [ -d "$DATA/imu" ] || { echo "need $DATA/{SLAM,imu}" >&2; exit 1; }
if [ ! -f "$KISS" ]; then
  echo "== KISS-ICP reference -> $(dirname "$KISS")"
  "$PY" "$BIG/kiss_gt.py" --session "$DATA/lidar" --out "$(dirname "$KISS")" | tail -1
fi

for set in covdefault covreal; do
  d="$OUT/$set"; mkdir -p "$d"
  [ "$set" = covdefault ] && COV="--no-odom-cov" || COV=""
  echo "== [$(date +%T)] $set"
  ( $RUN --slam rtabmap --session "$DATA/SLAM" --time-source frame --mode offline $COV \
         --out "$d/base_tum.txt" > "$d/base.log" 2>&1 ) &
  ( $RUN --slam rtabmap --session "$DATA/SLAM" --time-source frame --mode offline $COV \
         --gyro "$RIG" --imu "$DATA/imu" --out "$d/imu_tum.txt" > "$d/imu.log" 2>&1 ) &
  ( $RUN --slam rtabmap --session "$DATA/SLAM" --time-source frame --mode offline $COV \
         --rig "$RIG" --ext-odom "$KISS" --out "$d/ext_tum.txt" > "$d/ext.log" 2>&1 ) &
  wait
  for v in base imu ext; do printf "   %-5s " $v; grep -h "done:" "$d/$v.log" | cut -c15-140; done

  echo "   scoring against KISS-ICP"
  args=()
  for v in base imu ext; do
    args+=("${v}_live=$d/${v}_tum.txt" "${v}_opt=$d/${v}_tum.txt.optimized.txt")
  done
  ( cd "$BIG" && "$PY" eval_traj.py --rig "$RIG" --gt "$KISS" --out "$d/eval.json" \
        --series "$d/series.json" "${args[@]}" 2>/dev/null | tee "$d/eval.txt" | sed 's/^/   /' )

  # non-circular read of the ext run: how far the mapper moved the trajectory it was handed
  "$PY" "$HERE/mapper_delta.py" --rig "$RIG" --kiss "$KISS" \
        live="$d/ext_tum.txt" opt="$d/ext_tum.txt.optimized.txt" > "$d/delta.txt" 2>&1
  sed 's/^/   /' "$d/delta.txt" | tail -4

  # the per-run rtabmap databases are ~130 MB each and are not part of the record
  rm -f "$d"/*.rtabmap.db
done

echo "== report"
"$PY" "$HERE/make_report.py"
