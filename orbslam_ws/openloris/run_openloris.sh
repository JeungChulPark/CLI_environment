#!/usr/bin/env bash
# Run every variant on every sequence of one OpenLORIS-Scene scene, then score with eval_openloris.py.
#
#   orbslam_ws/openloris/run_openloris.sh SCENE_DIR OUT_DIR [REPS=3] [NFEATURES=1000] [VARIANTS="upstream ours ours_gyro"]
#     SCENE_DIR  e.g. ~/Documents/DefenseMeta/Dataset/openloris/office (holds office1-1 … office1-7)
#     REALTIME=0 in the environment tracks every frame back to back instead of at 30 Hz wall clock.
#     STRIDE=k RATE=k keeps every k-th frame and plays it at 30 Hz: k times the motion between frames.
#
# Variants (same settings yaml, same runner, same flags):
#   upstream   UZ-SLAMLab ORB-SLAM3 v1.0-release
#   ours       this repo's ORB-SLAM3 with its real-time config (Optimizer.PoseIterations [10,5,5,5] + early exit)
#   ours_gyro  ours + gyro aid (D435i gyro, rig from make_rig.py)
# Runs are interleaved (rep 1 of every variant, then rep 2 …) so slow drifts in machine load hit all variants alike.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
W=$HERE/../output/openloris
SCENE=$1 OUT=$2 REPS=${3:-3} NF=${4:-1000} VARIANTS=${5:-"upstream ours ours_gyro"}
VOCAB=$HERE/../output/mac_build/ORBvoc.txt
RIG=$OUT/d400_gyro_rig.json
mkdir -p "$OUT"
first=$(ls -d "$SCENE"/*1-* | head -1)
"$W/venv/bin/python" "$HERE/make_rig.py" "$first" "$RIG"

for rep in $(seq 1 "$REPS"); do
  for seq in $(ls -d "$SCENE"/*1-* | sort -V); do
    name=$(basename "$seq")
    for v in $VARIANTS; do
      run=$OUT/$v/$name/rep$rep
      [ -f "$run/summary.json" ] && continue          # resumable
      case $v in
        upstream)  bin=$W/build_upstream/openloris_rgbd; extra=() ;;
        ours)      bin=$W/build_ours/openloris_rgbd; extra=(--set 'Optimizer.PoseIterations=[10, 5, 5, 5]' --set Optimizer.PoseEarlyExit=1) ;;
        ours_gyro) bin=$W/build_ours/openloris_rgbd; extra=(--set 'Optimizer.PoseIterations=[10, 5, 5, 5]' --set Optimizer.PoseEarlyExit=1 --gyro "$RIG") ;;
        *) echo "unknown variant $v"; exit 1 ;;
      esac
      mkdir -p "$run"
      echo "[$(date +%H:%M:%S)] $v $name rep$rep"
      if ! "$bin" "$VOCAB" "$HERE/openloris_d400.yaml" "$seq" "$run" --realtime "${REALTIME:-1}" --rate "${RATE:-1}" --stride "${STRIDE:-1}" \
           --set "ORBextractor.nFeatures=$NF" ${extra[@]+"${extra[@]}"} > "$run/run.log" 2>&1; then
        echo "  FAILED (see $run/run.log)"; rm -f "$run/summary.json"
      else
        tail -1 "$run/run.log"
      fi
    done
  done
done
"$W/venv/bin/python" "$HERE/eval_openloris.py" "$OUT"
