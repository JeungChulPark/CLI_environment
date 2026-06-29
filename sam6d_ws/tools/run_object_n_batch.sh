#!/usr/bin/env bash
# Batch-run yolo_ism_object_n.py (multi-object ISM) over several ROS2 bags to
# validate color-attribute prompt generalization. Reuses template caches.
set -u
ROOT="/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
PY="/home/ldh9501/miniconda3/envs/sam_yolo/bin/python"
CFG="$ROOT/configs/yolo_ism_objects.yaml"
LOG="$ROOT/outputs/_logs"
mkdir -p "$LOG"
cd "$ROOT"

# multi-object table scenes (+ one different scene to stress generalization)
BAGS=(
  two_table_around_goback
  two_table_diagonal1
  two_table_diagonal2
  two_table_goback
  high_texture_around
)

for b in "${BAGS[@]}"; do
  echo "========== $b =========="
  $PY yolo_ism_object_n.py \
    --config "$CFG" \
    --bag "data/ros2_bag/$b" \
    --frames-dir "$ROOT/outputs/yolo_test/$b/frames" \
    2>&1 | tee "$LOG/object_n_${b}.log" | sed -n '/SUMMARY/,$p'
  echo
done
echo "===== BATCH DONE ====="
