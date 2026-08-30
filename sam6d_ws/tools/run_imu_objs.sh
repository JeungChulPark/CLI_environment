#!/usr/bin/env bash
# Re-run a subset of objects only, merge into existing delivery. Usage: run_imu_objs.sh "obj1 obj2"
set -u
OBJS="${1:?usage: run_imu_objs.sh \"obj1 obj2\"}"
ROOT="/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"; cd "$ROOT"
PYY="/home/ldh9501/miniconda3/envs/sam_yolo/bin/python"
PYH="/home/ldh9501/miniconda3/envs/sam6d_ros_humble/bin/python"
STAGE_ROOT="$ROOT/outputs/pem_inputs"
for BAG in SAM_circle SAM_loop1 SAM_loop2 SAM_occlusion; do
  echo "=========== $BAG objs=[$OBJS] : $(date) ==========="
  $PYY tools/build_ism_inputs_imu.py --bag "$BAG" --stride 10 --max-frames 99999 \
      --objects $OBJS --output-root "$STAGE_ROOT" 2>&1 \
    | grep -vE "xFormers|FutureWarning|warnings.warn|pynvml|UserWarning|Downloading|%\|" \
    | grep -E "templates|bag\]|accepted: \[.+\]|done"
  $PYH -u tools/run_pem_batch.py --bags "$BAG" 2>&1 \
    | grep -vE "FutureWarning|warnings.warn|pynvml|UserWarning|load pre-trained|Downloading|%\|" \
    | grep -E "OK |FAIL |batch done"
  $PYH tools/reorg_imu_outputs.py --stage "$STAGE_ROOT/$BAG" --final "outputs/rgbd_imu_sdk_bag/$BAG"
done
echo "=========== OBJS RERUN DONE : $(date) ==========="
