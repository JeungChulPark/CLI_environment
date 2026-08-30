#!/usr/bin/env bash
# Full SAM-6D (ISM->PEM->reorg) for one or more bags over ALL enabled config
# objects (config-driven, 10 objs) with the toy-prompt fix + imgsz=960.
# Usage: tools/run_imu_bags_10.sh [BAG ...]   (default: SAM_circle SAM_loop1 SAM_loop2)
set -u
BAGS=("$@"); [ ${#BAGS[@]} -eq 0 ] && BAGS=(SAM_circle SAM_loop1 SAM_loop2)
STRIDE=10
ROOT="/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"; cd "$ROOT"
PYY="/home/ldh9501/miniconda3/envs/sam_yolo/bin/python"
PYH="/home/ldh9501/miniconda3/envs/sam6d_ros_humble/bin/python"
STAGE_ROOT="$ROOT/outputs/pem_inputs"

for BAG in "${BAGS[@]}"; do
  FINAL="$ROOT/outputs/rgbd_imu_sdk_bag/$BAG"
  echo "################################ $BAG : $(date) ################################"
  echo "---- clear stale $BAG outputs ----"; rm -rf "$STAGE_ROOT/$BAG" "$FINAL"

  echo "---- [$BAG] STAGE A — ISM (all enabled objects, stride=$STRIDE, imgsz per config) ----"
  $PYY tools/build_ism_inputs_imu.py --bag "$BAG" --stride "$STRIDE" --max-frames 99999 \
      --output-root "$STAGE_ROOT" 2>&1 \
    | grep -vE "xFormers|FutureWarning|warnings.warn|pynvml|UserWarning|Downloading|%\|" \
    | grep -E "imgsz|templates|bag\]|done|skip-cad"
  [ ${PIPESTATUS[0]} -ne 0 ] && { echo "[$BAG] STAGE A failed"; continue; }

  echo "---- [$BAG] STAGE B — PEM 6D pose ----"
  $PYH -u tools/run_pem_batch.py --bags "$BAG" 2>&1 \
    | grep -vE "FutureWarning|warnings.warn|pynvml|UserWarning|load pre-trained|Downloading|%\|" \
    | grep -E "OK |FAIL |batch done|bundles"

  echo "---- [$BAG] STAGE C — reorganize delivery layout ----"
  $PYH tools/reorg_imu_outputs.py --stage "$STAGE_ROOT/$BAG" --final "$FINAL"
  echo "===== [$BAG] DONE -> $FINAL ====="
done
echo "################################ ALL BAGS DONE : $(date) ################################"
