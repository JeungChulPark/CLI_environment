#!/usr/bin/env bash
# End-to-end SAM-6D for rgbd_imu_sdk_bag SAM_* bags.
#   Stage A (sam_yolo):        ISM 8-object recognition + input bundle + seg overlays
#   Stage B (sam6d_ros_humble): PEM 6D pose per (frame,object)
#   Stage C (any):             reorganize into requested delivery layout
#
# Usage: tools/run_imu_e2e.sh <bag> [stride]
set -u
BAG="${1:?usage: run_imu_e2e.sh <bag> [stride]}"
STRIDE="${2:-10}"
ROOT="/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"; cd "$ROOT"
PYY="/home/ldh9501/miniconda3/envs/sam_yolo/bin/python"
PYH="/home/ldh9501/miniconda3/envs/sam6d_ros_humble/bin/python"
OBJS="milk Bear Rabbit Mugcup_high saffron choco_hazelnut_high Sauce_high Febreze_high"
STAGE_ROOT="$ROOT/outputs/pem_inputs"            # native path read by run_pem_batch
FINAL="$ROOT/outputs/rgbd_imu_sdk_bag/$BAG"

echo "############ [$BAG] STAGE A — ISM (8 objects, stride=$STRIDE) ############"
$PYY tools/build_ism_inputs_imu.py --bag "$BAG" --stride "$STRIDE" --max-frames 99999 \
    --objects $OBJS --output-root "$STAGE_ROOT" 2>&1 \
  | grep -vE "xFormers|FutureWarning|warnings.warn|pynvml|UserWarning|Downloading|%\|" \
  | grep -E "templates|bag\]|frame |done|skip-cad"
[ ${PIPESTATUS[0]} -ne 0 ] && { echo "[$BAG] STAGE A failed"; exit 1; }

echo "############ [$BAG] STAGE B — PEM 6D pose ############"
$PYH -u tools/run_pem_batch.py --bags "$BAG" 2>&1 \
  | grep -vE "FutureWarning|warnings.warn|pynvml|UserWarning|load pre-trained|Downloading|%\|" \
  | grep -E "creating|OK |FAIL |batch done|bundles"

echo "############ [$BAG] STAGE C — reorganize delivery layout ############"
$PYH tools/reorg_imu_outputs.py --stage "$STAGE_ROOT/$BAG" --final "$FINAL"

echo "===== [$BAG] DONE -> $FINAL ====="
