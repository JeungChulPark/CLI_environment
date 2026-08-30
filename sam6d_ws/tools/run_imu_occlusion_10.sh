#!/usr/bin/env bash
# One-off: full SAM-6D (ISM->PEM->reorg) for SAM_occlusion over ALL enabled
# config objects (config-driven, not the hardcoded 8). Applies the toy-prompt
# fix (Bear='brown teddy bear', Rabbit='white rabbit', Dinosaur added) -> 10 objs.
set -u
BAG="SAM_occlusion"; STRIDE=10
ROOT="/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"; cd "$ROOT"
PYY="/home/ldh9501/miniconda3/envs/sam_yolo/bin/python"
PYH="/home/ldh9501/miniconda3/envs/sam6d_ros_humble/bin/python"
STAGE_ROOT="$ROOT/outputs/pem_inputs"
FINAL="$ROOT/outputs/rgbd_imu_sdk_bag/$BAG"

echo "############ clear stale $BAG outputs ############"
rm -rf "$STAGE_ROOT/$BAG" "$FINAL"

echo "############ [$BAG] STAGE A — ISM (all enabled objects, stride=$STRIDE) ############"
$PYY tools/build_ism_inputs_imu.py --bag "$BAG" --stride "$STRIDE" --max-frames 99999 \
    --output-root "$STAGE_ROOT" 2>&1 \
  | grep -vE "xFormers|FutureWarning|warnings.warn|pynvml|UserWarning|Downloading|%\|" \
  | grep -E "templates|bag\]|frame |done|skip-cad|accepted"
[ ${PIPESTATUS[0]} -ne 0 ] && { echo "[$BAG] STAGE A failed"; exit 1; }

echo "############ [$BAG] STAGE B — PEM 6D pose ############"
$PYH -u tools/run_pem_batch.py --bags "$BAG" 2>&1 \
  | grep -vE "FutureWarning|warnings.warn|pynvml|UserWarning|load pre-trained|Downloading|%\|" \
  | grep -E "creating|OK |FAIL |batch done|bundles"

echo "############ [$BAG] STAGE C — reorganize delivery layout ############"
$PYH tools/reorg_imu_outputs.py --stage "$STAGE_ROOT/$BAG" --final "$FINAL"
echo "===== [$BAG] DONE -> $FINAL ====="
