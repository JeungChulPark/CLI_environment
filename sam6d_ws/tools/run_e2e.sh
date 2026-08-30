#!/usr/bin/env bash
# End-to-end SAM-6D: input frames -> ISM -> PEM -> saved 6D-pose images.
#
# ISM (YOLO-World+DINOv2+MobileSAM) needs the `sam_yolo` env; PEM (gorilla+
# pointnet2) needs `sam6d_ros_humble`. ultralytics differs between them
# (8.4 has YOLOWorld, 8.0 doesn't) so they CANNOT share one process. This
# orchestrates the two stages; within EACH stage all one-time data (models,
# template features, CAD point caches) is loaded ONCE, then every input frame
# is processed in a loop.
#
# Usage: tools/run_e2e.sh <bag> [max_frames]      (frames evenly spread across bag)
set -u
BAG="${1:-two_table_around}"
N="${2:-3}"
ROOT="/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"; cd "$ROOT"
PYY="/home/ldh9501/miniconda3/envs/sam_yolo/bin/python"
PYH="/home/ldh9501/miniconda3/envs/sam6d_ros_humble/bin/python"
F=2>&1  # noop

echo "############################################################"
echo "# PHASE 1+2 / STAGE A — ISM  (env: sam_yolo)"
echo "#   load YOLO-World + DINOv2 + MobileSAM ONCE, then per frame:"
echo "#   RGB + synced aligned-depth + cam -> detections -> RLE seg bundle"
echo "############################################################"
$PYY tools/build_pem_inputs.py --bag "$BAG" --max-frames "$N" 2>&1 \
  | grep -vE "xFormers|FutureWarning|warnings.warn|pynvml|UserWarning" \
  | grep -E "bag\]|frame |accepted|done|skip-cad"
[ ${PIPESTATUS[0]} -ne 0 ] && { echo "STAGE A failed"; exit 1; }

echo
echo "############################################################"
echo "# STAGE B — PEM  (env: sam6d_ros_humble)"
echo "#   load PEM model + template-feature cache + CAD point cache ONCE,"
echo "#   then per (frame,object): observed point cloud -> coarse+fine pose"
echo "############################################################"
$PYH -u tools/run_pem_batch.py --bags "$BAG" 2>&1 \
  | grep -vE "FutureWarning|warnings.warn|pynvml|UserWarning|load pre-trained|Downloading|%\|" \
  | grep -E "creating|OK |FAIL |batch done"

echo
echo "############################################################"
echo "# STAGE C — unified per-frame 6D-pose overlay"
echo "############################################################"
$PYH tools/viz_pem_multiobject.py --bags "$BAG" 2>&1 \
  | grep -vE "FutureWarning|warnings.warn|pynvml|UserWarning"

echo
echo "===== E2E DONE -> outputs/pem_inputs/$BAG/frame_*/vis_pem_all.png ====="
ls "$ROOT/outputs/pem_inputs/$BAG"/frame_*/vis_pem_all.png 2>/dev/null
