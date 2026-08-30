#!/usr/bin/env bash
# Re-run SAM-6D on the two diagonal bags with the unit-fixed (mm) CAD + fresh
# templates, for ALL 16 objects. Validates that the 14 previously-zero objects
# now produce detections. Clears prior _raw for these bags so results are clean.
set -e
source ~/miniconda3/etc/profile.d/conda.sh
conda activate sam6d_ros_humble
cd /home/ldh9501/temp_ws/CLI_environment/sam6d_ws

BAGS="two_table_diagonal1,two_table_diagonal2"
echo "[recheck] clearing prior _raw for $BAGS"
rm -rf outputs_e2e/_raw/two_table_diagonal1 outputs_e2e/_raw/two_table_diagonal2

echo "[recheck] inference (16 objects x 2 bags) $(date '+%F %T')"
python tools/e2e_pipeline/02_run_inference.py --bags "$BAGS" --objects ALL --gpus 0,1,2,3

echo "[recheck] postprocess $(date '+%F %T')"
python tools/e2e_pipeline/03_postprocess.py --bags "$BAGS"

echo "[recheck] DONE $(date '+%F %T')"
