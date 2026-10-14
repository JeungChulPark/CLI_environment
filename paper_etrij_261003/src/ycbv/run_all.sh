#!/usr/bin/env bash
# Runs the three methods one after another (GPU otherwise idle) and then the metrics.
set -euo pipefail
cd "$(dirname "$0")"
PY=${SAM6D_PY:-$HOME/anaconda3/envs/sam6d/bin/python}
OUT=${YCBV_OUT:-$(pwd)/out}; mkdir -p "$OUT"
$PY run_orig.py               > "$OUT/log_sam6d.txt" 2>&1
$PY run_ours.py --mode text   > "$OUT/log_ours_text.txt" 2>&1
$PY run_ours.py --mode gtbox  > "$OUT/log_ours_gtbox.txt" 2>&1
$PY evaluate.py               > "$OUT/log_evaluate.txt" 2>&1
