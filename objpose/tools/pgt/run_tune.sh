#!/usr/bin/env bash
# parameter tuning of the proposed pipeline on VALIDATION frames only (YCB-V non-BOP19 frames, YCB-M offset-4 frames)
#   bash run_tune.sh <tag> <gtbox|yolo> [env assignments...]
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
V=objpose/output/estcmp_260927/val
tag=$1; mode=$2; shift 2
G=""; [ $mode = gtbox ] && G="--gt-boxes"
for kv in "$@"; do export "$kv"; done
echo "=== $(date +%T) $tag $mode $*"
$PY $R --method proposed --dataset ycbv --mode still --stride 100 --exclude-targets $G --out $V/ycbv_$tag 2>&1 | grep -E "Traceback|Error:" | grep -v PEM
$PY $R --method proposed --dataset ycbm --mode still --stride 24 --offset 4 $G --out $V/ycbm_$tag 2>&1 | grep -E "Traceback|Error:" | grep -v PEM
$PY objpose/tools/pgt/val_score.py ycbv $V/ycbv_$tag 2>&1 | grep "^ycbv"
$PY objpose/tools/pgt/val_score.py ycbm $V/ycbm_$tag 2>&1 | grep "^ycbm"
