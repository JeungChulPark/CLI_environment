#!/usr/bin/env bash
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
O=objpose/output/cmp_260924_gt
run() { echo "=== $(date +%T) $*"; $PY $R --gt-boxes "$@" 2>&1 | grep -E --line-buffered "done:|Traceback|Error"; }
for ds in ycbv lmo; do
  run --method proposed_noverify         --dataset $ds --mode bop19 --out $O/proposednoverify_${ds}_bop19
  run --method proposed_noverify_fullpts --dataset $ds --mode bop19 --out $O/proposednoverifyfullpts_${ds}_bop19
done
echo "=== $(date +%T) ALL DONE"
