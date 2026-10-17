#!/usr/bin/env bash
# GT-box comparison: detector replaced by ground-truth boxes for every method.
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
O=objpose/output/cmp_260924_gt
run() { echo "=== $(date +%T) $*"; $PY $R --gt-boxes "$@" 2>&1 | grep -E --line-buffered "done:|Traceback|Error"; }
for ds in ycbv lmo; do
  run --method proposed        --dataset $ds --mode bop19 --out $O/proposed_${ds}_bop19
  run --method proposed_nogate --dataset $ds --mode bop19 --out $O/proposednogate_${ds}_bop19
  run --method orig            --dataset $ds --mode bop19 --out $O/orig_${ds}_bop19
done
run --method proposed --dataset ycbv --mode realtime --out $O/proposed_ycbv_realtime
run --method orig     --dataset ycbv --mode realtime --out $O/orig_ycbv_realtime
run --method proposed_nogate --dataset ycbv --mode realtime --out $O/proposednogate_ycbv_realtime
echo "=== $(date +%T) ALL DONE"
