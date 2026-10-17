#!/usr/bin/env bash
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
O=objpose/output/cmp_260924_gt
run() { echo "=== $(date +%T) $*"; $PY $R --gt-boxes "$@" 2>&1 | grep -E --line-buffered "done:|Traceback|Error"; }
run --method proposed_score --dataset ycbv --mode bop19    --out $O/proposedscore_ycbv_bop19
run --method proposed_score --dataset lmo  --mode bop19    --out $O/proposedscore_lmo_bop19
run --method proposed_score --dataset ycbv --mode realtime --out $O/proposedscore_ycbv_realtime
echo "=== $(date +%T) ALL DONE"
