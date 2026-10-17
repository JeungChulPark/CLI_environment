#!/usr/bin/env bash
# estimator comparison, part 1: pseudo-GT build runs on the recorded session, then YCB-V and YCB-M (3 methods each)
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
P=objpose/output/pgt_260915_inlong_middle
O=objpose/output/estcmp_260927
run() { echo "=== $(date +%T) $*"; $PY $R "$@" 2>&1 | grep -E --line-buffered "done:|Traceback|Error|processed"; }
run --method proposed --dataset real --mode still --all-objects --root $P/build --out $P/runs/build_proposed
run --method orig     --dataset real --mode still --all-objects --root $P/build --out $P/runs/build_orig
for m in proposed orig; do run --method $m --dataset ycbv --mode bop19 --out $O/ycbv_$m; done
run --method proposed --dataset ycbv --mode bop19 --gt-boxes --out $O/ycbv_gtbox
for m in proposed orig; do run --method $m --dataset ycbm --mode still --stride 8 --out $O/ycbm_$m; done
run --method proposed --dataset ycbm --mode still --stride 8 --gt-boxes --out $O/ycbm_gtbox
echo "=== $(date +%T) QUEUE1 DONE"
