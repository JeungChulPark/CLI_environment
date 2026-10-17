#!/usr/bin/env bash
# estimator comparison, part 2: the three methods on the recorded session's eval frames (pseudo GT from build_pgt.py)
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
P=objpose/output/pgt_260915_inlong_middle
run() { echo "=== $(date +%T) $*"; $PY $R "$@" 2>&1 | grep -E --line-buffered "done:|Traceback|Error|processed"; }
run --method orig     --dataset real --mode still --all-objects --root $P/eval --out $P/runs/eval_orig
run --method proposed --dataset real --mode still --all-objects --root $P/eval --out $P/runs/eval_proposed
run --method proposed --dataset real --mode still --all-objects --root $P/eval --gt-boxes --out $P/runs/eval_gtbox
echo "=== $(date +%T) QUEUE2 DONE"
