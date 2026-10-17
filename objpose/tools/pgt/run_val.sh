#!/usr/bin/env bash
# stage diagnosis of the proposed pipeline on VALIDATION frames (never the reported test frames)
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
V=objpose/output/estcmp_260927/val
run() { echo "=== $(date +%T) $*"; $PY $R "$@" 2>&1 | grep -E --line-buffered "Traceback|Error" ; }
VY="--dataset ycbv --mode still --stride 100 --exclude-targets"
VM="--dataset ycbm --mode still --stride 24 --offset 4"
for d in Y M; do
  eval A=\$V$d; t=$( [ $d = Y ] && echo ycbv || echo ycbm )
  run --method proposed $A --out $V/${t}_proposed
  run --method proposed $A --gt-boxes --out $V/${t}_gtbox
  run --method proposed_nogate $A --gt-boxes --out $V/${t}_gtbox_nogate
  run --method proposed_noverify $A --gt-boxes --out $V/${t}_gtbox_noverify
done
echo "=== $(date +%T) VAL DONE"
