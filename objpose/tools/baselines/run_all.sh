#!/usr/bin/env bash
# thesis comparison runs, sequential on one GPU. Re-running skips finished scenes.
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
O=objpose/output/cmp_260924
run() { echo "=== $(date +%T) $*"; $PY $R "$@" 2>&1 | grep -v -i "warn\|pynvml\|torchvision\|compatib\|^  import"; }
run --method orig     --dataset ycbv --mode bop19    --out $O/orig_ycbv_bop19
run --method proposed --dataset ycbv --mode bop19    --out $O/proposed_ycbv_bop19
run --method orig     --dataset lmo  --mode bop19    --out $O/orig_lmo_bop19
run --method proposed --dataset lmo  --mode bop19    --out $O/proposed_lmo_bop19
run --method orig     --dataset ycbv --mode realtime --out $O/orig_ycbv_realtime
run --method orig_vits --dataset ycbv --mode bop19   --out $O/origvits_ycbv_bop19
run --method orig_vits --dataset lmo  --mode bop19   --out $O/origvits_lmo_bop19
run --method orig     --dataset ycbv --mode bop19    --out $O/orig_ycbv_bop19
run --method proposed --dataset ycbv --mode bop19 --all-objects --out $O/proposed_ycbv_bop19_all21
run --method orig     --dataset ycbv --mode bop19 --all-objects --out $O/orig_ycbv_bop19_all21
echo "=== $(date +%T) ALL DONE"
