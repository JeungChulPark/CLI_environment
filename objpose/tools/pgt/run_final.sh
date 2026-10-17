#!/usr/bin/env bash
# test-frame rerun of the proposed pipeline with the settings picked on the validation frames (final.env), then BOP scoring
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
O=objpose/output/estcmp_260927
set -a; . objpose/tools/pgt/final.env; set +a
echo "=== $(date +%T) final with VERIFY_OVR=${VERIFY_OVR:-} ISM_OVR=${ISM_OVR:-} PROMPT_OVR=${PROMPT_OVR:-}"
$PY $R --method proposed --dataset ycbv --mode bop19 --out $O/ycbv_proposedT 2>&1 | grep -E "Traceback|Error:" | grep -v PEM
$PY $R --method proposed --dataset ycbv --mode bop19 --gt-boxes --out $O/ycbv_gtboxT 2>&1 | grep -E "Traceback|Error:" | grep -v PEM
$PY $R --method proposed --dataset ycbm --mode still --stride 8 --out $O/ycbm_proposedT 2>&1 | grep -E "Traceback|Error:" | grep -v PEM
$PY $R --method proposed --dataset ycbm --mode still --stride 8 --gt-boxes --out $O/ycbm_gtboxT 2>&1 | grep -E "Traceback|Error:" | grep -v PEM
echo "=== $(date +%T) runs done"
$PY objpose/tools/pgt/final_bop.py
echo "=== $(date +%T) FINAL DONE"
