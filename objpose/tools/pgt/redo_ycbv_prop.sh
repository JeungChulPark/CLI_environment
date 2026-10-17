#!/usr/bin/env bash
# YCB-V proposed (tuned) rerun alone on the GPU: the first run shared the GPU with a validation run, so its timing was inflated
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
O=objpose/output/estcmp_260927
set -a; . objpose/tools/pgt/final.env; set +a
rm -rf $O/ycbv_proposedT
$PY objpose/tools/baselines/run_compare.py --method proposed --dataset ycbv --mode bop19 --out $O/ycbv_proposedT 2>&1 | grep -E "Traceback|Error:" | grep -v PEM
unset VERIFY_OVR PROMPT_OVR
$PY objpose/tools/pgt/final_bop.py
EST_SUFFIX=T $PY objpose/tools/pgt/score_estcmp.py ycbv ycbm real
EST_SUFFIX=T $PY objpose/tools/pgt/occlusion_bins.py
echo REDO DONE
