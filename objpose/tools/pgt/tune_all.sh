#!/usr/bin/env bash
# validation-frame diagnosis + tuning, one queue (restart after the disk-full stop)
set -u
cd "$(dirname "$0")/../../.."
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
V=objpose/output/estcmp_260927/val
VM="--dataset ycbm --mode still --stride 24 --offset 4"
rm -rf $V/ycbm_gtbox_nogate $V/ycbm_gtbox_noverify
echo "=== $(date +%T) finish ycbm diag"
$PY $R --method proposed $VM --gt-boxes --out $V/ycbm_gtbox 2>&1 | grep -E "Traceback|Error:" | grep -v PEM
$PY $R --method proposed_nogate $VM --gt-boxes --out $V/ycbm_gtbox_nogate 2>&1 | grep -E "Traceback|Error:" | grep -v PEM
$PY $R --method proposed_noverify $VM --gt-boxes --out $V/ycbm_gtbox_noverify 2>&1 | grep -E "Traceback|Error:" | grep -v PEM
$PY objpose/tools/pgt/val_score.py ycbm $V/ycbm_gtbox $V/ycbm_gtbox_nogate $V/ycbm_gtbox_noverify 2>&1 | grep "^ycbm"
T=objpose/tools/pgt/run_tune.sh
$T v30t40 gtbox 'VERIFY_OVR={"mask_iou_min":0.30,"texture_min_score":0.40}'
$T v25t35 gtbox 'VERIFY_OVR={"mask_iou_min":0.25,"texture_min_score":0.35,"cluster_min_occupancy":0.4}'
$T v30t40g gtbox 'VERIFY_OVR={"mask_iou_min":0.30,"texture_min_score":0.40}' 'ISM_OVR={"similarity_threshold":0.30,"appe_v2_gate":0.55}'
$T pA yolo 'PROMPT_OVR={"ycbv_pudding_box":"chocolate pudding box","ycbv_gelatin_box":"jello box","ycbv_potted_meat_can":"spam can","ycbv_large_clamp":"spring clamp","ycbv_foam_brick":"red block"}'
$T pB yolo 'PROMPT_OVR={"ycbv_pudding_box":"small brown box","ycbv_gelatin_box":"small red box","ycbv_potted_meat_can":"small blue can","ycbv_large_clamp":"black spring clamp","ycbv_foam_brick":"red brick"}'
echo "=== $(date +%T) TUNE DONE"
