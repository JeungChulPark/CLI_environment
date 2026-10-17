#!/usr/bin/env bash
cd "$(dirname "$0")"
T=./run_tune.sh
until grep -q "VAL DONE" ../../output/estcmp_260927/val/val.log; do sleep 15; done
$T v30t40 gtbox 'VERIFY_OVR={"mask_iou_min":0.30,"texture_min_score":0.40}'
$T v25t35 gtbox 'VERIFY_OVR={"mask_iou_min":0.25,"texture_min_score":0.35,"cluster_min_occupancy":0.4}'
$T v30t40g gtbox 'VERIFY_OVR={"mask_iou_min":0.30,"texture_min_score":0.40}' 'ISM_OVR={"similarity_threshold":0.30,"appe_v2_gate":0.55}'
echo "=== TUNE1 DONE"
