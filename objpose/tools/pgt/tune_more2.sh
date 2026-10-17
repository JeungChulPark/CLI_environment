#!/usr/bin/env bash
cd "$(dirname "$0")/../../.."
until grep -q "MORE DONE" objpose/output/estcmp_260927/val/tune_more.log; do sleep 15; done
T=objpose/tools/pgt/run_tune.sh
$T g2v gtbox 'VERIFY_OVR={"mask_iou_min":0.30,"texture_min_score":0.40}' 'ISM_OVR={"similarity_threshold":0.25,"appe_v2_gate":0.50}'
$T g2 gtbox 'ISM_OVR={"similarity_threshold":0.25,"appe_v2_gate":0.50}'
echo "=== $(date +%T) MORE2 DONE"
