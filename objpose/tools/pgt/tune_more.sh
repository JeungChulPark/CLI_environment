#!/usr/bin/env bash
cd "$(dirname "$0")/../../.."
until grep -q "TUNE DONE" objpose/output/estcmp_260927/val/tune_all.log; do sleep 15; done
T=objpose/tools/pgt/run_tune.sh
$T occ03 gtbox 'VERIFY_OVR={"cluster_min_occupancy":0.3}'
$T occ00 gtbox 'VERIFY_OVR={"cluster_min_occupancy":0.0}'
echo "=== $(date +%T) MORE DONE"
