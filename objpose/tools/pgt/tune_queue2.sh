#!/usr/bin/env bash
cd "$(dirname "$0")"
T=./run_tune.sh
until grep -q "TUNE1 DONE" ../../output/estcmp_260927/val/tune1.log; do sleep 15; done
$T pA yolo 'PROMPT_OVR={"ycbv_pudding_box":"chocolate pudding box","ycbv_gelatin_box":"jello box","ycbv_potted_meat_can":"spam can","ycbv_large_clamp":"spring clamp","ycbv_foam_brick":"red block"}'
$T pB yolo 'PROMPT_OVR={"ycbv_pudding_box":"small brown box","ycbv_gelatin_box":"small red box","ycbv_potted_meat_can":"small blue can","ycbv_large_clamp":"black spring clamp","ycbv_foam_brick":"red brick"}'
echo "=== TUNE2 DONE"
