#!/bin/bash
# live replay of the improved recogniser (g13p) with pose verification limited to the 50 best
# geometry candidates (hub --verify-candidates 50); everything else as the earlier g13p run
cd "$(dirname "$0")"; PY=$HOME/anaconda3/envs/sam6d/bin/python
until grep -qE "VK_DONE|SMOKE_FAIL" ../ycbv/out/quick/verify_cand.log; do sleep 20; done
YCBV_ISM=$HOME/DeepLearning/Dataset/bop/ycbv_work/ycbv_objects_g13p.yaml HUB_EXTRA="--verify-candidates 50" bash run_live.sh g13p_k50
R=../../../objpose/output
$PY eval_live.py --runs "$R/ycbv_live_g13p_k50_0000??" --out results_g13p_k50.json > eval_g13p_k50.log 2>&1
$PY eval_penalized.py --tags $(echo orig orig_th03 orig_th04 orig_th05 orig_scene origS_th04 origS_th05 origS_th06 origS_th07 v2 m5 m15 hz05 hz025 g13p g13p_k50 | tr ' ' ,) --out results_penalized_k50.json > eval_penalized_k50.log 2>&1
$PY eval_strict_found.py orig orig_th03 orig_th04 orig_th05 orig_scene origS_th04 origS_th05 origS_th06 origS_th07 v2 m5 m15 hz05 hz025 g13p g13p_k50 > eval_strict_all.log 2>&1   # rewrites results_strict_found.json with every tag
echo K50_LIVE_DONE
