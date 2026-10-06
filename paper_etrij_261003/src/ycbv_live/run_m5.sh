#!/bin/bash
# method 5 (map-projected proposals) on the 12 YCB-V live replays, after the single-image queue
cd "$(dirname "$0")"; Q=../ycbv/det_study/out/queue.log
until grep -q QUEUE_DONE $Q; do sleep 30; done
HUB_EXTRA="--map-prior" bash run_live.sh m5
HUB_EXTRA="--map-prior" YCBV_ISM=$HOME/DeepLearning/Dataset/bop/ycbv_work/ycbv_objects_m1.yaml bash run_live.sh m15
PY=$HOME/anaconda3/envs/sam6d/bin/python; R=../../../objpose/output
for t in v2 m5 m15; do $PY eval_live.py --runs "$R/ycbv_live_${t}_0000??" --out results_$t.json > eval_$t.log 2>&1; done
echo M5_DONE
