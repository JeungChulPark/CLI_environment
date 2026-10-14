#!/bin/bash
# experiment C: our recogniser fed at a capped rate (frames per second handed to recognition)
cd "$(dirname "$0")"
until grep -q ORIG_DONE orig.log; do sleep 30; done
HUB_EXTRA="--sam-feed-hz 0.25" bash run_live.sh hz025
HUB_EXTRA="--sam-feed-hz 0.5" bash run_live.sh hz05
PY=$HOME/anaconda3/envs/sam6d/bin/python; R=../../../objpose/output
for t in hz025 hz05; do $PY eval_live.py --runs "$R/ycbv_live_${t}_0000??" --out results_$t.json > eval_$t.log 2>&1; done
echo RATE_DONE
