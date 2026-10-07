#!/bin/bash
# live baseline: the original SAM-6D as the recogniser of the same live system, after method 5
cd "$(dirname "$0")"
until grep -q M5_DONE m5.log; do sleep 30; done
HUB_EXTRA="--recognizer $PWD/orig_live_core.py" bash run_live.sh orig
PY=$HOME/anaconda3/envs/sam6d/bin/python; R=../../../objpose/output
for t in v2 m5 m15 orig; do $PY eval_live.py --runs "$R/ycbv_live_${t}_0000??" --out results_$t.json > eval_$t.log 2>&1; done
echo ORIG_DONE
