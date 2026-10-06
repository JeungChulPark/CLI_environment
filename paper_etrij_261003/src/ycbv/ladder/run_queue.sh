#!/bin/bash
# ablation ladder L0..L5 on the 900 YCB-V images, in one session, after the live baseline variants
set -u
cd "$(dirname "$0")/.."; PY=$HOME/anaconda3/envs/sam6d/bin/python
until grep -q VARIANTS_DONE ../ycbv_live/variants.log; do sleep 30; done
# smoke test (5 images) of the new code paths before the long runs
$PY ladder/run_ladder.py --level L2 --limit 5 --warmup 1 || { echo SMOKE_FAIL; exit 1; }
$PY ladder/run_ladder.py --level L1 --limit 5 --warmup 1 || { echo SMOKE_FAIL; exit 1; }
for L in L0 L1 L2; do $PY ladder/run_ladder.py --level $L || echo "FAIL $L"; done
$PY run_ours.py --mode text --no-verify --tag ladder_L3 --out out/pred_ladder_L3.json || echo "FAIL L3"
$PY run_ours.py --mode text --out out/pred_ladder_L4.json || echo "FAIL L4"
$PY run_ours.py --mode text --proposer "text:yolov8m-worldv2.pt:$PWD/det_study/out/prompts_best_m.json" --out out/pred_ladder_L5.json || echo "FAIL L5"
$PY evaluate.py --results ladder/results_ladder.json --methods ladder_L0,ladder_L1,ladder_L2,ladder_L3,ladder_L4,ladder_L5
echo LADDER_DONE
