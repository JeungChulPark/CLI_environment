#!/bin/bash
# quick check on every 25th image (36 images) of the 900: all ladder levels + options A/B, outputs in out/quick
cd "$(dirname "$0")/.."; PY=$HOME/anaconda3/envs/sam6d/bin/python
export YCBV_SUBSET=25 YCBV_OUT=$PWD/out/quick; mkdir -p $YCBV_OUT
for L in L0 L1 L2; do $PY ladder/run_ladder.py --level $L --warmup 2; done
$PY ladder/run_ladder.py --level L1 --thresh 0.4 --warmup 2
$PY ladder/run_ladder.py --level L1 --thresh 0.5 --warmup 2
$PY run_ours.py --mode text --no-verify --warmup 2 --out $YCBV_OUT/pred_ladder_L3.json
$PY run_ours.py --mode text --warmup 2 --out $YCBV_OUT/pred_ladder_L4.json
$PY run_ours.py --mode text --warmup 2 --proposer "text:yolov8m-worldv2.pt:$PWD/det_study/out/prompts_best_m.json" --out $YCBV_OUT/pred_ladder_L5.json
$PY run_ours.py --mode text --warmup 2 --proposer fastsam --top-k 200 --no-verify --out $YCBV_OUT/pred_ladder_A_noverify.json
$PY run_ours.py --mode text --warmup 2 --proposer fastsam --top-k 200 --out $YCBV_OUT/pred_ladder_A.json
echo QUICK_DONE
