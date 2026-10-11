#!/bin/bash
# pose verification cost study on the 36-image subset, configuration = our improved recogniser (G123 + method-1 prompts)
cd "$(dirname "$0")/.."; PY=$HOME/anaconda3/envs/sam6d/bin/python
export YCBV_SUBSET=25 YCBV_OUT=$PWD/out/quick
G="--mode text --warmup 2 --assign-fallback --size-gate 1.3 --gate-sem 0.25 --gate-appe 0.605 --gate-hsv 0.06 --proposer text:yolov8m-worldv2.pt:$PWD/det_study/out/prompts_best_m.json"
for k in 100 50 30 10; do $PY run_ours.py $G --verify-topk $k --out $YCBV_OUT/pred_vc_top$k.json || echo "FAIL top$k"; done
for s in 3 10; do $PY run_ours.py $G --verify-stride $s --out $YCBV_OUT/pred_vc_stride$s.json || echo "FAIL stride$s"; done
for p in fp16 bf16; do $PY run_ours.py $G --precision $p --out $YCBV_OUT/pred_vc_$p.json || echo "FAIL $p"; done
$PY run_ours.py $G --verify-topk 30 --precision bf16 --out $YCBV_OUT/pred_vc_top30_bf16.json || echo "FAIL top30bf16"
$PY run_ours.py --mode text --warmup 2 --verify-topk 30 --out $YCBV_OUT/pred_vc_L4_top30.json || echo "FAIL L4top30"
echo VC_DONE
