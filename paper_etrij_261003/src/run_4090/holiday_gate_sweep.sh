#!/bin/bash
# 연휴 실험 B (2026-10-08): 검증 관문 문턱 운영 곡선(윤곽 IoU, 질감) + 사다리 L2/L3/L5 4090 재채점, GPU1
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; SRC=$R/paper_etrij_261003/src/ycbv
O=$HOME/DeepLearning/server/CLI_environment/paper_etrij_261003/src/ycbv/out_4090; PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/holiday_gate.log
say(){ echo "[$(date +%F' '%T)] $*" | tee -a $LOG; }
source $W/env.sh; export OBJPOSE_REPO=$R SAM6D_ORIG=$R/sam6d_ws/sam6d_master/SAM-6D YCBV_OUT=$O YCBV_RUN_LABEL="gate sweep, RTX 4090 (GPU1), 2026-10-08" CUDA_VISIBLE_DEVICES=1
cd $SRC
run(){ tag=$1; shift; f=$O/pred_hybS05_cf_${tag}_900.json
  [ -s $f ] && { say "skip $tag"; return; }
  say "start $tag ($*)"; env "$@" $PY run_ours.py --mode text --warmup 3 --orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg fastsam_full --verify-cluster-first --out $f > $O/hybS05_cf_${tag}_900.log 2>&1
  say "done $tag rc=$? $(grep -o "{'median'.*" $O/hybS05_cf_${tag}_900.log | head -1)"; ln -sf pred_hybS05_cf_${tag}_900.json $O/pred_ladder_hybS05cf_${tag}.json; }
say "HOLIDAY GATE START"
run mask030 YCBV_MASK_IOU_MIN=0.30
run mask035 YCBV_MASK_IOU_MIN=0.35
run mask050 YCBV_MASK_IOU_MIN=0.50
run mask055 YCBV_MASK_IOU_MIN=0.55
run tex040 YCBV_TEXTURE_MIN=0.40
run tex042 YCBV_TEXTURE_MIN=0.42
run tex048 YCBV_TEXTURE_MIN=0.48
run tex050 YCBV_TEXTURE_MIN=0.50
TAGS=mask030,mask035,mask050,mask055,tex040,tex042,tex048,tex050
M=$(echo $TAGS | sed 's/[^,]*/ladder_hybS05cf_&/g')
say "evaluate (fast)"; CUDA_VISIBLE_DEVICES="" $PY evaluate.py --skip_bop --methods $M --results $O/results_gate_sweep_4090.json > $W/logs/holiday_gate_eval.log 2>&1 && say "eval fast ok" || say "FAIL eval fast"
P=pred_ladder_L0.json,pred_hybS05_cf_900.json$(echo $TAGS | sed 's/[^,]*/,pred_hybS05_cf_&_900.json/g' | tr -d '\n'); N=L0_원본,개선안3_cf_기본$(echo $TAGS | sed 's/[^,]*/,개선안3_cf_&/g' | tr -d '\n')
CUDA_VISIBLE_DEVICES="" $PY ladder/same_object_adds.py --preds $P --names $N --out $O/same_object_gate_sweep_4090.json > $W/logs/holiday_gate_sameobj.log 2>&1 && say "same-object ok" || say "FAIL same-object"
say "evaluate (BOP AR) gate sweep + ladder L2/L3/L5 re-score"
CUDA_VISIBLE_DEVICES="" $PY evaluate.py --methods $M --results $O/results_gate_sweep_4090.json > $W/logs/holiday_gate_bop.log 2>&1 && say "bop gate ok" || say "FAIL bop gate"
CUDA_VISIBLE_DEVICES="" $PY evaluate.py --methods ladder_L2,ladder_L3,ladder_L5 --results $O/results_ladder_rescore_4090.json > $W/logs/holiday_ladder_rescore.log 2>&1 && say "ladder rescore ok" || say "FAIL ladder rescore"
say "HOLIDAY GATE ALL DONE"
