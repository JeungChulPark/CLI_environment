#!/bin/bash
# 연휴 실험 B (이어서, .41 RTX PRO 6000 GPU3): 질감 문턱 4점 + 평가(BOP AR 포함) + 사다리 L2/L3/L5 재채점
W=$HOME/Dataset/bop/ycbv_work; O=$W/out_gate; LOG=$W/logs/gate41.log; PY=$HOME/anaconda3/envs/sam6d/bin/python
say(){ echo "[$(date +%F' '%T)] $*" | tee -a $LOG; }
source $W/env.sh; export YCBV_OUT=$O CUDA_VISIBLE_DEVICES=3 YCBV_RUN_LABEL="gate sweep (texture points), RTX PRO 6000 Blackwell GPU3, 2026-10-08"
cd $OBJPOSE_REPO/paper_etrij_261003/src/ycbv
run(){ tag=$1; shift; f=$O/pred_hybS05_cf_${tag}_900.json
  [ -s $f ] && { say "skip $tag"; return; }
  say "start $tag ($*)"; env "$@" $PY run_ours.py --mode text --warmup 3 --orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg fastsam_full --verify-cluster-first --out $f > $O/hybS05_cf_${tag}_900.log 2>&1
  say "done $tag rc=$? $(grep -o "{'median'.*" $O/hybS05_cf_${tag}_900.log | head -1)"; }
say "GATE41 START"
run tex040 YCBV_TEXTURE_MIN=0.40
run tex042 YCBV_TEXTURE_MIN=0.42
run tex048 YCBV_TEXTURE_MIN=0.48
run tex050 YCBV_TEXTURE_MIN=0.50
TAGS=mask030,mask035,mask050,mask055,tex040,tex042,tex048,tex050
for t in $(echo $TAGS | tr , ' '); do ln -sf pred_hybS05_cf_${t}_900.json $O/pred_ladder_hybS05cf_${t}.json; done; ln -sf pred_hybS05_cf_900.json $O/pred_ladder_hybS05cf.json
M=$(echo $TAGS | sed 's/[^,]*/ladder_hybS05cf_&/g')
say "evaluate fast"; CUDA_VISIBLE_DEVICES="" $PY evaluate.py --skip_bop --methods $M,ladder_hybS05cf,ladder_L0 --results $O/results_gate_sweep.json > $W/logs/gate41_eval.log 2>&1 && say "eval fast ok" || say "FAIL eval fast"
P=pred_ladder_L0.json,pred_hybS05_cf_900.json$(echo $TAGS | sed 's/[^,]*/,pred_hybS05_cf_&_900.json/g'); N=L0_원본,개선안3_cf_기본$(echo $TAGS | sed 's/[^,]*/,개선안3_cf_&/g')
CUDA_VISIBLE_DEVICES="" $PY ladder/same_object_adds.py --preds $P --names $N --out $O/same_object_gate_sweep.json > $W/logs/gate41_sameobj.log 2>&1 && say "same-object ok" || say "FAIL same-object"
say "evaluate BOP AR (gate sweep)"; CUDA_VISIBLE_DEVICES="" $PY evaluate.py --methods $M --results $O/results_gate_sweep.json > $W/logs/gate41_bop.log 2>&1 && say "bop gate ok" || say "FAIL bop gate"
say "ladder L2/L3/L5 re-score"; CUDA_VISIBLE_DEVICES="" $PY evaluate.py --methods ladder_L2,ladder_L3,ladder_L5 --results $O/results_ladder_rescore.json > $W/logs/gate41_ladder.log 2>&1 && say "ladder rescore ok" || say "FAIL ladder rescore"
say "GATE41 ALL DONE"
