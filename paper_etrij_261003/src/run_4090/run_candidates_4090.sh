#!/bin/bash
# 후보 영역 단계 비교(2026-10-07): fastsam_mask / fastsam_box / yolo_box, 뒤 단계 고정(ViT-S + 원본 판정 + PEM). 900장 ×3 → BOP AR 포함 채점 → 후보 재현율.
#   사용: GPU=3 bash run_candidates_4090.sh   (기본 GPU0; 08 체인이 GPU0 을 쓰는 동안은 GPU=3)
W=$HOME/Dataset/bop/ycbv_work; OLD=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003; S=$OLD/src/ycbv
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/cand_chain.log
say(){ echo "[$(date +%T)] $*" >> $LOG; }
source $W/env.sh; export CUDA_VISIBLE_DEVICES=${GPU:-0}; export YCBV_OUT=$S/out_4090
cd $S; say "start on GPU $CUDA_VISIBLE_DEVICES"
say "smoke fastsam_box"; $PY ladder/run_candidates.py --cand fastsam_box --limit 3 --warmup 1 > $W/logs/cand_smoke.log 2>&1 || { say "FAIL smoke"; exit 1; }
for c in fastsam_box fastsam_mask yolo_box; do say "run $c 900"; $PY ladder/run_candidates.py --cand $c > $W/logs/cand_$c.log 2>&1 || say "FAIL $c"; done
say "evaluate (BOP AR)"; $PY evaluate.py --methods ladder_cand_fastsam_mask,ladder_cand_fastsam_box,ladder_cand_yolo_box --results $S/results_candidates_eval.json > $W/logs/cand_evaluate.log 2>&1 || say "FAIL evaluate"
$PY ladder/score_candidates.py --results $S/results_candidates_eval.json --out $S/results_candidates.json > $W/logs/cand_score.log 2>&1 || say "FAIL score"
say "CAND DONE"
