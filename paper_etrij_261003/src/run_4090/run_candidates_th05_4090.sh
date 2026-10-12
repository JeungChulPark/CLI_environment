#!/bin/bash
# 후보 영역 단계 비교, 물체 판정 기준 0.5 (2026-10-07): fastsam_mask / fastsam_box / yolo_box, 뒤 단계 고정(DINOv2 ViT-S + 원본 판정 0.5 + PEM, 검증 없음).
# GPU0 단독(다른 GPU 작업과 동시 실행 금지 — 10-06/07 드라이버 멈춤 3회). 900장 ×3 → BOP AR 포함 채점 → 후보 재현율.
W=$HOME/Dataset/bop/ycbv_work; OLD=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003; S=$OLD/src/ycbv
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/cand05_chain.log
say(){ echo "[$(date +%T)] $*" >> $LOG; }
source $W/env.sh; export CUDA_VISIBLE_DEVICES=0; export YCBV_OUT=$S/out_4090
cd $S; say "start on GPU $CUDA_VISIBLE_DEVICES (thresh 0.5)"
say "smoke fastsam_box"; $PY ladder/run_candidates.py --cand fastsam_box --thresh 0.5 --limit 3 --warmup 1 > $W/logs/cand05_smoke.log 2>&1 || { say "FAIL smoke"; exit 1; }
rm -f $YCBV_OUT/pred_ladder_cand_fastsam_box_th05.json
for c in fastsam_mask fastsam_box yolo_box; do
  say "run $c 900"; $PY ladder/run_candidates.py --cand $c --thresh 0.5 > $W/logs/cand05_$c.log 2>&1 || say "FAIL $c"
done
M=ladder_cand_fastsam_mask_th05,ladder_cand_fastsam_box_th05,ladder_cand_yolo_box_th05
say "evaluate (BOP AR)"; $PY evaluate.py --methods $M --results $S/results_candidates_th05_eval.json > $W/logs/cand05_evaluate.log 2>&1 || say "FAIL evaluate"
say "score candidates"; $PY ladder/score_candidates.py --results $S/results_candidates_th05_eval.json --out $S/results_candidates_th05.json > $W/logs/cand05_score.log 2>&1 || say "FAIL score"
say "CAND05 DONE"
