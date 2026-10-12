#!/bin/bash
# 자세 검증 경량화: 기준(300개) / 후보 50개 / 군집 먼저 — 36장(YCBV_SUBSET=25), GPU1, 2026-10-07
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src/ycbv
O=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003/src/ycbv/det_study/out_4090/vc; mkdir -p $O
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/vc_cluster_first.log
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
source $W/env.sh; export OBJPOSE_REPO=$R CUDA_VISIBLE_DEVICES=1 YCBV_SUBSET=25
cd $S
G="--mode text --warmup 2 --assign-fallback --size-gate 1.3 --gate-sem 0.25 --gate-appe 0.605 --gate-hsv 0.06 --proposer text:yolov8m-worldv2.pt:$S/det_study/out/prompts_best_m.json"
run(){ name=$1; shift; say "start $name"; $PY run_ours.py $G "$@" --out $O/pred_$name.json > $O/$name.log 2>&1 && say "done $name $(grep -o "{'median'.*" $O/$name.log | head -1)" || say "FAIL $name"; grep -c 'PEM 건너뜀' $O/$name.log | sed "s/^/   PEM skipped: /"; }
run base300
run cand50 --verify-candidates 50
run clusterfirst --verify-cluster-first
run clusterfirst_cand100 --verify-cluster-first --verify-candidates 100
# L4(배포)에도 적용
say "start L4_base"; $PY run_ours.py --mode text --warmup 2 --out $O/pred_L4_base.json > $O/L4_base.log 2>&1 && say "done L4_base" || say "FAIL L4_base"
say "start L4_clusterfirst"; $PY run_ours.py --mode text --warmup 2 --verify-cluster-first --out $O/pred_L4_clusterfirst.json > $O/L4_clusterfirst.log 2>&1 && say "done L4_clusterfirst" || say "FAIL L4_clusterfirst"
say "VC_DONE"
