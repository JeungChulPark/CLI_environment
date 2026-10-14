#!/bin/bash
# 군집 먼저 + 되돌림(fallback) 36장, GPU1
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src/ycbv
O=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003/src/ycbv/det_study/out_4090/vc
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/vc_cluster_first.log
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
source $W/env.sh; export OBJPOSE_REPO=$R CUDA_VISIBLE_DEVICES=1 YCBV_SUBSET=25; cd $S
G="--mode text --warmup 2 --assign-fallback --size-gate 1.3 --gate-sem 0.25 --gate-appe 0.605 --gate-hsv 0.06 --proposer text:yolov8m-worldv2.pt:$S/det_study/out/prompts_best_m.json"
say "smoke cf_fallback"; $PY run_ours.py $G --limit 2 --verify-cluster-first --out $O/smoke_cff.json > $O/smoke_cff.log 2>&1 && ! grep -q 'PEM 건너뜀' $O/smoke_cff.log || { say "SMOKE_FAIL cf_fallback"; tail -5 $O/smoke_cff.log; exit 1; }
say "start clusterfirst_fb"; $PY run_ours.py $G --verify-cluster-first --out $O/pred_clusterfirst_fb.json > $O/clusterfirst_fb.log 2>&1 && say "done clusterfirst_fb $(grep -o "{'median'.*" $O/clusterfirst_fb.log | head -1)" || say "FAIL clusterfirst_fb"
say "start L4_clusterfirst_fb"; $PY run_ours.py --mode text --warmup 2 --verify-cluster-first --out $O/pred_L4_clusterfirst_fb.json > $O/L4_clusterfirst_fb.log 2>&1 && say "done L4_clusterfirst_fb" || say "FAIL L4_clusterfirst_fb"
say "VC2_DONE"
