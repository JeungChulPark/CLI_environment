#!/bin/bash
# 아까 못 한 실험: 노트북 코드(작업 트리 CLI_env_paper)로 (1) 개선안 900장 2종, (2) 실제 영상 260901 실시간 4구성. GPU0 단독, 사다리 완료 후.
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; OLD=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/missing_chain.log
say(){ echo "[$(date +%T)] $*" >> $LOG; }
say "wait ladder"; until grep -q "ALL DONE" $W/logs/ladder_chain.log; do sleep 30; done
source $W/env.sh; export OBJPOSE_REPO=$R; export YCBV_OUT=$OLD/src/ycbv/out_4090
# ---- (1) 개선안 한 장 평가 900장 ----
cd $R/paper_etrij_261003/src/ycbv
G="--mode text --assign-fallback --size-gate 1.3 --gate-sem 0.25 --gate-appe 0.605 --gate-hsv 0.06 --proposer text:yolov8m-worldv2.pt:$R/paper_etrij_261003/src/ycbv/det_study/out/prompts_best_m.json"
say "smoke g123p"; $PY run_ours.py $G --limit 2 --warmup 1 --out $W/logs/smoke_g123p.json > $W/logs/smoke_g123p.log 2>&1 || { say "FAIL smoke g123p"; exit 1; }
grep -q "PEM 건너뜀" $W/logs/smoke_g123p.log && { say "FAIL smoke g123p: PEM 건너뜀"; exit 1; }
say "start g123p 900"; $PY run_ours.py $G --out $YCBV_OUT/pred_ours_text_g123p.json > $W/logs/g123p.log 2>&1 || say "FAIL g123p"
say "start g123p_k50 900"; $PY run_ours.py $G --verify-candidates 50 --out $YCBV_OUT/pred_ours_text_g123p_k50.json > $W/logs/g123p_k50.log 2>&1 || say "FAIL g123p_k50"
say "evaluate g123p"; $PY evaluate.py --methods ours_text_g123p,ours_text_g123p_k50 --results $OLD/src/ycbv/results_4090.json > $W/logs/evaluate_g123p.log 2>&1 || say "FAIL evaluate g123p"
# ---- (2) 실제 영상 260901 실시간 (로컬 ORB-SLAM3 f2000 + 이 서버 인식, --no-mac) ----
NAS=/mnt/synology/Dataset/data_slam_converted/260901_cbnu_bigeightcircle
SET=$R/objpose/mac_slam/settings/orbslam3_260901_slam_nf2000.yaml
run_live(){ tag=$1; shift
  OUT=$R/objpose/output/real_260901_$tag; mkdir -p $OUT
  pkill -f "objpose/pc/hub.py"; pkill -f slam_stream; sleep 3
  $PY -u $R/objpose/pc/hub.py --slam orbslam3 --extrinsic $R/objpose/rt/260901/X_slam_sam_260901.json \
     --sam-session $NAS/SAM --slam-session $NAS/SLAM --mac-slam-session $NAS/SLAM --out $OUT --features 2000 --no-mac "$@" > $OUT.log 2>&1 &
  HUB=$!
  for i in $(seq 1 150); do grep -q "waiting for SLAM stream" $OUT.log 2>/dev/null && break; sleep 2; done
  for i in $(seq 1 300); do [ -f $OUT/sam6d/READY ] && break; kill -0 $HUB 2>/dev/null || break; sleep 2; done
  [ -f $OUT/sam6d/READY ] || { say "FAIL $tag: sam6d not READY"; kill $HUB 2>/dev/null; return; }
  $HOME/objpose/run_slam.sh --slam orbslam3 --session $NAS/SLAM --settings $SET --features 2000 --time-source frame \
     --mode live --connect 127.0.0.1:17001 --rate 1.0 > $OUT/pc_slam.log 2>&1 &
  for i in $(seq 1 400); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
  sleep 3; kill $HUB 2>/dev/null; pkill -f "objpose/pc/hub.py"; pkill -f slam_stream
  say "done $tag summary=$(grep -c 'summary written' $OUT.log)"
}
G13=$R/sam6d_realtime/configs/yolo_ism_objects_g13.yaml; REC=$R/paper_etrij_261003/src/realdata/live_cores.py
say "live ours";          run_live ours
say "live g13";           run_live g13 --ism-config $G13
say "live g13_noverify";  run_live g13_noverify --ism-config $G13 --no-verify
say "live origL05";       export OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5; run_live origL05 --recognizer $REC; unset OBJPOSE_LIVE_MODE OBJPOSE_ORIG_DESC OBJPOSE_ORIG_THRESH
cd $R/paper_etrij_261003/src/realdata
say "eval real"; $PY eval_real.py --runs real_260901_ours real_260901_g13 real_260901_g13_noverify real_260901_origL05 --out results_real_4090.json > $W/logs/eval_real_4090.log 2>&1 || say "FAIL eval real"
say "ALL DONE"
