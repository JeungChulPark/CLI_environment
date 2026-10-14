#!/bin/bash
# 간략 버전(2026-10-06 16:10, 사용자 요청): 개선안 36장 ×2 → 실제 영상 3구성 → YCB-V 실시간 3장면 × 3구성 → 채점
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src; OLD=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/brief_chain.log; BAGS=$HOME/DeepLearning/Dataset/bop/ycbv_bags
say(){ echo "[$(date +%T)] $*" >> $LOG; }
source $W/env.sh; export OBJPOSE_REPO=$R
if [ -z "${SKIP_QUICK:-}" ]; then
# ---- (1) 개선안 36장 ----
cd $S/ycbv; export YCBV_OUT=$OLD/src/ycbv/out_4090/quick; mkdir -p $YCBV_OUT
G="--mode text --warmup 2 --assign-fallback --size-gate 1.3 --gate-sem 0.25 --gate-appe 0.605 --gate-hsv 0.06 --proposer text:yolov8m-worldv2.pt:$S/ycbv/det_study/out/prompts_best_m.json"
say "quick g123p 36"; YCBV_SUBSET=25 $PY run_ours.py $G --out $YCBV_OUT/pred_ours_text_g123p.json > $W/logs/quick_g123p.log 2>&1 || say "FAIL quick g123p"
say "quick g123p_k50 36"; YCBV_SUBSET=25 $PY run_ours.py $G --verify-candidates 50 --out $YCBV_OUT/pred_ours_text_g123p_k50.json > $W/logs/quick_g123p_k50.log 2>&1 || say "FAIL quick k50"
say "quick L4 36"; YCBV_SUBSET=25 $PY run_ours.py --mode text --warmup 2 --out $YCBV_OUT/pred_ours_text.json > $W/logs/quick_L4.log 2>&1 || say "FAIL quick L4"
say "evaluate quick"; YCBV_SUBSET=25 $PY evaluate.py --skip_bop --methods ours_text,ours_text_g123p,ours_text_g123p_k50 --results $OLD/src/ycbv/results_4090_quick36.json > $W/logs/evaluate_quick.log 2>&1 || say "FAIL evaluate quick"
export YCBV_OUT=$OLD/src/ycbv/out_4090
say "evaluate g123p 900 (이미 완료분)"; $PY evaluate.py --skip_bop --methods ours_text_g123p --results $OLD/src/ycbv/results_4090.json > $W/logs/evaluate_g123p900.log 2>&1 || say "FAIL evaluate g123p 900"
fi
if [ -z "${SKIP_REAL:-}" ]; then
# ---- (2) 실제 영상 260901, 3구성 ----
NAS=/mnt/synology/Dataset/data_slam_converted/260901_cbnu_bigeightcircle; SET=$R/objpose/mac_slam/settings/orbslam3_260901_slam_nf2000.yaml
run_live(){ tag=$1; shift
  OUT=$R/objpose/output/real_260901_$tag; mkdir -p $OUT
  pkill -f "objpose/pc/hub[.]py"; pkill -f slam_stream; sleep 3
  $PY -u $R/objpose/pc/hub.py --slam orbslam3 --extrinsic $R/objpose/rt/260901/X_slam_sam_260901.json \
     --sam-session $NAS/SAM --slam-session $NAS/SLAM --mac-slam-session $NAS/SLAM --out $OUT --features 2000 --no-mac "$@" > $OUT.log 2>&1 &
  HUB=$!
  for i in $(seq 1 150); do grep -q "waiting for SLAM stream" $OUT.log 2>/dev/null && break; sleep 2; done
  for i in $(seq 1 300); do [ -f $OUT/sam6d/READY ] && break; kill -0 $HUB 2>/dev/null || break; sleep 2; done
  [ -f $OUT/sam6d/READY ] || { say "FAIL $tag: sam6d not READY"; kill $HUB 2>/dev/null; return; }
  $HOME/objpose/run_slam.sh --slam orbslam3 --session $NAS/SLAM --settings $SET --features 2000 --time-source frame \
     --mode live --connect 127.0.0.1:17001 --rate 1.0 > $OUT/pc_slam.log 2>&1 &
  for i in $(seq 1 400); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
  sleep 3; kill $HUB 2>/dev/null; pkill -f "objpose/pc/hub[.]py"; pkill -f slam_stream
  say "done $tag summary=$(grep -c 'summary written' $OUT.log)"
}
G13=$R/sam6d_realtime/configs/yolo_ism_objects_g13.yaml; REC=$R/paper_etrij_261003/src/realdata/live_cores.py
say "live real g13";          run_live g13 --ism-config $G13
say "live real g13_noverify"; run_live g13_noverify --ism-config $G13 --no-verify
say "live real origL05";      export OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5; run_live origL05 --recognizer $REC; unset OBJPOSE_LIVE_MODE OBJPOSE_ORIG_DESC OBJPOSE_ORIG_THRESH
cd $S/realdata; say "eval real"; $PY eval_real.py --runs real_260901_g13 real_260901_g13_noverify real_260901_origL05 --out results_real_4090.json > $W/logs/eval_real_4090.log 2>&1 || say "FAIL eval real"
fi
# ---- (3) YCB-V 실시간 3장면 × 3구성 ----
export SLAM_RUN=$HOME/objpose/run_slam.sh SAM6D_PY=$PY YCBV_ROOT=$HOME/Dataset/bop/ycbv
for s in 49 53 55; do d=$BAGS/ycbv_0000$s; [ -f $d/SLAM/SAM_0.db3 ] || { mkdir -p $d/SLAM; for f in $d/SAM/*; do ln -f "$f" $d/SLAM/ 2>/dev/null || cp "$f" $d/SLAM/; done; }; done
cd $S/ycbv_live
ycbv_run(){ tag=$1; shift; todo=""; for s in "$@"; do [ -f $R/objpose/output/ycbv_live_${tag}_0000$s/summary.json ] || todo="$todo $s"; done
  [ -n "$todo" ] && bash run_live.sh $tag $todo >> $W/logs/ycbv_live_runs.log 2>&1; say "ycbv $tag done ($(for s in "$@"; do [ -f $R/objpose/output/ycbv_live_${tag}_0000$s/summary.json ] && printf "%s " $s; done))"; }
say "ycbv live orig_th05";  HUB_EXTRA="--recognizer $S/ycbv_live/orig_live_core.py" OBJPOSE_ORIG_THRESH=0.5 OBJPOSE_ORIG_DESC=dinov2_vitl14 ycbv_run orig_th05 49 53 55
say "ycbv live g13p";       YCBV_ISM=$W/ycbv_objects_g13p.yaml ycbv_run g13p 49 53 55
say "ycbv live g13p_nover"; YCBV_ISM=$W/ycbv_objects_g13p.yaml HUB_EXTRA="--no-verify" ycbv_run g13p_nover 49 53 55
pkill -f "objpose/pc/hub[.]py"; pkill -f slam_stream
say "eval ycbv"; for t in orig_th05 g13p g13p_nover; do $PY eval_live.py --runs "$R/objpose/output/ycbv_live_${t}_0000??" --out results_$t.json > $W/logs/eval_live_$t.log 2>&1 || say "FAIL eval $t"; done
$PY eval_penalized.py --tags orig_th05,g13p,g13p_nover --bop $HOME/Dataset/bop/ycbv --out results_penalized_4090.json > $W/logs/eval_penalized_4090.log 2>&1 || say "FAIL eval_penalized"
mkdir -p $OLD/src/ycbv_live/results_4090; cp results_orig_th05.json results_g13p.json results_g13p_nover.json results_penalized_4090.json $OLD/src/ycbv_live/results_4090/ 2>/dev/null
say "ALL DONE"
