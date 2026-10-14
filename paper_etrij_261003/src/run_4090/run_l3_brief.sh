#!/bin/bash
# L3 간략 실험(사용자 요청 16:46): 문구 상자+MobileSAM → ViT-S → 관문+독점 배정 → PEM 기하 1위 (자세 검증 끔)
#   (1) 36장 한 장 평가  (2) 실제 영상 260901 실시간 (배포 설정 + --no-verify)  (3) YCB-V 49·53·55 실시간 (배포 YCB 설정 + --no-verify)
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src; OLD=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/brief_chain.log; O=$R/objpose/output; BAGS=$HOME/DeepLearning/Dataset/bop/ycbv_bags
say(){ echo "[$(date +%T)] $*" >> $LOG; }
until grep -q "RERUN DONE" $LOG; do sleep 30; done
source $W/env.sh; export OBJPOSE_REPO=$R
# (1) 36장
cd $S/ycbv; export YCBV_OUT=$OLD/src/ycbv/out_4090/quick
say "L3 quick 36"; YCBV_SUBSET=25 $PY run_ours.py --mode text --warmup 2 --no-verify --out $YCBV_OUT/pred_ours_text_L3.json > $W/logs/quick_L3.log 2>&1 || say "FAIL quick L3"
YCBV_SUBSET=25 $PY evaluate.py --skip_bop --methods ours_text,ours_text_L3,ours_text_g123p,ours_text_g123p_k50 --results $OLD/src/ycbv/results_4090_quick36.json > $W/logs/evaluate_quick2.log 2>&1 || say "FAIL evaluate quick2"
unset YCBV_OUT
# (2) 실제 영상
NAS=/mnt/synology/Dataset/data_slam_converted/260901_cbnu_bigeightcircle; SET=$R/objpose/mac_slam/settings/orbslam3_260901_slam_nf2000.yaml
for attempt in 1 2; do
  OUT=$O/real_260901_ours_noverify; mkdir -p $OUT; say "live real ours_noverify attempt $attempt"
  pkill -f "objpose/pc/hub[.]py"; pkill -f slam_stream; sleep 3
  $PY -u $R/objpose/pc/hub.py --slam orbslam3 --extrinsic $R/objpose/rt/260901/X_slam_sam_260901.json \
     --sam-session $NAS/SAM --slam-session $NAS/SLAM --mac-slam-session $NAS/SLAM --out $OUT --features 2000 --no-mac --no-verify > $OUT.log 2>&1 &
  HUB=$!
  for i in $(seq 1 300); do [ -f $OUT/sam6d/READY ] && break; kill -0 $HUB 2>/dev/null || break; sleep 2; done
  $HOME/objpose/run_slam.sh --slam orbslam3 --session $NAS/SLAM --settings $SET --features 2000 --time-source frame --mode live --connect 127.0.0.1:17001 --rate 1.0 > $OUT/pc_slam.log 2>&1 &
  for i in $(seq 1 400); do grep -q "summary written" $OUT.log 2>/dev/null && break; sleep 3; done
  sleep 3; kill $HUB 2>/dev/null; pkill -f "objpose/pc/hub[.]py"; pkill -f slam_stream
  corr=$(grep -o "max correction so far [0-9.]* cm" $OUT.log | tail -1 | grep -o "[0-9.]* cm"); say "done ours_noverify attempt $attempt, max loop correction $corr"
  big=$(echo "${corr%% cm}" | awk '{print ($1>60)?1:0}'); [ "$big" = "1" ] && { mv $OUT $O/real_260901_ours_noverify_slamfail$attempt; mv $OUT.log $O/real_260901_ours_noverify_slamfail$attempt.log; continue; }
  break
done
cd $S/realdata; $PY eval_real.py --runs real_260901_g13 real_260901_g13_noverify real_260901_origL05 real_260901_ours_noverify --out results_real_4090.json > $W/logs/eval_real_4090.log 2>&1 || say "FAIL eval real (L3)"
# (3) YCB-V 실시간 3장면
export SLAM_RUN=$HOME/objpose/run_slam.sh SAM6D_PY=$PY YCBV_ROOT=$HOME/Dataset/bop/ycbv
cd $S/ycbv_live; say "ycbv live v2_nover"; HUB_EXTRA="--no-verify" bash run_live.sh v2_nover 49 53 55 >> $W/logs/ycbv_live_runs.log 2>&1
pkill -f "objpose/pc/hub[.]py"; pkill -f slam_stream
$PY eval_live.py --runs "$R/objpose/output/ycbv_live_v2_nover_0000??" --out results_v2_nover.json > $W/logs/eval_live_v2_nover.log 2>&1 || say "FAIL eval v2_nover"
$PY eval_penalized.py --tags orig_th05,g13p,g13p_nover,v2_nover --bop $HOME/Dataset/bop/ycbv --out results_penalized_4090.json > $W/logs/eval_penalized_4090.log 2>&1 || say "FAIL eval_penalized (L3)"
cp results_v2_nover.json results_penalized_4090.json $OLD/src/ycbv_live/results_4090/ 2>/dev/null
say "L3 BRIEF DONE"
