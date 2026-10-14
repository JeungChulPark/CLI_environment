#!/bin/bash
# .41 단독 실시간 실행 (2026-10-14): hub(.41) + ORB-SLAM3 도 .41(~/objpose, localhost ssh 역터널). .192 불필요.
#   run_real08_lan41.sh 의 real_run 과 같은 절차·설정(개선안 3 + 군집 먼저 = hybS05)이고 SLAM 호스트만 localhost.
#   사용: bash run_live_41solo.sh <session> <run_name> [extra hub args]   (기본: 260901_cbnu_eightcircle, *_hybS05_41solo)
set -u
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/CLI_environment; S=$R/paper_etrij_261003/src
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/live_41solo.log; O=$R/objpose/output
NASR=$HOME/DeepLearning/Dataset/data_slam_converted
SLAM_SSH=localhost; SLAM_NASR=$NASR          # SLAM 호스트 = 이 머신
SET_DIR=$R/objpose/mac_slam/settings
mkdir -p $W/logs
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
export OBJPOSE_REPO=$R SAM6D_PY=$PY SAM6D_ORIG=$R/sam6d_ws/sam6d_master/SAM-6D
export YCBV_WORK=$W YCBV_DIR=$HOME/DeepLearning/Dataset/bop/ycbv YCBV_ROOT=$HOME/Dataset/bop/ycbv
export FASTSAM_X=$W/weights_dl/FastSAM-x.pt DINOV2_VITL_DIR=$W/dinov2_ckpt
export YCBV_RUN_LABEL="single machine, RTX PRO 6000 (.41 hub + .41 ORB-SLAM3), 2026-10-14"
export CUDA_VISIBLE_DEVICES=${OBJMAP_GPU:-0}
REC=$S/realdata/live_cores.py
ses=${1:-260901_cbnu_eightcircle}; name=${2:-real_${ses}_hybS05_41solo}; shift 2 2>/dev/null; extra="$*"
HYBS="OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5"

wait_free(){ pkill -f "objpose/pc/hub[.]py"; pkill -f "sam6d_infe[r]"; pkill -f "slam_strea[m]"
  for i in $(seq 1 60); do pgrep -f "pc/hub[.]py|sam6d_infe[r]|slam_strea[m]" >/dev/null || ss -ltn | grep -qE ":(8766|17001) " || break; sleep 2; done; sleep 3; }
xt_for(){ d=${1:0:6}; f=$R/objpose/rt/$d/X_slam_sam_$d.json; [ -f $f ] && echo $f || echo $R/objpose/rt/X_slam_sam.json; }
set_for(){ d=${1:0:6}; f=$SET_DIR/orbslam3_${d}_slam_nf2000.yaml; [ -f $f ] && echo "--slam-arg=--settings --slam-arg=$f" || echo ""; }
valid(){ [ -f $1/summary.json ] && [ "$(grep -c '^#' $1/sam6d_infer.log 2>/dev/null)" -gt 0 ]; }

OUT=$O/$name
for attempt in 1 2; do
  [ -d $OUT ] && { mv $OUT ${OUT}_bad$(date +%H%M%S); mv $OUT.log ${OUT}_bad$(date +%H%M%S).log 2>/dev/null; }
  mkdir -p $OUT; wait_free; say "real $name attempt $attempt (SLAM host $SLAM_SSH)"
  env $HYBS $PY -u $R/objpose/pc/hub.py --slam orbslam3 --extrinsic $(xt_for $ses) \
      --sam-session $NASR/$ses/SAM --slam-session $NASR/$ses/SLAM \
      --mac-host $SLAM_SSH --mac-runner "$HOME/objpose/run_slam.sh" \
      --mac-slam-session $SLAM_NASR/$ses/SLAM $(set_for $ses) \
      --http-port 8766 --out $OUT --features 2000 --recognizer $REC $extra > $OUT.log 2>&1 &
  HUB=$!
  for i in $(seq 1 300); do [ -f $OUT/sam6d/READY ] && break; kill -0 $HUB 2>/dev/null || break; sleep 2; done
  [ -f $OUT/sam6d/READY ] || { say "FAIL $name: sam6d not READY"; kill $HUB 2>/dev/null; continue; }
  for i in $(seq 1 160); do grep -q "SLAM hello" $OUT.log 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
  grep -q "SLAM hello" $OUT.log 2>/dev/null || { say "FAIL $name: no SLAM hello in 8min"; tail -5 $OUT/mac_slam.log 2>/dev/null | tee -a $LOG; kill $HUB 2>/dev/null; continue; }
  say "SLAM hello ok"
  for i in $(seq 1 500); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
  sleep 3; kill $HUB 2>/dev/null; wait_free
  corr=$(grep -o "max correction so far [0-9.]* cm" $OUT.log | tail -1 | grep -o "[0-9.]*"); n=$(grep -c '^#' $OUT/sam6d_infer.log 2>/dev/null)
  say "done $name: frames=${n:-0} loop_corr=${corr:-?}cm"
  valid $OUT && { say "LIVE 41SOLO OK"; exit 0; }
done; say "GIVEUP $name"; exit 1
