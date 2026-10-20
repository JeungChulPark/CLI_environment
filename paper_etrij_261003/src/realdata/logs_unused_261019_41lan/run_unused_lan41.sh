#!/bin/bash
# 2026-10-19 밤: 29 문서 §2(미사용 실데이터 녹화) 재현 — .41 = hub + 인식기, .192 = 수정 ORB-SLAM3(자이로 끔, 로컬 세션 복사본).
#   녹화마다 세 실행(노트북 run_unused_l5090.sh 와 같음):
#     ref_<ses>_ours_41lan            배포 인식기(기준 위치)
#     live_261019_<ses>_41lan_origL05 원본 0.5 (ViT-L)
#     live_261019_<ses>_41lan_hyb3cf  개선안 3 + 군집 먼저
#   NAS 에만 있던 260915 7개는 rsync(.41)·cp(.192 로컬) 완료 표시를 기다린 뒤 돈다. 260915_eight_fast · eight_middle 은 NAS 에 없어 제외.
#   사용: bash run_unused_lan41.sh [session ...]
set -u
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/CLI_environment; S=$R/paper_etrij_261003/src
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/unused_lan41.log; O=$R/objpose/output
NASR=$HOME/DeepLearning/Dataset/data_slam_converted
SLAM_SSH=slam192; SLAM_NASR=/home/jucpark/local_sessions/data_slam_converted
SLAM_SET=/home/jucpark/objpose/mac_slam/settings
SP=/tmp/claude-1001/-home-jucpark-DeepLearning-CLI-environment/b9bcf138-66e5-47a1-a804-9ebb7618481c/scratchpad
mkdir -p $W/logs
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
export OBJPOSE_REPO=$R SAM6D_PY=$PY SAM6D_ORIG=$R/sam6d_ws/sam6d_master/SAM-6D
export YCBV_WORK=$W YCBV_DIR=$HOME/DeepLearning/Dataset/bop/ycbv YCBV_ROOT=$HOME/Dataset/bop/ycbv
export FASTSAM_X=$W/weights_dl/FastSAM-x.pt DINOV2_VITL_DIR=$W/dinov2_ckpt
export YCBV_RUN_LABEL="server-client: .41 hub (RTX PRO 6000) + .192 modified ORB-SLAM3, unused recordings, 2026-10-19"
export CUDA_VISIBLE_DEVICES=${OBJMAP_GPU:-0} SAM6D_SHM_SUFFIX=_g0   # 20:05: hub↔SAM-6D 공유메모리 이름을 체인별로(다른 hub 와 충돌 방지)
REC=$S/realdata/live_cores.py
HYBS="OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5"
ORIG="OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5"
SESSIONS=${@:-260901_cbnu_digut 260915_longcircle 260826_etri_longcircle_dark 260915_inlong_fast 260915_inlong_middle 260915_outlong_fast 260915_outlong_middle 260915_height 260915_dense 260915_distance}

wait_free(){   # 17:55 수정: 이 스크립트가 띄운 hub(와 그 SAM-6D)만 끝낸다. pkill 로 모든 hub 를 죽이던 예전 판은 같은 머신의
  # ObChange 방 체인(run_ours.py 의 hub, 포트 8775/8785)까지 죽여 Office·BigRoom 제안 실행을 망가뜨렸다.
  if [ -n "${HUB:-}" ] && kill -0 $HUB 2>/dev/null; then kill -INT $HUB 2>/dev/null; for i in $(seq 1 20); do kill -0 $HUB 2>/dev/null || break; sleep 1; done; kill -KILL $HUB 2>/dev/null; fi
  for p in $(pgrep -f "sam6d_infe[r].*/$(basename $O)/"); do kill $p 2>/dev/null; done
  ssh -o BatchMode=yes -o ConnectTimeout=10 $SLAM_SSH "pkill -f slam_strea[m]" 2>/dev/null
  for i in $(seq 1 60); do ss -ltn | grep -qE ":(8766|17001) " || break; sleep 2; done; sleep 3; }
wait_data(){ ses=$1
  until [ -f $NASR/$ses/info.json ] && [ -d $NASR/$ses/SAM ] && [ -d $NASR/$ses/SLAM ] && \
        { grep -q "RSYNC_DONE $ses rc=0" $SP/rsync_unused.log 2>/dev/null || [ $ses = 260901_cbnu_digut ] || [ $ses = 260915_longcircle ] || [ $ses = 260826_etri_longcircle_dark ]; } && \
        { grep -q "TO192_DONE $ses rc=0" $SP/to192_unused.log 2>/dev/null || grep -q "TO192_DONE $ses rc=0" $SP/to192.log 2>/dev/null; }; do
    say "waiting for $ses data"; sleep 120; done; }
xt_for(){ d=${1:0:6}; f=$R/objpose/rt/$d/X_slam_sam_$d.json; [ -f $f ] && echo $f || echo $R/objpose/rt/X_slam_sam.json; }
set_for(){ d=${1:0:6}; f=$SLAM_SET/orbslam3_${d}_slam_nf2000.yaml
  ssh -o BatchMode=yes $SLAM_SSH "[ -f $f ]" && echo "--slam-arg=--settings --slam-arg=$f" || echo ""; }
valid(){ [ -f $1/summary.json ] && [ "$(grep -c '^#' $1/sam6d_infer.log 2>/dev/null)" -gt 0 ]; }

run_one(){ ses=$1; name=$2; envs=$3; recarg=$4; OUT=$O/$name
  valid $OUT && { say "skip $name (valid)"; return 0; }
  for attempt in 1 2; do
    [ -d $OUT ] && { mv $OUT ${OUT}_bad$(date +%H%M%S); mv $OUT.log ${OUT}_bad$(date +%H%M%S).log 2>/dev/null; }
    mkdir -p $OUT; wait_free; say "start $name attempt $attempt"
    env $envs $PY -u $R/objpose/pc/hub.py --slam orbslam3 --extrinsic $(xt_for $ses) \
        --sam-session $NASR/$ses/SAM --slam-session $NASR/$ses/SLAM \
        --mac-host $SLAM_SSH --mac-runner /home/jucpark/objpose/run_slam.sh \
        --mac-slam-session $SLAM_NASR/$ses/SLAM $(set_for $ses) \
        --http-port 8766 --out $OUT --features 2000 $recarg > $OUT.log 2>&1 &
    HUB=$!
    for i in $(seq 1 300); do [ -f $OUT/sam6d/READY ] && break; kill -0 $HUB 2>/dev/null || break; sleep 2; done
    [ -f $OUT/sam6d/READY ] || { say "FAIL $name: sam6d not READY"; kill $HUB 2>/dev/null; continue; }
    for i in $(seq 1 160); do grep -q "SLAM hello" $OUT.log 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
    grep -q "SLAM hello" $OUT.log 2>/dev/null || { say "FAIL $name: no SLAM hello in 8min"; tail -3 $OUT/mac_slam.log 2>/dev/null | tee -a $LOG; kill $HUB 2>/dev/null; continue; }
    for i in $(seq 1 600); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
    sleep 3; kill $HUB 2>/dev/null; wait_free
    corr=$(grep -o "max correction so far [0-9.]* cm" $OUT.log | tail -1 | grep -o "[0-9.]*"); n=$(grep -c '^#' $OUT/sam6d_infer.log 2>/dev/null)
    say "done $name: summary=$(grep -c 'summary written' $OUT.log) frames=${n:-0} loop_corr=${corr:-?}cm dropped=$(python3 -c "import json;print(json.load(open('$OUT/summary.json')).get('slam_dropped'))" 2>/dev/null)"
    valid $OUT && return 0
  done; say "GIVEUP $name"; return 1; }

for ses in $SESSIONS; do
  wait_data $ses
  run_one $ses ref_${ses}_ours_41lan            "X=1"   ""
  run_one $ses live_261019_${ses}_41lan_origL05 "$ORIG" "--recognizer $REC"
  run_one $ses live_261019_${ses}_41lan_hyb3cf  "$HYBS" "--recognizer $REC --verify-cluster-first"
done
say "UNUSED_ALL_DONE"
