#!/bin/bash
# 2026-10-19 자이로 실험(30 문서 구성) — .41 = hub + 개선안 3 + 군집 먼저(hybS05 + --verify-cluster-first) 인식기,
# .192 = 수정 ORB-SLAM3 + 자이로 보정(slam_stream --gyro rig --imu <ses>/imu, 10-19 14:54 빌드).
#   run_live_41solo.sh 의 절차에 SLAM 호스트만 slam192 로, 자이로 인자를 더한 것. 녹화는 IMU 가 있는 260901 cbnu 4개.
#   digut · longcircle 은 .192 에서 rsync 중이므로 RSYNC_DONE 표시를 기다린 뒤 돈다.
#   사용: bash run_gyro_lan41.sh [session ...]      SUF=_r2 bash run_gyro_lan41.sh   (반복)
set -u
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/CLI_environment; S=$R/paper_etrij_261003/src
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/gyro_lan41.log; O=$R/objpose/output
NASR=$HOME/DeepLearning/Dataset/data_slam_converted
SLAM_SSH=slam192; SLAM_NASR=/home/jucpark/local_sessions/data_slam_converted   # .192 로컬 복사본(NAS 는 다른 작업과 겹쳐 SLAM 이 프레임을 절반 놓침, 16:38)
TO192_LOG=/tmp/claude-1001/-home-jucpark-DeepLearning-CLI-environment/b9bcf138-66e5-47a1-a804-9ebb7618481c/scratchpad/to192.log
SLAM_SET=/home/jucpark/objpose/mac_slam/settings          # .192 쪽 설정 · rig 파일
RSYNC_LOG=/mnt/nvme1/Dataset/data_slam_converted/rsync_260901_gyro.log
mkdir -p $W/logs
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
export OBJPOSE_REPO=$R SAM6D_PY=$PY SAM6D_ORIG=$R/sam6d_ws/sam6d_master/SAM-6D
export YCBV_WORK=$W YCBV_DIR=$HOME/DeepLearning/Dataset/bop/ycbv YCBV_ROOT=$HOME/Dataset/bop/ycbv
export FASTSAM_X=$W/weights_dl/FastSAM-x.pt DINOV2_VITL_DIR=$W/dinov2_ckpt
export YCBV_RUN_LABEL="server-client: .41 hub (RTX PRO 6000) + .192 modified ORB-SLAM3 + gyro, 2026-10-19"
export CUDA_VISIBLE_DEVICES=${OBJMAP_GPU:-0} SAM6D_SHM_SUFFIX=_g0   # 20:05: hub↔SAM-6D 공유메모리 이름을 체인별로(다른 hub 와 충돌 방지)
REC=$S/realdata/live_cores.py
HYBS="OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5"
ORIG="OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5"
SESSIONS=${@:-260901_cbnu_eightcircle 260901_cbnu_bigeightcircle 260901_cbnu_digut 260901_cbnu_longcircle}
SUF=${SUF:-}
CONFIG=${CONFIG:-gyro}        # gyro: 수정 ORB + 자이로 | orb: 수정 ORB(자이로 끔) | orig: 원본 0.5 + 수정 ORB (30 문서의 세 구성)

wait_free(){   # 17:55 수정: 이 스크립트가 띄운 hub(와 그 SAM-6D)만 끝낸다. pkill 로 모든 hub 를 죽이던 예전 판은 같은 머신의
  # ObChange 방 체인(run_ours.py 의 hub, 포트 8775/8785)까지 죽여 Office·BigRoom 제안 실행을 망가뜨렸다.
  if [ -n "${HUB:-}" ] && kill -0 $HUB 2>/dev/null; then kill -INT $HUB 2>/dev/null; for i in $(seq 1 20); do kill -0 $HUB 2>/dev/null || break; sleep 1; done; kill -KILL $HUB 2>/dev/null; fi
  for p in $(pgrep -f "sam6d_infe[r].*/$(basename $O)/"); do kill $p 2>/dev/null; done
  ssh -o BatchMode=yes -o ConnectTimeout=10 $SLAM_SSH "pkill -f slam_strea[m]" 2>/dev/null
  for i in $(seq 1 60); do ss -ltn | grep -qE ":(8766|17001) " || break; sleep 2; done; sleep 3; }
wait_data(){ ses=$1
  until [ -f $NASR/$ses/info.json ] && [ -d $NASR/$ses/SAM ] && [ -d $NASR/$ses/SLAM ] && [ -d $NASR/$ses/imu ] && \
        { grep -q "RSYNC_DONE $ses rc=0" $RSYNC_LOG 2>/dev/null || [ $ses = 260901_cbnu_eightcircle ] || [ $ses = 260901_cbnu_bigeightcircle ] || [ $ses = 260901_cbnu_digut ]; } && \
        grep -q "TO192_DONE $ses rc=0" $TO192_LOG 2>/dev/null; do
    say "waiting for $ses data (rsync)"; sleep 60; done; }
xt_for(){ d=${1:0:6}; f=$R/objpose/rt/$d/X_slam_sam_$d.json; [ -f $f ] && echo $f || echo $R/objpose/rt/X_slam_sam.json; }
valid(){ [ -f $1/summary.json ] && [ "$(grep -c '^#' $1/sam6d_infer.log 2>/dev/null)" -gt 0 ]; }

for ses in $SESSIONS; do
  d=${ses:0:6}
  SETARG="--slam-arg=--settings --slam-arg=$SLAM_SET/orbslam3_${d}_slam_nf2000.yaml"
  case $CONFIG in
    gyro) name=live_261019_${ses}_41lan_hyb3cf_orbgyro$SUF; GYRO="$SETARG --slam-arg=--gyro --slam-arg=$SLAM_SET/rig_${d}_gyro.json --slam-arg=--imu --slam-arg=$SLAM_NASR/$ses/imu"; ENVS=$HYBS; RECARG="--recognizer $REC --verify-cluster-first";;
    orb)  name=live_261019_${ses}_41lan_hyb3cf_orb$SUF;     GYRO="$SETARG"; ENVS=$HYBS; RECARG="--recognizer $REC --verify-cluster-first";;
    orig) name=live_261019_${ses}_41lan_origL05_orb$SUF;    GYRO="$SETARG"; ENVS=$ORIG; RECARG="--recognizer $REC";;
    *) echo "bad CONFIG $CONFIG"; exit 2;;
  esac
  OUT=$O/$name
  valid $OUT && [ "$(grep -c '^#' $OUT/sam6d_infer.log)" -gt 300 ] && { say "skip $name (valid)"; continue; }   # 17:58 추가: 재시작 시 끝난 실행은 건너뜀
  wait_data $ses
  ok=0
  for attempt in 1 2; do
    [ -d $OUT ] && { mv $OUT ${OUT}_bad$(date +%H%M%S); mv $OUT.log ${OUT}_bad$(date +%H%M%S).log 2>/dev/null; }
    mkdir -p $OUT; wait_free; say "start $name attempt $attempt (SLAM host $SLAM_SSH, config $CONFIG)"
    env $ENVS $PY -u $R/objpose/pc/hub.py --slam orbslam3 --extrinsic $(xt_for $ses) \
        --sam-session $NASR/$ses/SAM --slam-session $NASR/$ses/SLAM \
        --mac-host $SLAM_SSH --mac-runner /home/jucpark/objpose/run_slam.sh \
        --mac-slam-session $SLAM_NASR/$ses/SLAM $GYRO \
        --http-port 8766 --out $OUT --features 2000 $RECARG > $OUT.log 2>&1 &
    HUB=$!
    for i in $(seq 1 300); do [ -f $OUT/sam6d/READY ] && break; kill -0 $HUB 2>/dev/null || break; sleep 2; done
    [ -f $OUT/sam6d/READY ] || { say "FAIL $name: sam6d not READY"; kill $HUB 2>/dev/null; continue; }
    for i in $(seq 1 160); do grep -q "SLAM hello" $OUT.log 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
    grep -q "SLAM hello" $OUT.log 2>/dev/null || { say "FAIL $name: no SLAM hello in 8min"; tail -5 $OUT/mac_slam.log 2>/dev/null | tee -a $LOG; kill $HUB 2>/dev/null; continue; }
    say "SLAM hello ok; gyro line: $(grep -m1 -oiE 'gyro[^,]*' $OUT/mac_slam.log 2>/dev/null)"
    for i in $(seq 1 600); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
    sleep 3; kill $HUB 2>/dev/null; wait_free
    corr=$(grep -o "max correction so far [0-9.]* cm" $OUT.log | tail -1 | grep -o "[0-9.]*"); n=$(grep -c '^#' $OUT/sam6d_infer.log 2>/dev/null)
    say "done $name: summary=$(grep -c 'summary written' $OUT.log) frames=${n:-0} loop_corr=${corr:-?}cm slam_log_gyro=$(grep -ciE 'gyro' $OUT/mac_slam.log 2>/dev/null)"
    valid $OUT && { ok=1; break; }
  done
  [ $ok = 1 ] || say "GIVEUP $name"
done
say "ALL_DONE $CONFIG$SUF"
