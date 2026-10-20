#!/bin/bash
# 2026-10-20 밤 실험, S 줄 — 29 §2 의 미사용 녹화 10개를 "주력 구성"으로 3회 반복 + 원본 0.5 1회.
#   .41 = hub + 인식기(GPU 0, 공유메모리 접미사 _g0, 포트 8766/17001), .192 = 수정 ORB-SLAM3(10-19 14:54 빌드).
#   주력 구성(10-20 사용자 결정): 자이로(IMU)가 있는 녹화(260901_digut)는 수정 ORB + 자이로(특징점 2000, 30 문서와 같음),
#   IMU 가 없는 나머지 9개는 수정 ORB 특징점 3000(hub --features 3000, slam_stream 이 설정의 nFeatures 를 덮어씀).
#   기준 위치는 10-19 의 ref_<ses>_ours_41lan 을 그대로 씀. run_unused_lan41.sh 의 절차를 따르고, 끝난 실행은 건너뜀.
#   사용: bash run_main_lan41.sh            → _r1 _r2 _r3 (개선안 3) 뒤 원본 0.5 1회
set -u
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/CLI_environment; S=$R/paper_etrij_261003/src
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/overnight_261020.log; O=$R/objpose/output
NASR=$HOME/DeepLearning/Dataset/data_slam_converted
SLAM_SSH=slam192; SLAM_NASR=/home/jucpark/local_sessions/data_slam_converted
SLAM_SET=/home/jucpark/objpose/mac_slam/settings
mkdir -p $W/logs
say(){ echo "[$(date +%T)] S $*" | tee -a $LOG; }
export OBJPOSE_REPO=$R SAM6D_PY=$PY SAM6D_ORIG=$R/sam6d_ws/sam6d_master/SAM-6D
export YCBV_WORK=$W YCBV_DIR=$HOME/DeepLearning/Dataset/bop/ycbv YCBV_ROOT=$HOME/Dataset/bop/ycbv
export FASTSAM_X=$W/weights_dl/FastSAM-x.pt DINOV2_VITL_DIR=$W/dinov2_ckpt
export YCBV_RUN_LABEL="server-client: .41 hub (RTX PRO 6000) + .192 modified ORB-SLAM3 (gyro where IMU, else 3000 features), 2026-10-20"
export CUDA_VISIBLE_DEVICES=${OBJMAP_GPU:-0} SAM6D_SHM_SUFFIX=_g0
REC=$S/realdata/live_cores.py
HYBS="OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5"
ORIG="OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5"
SESSIONS="260901_cbnu_digut 260915_longcircle 260826_etri_longcircle_dark 260915_inlong_fast 260915_inlong_middle 260915_outlong_fast 260915_outlong_middle 260915_height 260915_dense 260915_distance"

wait_free(){   # 이 스크립트가 띄운 hub(와 그 SAM-6D)만 끝낸다. 전역 pkill 금지(31 §4.3).
  if [ -n "${HUB:-}" ] && kill -0 $HUB 2>/dev/null; then kill -INT $HUB 2>/dev/null; for i in $(seq 1 20); do kill -0 $HUB 2>/dev/null || break; sleep 1; done; kill -KILL $HUB 2>/dev/null; fi
  for p in $(pgrep -f "sam6d_infe[r].*/$(basename $O)/"); do kill $p 2>/dev/null; done
  ssh -o BatchMode=yes -o ConnectTimeout=10 $SLAM_SSH "pkill -f slam_strea[m]" 2>/dev/null   # .192 의 slam_stream 은 이 줄만 쓴다
  for i in $(seq 1 60); do ss -ltn | grep -qE ":(8766|17001) " || break; sleep 2; done; sleep 3; }
xt_for(){ d=${1:0:6}; f=$R/objpose/rt/$d/X_slam_sam_$d.json; [ -f $f ] && echo $f || echo $R/objpose/rt/X_slam_sam.json; }
set_for(){ d=${1:0:6}; f=$SLAM_SET/orbslam3_${d}_slam_nf2000.yaml
  ssh -o BatchMode=yes $SLAM_SSH "[ -f $f ]" && echo "--slam-arg=--settings --slam-arg=$f" || echo ""; }
slam_for(){ ses=$1; d=${ses:0:6}   # 주력 구성: IMU 있으면 자이로(2000), 없으면 특징점 3000
  if ssh -o BatchMode=yes $SLAM_SSH "[ -d $SLAM_NASR/$ses/imu ] && [ -f $SLAM_SET/rig_${d}_gyro.json ]"; then
    echo "$(set_for $ses) --slam-arg=--gyro --slam-arg=$SLAM_SET/rig_${d}_gyro.json --slam-arg=--imu --slam-arg=$SLAM_NASR/$ses/imu --features 2000"
  else echo "$(set_for $ses) --features 3000"; fi; }
valid(){ [ -f $1/summary.json ] && [ "$(grep -c '^#' $1/sam6d_infer.log 2>/dev/null)" -gt 300 ]; }

run_one(){ ses=$1; name=$2; envs=$3; recarg=$4; OUT=$O/$name
  valid $OUT && { say "skip $name (valid)"; return 0; }
  [ -f $NASR/$ses/info.json ] && [ -d $NASR/$ses/SAM ] && [ -d $NASR/$ses/SLAM ] || { say "NO DATA $ses"; return 1; }
  SLAMARGS=$(slam_for $ses)
  for attempt in 1 2; do
    [ -d $OUT ] && { mv $OUT ${OUT}_bad$(date +%H%M%S); mv $OUT.log ${OUT}_bad$(date +%H%M%S).log 2>/dev/null; }
    mkdir -p $OUT; wait_free; say "start $name attempt $attempt ($SLAMARGS)"
    env $envs $PY -u $R/objpose/pc/hub.py --slam orbslam3 --extrinsic $(xt_for $ses) \
        --sam-session $NASR/$ses/SAM --slam-session $NASR/$ses/SLAM \
        --mac-host $SLAM_SSH --mac-runner /home/jucpark/objpose/run_slam.sh \
        --mac-slam-session $SLAM_NASR/$ses/SLAM $SLAMARGS \
        --http-port 8766 --out $OUT $recarg > $OUT.log 2>&1 &
    HUB=$!
    for i in $(seq 1 300); do [ -f $OUT/sam6d/READY ] && break; kill -0 $HUB 2>/dev/null || break; sleep 2; done
    [ -f $OUT/sam6d/READY ] || { say "FAIL $name: sam6d not READY"; kill $HUB 2>/dev/null; continue; }
    for i in $(seq 1 160); do grep -q "SLAM hello" $OUT.log 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
    grep -q "SLAM hello" $OUT.log 2>/dev/null || { say "FAIL $name: no SLAM hello in 8min"; tail -3 $OUT/mac_slam.log 2>/dev/null | tee -a $LOG; kill $HUB 2>/dev/null; continue; }
    for i in $(seq 1 600); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
    sleep 3; kill $HUB 2>/dev/null; wait_free
    corr=$(grep -o "max correction so far [0-9.]* cm" $OUT.log | tail -1 | grep -o "[0-9.]*"); n=$(grep -c '^#' $OUT/sam6d_infer.log 2>/dev/null)
    say "done $name: summary=$(grep -c 'summary written' $OUT.log) frames=${n:-0} loop_corr=${corr:-?}cm dropped=$(python3 -c "import json;print(json.load(open('$OUT/summary.json')).get('slam_dropped'))" 2>/dev/null) gyro_lines=$(grep -ciE 'gyro' $OUT/mac_slam.log 2>/dev/null)"
    valid $OUT && return 0
  done; say "GIVEUP $name"; return 1; }

say "LANE S start (GPU $CUDA_VISIBLE_DEVICES, .192 modified ORB; gyro where IMU, else 3000 features)"
for rep in _r1 _r2 _r3; do
  for ses in $SESSIONS; do run_one $ses live_261020_${ses}_41lan_hyb3cf_main$rep "$HYBS" "--recognizer $REC --verify-cluster-first"; done
  say "REP $rep done"
done
for ses in $SESSIONS; do run_one $ses live_261020_${ses}_41lan_origL05_main "$ORIG" "--recognizer $REC"; done
$PY score_main_41lan.py > score_main_41lan.log 2>&1; say "score rc=$? -> $(pwd)/results_main_41lan.json"
say "LANE_S_DONE"
