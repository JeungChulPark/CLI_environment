#!/bin/bash
# 2026-10-12: ports 17002/8766 and only our own slam_stream is killed — .41 streams ORB-SLAM3 from this PC on 17001 (ssh -R) at the same time.
# 연휴 실험 A (2026-10-08): 실제 영상 4개 × (기준 2회 + 원본 0.5 3회 + 개선안 3+군집 먼저 3회), RTX 4090 GPU0
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/holiday_real.log; O=$R/objpose/output; NASR=/mnt/synology/Dataset/data_slam_converted
say(){ echo "[$(date +%F' '%T)] $*" | tee -a $LOG; }
source $W/env.sh; export OBJPOSE_REPO=$R SAM6D_ORIG=$R/sam6d_ws/sam6d_master/SAM-6D SLAM_RUN=$HOME/objpose/run_slam.sh SAM6D_PY=$PY YCBV_ROOT=$HOME/Dataset/bop/ycbv YCBV_BAGS=$HOME/DeepLearning/Dataset/bop/ycbv_bags CUDA_VISIBLE_DEVICES=0
REC=$S/realdata/live_cores.py
wait_free(){ pkill -f "objpose/pc/hub[.]py"; pkill -f "slam_stream.*127.0.0.1:1700[2]"; pkill -f "sam6d_infe[r]"; for i in $(seq 1 60); do
  pgrep -f "pc/hub[.]py|slam_strea[m]|sam6d_infe[r]" >/dev/null || ss -ltn | grep -qE ":(8766|17002) " || break; sleep 2; done; sleep 3; }
xt_for(){ d=${1:0:6}; f=$R/objpose/rt/$d/X_slam_sam_$d.json; [ -f $f ] && echo $f || echo $R/objpose/rt/X_slam_sam.json; }
set_for(){ d=${1:0:6}; f=$R/objpose/mac_slam/settings/orbslam3_${d}_slam_nf2000.yaml; [ -f $f ] && echo "--settings $f" || echo ""; }
valid(){ [ -f $1/summary.json ] && [ "$(grep -c '^#' $1/sam6d_infer.log 2>/dev/null)" -gt 0 ]; }
real_run(){ ses=$1; name=$2; extra=$3; shift 3
  OUT=$O/$name; valid $OUT && { say "skip $name (valid)"; return 0; }
  for attempt in 1 2; do
    [ -d $OUT ] && { mv $OUT ${OUT}_bad$(date +%H%M%S); mv $OUT.log ${OUT}_bad$(date +%H%M%S).log 2>/dev/null; }
    mkdir -p $OUT; wait_free; say "real $name attempt $attempt"
    env "$@" $PY -u $R/objpose/pc/hub.py --slam orbslam3 --extrinsic $(xt_for $ses) --sam-session $NASR/$ses/SAM --slam-session $NASR/$ses/SLAM \
        --mac-slam-session $NASR/$ses/SLAM --out $OUT --features 2000 --no-mac --slam-port 17002 --http-port 8766 $extra > $OUT.log 2>&1 &
    HUB=$!
    for i in $(seq 1 300); do [ -f $OUT/sam6d/READY ] && break; kill -0 $HUB 2>/dev/null || break; sleep 2; done
    [ -f $OUT/sam6d/READY ] || { say "FAIL $name: sam6d not READY"; kill $HUB 2>/dev/null; continue; }
    $HOME/objpose/run_slam.sh --slam orbslam3 --session $NASR/$ses/SLAM $(set_for $ses) --features 2000 --time-source frame --mode live --connect 127.0.0.1:17002 --rate 1.0 > $OUT/pc_slam.log 2>&1 &
    for i in $(seq 1 500); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
    sleep 3; kill $HUB 2>/dev/null; wait_free
    corr=$(grep -o "max correction so far [0-9.]* cm" $OUT.log | tail -1 | grep -o "[0-9.]*"); n=$(grep -c '^#' $OUT/sam6d_infer.log 2>/dev/null)
    say "done $name: summary=$(grep -c 'summary written' $OUT.log) frames=${n:-0} loop_corr=${corr:-?}cm"
    valid $OUT && [ "$(echo "${corr:-0}" | awk '{print ($1>100)?1:0}')" = "0" ] && return 0
  done; say "GIVEUP $name"; return 1; }
ORIG="OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5"
HYBS="OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5"
say "HOLIDAY REAL START"
for ses in 260901_cbnu_eightcircle 260915_eightcircle 260910_object 260826_etri_eightcircle_dark; do
  short=${ses}
  # 기준 위치: 배포 인식기 2회 (eval_real.py 가 중앙값을 기준으로 씀)
  real_run $ses ref_${short}_ours_r1 "" X=1
  real_run $ses ref_${short}_ours_r2 "" X=1
  for r in r1 r2 r3; do
    real_run $ses hol_${short}_origL05_$r   "--recognizer $REC" $ORIG
    real_run $ses hol_${short}_hybS05_cf_$r "--recognizer $REC --verify-cluster-first" $HYBS
  done
  (cd $S/realdata && EVAL_REAL_SESSION=$ses EVAL_REAL_REFS=ref_${short}_ours_r1,ref_${short}_ours_r2 $PY eval_real.py \
     --runs ref_${short}_ours_r1 ref_${short}_ours_r2 hol_${short}_origL05_r1 hol_${short}_origL05_r2 hol_${short}_origL05_r3 hol_${short}_hybS05_cf_r1 hol_${short}_hybS05_cf_r2 hol_${short}_hybS05_cf_r3 \
     --out results_holiday_${short}_4090.json > $W/logs/eval_holiday_${short}.log 2>&1) && say "eval $ses ok" || say "FAIL eval $ses"
done
say "HOLIDAY REAL ALL DONE"
