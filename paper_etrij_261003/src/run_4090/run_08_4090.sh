#!/bin/bash
# 2026-10-12: ports 17002/8766 and only our own slam_stream is killed — .41 streams ORB-SLAM3 from this PC on 17001 (ssh -R) at the same time.
export SLAM_PORT=17002 HTTP_PORT=8766   # for src/ycbv_live/run_live.sh (YCB-V part)
# 08_실시간_비교표 와 같은 실험을 RTX 4090(GPU0 단독, 로컬 ORB-SLAM3 f2000, 노트북 코드 CLI_env_paper)에서. 2026-10-07.
#   실제 녹화: 260901_cbnu_bigeightcircle 6구성(real5_*), 260826_etri_eightcircle_dark / 260901_cbnu_eightcircle / 260910_object / 260915_eightcircle 4구성 + 기준 실행
#   YCB-V 12영상 × 6구성 (orig_th05, g13p, origS_th05, hybS05, hybS05_nover, gateA); 기존 유효 장면(49·53·55 orig/g13p)은 재사용
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src; OLD=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/run08_chain.log; O=$R/objpose/output; NASR=/mnt/synology/Dataset/data_slam_converted
say(){ echo "[$(date +%T)] $*" >> $LOG; }
source $W/env.sh; export OBJPOSE_REPO=$R SLAM_RUN=$HOME/objpose/run_slam.sh SAM6D_PY=$PY YCBV_ROOT=$HOME/Dataset/bop/ycbv
REC=$S/realdata/live_cores.py; G13=$R/sam6d_realtime/configs/yolo_ism_objects_g13.yaml
wait_free(){ pkill -f "objpose/pc/hub[.]py"; pkill -f "slam_stream.*127.0.0.1:1700[2]"; pkill -f "sam6d_infe[r]"; for i in $(seq 1 60); do
  pgrep -f "pc/hub[.]py|slam_strea[m]|sam6d_infe[r]" >/dev/null || ss -ltn | grep -qE ":(8766|17002) " || break; sleep 2; done; sleep 3; }
# ---------- 실제 녹화 ----------
xt_for(){ d=${1:0:6}; f=$R/objpose/rt/$d/X_slam_sam_$d.json; [ -f $f ] && echo $f || echo $R/objpose/rt/X_slam_sam.json; }
set_for(){ d=${1:0:6}; f=$R/objpose/mac_slam/settings/orbslam3_${d}_slam_nf2000.yaml; [ -f $f ] && echo "--settings $f" || echo ""; }
real_run(){ ses=$1; name=$2; extra=$3; shift 3   # 나머지: env VAR=val ...
  OUT=$O/$name; [ -f $OUT/summary.json ] && { say "skip $name (exists)"; return 0; }
  for attempt in 1 2; do
    rm -rf $OUT $OUT.log; mkdir -p $OUT; wait_free; say "real $name attempt $attempt"
    env "$@" $PY -u $R/objpose/pc/hub.py --slam orbslam3 --extrinsic $(xt_for $ses) --sam-session $NASR/$ses/SAM --slam-session $NASR/$ses/SLAM \
        --mac-slam-session $NASR/$ses/SLAM --out $OUT --features 2000 --no-mac --slam-port 17002 --http-port 8766 $extra > $OUT.log 2>&1 &
    HUB=$!
    for i in $(seq 1 300); do [ -f $OUT/sam6d/READY ] && break; kill -0 $HUB 2>/dev/null || break; sleep 2; done
    [ -f $OUT/sam6d/READY ] || { say "FAIL $name: sam6d not READY ($(grep -c 'exited rc' $OUT.log) exits)"; kill $HUB 2>/dev/null; continue; }
    $HOME/objpose/run_slam.sh --slam orbslam3 --session $NASR/$ses/SLAM $(set_for $ses) --features 2000 --time-source frame --mode live --connect 127.0.0.1:17002 --rate 1.0 > $OUT/pc_slam.log 2>&1 &
    for i in $(seq 1 500); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
    sleep 3; kill $HUB 2>/dev/null; wait_free
    corr=$(grep -o "max correction so far [0-9.]* cm" $OUT.log | tail -1 | grep -o "[0-9.]*"); n=$(grep -c '^#' $OUT/sam6d_infer.log 2>/dev/null)
    say "done $name: summary=$(grep -c 'summary written' $OUT.log) frames=${n:-0} loop_corr=${corr:-?}cm"
    [ -f $OUT/summary.json ] && [ "${n:-0}" -gt 0 ] && [ "$(echo "${corr:-0}" | awk '{print ($1>100)?1:0}')" = "0" ] && return 0
    mv $OUT ${OUT}_bad$attempt; mv $OUT.log ${OUT}_bad$attempt.log 2>/dev/null
  done; say "GIVEUP $name"; return 1; }
ORIG="OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5"
ORIGS="OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5"
HYBS="OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5"
GATEA="OBJPOSE_LIVE_MODE=gateA"
# (R1) 260901 bigeightcircle, 5+1 구성 한 세션에 (08 의 5절·8절)
ses=260901_cbnu_bigeightcircle
real_run $ses real5_260901_origL05      "--recognizer $REC" $ORIG
real_run $ses real5_260901_g13          "--ism-config $G13" X=1
real_run $ses real5_260901_origS05      "--recognizer $REC" $ORIGS
real_run $ses real5_260901_hybS05       "--recognizer $REC" $HYBS
real_run $ses real5_260901_hybS05_nover "--recognizer $REC --no-verify" $HYBS
real_run $ses real5_260901_gateA        "--recognizer $REC" $GATEA
(cd $S/realdata && $PY eval_real.py --runs real5_260901_origL05 real5_260901_g13 real5_260901_origS05 real5_260901_hybS05 real5_260901_hybS05_nover real5_260901_gateA --out results_real5_4090.json > $W/logs/eval_real5_4090.log 2>&1) || say "FAIL eval real5"
say "R1 DONE"
# (R2) 나머지 녹화 4개: 기준 실행(우리 기술 배포) + 원본 0.5 / 개선안 / 개선안 3 / A안
declare -A REF=( [260915_eightcircle]=live_260915_eightcircle_orbslam3 [260826_etri_eightcircle_dark]=live_260826_etri_dark_orbslam3_f2000 [260901_cbnu_eightcircle]=live_260901_cbnu_eightcircle_orbslam3 [260910_object]=ref_260910_object_ours )
for ses in 260915_eightcircle 260901_cbnu_eightcircle 260910_object 260826_etri_eightcircle_dark; do
  real_run $ses ${REF[$ses]} "" X=1
  real_run $ses real_${ses}_origL05 "--recognizer $REC" $ORIG
  real_run $ses real_${ses}_g13     "--ism-config $G13" X=1
  real_run $ses real_${ses}_hybS05  "--recognizer $REC" $HYBS
  real_run $ses real_${ses}_gateA   "--recognizer $REC" $GATEA
  (cd $S/realdata && EVAL_REAL_SESSION=$ses $PY eval_real.py --runs ${REF[$ses]} real_${ses}_origL05 real_${ses}_g13 real_${ses}_hybS05 real_${ses}_gateA --out results_real_${ses}_4090.json > $W/logs/eval_real_${ses}_4090.log 2>&1) || say "FAIL eval $ses"
  say "R2 $ses DONE"
done
say "REAL DONE"
# ---------- YCB-V 12영상 × 6구성 ----------
ycbv_one(){ tag=$1; sc=$2; d=$O/ycbv_live_${tag}_0000$sc
  [ -f $d/summary.json ] && [ "$(grep -c '^#' $d/sam6d_infer.log 2>/dev/null)" -gt 0 ] && return 0
  for attempt in 1 2; do
    [ -d $d ] && { mv $d ${d}_bad$(date +%H%M%S); mv $d.log ${d}_bad$(date +%H%M%S).log 2>/dev/null; }
    wait_free; say "ycbv $tag $sc attempt $attempt"
    case $tag in
      orig_th05)    (cd $S/ycbv_live && HUB_EXTRA="--recognizer $S/ycbv_live/orig_live_core.py" OBJPOSE_ORIG_THRESH=0.5 OBJPOSE_ORIG_DESC=dinov2_vitl14 bash run_live.sh $tag $sc) ;;
      g13p)         (cd $S/ycbv_live && YCBV_ISM=$W/ycbv_objects_g13p.yaml bash run_live.sh $tag $sc) ;;
      origS_th05)   (cd $S/ycbv_live && HUB_EXTRA="--recognizer $S/ycbv_live/orig_live_core.py" OBJPOSE_ORIG_THRESH=0.5 OBJPOSE_ORIG_DESC=dinov2_vits14 bash run_live.sh $tag $sc) ;;
      hybS05)       (cd $S/ycbv_live && HUB_EXTRA="--recognizer $REC" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5 bash run_live.sh $tag $sc) ;;
      hybS05_nover) (cd $S/ycbv_live && HUB_EXTRA="--recognizer $REC --no-verify" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5 bash run_live.sh $tag $sc) ;;
      gateA)        (cd $S/ycbv_live && HUB_EXTRA="--recognizer $REC" OBJPOSE_LIVE_MODE=gateA bash run_live.sh $tag $sc) ;;
    esac >> $W/logs/ycbv_live_runs.log 2>&1
    n=$(grep -c '^#' $d/sam6d_infer.log 2>/dev/null); [ -f $d/summary.json ] && [ "${n:-0}" -gt 0 ] && { say "ycbv $tag $sc ok (frames $n)"; return 0; }
    say "ycbv $tag $sc invalid (summary=$([ -f $d/summary.json ] && echo y || echo n) frames=${n:-0})"
  done; return 1; }
for tag in orig_th05 g13p origS_th05 hybS05 hybS05_nover gateA; do
  for sc in 48 49 50 51 52 53 54 55 56 57 58 59; do ycbv_one $tag $sc; done; say "ycbv $tag all scenes done"
done
wait_free
cd $S/ycbv_live; say "eval ycbv (장면 GT 있는 것만)"
for t in orig_th05 g13p origS_th05 hybS05 hybS05_nover gateA g13p_nover; do $PY eval_live.py --runs "$O/ycbv_live_${t}_0000??" --out results_$t.json > $W/logs/eval_live_$t.log 2>&1 || say "FAIL eval $t"; done
$PY eval_penalized.py --tags orig_th05,g13p,origS_th05,hybS05,hybS05_nover,gateA,g13p_nover --bop $HOME/Dataset/bop/ycbv --out results_penalized_4090.json > $W/logs/eval_penalized_4090.log 2>&1 || say "FAIL eval_penalized"
mkdir -p $OLD/src/ycbv_live/results_4090 $OLD/src/realdata/results_4090; cp results_*.json $OLD/src/ycbv_live/results_4090/ 2>/dev/null; cp $S/realdata/results_real*_4090.json $OLD/src/realdata/results_4090/ 2>/dev/null
say "RUN08 ALL DONE"
