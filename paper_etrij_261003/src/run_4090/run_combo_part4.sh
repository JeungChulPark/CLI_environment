#!/bin/bash
# 4부: 검증 코드 수정(기록 벡터화) 뒤 재측정 + 군집 먼저(cluster_first) 실시간. 1~3부(run_combo_4090.sh) 완료 뒤 실행.
#   _fix = 같은 설정, 수정된 코드(기록 벡터화만)   _cf = + 군집 먼저 + 되돌림
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src; OLD=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/combo_chain.log; O=$R/objpose/output; NASR=/mnt/synology/Dataset/data_slam_converted
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
until grep -q "COMBO ALL DONE" $LOG; do sleep 30; done
source $W/env.sh; export OBJPOSE_REPO=$R SLAM_RUN=$HOME/objpose/run_slam.sh SAM6D_PY=$PY YCBV_ROOT=$HOME/Dataset/bop/ycbv YCBV_BAGS=$HOME/DeepLearning/Dataset/bop/ycbv_bags CUDA_VISIBLE_DEVICES=0
REC=$S/realdata/live_cores.py; G13=$R/sam6d_realtime/configs/yolo_ism_objects_g13.yaml
wait_free(){ pkill -f "objpose/pc/hub[.]py"; pkill -f slam_stream; pkill -f "sam6d_infe[r]"; for i in $(seq 1 60); do
  pgrep -f "pc/hub[.]py|slam_strea[m]|sam6d_infe[r]" >/dev/null || ss -ltn | grep -qE ":(8765|17001) " || break; sleep 2; done; sleep 3; }
xt_for(){ d=${1:0:6}; f=$R/objpose/rt/$d/X_slam_sam_$d.json; [ -f $f ] && echo $f || echo $R/objpose/rt/X_slam_sam.json; }
set_for(){ d=${1:0:6}; f=$R/objpose/mac_slam/settings/orbslam3_${d}_slam_nf2000.yaml; [ -f $f ] && echo "--settings $f" || echo ""; }
valid(){ [ -f $1/summary.json ] && [ "$(grep -c '^#' $1/sam6d_infer.log 2>/dev/null)" -gt 0 ]; }
real_run(){ ses=$1; name=$2; extra=$3; shift 3
  OUT=$O/$name; valid $OUT && { say "skip $name (valid)"; return 0; }
  for attempt in 1 2; do
    [ -d $OUT ] && { mv $OUT ${OUT}_bad$(date +%H%M%S); mv $OUT.log ${OUT}_bad$(date +%H%M%S).log 2>/dev/null; }
    mkdir -p $OUT; wait_free; say "real $name attempt $attempt"
    env "$@" $PY -u $R/objpose/pc/hub.py --slam orbslam3 --extrinsic $(xt_for $ses) --sam-session $NASR/$ses/SAM --slam-session $NASR/$ses/SLAM \
        --mac-slam-session $NASR/$ses/SLAM --out $OUT --features 2000 --no-mac $extra > $OUT.log 2>&1 &
    HUB=$!
    for i in $(seq 1 300); do [ -f $OUT/sam6d/READY ] && break; kill -0 $HUB 2>/dev/null || break; sleep 2; done
    [ -f $OUT/sam6d/READY ] || { say "FAIL $name: sam6d not READY"; kill $HUB 2>/dev/null; continue; }
    $HOME/objpose/run_slam.sh --slam orbslam3 --session $NASR/$ses/SLAM $(set_for $ses) --features 2000 --time-source frame --mode live --connect 127.0.0.1:17001 --rate 1.0 > $OUT/pc_slam.log 2>&1 &
    for i in $(seq 1 500); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HUB 2>/dev/null || break; sleep 3; done
    sleep 3; kill $HUB 2>/dev/null; wait_free
    corr=$(grep -o "max correction so far [0-9.]* cm" $OUT.log | tail -1 | grep -o "[0-9.]*"); n=$(grep -c '^#' $OUT/sam6d_infer.log 2>/dev/null)
    say "done $name: summary=$(grep -c 'summary written' $OUT.log) frames=${n:-0} loop_corr=${corr:-?}cm"
    valid $OUT && [ "$(echo "${corr:-0}" | awk '{print ($1>100)?1:0}')" = "0" ] && return 0
  done; say "GIVEUP $name"; return 1; }
HYBS="OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5"
say "PART4 START (code: rows vectorised + cluster_first)"
ses=260901_cbnu_bigeightcircle
real_run $ses real5_260901_g13_fix    "--ism-config $G13" X=1
real_run $ses real5_260901_g13_cf     "--ism-config $G13 --verify-cluster-first" X=1
real_run $ses real5_260901_hybS05_cf  "--recognizer $REC --verify-cluster-first" $HYBS
real_run $ses real5_260901_ours_cf    "--verify-cluster-first" X=1
(cd $S/realdata && $PY eval_real.py --runs real5_260901_origL05 real5_260901_g13 real5_260901_g13_fix real5_260901_g13_cf real5_260901_hybS05 real5_260901_hybS05_cf real5_260901_ours real5_260901_ours_cf --out results_real5_part4_4090.json > $W/logs/eval_real5_part4.log 2>&1) && say "eval real5 part4 ok" || say "FAIL eval real5 part4"
cp $S/realdata/results_real5_part4_4090.json $OLD/src/realdata/results_4090/ 2>/dev/null
say "PART4 REAL DONE"
ycbv_one(){ tag=$1; sc=$2; d=$O/ycbv_live_${tag}_0000$sc
  valid $d && return 0
  for attempt in 1 2; do
    [ -d $d ] && { mv $d ${d}_bad$(date +%H%M%S); mv $d.log ${d}_bad$(date +%H%M%S).log 2>/dev/null; }
    wait_free; say "ycbv $tag $sc attempt $attempt"
    case $tag in
      g13p_fix) (cd $S/ycbv_live && YCBV_ISM=$W/ycbv_objects_g13p.yaml bash run_live.sh $tag $sc) ;;
      g13p_cf)  (cd $S/ycbv_live && YCBV_ISM=$W/ycbv_objects_g13p.yaml HUB_EXTRA="--verify-cluster-first" bash run_live.sh $tag $sc) ;;
      ours_fix) (cd $S/ycbv_live && bash run_live.sh $tag $sc) ;;
      hybS05_cf) (cd $S/ycbv_live && HUB_EXTRA="--recognizer $REC --verify-cluster-first" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5 bash run_live.sh $tag $sc) ;;
    esac >> $W/logs/ycbv_live_runs.log 2>&1
    valid $d && { say "ycbv $tag $sc ok (frames $(grep -c '^#' $d/sam6d_infer.log))"; return 0; }
    say "ycbv $tag $sc invalid"
  done; return 1; }
TAGS="g13p_cf g13p_fix ours_fix hybS05_cf"
for tag in $TAGS; do for sc in 48 49 50 51 52 53 54 55 56 57 58 59; do ycbv_one $tag $sc; done; say "ycbv $tag all scenes done"; done
wait_free; cd $S/ycbv_live
ALL="orig_th05 g13p ours ours_nover origS_th05 hybS05 $TAGS"
for t in $ALL; do $PY eval_live.py --runs "$O/ycbv_live_${t}_0000??" --out results_$t.json > $W/logs/eval_live_$t.log 2>&1 || say "FAIL eval $t"; done
$PY eval_penalized.py --tags $(echo $ALL | tr ' ' ',') --bop $HOME/Dataset/bop/ycbv --out results_penalized_4090.json > $W/logs/eval_penalized_4090.log 2>&1 || say "FAIL eval_penalized"
mkdir -p $OLD/src/ycbv_live/results_4090; cp results_*.json $OLD/src/ycbv_live/results_4090/ 2>/dev/null
say "PART4 ALL DONE"
