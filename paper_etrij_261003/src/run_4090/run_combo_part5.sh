#!/bin/bash
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src; OLD=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/combo_chain.log; O=$R/objpose/output; NASR=/mnt/synology/Dataset/data_slam_converted
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
until grep -q "PART4 ALL DONE" $LOG; do sleep 30; done
source $W/env.sh; export OBJPOSE_REPO=$R SLAM_RUN=$HOME/objpose/run_slam.sh SAM6D_PY=$PY YCBV_ROOT=$HOME/Dataset/bop/ycbv YCBV_BAGS=$HOME/DeepLearning/Dataset/bop/ycbv_bags CUDA_VISIBLE_DEVICES=0
REC=$S/realdata/live_cores.py
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
ORIG="OBJPOSE_LIVE_MODE=orig OBJPOSE_ORIG_DESC=dinov2_vitl14 OBJPOSE_ORIG_THRESH=0.5"
HYBS="OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5"
say "PART5 START (repeats r2 r3: 원본 0.5, 개선안3 + 군집 먼저)"
ses=260901_cbnu_bigeightcircle
for r in r2 r3; do
  real_run $ses real5_260901_origL05_$r   "--recognizer $REC" $ORIG
  real_run $ses real5_260901_hybS05_cf_$r "--recognizer $REC --verify-cluster-first" $HYBS
done
(cd $S/realdata && $PY eval_real.py --runs real5_260901_origL05 real5_260901_origL05_r2 real5_260901_origL05_r3 real5_260901_hybS05_cf real5_260901_hybS05_cf_r2 real5_260901_hybS05_cf_r3 --out results_real5_repeats_4090.json > $W/logs/eval_real5_repeats.log 2>&1) && say "eval real5 repeats ok" || say "FAIL eval real5 repeats"
cp $S/realdata/results_real5_repeats_4090.json $OLD/src/realdata/results_4090/ 2>/dev/null; say "PART5 REAL DONE"
ycbv_one(){ tag=$1; sc=$2; d=$O/ycbv_live_${tag}_0000$sc
  valid $d && return 0
  for attempt in 1 2; do
    [ -d $d ] && { mv $d ${d}_bad$(date +%H%M%S); mv $d.log ${d}_bad$(date +%H%M%S).log 2>/dev/null; }
    wait_free; say "ycbv $tag $sc attempt $attempt"
    case $tag in
      orig_th05_r*) (cd $S/ycbv_live && HUB_EXTRA="--recognizer $S/ycbv_live/orig_live_core.py" OBJPOSE_ORIG_THRESH=0.5 OBJPOSE_ORIG_DESC=dinov2_vitl14 bash run_live.sh $tag $sc) ;;
      hybS05_cf_r*) (cd $S/ycbv_live && HUB_EXTRA="--recognizer $REC --verify-cluster-first" OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5 bash run_live.sh $tag $sc) ;;
    esac >> $W/logs/ycbv_live_runs.log 2>&1
    valid $d && { say "ycbv $tag $sc ok (frames $(grep -c '^#' $d/sam6d_infer.log))"; return 0; }
    say "ycbv $tag $sc invalid"
  done; return 1; }
TAGS="orig_th05_r2 hybS05_cf_r2 orig_th05_r3 hybS05_cf_r3"
for tag in $TAGS; do for sc in 48 49 50 51 52 53 54 55 56 57 58 59; do ycbv_one $tag $sc; done; say "ycbv $tag all scenes done"; done
wait_free; cd $S/ycbv_live
for t in $TAGS; do $PY eval_live.py --runs "$O/ycbv_live_${t}_0000??" --out results_$t.json > $W/logs/eval_live_$t.log 2>&1 || say "FAIL eval $t"; done
$PY eval_penalized.py --tags orig_th05,hybS05_cf,$(echo $TAGS | tr ' ' ',') --bop $HOME/Dataset/bop/ycbv --out results_penalized_repeats_4090.json > $W/logs/eval_penalized_repeats.log 2>&1 || say "FAIL eval_penalized repeats"
cp results_*.json $OLD/src/ycbv_live/results_4090/ 2>/dev/null; say "PART5 ALL DONE"
