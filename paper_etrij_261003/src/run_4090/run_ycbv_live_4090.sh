#!/bin/bash
# YCB-V 실시간 3구성(원본 0.5 / 개선안 검증 켬 / 개선안 검증 끔) × 12 영상 — 복사가 끝난 장면부터 bag 을 만들고, GPU 가 비면 bag 있는 장면부터 차례로.
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src; OLD=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/ycbv_live_chain.log; T=$HOME/DeepLearning/Dataset/bop/ycbv/full/test; BAGS=$HOME/DeepLearning/Dataset/bop/ycbv_bags
SCENES="48 49 50 51 52 53 54 55 56 57 58 59"; TAGS="orig_th05 g13p g13p_nover"
say(){ echo "[$(date +%T)] $*" >> $LOG; }
scene_done(){ d=$T/0000$1; [ -f $d/scene_gt.json ] && [ -f $d/scene_gt_info.json ] && [ -f $d/scene_camera.json ] || return 1
  n=$($PY -c "import json;print(len(json.load(open('$d/scene_camera.json'))))" 2>/dev/null) || return 1
  [ "$(ls $d/rgb 2>/dev/null | wc -l)" = "$n" ] && [ "$(ls $d/depth 2>/dev/null | wc -l)" = "$n" ] || return 1
  [ -z "$(find $d -mmin -2 -type f -print -quit)" ]; }    # 2분간 변화 없음
make_bags(){ for s in $SCENES; do [ -f $BAGS/ycbv_0000$s/SAM/SAM_0.db3 ] && continue; scene_done $s || continue
  say "make_bag $s"; (cd $S/ycbv_live && $PY make_bags.py --scenes $s >> $W/logs/make_bags.log 2>&1) || say "FAIL make_bag $s"; done; }
make_bags
say "wait gpu (missing chain)"; until grep -q "ALL DONE" $W/logs/missing_chain.log; do make_bags; sleep 60; done
source $W/env.sh; export OBJPOSE_REPO=$R SLAM_RUN=$HOME/objpose/run_slam.sh SAM6D_PY=$PY YCBV_ROOT=$HOME/Dataset/bop/ycbv
cd $S/ycbv_live; mkdir -p $R/objpose/output
run_tag(){ tag=$1; s=$2   # env/HUB_EXTRA set by caller
  # 노트북에서 복사해 온 bag 은 SAM 만 있음 → SLAM 쪽은 같은 파일을 하드링크(make_bags.py 와 같은 구조)
  if [ ! -f $BAGS/ycbv_0000$s/SLAM/SAM_0.db3 ]; then mkdir -p $BAGS/ycbv_0000$s/SLAM; for f in $BAGS/ycbv_0000$s/SAM/*; do ln -f "$f" $BAGS/ycbv_0000$s/SLAM/ 2>/dev/null || cp "$f" $BAGS/ycbv_0000$s/SLAM/; done; say "linked SLAM for $s"; fi
  case $tag in
    orig_th05)  HUB_EXTRA="--recognizer $S/ycbv_live/orig_live_core.py" OBJPOSE_ORIG_THRESH=0.5 OBJPOSE_ORIG_DESC=dinov2_vitl14 bash run_live.sh $tag $s ;;
    g13p)       YCBV_ISM=$W/ycbv_objects_g13p.yaml bash run_live.sh $tag $s ;;
    g13p_nover) YCBV_ISM=$W/ycbv_objects_g13p.yaml HUB_EXTRA="--no-verify" bash run_live.sh $tag $s ;;
  esac >> $W/logs/ycbv_live_runs.log 2>&1
  L=$R/objpose/output/ycbv_live_${tag}_0000$s
  grep -q "summary written" $L.log 2>/dev/null || { say "FAIL $tag $s: no summary"; return 1; }
  pem=$(grep -c 'PEM 건너뜀' $L/sam6d_infer.log 2>/dev/null); say "done $tag $s (PEM 건너뜀 $pem)"; }
declare -A FAILED
while true; do
  make_bags; left=0; ran=0
  for tag in $TAGS; do for s in $SCENES; do
    [ -f $R/objpose/output/ycbv_live_${tag}_0000$s/summary.json ] && continue
    [ -n "${FAILED[$tag:$s]:-}" ] && continue
    if [ -f $BAGS/ycbv_0000$s/SAM/SAM_0.db3 ] && [ -f $BAGS/ycbv_0000$s/SAM/metadata.yaml ]; then run_tag $tag $s || FAILED[$tag:$s]=1; ran=1; else left=$((left+1)); fi
  done; done
  [ $left -eq 0 ] && break
  [ $ran -eq 0 ] && { say "waiting for copy: $left runs need bags"; sleep 120; }
done
pkill -f "objpose/pc/hub.py"; pkill -f slam_stream
say "eval"; for t in $TAGS; do $PY eval_live.py --runs "$R/objpose/output/ycbv_live_${t}_0000??" --out results_$t.json > $W/logs/eval_live_$t.log 2>&1 || say "FAIL eval $t"; done
$PY eval_penalized.py --tags orig_th05,g13p,g13p_nover --bop $HOME/Dataset/bop/ycbv --out results_penalized_4090.json > $W/logs/eval_penalized_4090.log 2>&1 || say "FAIL eval_penalized"
mkdir -p $OLD/src/ycbv_live/results_4090; cp results_orig_th05.json results_g13p.json results_g13p_nover.json results_penalized_4090.json $OLD/src/ycbv_live/results_4090/ 2>/dev/null
say "ALL DONE"
