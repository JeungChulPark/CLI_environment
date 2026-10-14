#!/bin/bash
# YCB-V 재실행: 허브가 완전히 내려간 뒤(포트 8765/17001 해제, hub·sam6d_infer·slam_stream 없음) 한 장면씩. 처리 프레임 0 이면 재시도.
# 대상: g13p 53·55 (이전 허브 포트 충돌로 SAM-6D 미실행), g13p_nover 55 (요약 없이 멈춤). 모든 체인 뒤에.
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src; OLD=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/brief_chain.log; O=$R/objpose/output
say(){ echo "[$(date +%T)] $*" >> $LOG; }
until grep -q "L3 REAL DONE" $LOG; do sleep 30; done
source $W/env.sh; export OBJPOSE_REPO=$R SLAM_RUN=$HOME/objpose/run_slam.sh SAM6D_PY=$PY YCBV_ROOT=$HOME/Dataset/bop/ycbv
wait_free(){ pkill -f "objpose/pc/hub[.]py"; pkill -f slam_stream; pkill -f "sam6d_infe[r]"; for i in $(seq 1 60); do
  pgrep -f "pc/hub[.]py|slam_strea[m]|sam6d_infe[r]" >/dev/null || ss -ltn | grep -qE ":(8765|17001) " || break; sleep 2; done; sleep 3; }
one(){ tag=$1; sc=$2; d=$O/ycbv_live_${tag}_0000$sc
  for attempt in 1 2; do
    [ -d $d ] && { mv $d ${d}_bad$(date +%H%M); mv $d.log ${d}_bad$(date +%H%M).log 2>/dev/null; }
    wait_free; say "ycbv $tag $sc attempt $attempt"
    case $tag in
      g13p)       (cd $S/ycbv_live && YCBV_ISM=$W/ycbv_objects_g13p.yaml bash run_live.sh $tag $sc) >> $W/logs/ycbv_live_runs.log 2>&1 ;;
      g13p_nover) (cd $S/ycbv_live && YCBV_ISM=$W/ycbv_objects_g13p.yaml HUB_EXTRA="--no-verify" bash run_live.sh $tag $sc) >> $W/logs/ycbv_live_runs.log 2>&1 ;;
      orig_th05)  (cd $S/ycbv_live && HUB_EXTRA="--recognizer $S/ycbv_live/orig_live_core.py" OBJPOSE_ORIG_THRESH=0.5 OBJPOSE_ORIG_DESC=dinov2_vitl14 bash run_live.sh $tag $sc) >> $W/logs/ycbv_live_runs.log 2>&1 ;;
    esac
    n=$(grep -c '^#' $d/sam6d_infer.log 2>/dev/null); [ -f $d/summary.json ] && [ "${n:-0}" -gt 0 ] && { say "ycbv $tag $sc ok (frames $n)"; return 0; }
    say "ycbv $tag $sc invalid (summary=$([ -f $d/summary.json ] && echo y || echo n) frames=${n:-0})"
  done; return 1; }
one g13p 53; one g13p 55; one g13p_nover 55
wait_free
cd $S/ycbv_live; say "eval ycbv (final)"
for t in orig_th05 g13p g13p_nover; do $PY eval_live.py --runs "$O/ycbv_live_${t}_0000??" --out results_$t.json > $W/logs/eval_live_$t.log 2>&1 || say "FAIL eval $t"; done
$PY eval_penalized.py --tags orig_th05,g13p,g13p_nover --bop $HOME/Dataset/bop/ycbv --out results_penalized_4090.json > $W/logs/eval_penalized_4090.log 2>&1 || say "FAIL eval_penalized"
mkdir -p $OLD/src/ycbv_live/results_4090; cp results_orig_th05.json results_g13p.json results_g13p_nover.json results_penalized_4090.json $OLD/src/ycbv_live/results_4090/ 2>/dev/null
say "FIX55 DONE"
