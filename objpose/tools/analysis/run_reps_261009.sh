#!/usr/bin/env bash
# run_reps_261009.sh <반복 번호> — 전체 데이터(YCB-Video 12장면, YCB-M 32장면, ObChange 4개 방의 정답 있는 방문 20회)를 실시간 재생 조건으로 한 번 돌린다.
#   A) 비교 방법 지도 + CAD 이름: FM-Fusion · DualMap 이 만든 지도 객체에 제안 방법의 CAD 템플릿 검증으로 이름을 다시 붙인다(cad_relabel.py, 실시간 재생)
#   B) 제안 방법            : run_compare.py --method proposed --mode realtime (벽시계 재생, 추론이 끝나면 가장 최신 프레임만 처리)
#   C) 제안 방법, 검증 없음  : --method proposed_noverify (후보 검증 문턱과 자세 검증을 모두 끔, 지도 구성은 같음)
# 카메라 자세(ORB-SLAM3 저장본 / ObChange 는 데이터셋 tf)와 FM-Fusion · DualMap 지도는 기존 것을 그대로 쓴다. 벽시계 재생이라 한 번에 하나씩만 돈다.
# 결과 objpose/output/reps_261009/rep<번호>/{ycbv,ycbm,obchange}/…, 진행 기록 rep<번호>/log.txt (끝나면 REP_DONE)
set -u
REP=${1:?}; cd "$(dirname "$0")/../../.."; REPO=$(pwd)
R=$REPO/objpose/output/reps_261009/rep$REP; mkdir -p $R; exec >> $R/log.txt 2>&1
PY=$HOME/anaconda3/envs/sam6d/bin/python; DPY=$HOME/anaconda3/envs/dualmap/bin/python; A=objpose/tools/analysis; T=objpose/tools/obchange
V=$REPO/objpose/output/fmf_cmp_260928; M=$REPO/objpose/output/fmf_cmp_ycbm_260929; OB=$HOME/DeepLearning/Datasets/ObChange
ROOMS="kitchen office livingarea smallroom"
OBS=""; for r in $ROOMS; do for n in 2 3 4 5 6; do OBS="$OBS ${r}_s$n"; done; done
log() { echo "=== $(date '+%F %T') $*"; }
q() { grep -v -i "warn\|pynvml\|xformers" | cut -c1-400; }
rc() { timeout 7200 $PY objpose/tools/baselines/run_compare.py "$@" 2>&1 | grep -E --line-buffered "done|Traceback|Error" | grep --line-buffered -v "PEM 건너뜀" | cut -c1-200; }
ycb_env() {   # $1 = ycbv | ycbm
  if [ $1 = ycbv ]; then unset BENCH_ROOT BENCH_TABLE BENCH_FPS; export BENCH_SLAM=$V/slam
  else export BENCH_ROOT=$HOME/DeepLearning/Datasets/paper_benchmarks/YCB_M/xtion_bop BENCH_TABLE=$REPO/objpose/rt/bop/ycbm_objects.json BENCH_FPS=6 BENCH_SLAM=$M/slam; fi
}

log "A 비교 방법 지도 + CAD 이름"
for d in ycbv ycbm; do
  ycb_env $d; [ $d = ycbv ] && { E=$V/eval_own; G=10; } || { E=$M/eval_own; G=2; }
  mkdir -p $R/$d
  timeout 5400 $PY $A/cad_relabel.py --data ycb --dump $E/dump_final --out $R/$d/dump_cad --gap FM-Fusion=$G DualMap=$G 2>&1 | grep -E "^[0-9]{6} |Traceback|Error" | cut -c1-300
  timeout 3600 $PY $A/inst_iou.py --dump $R/$d/dump_cad --tau 0.02 --out $R/$d/inst_iou_cad.json 2>&1 | q | tail -3
done
unset BENCH_ROOT BENCH_TABLE BENCH_FPS BENCH_SLAM
mkdir -p $R/obchange
timeout 10800 $PY $A/cad_relabel.py --data obchange --out $R/obchange/cad --gap FM-Fusion=3 DualMap=2 --scenes $OBS 2>&1 | grep -E "^[a-z]+_s[0-9] |Traceback|Error" | cut -c1-300
for r in $ROOMS; do
  DUALMAP_MAPS=$R/obchange/cad/dualmap FMF_NAMES=$R/obchange/cad/fmf_names.json timeout 3600 $DPY $T/inst_iou_obchange.py --scenes ${r}_s2 ${r}_s3 ${r}_s4 ${r}_s5 ${r}_s6 --tau 0.02 --out $R/obchange/cad_$r.json 2>&1 | q | tail -3
done

for d in ycbv ycbm; do
  ycb_env $d
  log "B $d 제안 방법";            rc --method proposed          --dataset $d --mode realtime --out $R/$d/proposed
  log "C $d 제안 방법, 검증 없음";  rc --method proposed_noverify --dataset $d --mode realtime --out $R/$d/noverify
  timeout 3600 $PY objpose/tools/baselines/fmfusion/eval_maps_f1.py --ours $R/$d/proposed --orig $R/$d/noverify --out $R/$d/eval --dump $R/$d/dump 2>&1 | q | tail -2
  timeout 3600 $PY $A/inst_iou.py --dump $R/$d/dump --tau 0.02 --out $R/$d/inst_iou.json 2>&1 | q | tail -2
  timeout 1800 $PY $A/pose_metrics.py --dump $R/$d/dump --inst $R/$d/inst_iou.json --out $R/$d/pose_proposed.json 2>&1 | q | tail -1
  timeout 1800 $PY $A/pose_metrics.py --dump $R/$d/dump --inst $R/$d/inst_iou.json --out $R/$d/pose_noverify.json --method "SAM-6D + 같은 지도" 2>&1 | q | tail -1
done
unset BENCH_ROOT BENCH_TABLE BENCH_FPS BENCH_SLAM

log "B obchange 제안 방법";           rc --method proposed          --dataset obchange --mode realtime --all-objects --scenes $OBS --out $R/obchange/proposed
log "C obchange 제안 방법, 검증 없음"; rc --method proposed_noverify --dataset obchange --mode realtime --all-objects --scenes $OBS --out $R/obchange/noverify
for tag in proposed noverify; do
  for r in $ROOMS; do
    BENCH_ROOT=$OB/bop BENCH_TABLE=$REPO/objpose/rt/bop/obchange_objects.json BENCH_FPS=10 timeout 3600 $PY $T/proposed_maps.py --run $R/obchange/$tag --out $R/obchange --tag $tag --room $r --no-online 2>&1 | q | grep -c "_s"
    PROPOSED_MAPS=$R/obchange/maps/$tag timeout 3600 $DPY $T/inst_iou_obchange.py --scenes ${r}_s2 ${r}_s3 ${r}_s4 ${r}_s5 ${r}_s6 --tau 0.02 --out $R/obchange/${tag}_$r.json 2>&1 | q | grep "제안 방법"
  done
done
log "REP_DONE"
