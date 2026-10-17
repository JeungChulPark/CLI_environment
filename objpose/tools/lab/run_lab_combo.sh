#!/usr/bin/env bash
# run_lab_combo.sh — 직접 녹화한 8개 세션에 자세 추정 구성 7개를 실시간 재생으로 돌리고 구성마다 객체 지도를 만든다(ObChange 의 run_combo_obchange.sh 와 같은 구성).
#   결과 objpose/output/lab_combo_261010/<구성>/{<세션>/sam6d_frames.jsonl, maps/<세션>.json}, 진행 기록 log.txt (끝나면 COMBO_DONE)
set -u
cd "$(dirname "$0")/../../.."; REPO=$(pwd); O=$REPO/objpose/output/lab_combo_261010; mkdir -p $O; exec >> $O/log.txt 2>&1
PY=$HOME/anaconda3/envs/sam6d/bin/python
log() { echo "=== $(date '+%F %T') $*"; }
for c in E E_off C C_off D B A; do
  [ -d $O/$c/maps ] && [ $(ls $O/$c/maps | wc -l) -ge 8 ] && { log "$c 이미 끝남"; continue; }
  log "$c 시작"; rm -rf $O/$c; mkdir -p $O/$c
  timeout 4000 $PY objpose/tools/lab/lab_rt.py --config $c --out $O/$c > $O/$c/run.log 2>&1; echo "exit=$? $(grep -a -c '^\[lab_rt\] 26' $O/$c/run.log) 세션"
  BENCH_ROOT=$HOME/DeepLearning/Datasets/lab_sessions_bop BENCH_TABLE=$REPO/objpose/rt/bop/lab8_objects.json BENCH_FPS=10 timeout 3000 $PY objpose/tools/lab/lab_maps.py --run $O/$c --poses $O/slam > $O/$c/maps.log 2>&1
  echo "지도 $(ls $O/$c/maps 2>/dev/null | wc -l)개"; log "$c 끝"
done
log "COMBO_DONE"
