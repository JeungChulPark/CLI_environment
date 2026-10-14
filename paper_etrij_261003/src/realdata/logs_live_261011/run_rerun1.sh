#!/bin/bash
# 2026-10-11: 실데이터 5개를 23 문서와 같은 설정(Mac ORB-SLAM3 f2000 + 개선안 3 + 군집 먼저)으로 실시간 재실행
set -u
REPO=/home/jucpark/DeepLearning/CLI_environment; PY=$HOME/anaconda3/envs/sam6d/bin/python
RD=$REPO/paper_etrij_261003/src/realdata
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
ssh -o BatchMode=yes mac 'pgrep -q caffeinate || (nohup caffeinate -dimsu -t 14400 >/dev/null 2>&1 &)'
RUNS=""
for ses in 260901_cbnu_bigeightcircle; do
  name=live_261011_${ses}_l3080_hyb3cf_r2; RUNS="$RUNS $name"
  until timeout 8 ssh -o ConnectTimeout=5 -o BatchMode=yes mac true 2>/dev/null; do echo "mac unreachable $(date +%T)"; sleep 60; done
  BASE=$(cd $REPO/objpose/pc && $PY -c "import catalog as c;print(' '.join(c.hub_args('$ses','orbslam3',2000)))" | sed 's/--out [^ ]*//')
  OUT=$REPO/objpose/output/$name; rm -rf $OUT; mkdir -p $OUT
  for p in $(pgrep -f "objpose/pc/hub.py"); do kill $p; done; sleep 3
  echo "start $name $(date +%T)"
  env OBJPOSE_LIVE_MODE=hybrid OBJPOSE_ORIG_DESC=dinov2_vits14 OBJPOSE_ORIG_THRESH=0.5 \
    $PY -u $REPO/objpose/pc/hub.py $BASE --out $OUT --recognizer $RD/live_cores.py --verify-cluster-first > $OUT.log 2>&1 &
  HP=$!
  for i in $(seq 1 500); do grep -q "summary written" $OUT.log 2>/dev/null && break; kill -0 $HP 2>/dev/null || break; sleep 3; done
  sleep 3; for p in $(pgrep -f "objpose/pc/hub.py"); do kill $p; done
  echo "done $name $(date +%T) summary=$(grep -c 'summary written' $OUT.log) $(grep -o 'max correction so far [0-9.]* cm' $OUT.log | tail -1)"
done
echo ALL_DONE
