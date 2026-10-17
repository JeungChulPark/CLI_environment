#!/usr/bin/env bash
# 정적 장면 영상 데이터셋 실시간 비교: 데이터셋 프레임률로 재생, 최신 프레임만. 기본은 두 방법 모두 정답 상자, GTB= (빈 값)이면 각자 검출.
#   bash objpose/tools/baselines/run_bench_rt.sh hope     (HOPE-Video, 6 fps)
#   bash objpose/tools/baselines/run_bench_rt.sh ycbm     (YCB-M xtion trajectory)
set -u
cd "$(dirname "$0")/../../.."
DS=$1
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
R=objpose/tools/baselines/run_compare.py
O=objpose/output/bench_${DS}_rt_${STAMP:-260925}
mkdir -p $O
run() { echo "=== $(date +%T) $*"; $PY $R ${GTB---gt-boxes} "$@" 2>&1 | grep -E --line-buffered "done:|Traceback|Error"; }
run --method orig     --dataset $DS --mode realtime --out $O/orig_${DS}_realtime
run --method proposed --dataset $DS --mode realtime --out $O/proposed_${DS}_realtime
echo "=== $(date +%T) ALL DONE"
