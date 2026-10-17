#!/usr/bin/env bash
# run_dualmap.sh <room> <out> — ObChange 한 방의 방문들에 DualMap 을 돌려 <out>/maps/dualmap/<scene>.json 을 만든다.
#   물체 이름 = 제안 방법 · FM-Fusion 과 같은 11종 문구 + 마지막 줄 "unknown" (DualMap 의 목록 규칙).
#   2026-10-05 이전 결과(maps/dualmap_oldlist)는 목록 끝에 unknown 이 없어 잘못된 설정이었다.
#   입력 5 fps (10 fps 의 2장마다 1장), 카메라 자세 = 데이터셋 tf (scene_camera.json).
set -u
ROOM=${1:?}; OUT=$(realpath ${2:?}); REPO=$(cd "$(dirname "$0")/../../.." && pwd)
BOP=$HOME/DeepLearning/Datasets/ObChange/bop/test
$HOME/anaconda3/envs/dualmap/bin/python $REPO/objpose/tools/baselines/dualmap/run_dualmap_bop.py --bop-test $BOP --table $REPO/objpose/rt/bop/obchange_objects.json \
  --poses gt --fps 10 --stride 2 --out $OUT --tag dualmap --scenes $(ls $BOP | grep "^${ROOM}_s" | sort -V) 2>&1 | grep -v -i warn
