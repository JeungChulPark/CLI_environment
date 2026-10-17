#!/usr/bin/env bash
# FM-Fusion 을 YCB-V 12영상에 돌린다. 입력은 make_ycbv_scans.py 로 만든 폴더.
#   카메라 자세 = Object Pose Map 과 같은 ORB-SLAM3 결과, 검출 = PRED 폴더(prediction_gt: 인식 정확 가정)
#   frame_gap 10 = FM-Fusion 논문 실험 설정 (10프레임마다 검출 통합)
#   CFG 기본 ycbv_small.yaml (복셀 5 mm, 절단 2 cm, 내보내기 최소 점 100 — 원래 2 cm / 4 cm / 500 은 방 규모용이라 YCB 물체가 모두 걸러짐)
# 사용: bash run_ycbv.sh [OUT] [PRED] [CFG] [GAP]
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../../.." && pwd)
OUT=${1:-$REPO/objpose/output/fmfusion_ycbv_gt}
PRED=${2:-prediction_gt}
CFG=${3:-$HERE/ycbv_small.yaml}
GAP=${4:-10}
DATA=${FMF_DATA:-$HOME/DeepLearning/Datasets/fmfusion_ycbv/scans}
BIN=$HOME/DeepLearning/third_party/FM-Fusion/build/src/IntegrateInstanceMap
mkdir -p "$OUT"
for s in $(ls "$DATA"); do
  t0=$(date +%s.%N)
  env -i HOME=$HOME PATH=/usr/bin:/bin LD_LIBRARY_PATH=$HOME/DeepLearning/third_party/install/lib \
    "$BIN" --config "$CFG" --root "$DATA/$s" --prediction "$PRED" --frame_gap "$GAP" --output "$OUT" --verbose 2 > "$OUT/$s.log" 2>&1
  t1=$(date +%s.%N)
  echo "$s $(grep -c 'Processing frame' "$OUT/$s.log") frames $(printf '%.1f' $(echo "$t1-$t0" | bc)) s  $(grep -o 'Save [0-9]* semantic instances' "$OUT/$s.log")"
  [ -f "$OUT/$s/instance_info.txt" ] || echo "$s FAILED: $(grep -m1 -o 'Error.*' "$OUT/$s.log" | head -c 200)"
done
