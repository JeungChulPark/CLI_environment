#!/usr/bin/env bash
# FM-Fusion 을 직접 녹화한 세션에 돌린다. 입력은 make_real_scans.py + detect_gsam_real.py 로 만든 폴더.
#   설정은 ycbm_small.yaml 에서 카메라 내부 파라미터(meta.json), 깊이 단위(mm), 복셀만 바꾼다:
#   복셀 1 cm / 절단 4 cm (물체까지 1.5~3 m 라 깊이 잡음이 탁상 데이터보다 큼), 최소 마스크 100 px, 내보내기 최소 점 100
# 사용: bash run_real.sh OUT [PRED=prediction_gsam] [GAP=5]
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:?out dir}; PRED=${2:-prediction_gsam}; GAP=${3:-5}
DATA=${FMF_DATA:-$HOME/DeepLearning/Datasets/fmfusion_real/scans}
BIN=$HOME/DeepLearning/third_party/FM-Fusion/build/src/IntegrateInstanceMap
mkdir -p "$OUT"
for s in $(ls "$DATA"); do
  read FX FY CX CY < <(python3 -c "import json;K=json.load(open('$DATA/$s/meta.json'))['K'];print(K[0][0],K[1][1],K[0][2],K[1][2])")
  sed -e "s/^camera_fx:.*/camera_fx: $FX/" -e "s/^camera_fy:.*/camera_fy: $FY/" -e "s/^camera_cx:.*/camera_cx: $CX/" -e "s/^camera_cy:.*/camera_cy: $CY/" \
      -e "s/^  voxel_length:.*/  voxel_length: ${VOXEL:-0.01}/" -e "s/^  sdf_trunc:.*/  sdf_trunc: ${TRUNC:-0.04}/" \
      -e "s/^  depth_scale:.*/  depth_scale: 1000.0/" -e "s/^  min_det_masks:.*/  min_det_masks: 100/" "$HERE/ycbm_small.yaml" > "$OUT/$s.yaml"
  t0=$(date +%s.%N)
  env -i HOME=$HOME PATH=/usr/bin:/bin LD_LIBRARY_PATH=$HOME/DeepLearning/third_party/install/lib \
    "$BIN" --config "$OUT/$s.yaml" --root "$DATA/$s" --prediction "$PRED" --frame_gap "$GAP" --output "$OUT" --verbose 2 > "$OUT/$s.log" 2>&1
  t1=$(date +%s.%N)
  echo "$s $(grep -c 'Processing frame' "$OUT/$s.log") frames $(printf '%.1f' $(echo "$t1-$t0" | bc)) s  $(grep -o 'Save [0-9]* semantic instances' "$OUT/$s.log")"
  [ -f "$OUT/$s/instance_info.txt" ] || echo "$s FAILED: $(grep -m1 -o 'Error.*' "$OUT/$s.log" | head -c 200)"
done
