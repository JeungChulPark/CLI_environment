#!/usr/bin/env bash
# Validation Plan V1~V8 runner (research report 2026-07-06 기준).
# 각 런의 총 wall time은 validation_walltimes.txt에 기록된다.
set -u
cd "$(dirname "$0")/.."   # sam6d_ws root

PY=/home/ldh9501/miniconda3/envs/sam_yolo/bin/python
S=_perf_bottleneck_probe_2026_07_06/copied/yolo_ism_object_n.py
OUT=_perf_bottleneck_probe_2026_07_06/outputs
CFG=_perf_bottleneck_probe_2026_07_06/configs
COMMON="--bag data/ros2_bag/two_table_around --max-frames 30 --perf-warmup-frames 3"
WT=_perf_bottleneck_probe_2026_07_06/outputs/validation_walltimes.txt
: > "$WT"

run() {  # run <label> <args...>
  local label=$1; shift
  echo "===== $label ====="
  local t0=$SECONDS
  "$PY" "$S" "$@" || echo "[FAIL] $label" >> "$WT"
  echo "$label wall_s=$((SECONDS-t0))" >> "$WT"
}

# V1: object 1 / 5 / 10 (10 = 원본 config 그대로, read-only)
run v1_obj1     $COMMON --config "$CFG/cfg_obj1.yaml"     --perf-out "$OUT" --perf-run-id v1_obj1
run v1_obj5     $COMMON --config "$CFG/cfg_obj5.yaml"     --perf-out "$OUT" --perf-run-id v1_obj5
run v1_obj10    $COMMON                                    --perf-out "$OUT" --perf-run-id v1_obj10
# V2: top_k 1 (vs v1_obj10의 top_k 3)
run v2_topk1    $COMMON --config "$CFG/cfg_topk1.yaml"    --perf-out "$OUT" --perf-run-id v2_topk1
# V3: imgsz 640 (vs v1_obj10의 960)
run v3_imgsz640 $COMMON --config "$CFG/cfg_imgsz640.yaml" --perf-out "$OUT" --perf-run-id v3_imgsz640
# V5: 이미지 저장 off (vs v1_obj10)
run v5_nosave   $COMMON --no-save-images                   --perf-out "$OUT" --perf-run-id v5_nosave
# V4: template feature cache COLD (probe 내부로 재빌드; frame 수 축소, startup 비교용)
run v4_cold     --bag data/ros2_bag/two_table_around --max-frames 6 --perf-warmup-frames 3 \
                --rebuild-features --perf-out "$OUT" --perf-run-id v4_cold
# V8: 계측 OFF 대조군 (wall time만; v1_obj10과 동일 조건에서 --perf-out 제거)
run v8_perfoff  $COMMON

echo "ALL_DONE"
cat "$WT"
