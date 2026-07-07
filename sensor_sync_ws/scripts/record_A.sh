#!/usr/bin/env bash
# record_A.sh — V3/V4: 단일머신 통합 MCAP 기록 (Jazzy는 mcap이 기본 저장포맷)
#   사용: bash scripts/record_A.sh   (Ctrl-C로 종료)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SESSION="${HERE}/../session/$(hostname)_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$(dirname "$SESSION")"

echo "recording → $SESSION"
ros2 bag record -o "$SESSION" \
  --max-cache-size 1073741824 \
  --qos-profile-overrides-path "${HERE}/../config/qos_override.yaml" \
  /cam_A/color/image_raw \
  /cam_A/aligned_depth_to_color/image_raw \
  /cam_A/color/camera_info \
  /cam_A/imu \
  /velodyne_points
# TODO(실기): 토픽명 실제 발행명으로 확인(ros2 topic list). Jazzy 기본 mcap → -s 생략.
