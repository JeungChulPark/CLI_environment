#!/usr/bin/env bash
# check_global_time.sh — V1/V6: 카메라 header.stamp 가 호스트 epoch 도메인인지 확인
#   global_time_enabled=true 면 stamp ≈ 시스템시각 이어야 한다.
#   사용: bash scripts/check_global_time.sh [topic]
set -u
TOPIC="${1:-/cam_A/color/image_raw}"

echo "topic: $TOPIC"
echo "-- header.stamp (sec.nsec) --"
STAMP=$(ros2 topic echo "$TOPIC" --field header.stamp --once 2>/dev/null)
echo "$STAMP"
echo "-- system time (date) --"
date +%s.%N
echo
echo "→ stamp의 sec 값과 date의 앞자리가 근접하면 global_time(host epoch) 동작."
echo "  크게 다르면(예: 50년 차이) sensor/USB 도메인 → global_time 파라미터 재확인."
echo "  V6: 10분 기록 중 주기적으로 실행해 차이가 발산하지 않는지 관찰."
