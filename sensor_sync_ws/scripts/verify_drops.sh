#!/usr/bin/env bash
# verify_drops.sh — V4: 기록된 bag의 메시지 수로 드롭 점검
#   사용: bash scripts/verify_drops.sh <bag_dir>
set -u
BAG="${1:-}"
if [ -z "$BAG" ]; then echo "usage: bash scripts/verify_drops.sh <bag_dir>"; exit 1; fi

echo "==== ros2 bag info ===="
ros2 bag info "$BAG"

cat <<'EOF'

==== 드롭 판정 방법 ====
1) 기록 전 각 토픽의 실제 rate를 미리 측정:  ros2 topic hz <topic>
2) 기대 메시지수 = rate(Hz) × 기록시간(s)
3) 위 'Count' ≈ 기대치(오차 <1%) 면 드롭 없음.
   예) color 30Hz × 300s ≈ 9000, velodyne 10Hz × 300s ≈ 3000
4) 부족하면: 해상도/프레임 하향, 압축(lz4), --max-cache-size 상향,
   D435i를 단독 USB3 컨트롤러로 이동.

(기록 중 디스크 병목은 별도 터미널에서:  iostat -x 5  → %util 확인)
EOF
