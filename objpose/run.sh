#!/bin/bash
# 객체 위치 추정 시스템 실행 (PC 219.100 에서).
#
#   1) 카메라 RT 가 없으면 먼저: objpose/pc/estimate_rt.py  (Mac ORB-SLAM3 궤적 2개로 추정)
#   2) bash objpose/run.sh [hub.py 옵션...]
#      → Mac(219.113) 에 ssh 로 ORB-SLAM3(--features 2000) 실행 명령을 내리고,
#        SAM-6D 를 띄운 뒤 두 재생을 같은 데이터 시각에서 시작한다.
#      → 브라우저: http://localhost:8765
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
X="$REPO/objpose/rt/X_slam_sam.json"
if [ ! -f "$X" ]; then
  echo "카메라 RT($X)가 없습니다. 먼저 objpose/pc/estimate_rt.py 를 실행하세요." >&2
  exit 1
fi
exec "$PY" -u "$REPO/objpose/pc/hub.py" --extrinsic "$X" "$@"
