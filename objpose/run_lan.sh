#!/bin/bash
# run_lan.sh — LAN 직결 프로필 (2026-09-14). 기존 run.sh(ssh -R 역터널 + localhost 뷰어)는 그대로 두고
# 따로 만든 실행 설정이다. 두 스크립트는 같은 hub.py 를 쓰고 기본값만 다르다.
#
#   SLAM   192.168.20.55 (Mac)   ORB-SLAM3 f2000 — SLAM 세션을 30 fps(1x)로 재생하고
#                                포즈를 이 PC 의 192.168.20.5:17001 로 TCP 직접 전송
#   SAM-6D 192.168.20.5  (이 PC) SAM 세션(변환본) 재생 + SAM-6D + SLAM 포즈 융합 + 객체 메모리
#   뷰어   http://192.168.20.5:8765   (영상: 객체 박스 + 3축, 3D: SLAM 지도 · 궤적 · 객체 박스)
#
# 입력
#   이 PC : /mnt/synology/Dataset/data_slam_converted/260826_etri_eightcircle_dark/{SAM,SLAM}   (변환본, 표준 토픽)
#   Mac   : ~/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark/SLAM                     (Mac 쪽 사본, 2026-09-14 실측 위치)
#
# 연결 (LINK)
#   auto    (기본) ssh 키 인증이 되면 direct, 안 되면 manual
#   direct  ssh 로 Mac 에 스트리머를 띄우고 --connect 192.168.20.5:17001 (역터널 없음)
#   manual  Mac 쪽은 사람이 직접 띄운다. 허브가 정확한 명령을 출력하고 접속을 기다린다 (ssh 키 불필요)
#   tunnel  기존 방식(ssh -R). run.sh 와 같다
#
# 사용
#   bash objpose/run_lan.sh                       # 브라우저: http://192.168.20.5:8765 , 종료: Ctrl+C
#   LINK=manual bash objpose/run_lan.sh           # 그 뒤 Mac 에서 출력된 명령을 실행
#   bash objpose/run_lan.sh --duration-s 60       # hub.py 옵션은 그대로 뒤에 붙인다
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python

SLAM_HOST="${SLAM_HOST:-192.168.20.55}"
PC_HOST="${PC_HOST:-192.168.20.5}"
MAC_SSH="${MAC_SSH:-mac}"                 # ssh 목적지: ~/.ssh/config 의 별칭 (HostName 192.168.20.55, User user)
SLAM_PORT="${SLAM_PORT:-17001}"
HTTP_PORT="${HTTP_PORT:-8765}"
LINK="${LINK:-auto}"

PC_DATASET="${PC_DATASET:-/mnt/synology/Dataset/data_slam_converted/260826_etri_eightcircle_dark}"
MAC_DATASET="${MAC_DATASET:-~/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark}"   # Mac 셸이 ~ 를 푼다 (따옴표로 보존)
# 변환본 SAM bag 의 스탬프 = 컬러 메타데이터 시각 − 18 226 215 158 ns (info.json). 카메라 RT(rt/X_slam_sam.json)는
# 프레임 시계(−18 225 662 057 ns)에 τ = −48 ms 를 더한 시각으로 추정됐으므로 변환본에는 −48 + 0.553 = −47.4 ms 를 준다.
SAM_TAU_S="${SAM_TAU_S:--0.0474}"
X="${EXTRINSIC:-$REPO/objpose/rt/X_slam_sam.json}"
OUT="${OUT:-$REPO/objpose/output/lan_$(date +%y%m%d_%H%M%S)}"

[ -f "$X" ] || { echo "카메라 RT($X)가 없습니다. 먼저 objpose/pc/estimate_rt_offline.py 를 실행하세요." >&2; exit 1; }
[ -d "$PC_DATASET/SAM" ] || { echo "SAM 세션이 없습니다: $PC_DATASET/SAM" >&2; exit 1; }

if [ "$LINK" = auto ]; then
  if ssh -o BatchMode=yes -o ConnectTimeout=5 "$MAC_SSH" true 2>/dev/null; then
    LINK=direct
  else
    LINK=manual
    echo "[run_lan] ssh $MAC_SSH 키 인증이 안 되어 manual 모드로 갑니다. (자동화하려면: ssh-copy-id $MAC_SSH)" >&2
  fi
fi

echo "[run_lan] SLAM=$SLAM_HOST  SAM-6D/뷰어=$PC_HOST  link=$LINK  out=$OUT" >&2
echo "[run_lan] 뷰어: http://$PC_HOST:$HTTP_PORT" >&2
exec "$PY" -u "$REPO/objpose/pc/hub.py" \
  --link "$LINK" --slam-host "$SLAM_HOST" --pc-host "$PC_HOST" --mac-host "$MAC_SSH" \
  --slam-port "$SLAM_PORT" --http-host 0.0.0.0 --http-port "$HTTP_PORT" \
  --sam-session "$PC_DATASET/SAM" --slam-session "$PC_DATASET/SLAM" \
  --mac-slam-session "$MAC_DATASET/SLAM" \
  --extrinsic "$X" --sam-tau-s "$SAM_TAU_S" --slam orbslam3 --features 2000 --rate 1.0 \
  --out "$OUT" "$@"
