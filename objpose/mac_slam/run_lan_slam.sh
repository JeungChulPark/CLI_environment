#!/bin/bash
# run_lan_slam.sh — Mac(192.168.20.55) 쪽 LAN 직결 프로필. ~/objpose/ 에 두고 실행한다 (run_slam.sh 옆).
#
#   PC(192.168.20.5) 에서 `bash objpose/run_lan.sh` (LINK=manual) 로 허브를 먼저 띄운 뒤, Mac 에서:
#       ~/objpose/run_lan_slam.sh                       # 기본: ~/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark/SLAM, 1x(30 fps)
#       ~/objpose/run_lan_slam.sh --session <다른 SLAM 세션>   # run_slam.sh 옵션은 그대로 덧붙는다
#
# ORB-SLAM3(f2000) 가 SLAM 세션을 실시간(1x)으로 재생하며 포즈를 PC 의 17001 포트로 직접 TCP 전송한다.
# 허브의 start 메시지를 받은 뒤 같은 데이터 시각 t0 에서 재생이 시작된다.
set -eu
PC_HOST="${PC_HOST:-192.168.20.5}"
SLAM_PORT="${SLAM_PORT:-17001}"
SESSION="${SLAM_SESSION:-$HOME/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark/SLAM}"
RATE="${RATE:-1.0}"
FEATURES="${FEATURES:-2000}"
exec "$HOME/objpose/run_slam.sh" --slam orbslam3 --session "$SESSION" --features "$FEATURES" \
     --time-source frame --mode live --connect "$PC_HOST:$SLAM_PORT" --rate "$RATE" "$@"
