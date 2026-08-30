#!/usr/bin/env bash
# Smoke run: 노드만 띄우고(bag.play:false config) 짧은 bag 구간만 재생해 디버그 로그를 생성한다.
# 코드 변경 아님 — 실행 오케스트레이션. 전체 bag 완주 금지(timeout 으로 짧게 재생).
#   사용: smoke_node_run.sh <config> <bag_path> [play_sec] [drain_sec]
# 주의: set -u 금지 — conda ros-humble activate.d 가 미설정 변수($CONDA_BUILD)를 참조함.
WS="$HOME/temp_ws/CLI_environment/sam6d_ws"
CFG="$1"; BAG="$2"; PLAY_SEC="${3:-25}"; DRAIN="${4:-30}"
cd "$WS" || exit 2
source ~/miniconda3/etc/profile.d/conda.sh
conda activate sam6d_ros_humble
source ~/ros2_ws/install/setup.bash
source install/setup.bash
LOG="/tmp/smoke_$(basename "$CFG" .yaml).log"
: > "$LOG"
echo "[smoke] launch node-only: $CFG"
setsid ros2 launch sam6d_ros sam6d_inference.launch.py config:="$CFG" > "$LOG" 2>&1 &
LPID=$!
ready=0
for i in $(seq 1 360); do
  if grep -q "준비 완료" "$LOG"; then ready=1; break; fi
  if ! kill -0 "$LPID" 2>/dev/null; then echo "[smoke] node exited early"; break; fi
  sleep 1
done
if [ "$ready" != 1 ]; then echo "[smoke] READY FAIL (model load)"; tail -60 "$LOG"; pkill -f sam6d_inference_node 2>/dev/null; exit 1; fi
echo "[smoke] READY after ~${i}s. playing ${PLAY_SEC}s of $BAG"
timeout "$PLAY_SEC" ros2 bag play "$BAG" --rate 1.0 >/dev/null 2>&1
echo "[smoke] bag play stopped; draining ${DRAIN}s for in-flight inference"
sleep "$DRAIN"
echo "[smoke] stopping node"
pkill -INT -f sam6d_inference_node 2>/dev/null; sleep 3
pkill -f sam6d_inference_node 2>/dev/null; pkill -f "ros2 bag play" 2>/dev/null
sleep 2
echo "[smoke] node log tail (last 20):"; tail -20 "$LOG"
echo "[smoke] DONE"
