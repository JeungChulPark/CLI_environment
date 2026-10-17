#!/usr/bin/env bash
# slamctl.sh — 맥(.113)에서 전부 제어한다. 이 파일은 맥에 둔다.
#
#   ./slamctl.sh check                 연결 점검
#   ./slamctl.sh demo [N]              SLAM 없이 전송 경로만 확인
#   ./slamctl.sh start <traj_file>     ORB-SLAM3 출력을 따라가며 전송
#   ./slamctl.sh remote '<cmd>'        WSL2 에서 명령 실행 (sshd 필요)
#   ./slamctl.sh fetch [dir]           .100 의 결과를 가져온다 (sshd 필요)
#   ./slamctl.sh status
DATA_PORT="${DATA_PORT:-5765}"
CTRL_PORT="${CTRL_PORT:-2222}"
WSL_USER="${WSL_USER:-jucpark}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RSSH=(ssh -p "$CTRL_PORT" -o ConnectTimeout=5 "${WSL_USER}@127.0.0.1")

case "${1:-status}" in
  check)  bash "$HERE/../setup/check_link.sh" 2>/dev/null || bash "$HERE/check_link.sh" ;;
  demo)   python3 "$HERE/slam_sender.py" --demo "${2:-300}" --port "$DATA_PORT" ;;
  start)
    [ -n "${2:-}" ] || { echo "usage: $0 start <CameraTrajectory_live.txt>"; exit 2; }
    python3 "$HERE/slam_sender.py" --follow "$2" --port "$DATA_PORT" ;;
  remote)
    [ -n "${2:-}" ] || { echo "usage: $0 remote '<command>'"; exit 2; }
    "${RSSH[@]}" "$2" ;;
  fetch)
    dst="${2:-./slam_results}"; mkdir -p "$dst"
    rsync -av -e "ssh -p $CTRL_PORT" "${WSL_USER}@127.0.0.1:slam_results/" "$dst/" \
      || scp -P "$CTRL_PORT" -r "${WSL_USER}@127.0.0.1:slam_results/*" "$dst/"
    echo "받음 -> $dst" ;;
  status)
    echo "-- 맥 --"
    nc -z 127.0.0.1 "$DATA_PORT" 2>/dev/null && echo "  데이터 터널 OK" || echo "  데이터 터널 없음"
    nc -z 127.0.0.1 "$CTRL_PORT" 2>/dev/null && echo "  제어 터널 OK"   || echo "  제어 터널 없음"
    echo "-- .100 (WSL2) --"
    "${RSSH[@]}" 'bash ~/CLI_environment/slam_net/receiver/link.sh status' 2>/dev/null \
      || echo "  제어 터널이 없어 조회 불가" ;;
  *) echo "usage: $0 {check|demo|start|remote|fetch|status}"; exit 2 ;;
esac
