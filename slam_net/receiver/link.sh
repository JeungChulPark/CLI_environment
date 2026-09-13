#!/usr/bin/env bash
# link.sh — WSL2(.100) 에서 맥(.113) 으로 나가는 SSH 역터널을 세우고 수신기를 띄운다.
#
# 왜 이 방향인가
#   WSL2 는 NAT 뒤에 있어 **밖에서 들어오는** 연결을 받지 못한다(실측: 맥에서
#   192.168.219.100:8899 는 닫혀 보인다). 그러나 **나가는** 연결은 자유롭다
#   (실측: WSL2 -> 192.168.219.113:5900 연결 성공). 그래서 WSL2 가 걸고,
#   `-R` 역터널로 맥이 되돌아 들어오게 한다. Windows 관리자 권한도,
#   netsh portproxy 도, .wslconfig 변경도 필요 없다.
#
# 세우는 터널
#   맥의 localhost:5765  ->  WSL2 의 수신기 5765   (포즈 데이터)
#   맥의 localhost:2222  ->  WSL2 의 sshd 22       (제어 · sshd 가 있을 때만)
#
# 사용
#   ./link.sh up            터널 + 수신기 기동 (foreground, Ctrl-C 로 정지)
#   ./link.sh up --daemon   백그라운드
#   ./link.sh down
#   ./link.sh status
set -o pipefail

MAC_HOST="${MAC_HOST:-192.168.219.113}"
MAC_USER="${MAC_USER:-$(whoami)}"
DATA_PORT="${DATA_PORT:-5765}"
CTRL_PORT="${CTRL_PORT:-2222}"
OUT_DIR="${OUT_DIR:-$HOME/slam_results}"
RUN_DIR="${RUN_DIR:-$HOME/.slam_net}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

mkdir -p "$RUN_DIR" "$OUT_DIR"
TUNNEL_PID="$RUN_DIR/tunnel.pid"
RECV_PID="$RUN_DIR/receiver.pid"
LOG="$RUN_DIR/link.log"

log() { printf '%s %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }

have_sshd() { ss -tln 2>/dev/null | grep -q ':22 '; }

start_receiver() {
  if [ -f "$RECV_PID" ] && kill -0 "$(cat "$RECV_PID")" 2>/dev/null; then
    log "수신기 이미 동작 중 (pid $(cat "$RECV_PID"))"; return 0
  fi
  nohup python3 "$HERE/slam_receiver.py" \
      --host 127.0.0.1 --port "$DATA_PORT" --out "$OUT_DIR" \
      --pidfile "$RECV_PID" >>"$RUN_DIR/receiver.log" 2>&1 &
  sleep 1
  if [ -f "$RECV_PID" ] && kill -0 "$(cat "$RECV_PID")" 2>/dev/null; then
    log "수신기 기동 127.0.0.1:$DATA_PORT -> $OUT_DIR"
  else
    log "수신기 기동 실패 — $RUN_DIR/receiver.log 확인"; return 1
  fi
}

tunnel_args() {
  # -N 명령 실행 안 함 / -T tty 없음
  # ServerAlive*: 회선이 조용히 죽는 것을 감지해 재접속이 돌게 한다
  # ExitOnForwardFailure: 포트 전달이 실패하면 조용히 붙어 있지 않고 죽는다
  echo -n "-N -T \
-o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
-o ExitOnForwardFailure=yes -o StrictHostKeyChecking=accept-new \
-o ConnectTimeout=8 \
-R ${DATA_PORT}:127.0.0.1:${DATA_PORT}"
  if have_sshd; then echo -n " -R ${CTRL_PORT}:127.0.0.1:22"; fi
}

tunnel_loop() {
  local backoff=2
  while :; do
    log "터널 연결 시도 -> ${MAC_USER}@${MAC_HOST}"
    # shellcheck disable=SC2046
    ssh $(tunnel_args) "${MAC_USER}@${MAC_HOST}" >>"$LOG" 2>&1
    local rc=$?
    log "터널 끊김 (rc=$rc). ${backoff}s 후 재시도"
    sleep "$backoff"
    backoff=$(( backoff < 30 ? backoff * 2 : 30 ))
  done
}

case "${1:-up}" in
  up)
    if ! have_sshd; then
      log "주의: WSL2 에 sshd 가 없다 — 데이터 터널만 세운다."
      log "      맥에서 WSL2 를 제어하려면:  bash $HERE/../setup/install_sshd.sh"
    fi
    start_receiver || exit 1
    if [ "${2:-}" = "--daemon" ]; then
      nohup bash "$0" _loop >>"$LOG" 2>&1 &
      echo $! > "$TUNNEL_PID"
      log "터널 데몬 기동 (pid $(cat "$TUNNEL_PID"))"
    else
      trap 'log "정지"; exit 0' INT TERM
      tunnel_loop
    fi
    ;;
  _loop) tunnel_loop ;;
  down)
    for f in "$TUNNEL_PID" "$RECV_PID"; do
      if [ -f "$f" ]; then
        p=$(cat "$f"); pkill -P "$p" 2>/dev/null; kill "$p" 2>/dev/null
        rm -f "$f"; log "정지 pid $p"
      fi
    done
    pkill -f "ssh .*-R ${DATA_PORT}:" 2>/dev/null
    ;;
  status)
    echo "맥        : ${MAC_USER}@${MAC_HOST}"
    echo "데이터포트: ${DATA_PORT}   제어포트: ${CTRL_PORT}"
    echo -n "WSL2 sshd : "; have_sshd && echo "동작 중" || echo "없음 (제어 터널 비활성)"
    for n in tunnel receiver; do
      f="$RUN_DIR/$n.pid"
      if [ -f "$f" ] && kill -0 "$(cat "$f")" 2>/dev/null;
        then echo "$n : 동작 중 (pid $(cat "$f"))"
        else echo "$n : 정지"; fi
    done
    echo "결과      : $OUT_DIR"
    ls -1t "$OUT_DIR" 2>/dev/null | head -3 | sed 's/^/            /'
    ;;
  *) echo "usage: $0 {up [--daemon]|down|status}"; exit 2 ;;
esac
