#!/bin/bash
# launcherctl.sh — start/stop/restart/status of objpose/pc/launcher.py (viewer front door on 8765).
#   bash objpose/pc/launcherctl.sh status
#   bash objpose/pc/launcherctl.sh start [backend]                 # backend of the default dataset (260910_object)
#   bash objpose/pc/launcherctl.sh start <dataset> <backend>       # e.g. 260826_etri_eightcircle_dark orb
#   bash objpose/pc/launcherctl.sh stop                 # SIGTERM: the launcher stops its hub (summary.json) and exits
#   bash objpose/pc/launcherctl.sh restart [[dataset] backend]
# Match the launcher by its script path only (never a pattern that a caller's own command line could contain).
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
PORT="${PORT:-8765}"; HUB_PORT="${HUB_PORT:-8766}"
LOG="$REPO/objpose/output/launcher.log"
pids() { pgrep -f "pc/launcher[.]py --port $PORT" || true; }
case "${1:-status}" in
  status)
    P=$(pids); [ -n "$P" ] && echo "launcher pid $P" || echo "launcher not running"
    curl -s -m 3 "http://127.0.0.1:$PORT/api/state" | python3 -c 'import sys,json; d=json.load(sys.stdin); print({k:d[k] for k in ("run_id","backend","hub_alive","busy","out")})' 2>/dev/null
    ;;
  stop)
    P=$(pids); [ -n "$P" ] && kill -TERM $P
    for i in $(seq 1 40); do [ -z "$(pids)" ] && break; sleep 1; done
    [ -z "$(pids)" ] && echo "launcher stopped" || { echo "launcher still alive: $(pids)" >&2; exit 1; }
    ;;
  start)
    [ -n "$(pids)" ] && { echo "already running: $(pids)" >&2; exit 1; }
    ARGS=(--port "$PORT" --hub-port "$HUB_PORT")
    # start [backend]  or  start <dataset> <backend>  (autostart is optional)
    [ -n "${3:-}" ] && ARGS+=(--dataset "$2" --backend "$3") || { [ -n "${2:-}" ] && ARGS+=(--backend "$2"); }
    cd "$REPO" && setsid nohup "$PY" -u objpose/pc/launcher.py "${ARGS[@]}" >> "$LOG" 2>&1 < /dev/null &
    sleep 3; echo "launcher pid $(pids)"; tail -2 "$LOG"
    ;;
  restart) "$0" stop && "$0" start "${2:-}" "${3:-}" ;;
  *) echo "usage: $0 status|start [[dataset] backend]|stop|restart [[dataset] backend]" >&2; exit 2 ;;
esac
