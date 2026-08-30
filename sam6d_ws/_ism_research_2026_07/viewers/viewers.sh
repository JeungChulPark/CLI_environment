#!/bin/bash
# viewers.sh — 이 연구에서 만든 URL 뷰어들을 한 번에 관리한다.
#   ./viewers.sh status        살아있는 포트 확인
#   ./viewers.sh start [port…] 지정 포트(생략=전부) 기동
#   ./viewers.sh stop  [port…] 지정 포트(생략=전부) 종료
#   ./viewers.sh list          등록된 뷰어 목록
# 등록 정보는 옆의 registry.csv 하나뿐이다 — 뷰어를 추가하면 거기에 한 줄 넣으면 된다.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RSRCH="$(dirname "$HERE")"; REPO="$(dirname "$RSRCH")"
PY=/home/ldh9501/miniconda3/envs/sam_yolo/bin/python
REG="$HERE/registry.csv"
LOGDIR="$REPO/outputs/viewer_logs"; mkdir -p "$LOGDIR"

rows() { tail -n +2 "$REG"; }
field() { echo "$1" | cut -d, -f"$2"; }

alive() { curl -s --noproxy '*' --max-time 2 "http://127.0.0.1:$1/healthz" 2>/dev/null; }

# 지정한 포트를 쓰는 뷰어 프로세스만 골라 종료한다. pkill -f 는 자기 셸까지
# 잡아먹은 적이 있어 /proc 를 직접 훑는다.
kill_port() {
  local script="$1" port="$2"
  for pid in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
    grep -qa "$script" "/proc/$pid/cmdline" 2>/dev/null || continue
    tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "GT_PORT=$port" || continue
    kill -9 "$pid" 2>/dev/null
  done
}

case "${1:-status}" in
  list)
    printf "%-6s %-18s %s\n" PORT NAME PURPOSE
    rows | while IFS= read -r r; do
      printf "%-6s %-18s %s\n" "$(field "$r" 1)" "$(field "$r" 2)" "$(field "$r" 4)"
    done ;;
  status)
    rows | while IFS= read -r r; do
      p=$(field "$r" 1); h=$(alive "$p")
      printf "%-6s %-18s %s\n" "$p" "$(field "$r" 2)" "${h:-DOWN}"
    done ;;
  start|stop)
    act=$1; shift
    want="$*"
    rows | while IFS= read -r r; do
      p=$(field "$r" 1); nm=$(field "$r" 2); sc=$(field "$r" 3)
      [ -n "$want" ] && ! echo " $want " | grep -q " $p " && continue
      base=$(basename "$sc")
      if [ "$act" = stop ]; then
        kill_port "$base" "$p"; echo "stop  $p $nm"
      else
        [ -n "$(alive "$p")" ] && { echo "skip  $p $nm (이미 실행중)"; continue; }
        [ -f "$RSRCH/$sc" ] || { echo "miss  $p $nm ($sc 없음)"; continue; }
        ( cd "$REPO" && setsid nohup env no_proxy=localhost,127.0.0.1,::1 \
            GT_PORT="$p" PYTHONUNBUFFERED=1 "$PY" "$RSRCH/$sc" \
            > "$LOGDIR/$nm.log" 2>&1 < /dev/null & )
        echo "start $p $nm -> http://127.0.0.1:$p"
      fi
    done ;;
  *) echo "usage: $0 {list|status|start|stop} [port...]"; exit 1 ;;
esac
