#!/bin/bash
# autostart_when_deps_ok.sh — run on the PC, detached. Re-tries the SLAM-host build (build_host.sh exits
# fast while apt packages are missing) until it succeeds, then starts the pipeline (run_pc3.sh).
# Log: objpose/output/pc3_autostart.log
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG="$REPO/objpose/output/pc3_autostart.log"; exec >> "$LOG" 2>&1
HOST="${HOST:-slam3}"; MAX_WAIT_S="${MAX_WAIT_S:-21600}"; t=0
echo "$(date '+%F %T') waiting for the apt packages on $HOST, then building (max ${MAX_WAIT_S}s)"
until ssh -o BatchMode=yes -o ConnectTimeout=5 "$HOST" '~/objpose/linux_slam/build_host.sh' </dev/null > "$REPO/objpose/output/pc3_remote_build.log" 2>&1; do
  sleep 60; t=$((t + 60))
  [ $t -ge "$MAX_WAIT_S" ] && { echo "$(date '+%F %T') gave up (see pc3_remote_build.log)"; exit 1; }
done
echo "$(date '+%F %T') build OK on $HOST: $(tail -1 "$REPO/objpose/output/pc3_remote_build.log")"
if ss -tln | grep -qE ':(17001|8765) '; then echo "$(date '+%F %T') ports 17001/8765 busy; not starting"; exit 1; fi
echo "$(date '+%F %T') == starting pipeline: bash objpose/run_pc3.sh (viewer http://192.168.20.5:8765) =="
cd "$REPO" && exec bash objpose/run_pc3.sh
