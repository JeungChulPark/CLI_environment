#!/bin/bash
# autostart_when_key_ok.sh — run on the PC (192.168.20.5), detached. Polls until this PC's key is
# accepted by the SLAM host (ssh slam3), then deploys/builds the bundle there (deploy.sh) and starts
# the whole pipeline (run_pc3.sh: ORB-SLAM3 on .3 -> poses over TCP -> SAM-6D + fusion here ->
# http://192.168.20.5:8765). Log: objpose/output/pc3_autostart.log
#   nohup bash objpose/linux_slam/autostart_when_key_ok.sh > /dev/null 2>&1 &
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG="$REPO/objpose/output/pc3_autostart.log"
mkdir -p "$(dirname "$LOG")"; exec >> "$LOG" 2>&1
HOST="${HOST:-slam3}"; MAX_WAIT_S="${MAX_WAIT_S:-21600}"   # 6 h
echo "$(date '+%F %T') waiting for key login to $HOST (max ${MAX_WAIT_S}s)"
t=0
until ssh -o BatchMode=yes -o ConnectTimeout=5 "$HOST" true 2>/dev/null; do
  sleep 30; t=$((t + 30))
  [ $t -ge "$MAX_WAIT_S" ] && { echo "$(date '+%F %T') gave up: key still not accepted by $HOST"; exit 1; }
done
echo "$(date '+%F %T') key OK on $HOST: $(ssh "$HOST" 'hostname; nproc; lscpu | sed -n "s/Model name: *//p"' | paste -sd' ')"
if ss -tln | grep -qE ':(17001|8765) '; then
  echo "$(date '+%F %T') ports 17001/8765 busy on this PC (another hub running); not starting"; exit 1
fi
echo "== deploy + remote build =="
HOST="$HOST" bash "$REPO/objpose/linux_slam/deploy.sh" || { echo "$(date '+%F %T') deploy/build FAILED (see above)"; exit 1; }
echo "$(date '+%F %T') == starting pipeline: bash objpose/run_pc3.sh (viewer http://192.168.20.5:8765) =="
cd "$REPO" && exec bash objpose/run_pc3.sh
