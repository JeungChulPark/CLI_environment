#!/usr/bin/env bash
# watch_rooms.sh — 남은 방을 내려받는 대로 처리한다: zip 완료 대기 → run_room.sh → finalize.sh
REPO=$(cd "$(dirname "$0")/../../.." && pwd); cd "$REPO"
O=objpose/output/obchange_261005; L=$HOME/DeepLearning/Datasets/ObChange/download.log
for ROOM in "$@"; do
  room=$(echo $ROOM | tr 'A-Z' 'a-z')
  until grep -q "done $ROOM.zip" $L; do
    pgrep -f "ObChange.*download.sh|download.sh" > /dev/null || { echo "$(date '+%F %T') 다운로드가 멈췄다 — 다시 시작"; (cd $HOME/DeepLearning/Datasets/ObChange && setsid nohup ./download.sh >> download.log 2>&1 < /dev/null &); }
    sleep 120
  done
  while pgrep -f "run_room.sh" > /dev/null; do sleep 30; done
  echo "$(date '+%F %T') start $ROOM"
  bash objpose/tools/obchange/run_room.sh $ROOM > $O/logs/$room.log 2>&1
  bash objpose/tools/obchange/finalize.sh > $O/logs/finalize_$room.log 2>&1
  echo "$(date '+%F %T') done $ROOM"
done
echo "$(date '+%F %T') WATCH_DONE"
