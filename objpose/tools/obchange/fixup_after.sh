#!/usr/bin/env bash
# fixup_after.sh — watch_rooms.sh 가 끝난 뒤: DualMap 이 실패했던 방문(지도 폴더 없음)을 다시 돌리고 표·그림을 다시 만든다.
REPO=$(cd "$(dirname "$0")/../../.." && pwd); cd "$REPO"
O=objpose/output/obchange_261005
until grep -q "WATCH_DONE" $O/logs/watch.log; do sleep 120; done
for r in kitchen office livingarea smallroom bigroom; do bash objpose/tools/obchange/run_dualmap.sh $r $O 2>&1 | cut -c1-160; done
bash objpose/tools/obchange/finalize.sh
echo "FIXUP_DONE $(date '+%F %T')"
python3 objpose/docs/ppt/arch_261005/build_results.py "$HOME/다운로드/논문발표자료_0.04v_구성도추가_261005.pptx" --replace; echo "SLIDES_UPDATED $(date "+%F %T")"
