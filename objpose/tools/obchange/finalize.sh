#!/usr/bin/env bash
# finalize.sh — 지금까지 처리된 모든 방으로 채점·그림·표를 다시 만든다.
set -u
REPO=$(cd "$(dirname "$0")/../../.." && pwd); cd "$REPO"
O=objpose/output/obchange_261005; T=objpose/tools/obchange; PY=$HOME/anaconda3/envs/sam6d/bin/python
ROOMS=""; for r in kitchen office livingarea smallroom bigroom; do
  n=$(ls $O/maps/proposed 2>/dev/null | grep -c "^${r}_s"); m=$(ls $O/maps/fmfusion 2>/dev/null | grep -c "^${r}_s"); d=$(ls $O/maps/dualmap 2>/dev/null | grep -c "^${r}_s")
  [ "$n" -ge 5 ] && [ "$m" -ge 5 ] && [ "$d" -ge 5 ] && ROOMS="$ROOMS $r"; done
echo "rooms:$ROOMS"
M="proposed fmfusion dualmap proposed_online"
python3 $T/change_eval.py --maps $O/maps --methods $M --room $ROOMS --out $O/eval/change_nobg.json
python3 $T/change_eval.py --maps $O/maps --methods $M --room $ROOMS --bg --out $O/eval/change_bg.json
python3 $T/map_eval.py --maps $O/maps --methods proposed fmfusion dualmap --room $ROOMS --out $O/eval/map_nobg.json
python3 $T/map_eval.py --maps $O/maps --methods proposed fmfusion dualmap --room $ROOMS --bg --out $O/eval/map_bg.json
for r in $ROOMS; do   # 방마다 대표 방문 쌍 하나(정답 물체가 가장 많은 쌍)를 그린다
  pair=$(python3 - "$r" <<'PYEOF'
import sys, json, itertools
sys.path.insert(0, "objpose/tools/obchange")
from change_eval import OB, CATALOG, load_gt
r = sys.argv[1]; sc = sorted(p.stem for p in (OB / "gt").glob(f"{r}_s*.json") if load_gt(p.stem, CATALOG))
best = max(itertools.combinations(sc, 2), key=lambda ab: len(load_gt(ab[0], CATALOG)) + len(load_gt(ab[1], CATALOG)))
print(best[0], best[1])
PYEOF
)
  [ -f $O/figs/change_${pair/ /__}.png ] || $PY $T/render_change.py --maps $O/maps --pair $pair --out $O/figs 2>&1 | tail -1
done
mkdir -p $HOME/콜로키움/객체변화_실험
python3 $T/make_tables.py $O $HOME/콜로키움/객체변화_실험/비교표_자동생성.md
cp -u $O/figs/change_*.png $HOME/콜로키움/객체변화_실험/ 2>/dev/null
echo "FINALIZE_DONE $(date '+%F %T')"
