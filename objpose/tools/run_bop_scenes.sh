#!/usr/bin/env bash
# run_bop_scenes.sh — run every test scene of a BOP dataset through run_bop.sh and score each.
# The recorded demo is a single scene (run_bop.sh); this is the table for the paper.
#   bash objpose/tools/run_bop_scenes.sh                              # YCB-V, all 12 test scenes
#   bash objpose/tools/run_bop_scenes.sh 000048 000052                # only these
#   TABLE=objpose/rt/bop/lmo_objects.json bash objpose/tools/run_bop_scenes.sh
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PY=/home/jucpark/anaconda3/envs/sam6d/bin/python
TABLE="${TABLE:-$REPO/objpose/rt/bop/ycbv_objects.json}"
case "$TABLE" in /*) ;; *) TABLE="$REPO/$TABLE" ;; esac
read -r DATASET SCENES_DIR < <("$PY" -c "
import json,sys; d=json.load(open(sys.argv[1])); print(d['dataset'], d['scenes_dir'])" "$TABLE")
STAMP="$(date +%y%m%d_%H%M%S)"
ROOT="$REPO/objpose/output/${DATASET}_batch_$STAMP"
mkdir -p "$ROOT"

SCENES=("$@")
[ ${#SCENES[@]} -eq 0 ] && SCENES=($(ls "$SCENES_DIR"))

for sc in "${SCENES[@]}"; do
  out="$ROOT/$sc"
  echo "===== scene $sc -> $out  $(date '+%T') ====="
  OUT="$out" bash "$REPO/objpose/run_bop.sh" --scene "$SCENES_DIR/$sc" --table "$TABLE" --no-record
  "$PY" "$REPO/objpose/tools/eval_bop_run.py" --run "$out" --scene "$SCENES_DIR/$sc" \
      --table "$TABLE" --out "$out/eval.json" | tee "$out/eval.txt"
done

echo "===== batch summary ====="
"$PY" - "$ROOT" <<'PY'
import json, sys
from pathlib import Path
root = Path(sys.argv[1])
rows = []
for f in sorted(root.glob("*/eval.json")):
    r = json.loads(f.read_text())
    o = r.get("overall", {})
    rows.append((f.parent.name, o.get("scored", 0), o.get("add_recall"), o.get("adds_recall"),
                 r.get("memory_median_err_mm")))
print(f"{'scene':10s} {'scored':>7s} {'ADD<0.1d':>9s} {'ADD-S<0.1d':>11s} {'memory err':>11s}")
for n, s, a, b, m in rows:
    print(f"{n:10s} {s:7d} {(a or 0)*100:8.1f}% {(b or 0)*100:10.1f}% "
          f"{'—' if m is None else f'{m:8.1f} mm'}")
tot = sum(r[1] for r in rows) or 1
print(f"\nweighted mean: ADD {sum((r[2] or 0)*r[1] for r in rows)/tot*100:.1f}%  "
      f"ADD-S {sum((r[3] or 0)*r[1] for r in rows)/tot*100:.1f}%  over {tot} estimates")
(root / "batch_summary.json").write_text(json.dumps(
    [{"scene": n, "scored": s, "add_recall": a, "adds_recall": b, "memory_median_err_mm": m}
     for n, s, a, b, m in rows], indent=1))
PY
echo "batch -> $ROOT"
