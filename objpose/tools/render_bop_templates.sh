#!/usr/bin/env bash
# render_bop_templates.sh — SAM-6D CAD templates (42 views) for a BOP dataset's objects.
# Uses the repo's CPU renderer (Blender 3.3.1 via blenderproc); ~1 min per object.
#   bash objpose/tools/render_bop_templates.sh                       # YCB-V, all 21
#   TABLE=objpose/rt/bop/lmo_objects.json bash objpose/tools/render_bop_templates.sh
#   bash objpose/tools/render_bop_templates.sh 1 6 14                # only these obj_ids
set -uo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT_ROOT="${OUT_ROOT:-$REPO/sam6d_realtime/template}"
TABLE="${TABLE:-$REPO/objpose/rt/bop/ycbv_objects.json}"
case "$TABLE" in /*) ;; *) TABLE="$REPO/$TABLE" ;; esac
MODELS="${MODELS:-$(python3 -c "import json;print(json.load(open('$TABLE'))['models_dir'])")}"
RENDER_PY="$REPO/sam6d_ws/sam6d_master/SAM-6D/Render/render_custom_templates_cpu.py"
BP=/home/jucpark/anaconda3/envs/sam6d/bin/blenderproc
LOGS="$OUT_ROOT/_logs_$(python3 -c "import json;print(json.load(open('$TABLE'))['dataset'])")"; mkdir -p "$LOGS"

mapfile -t ROWS < <(python3 -c "
import json,sys
sel=set(int(x) for x in sys.argv[1:])
for o in json.load(open('$TABLE'))['objects']:
    if not sel or o['obj_id'] in sel: print(o['obj_id'], o['name'])
" "$@")

for row in "${ROWS[@]}"; do
  set -- $row; oid=$1; name=$2
  out="$OUT_ROOT/$name"
  n=$(ls "$out/templates"/rgb_*.png 2>/dev/null | wc -l)
  if [ "$n" -ge 42 ]; then echo "[DONE] $name ($n views)"; continue; fi
  ply=$(printf '%s/obj_%06d.ply' "$MODELS" "$oid")
  [ -f "$ply" ] || { echo "[SKIP] $name: no $ply"; continue; }
  echo "[RENDER] obj_id=$oid $name  $(date '+%T')"
  "$BP" run --custom-blender-path "$HOME/blender/blender-3.3.1-linux-x64" \
      "$RENDER_PY" --cad_path "$ply" --output_dir "$out" > "$LOGS/$name.log" 2>&1
  rc=$?; n=$(ls "$out/templates"/rgb_*.png 2>/dev/null | wc -l)
  [ $rc -eq 0 ] && [ "$n" -ge 42 ] && echo "  [OK] $n views" || echo "  [FAIL] rc=$rc views=$n (see $LOGS/$name.log)"
done
echo "RENDER_ALL_DONE $(date '+%T')"
