#!/usr/bin/env bash
# Render SAM-6D templates for every CAD folder under data/cad.
# Output: sam6d_ws/template/<obj>/templates/{rgb_*,mask_*,xyz_*}
export CONDA_BUILD="${CONDA_BUILD:-}"
source ~/miniconda3/etc/profile.d/conda.sh
conda activate sam6d_ros_humble

WS="/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
CAD_DIR="$WS/data/cad"
OUT_ROOT="$WS/template"
RENDER_PY="$WS/sam6d_master/SAM-6D/Render/render_custom_templates_cpu.py"
LOG_DIR="$OUT_ROOT/_logs"
mkdir -p "$OUT_ROOT" "$LOG_DIR"

# Optional: restrict to a single object by passing its folder name as $1
ONLY="${1:-}"

for d in "$CAD_DIR"/*/; do
    obj="$(basename "$d")"
    [ -n "$ONLY" ] && [ "$obj" != "$ONLY" ] && continue
    ply="$(find "$d" -maxdepth 1 -name '*.ply' | head -1)"
    if [ -z "$ply" ]; then
        echo "[SKIP] $obj : no .ply"
        continue
    fi
    out="$OUT_ROOT/$obj"
    existing=$(ls "$out/templates"/rgb_*.png 2>/dev/null | wc -l)
    if [ "$existing" -ge 42 ]; then
        echo "[DONE] $obj : already has $existing views, skipping"
        continue
    fi
    echo "=================================================="
    echo "[RENDER] $obj"
    echo "  cad = $ply"
    echo "  out = $out/templates"
    echo "  $(date '+%F %T')"
    blenderproc run --custom-blender-path "$HOME/blender/blender-3.3.1-linux-x64" \
        "$RENDER_PY" \
        --cad_path "$ply" \
        --output_dir "$out" \
        > "$LOG_DIR/$obj.log" 2>&1
    rc=$?
    n=$(ls "$out/templates"/rgb_*.png 2>/dev/null | wc -l)
    if [ $rc -eq 0 ] && [ "$n" -gt 0 ]; then
        echo "  [OK] rc=$rc  rgb count=$n"
    else
        echo "  [FAIL] rc=$rc  rgb count=$n  (see $LOG_DIR/$obj.log)"
    fi
done
echo "=================================================="
echo "DONE $(date '+%F %T')"
