#!/usr/bin/env bash
# render_high_ply.sh — 압축본 *_high.ply 로 42-view 템플릿을 렌더한다 (격리 출력).
#
# 운영 tools/render_all_templates.sh 와 **완전히 동일한** 렌더러/스크립트/인자를 쓴다.
#   렌더러  : blenderproc + blender-3.3.1 (CPU 강제)
#   스크립트: sam6d_master/SAM-6D/Render/render_custom_templates_cpu.py
#   카메라  : cam_poses_level0.npy (42 view), 조명 POINT energy=1000 scale=2.5
#   샘플    : set_max_amount_of_samples(50)
# 즉 달라지는 변수는 오직 입력 PLY 하나다.
#
# 출력은 sam6d_ws/template/ 이 아니라 이 격리 폴더 아래로만 나간다. 기존 템플릿 무수정.
# set -u 는 쓰지 않는다: ros-humble conda activate 훅이 미정의 변수를 참조해 실패한다
export CONDA_BUILD="${CONDA_BUILD:-}"
source ~/miniconda3/etc/profile.d/conda.sh
conda activate sam6d_ros_humble

WS="/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
SRC="$WS/data/ply_files_excluding_20260702_extracted"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RENDER_PY="$WS/sam6d_master/SAM-6D/Render/render_custom_templates_cpu.py"
LOG_DIR="$HERE/_logs"
mkdir -p "$LOG_DIR"

# 렌더 대상: verify_identity.py 결과상 "운영과 다른 메시" 인 것만.
# Dinosaur/Rabbit/Sauce_high/Sikhye_high 는 운영이 이미 *_high.ply 와 동일(색 100% 일치)이라
# 재렌더해도 같은 이미지가 나오므로 제외한다 — 근거: results/ply_identity.csv
TARGETS="${*:-Bear_high saffron_high Febreze_high Mugcup_high}"

for name in $TARGETS; do
    ply="$SRC/$name.ply"
    out="$HERE/$name"
    if [ ! -f "$ply" ]; then echo "[SKIP] $name : PLY 없음"; continue; fi
    n=$(ls "$out/templates"/rgb_*.png 2>/dev/null | wc -l)
    if [ "$n" -ge 42 ]; then echo "[DONE] $name : 이미 $n view"; continue; fi
    echo "=================================================="
    echo "[RENDER] $name   ($(du -h "$ply" | cut -f1))   $(date '+%F %T')"
    blenderproc run --custom-blender-path "$HOME/blender/blender-3.3.1-linux-x64" \
        "$RENDER_PY" --cad_path "$ply" --output_dir "$out" \
        > "$LOG_DIR/$name.log" 2>&1
    rc=$?
    n=$(ls "$out/templates"/rgb_*.png 2>/dev/null | wc -l)
    [ $rc -eq 0 ] && [ "$n" -ge 42 ] && echo "  [OK] rgb=$n" \
        || echo "  [FAIL] rc=$rc rgb=$n  (로그: $LOG_DIR/$name.log)"
done
echo "=================================================="
echo "DONE $(date '+%F %T')"
