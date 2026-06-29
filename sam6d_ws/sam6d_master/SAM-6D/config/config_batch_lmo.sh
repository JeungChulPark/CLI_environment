# SAM-6D Batch Pipeline - LM-O BOP Dataset 설정 예시
# Usage (Git Bash):
#   source config/config_batch_lmo.sh

# ── 경로 설정 (다운로드 후 수정) ──────────────────────────
LMO_ROOT="D:/LDH_ws/sam_6d/SAM-6D/Data/BOP/lmo"
SCENE_ID="000002"   # 테스트 시퀀스 번호
OBJ_ID="000008"     # 대상 객체 번호

# ── 입력 ──────────────────────────────────────────────────
export RGB_DIR="$LMO_ROOT/test/$SCENE_ID/rgb"
export DEPTH_DIR="$LMO_ROOT/test/$SCENE_ID/depth"
export SCENE_CAMERA_PATH="$LMO_ROOT/test/$SCENE_ID/scene_camera.json"
export CAD_PATH="$LMO_ROOT/models/obj_$OBJ_ID.ply"

# ── 템플릿: render_custom_templates.py 로 미리 생성한 폴더 ──
# 렌더링 명령 예시:
#   conda activate sam_6d
#   cd SAM-6D/Render
#   python render_custom_templates.py \
#       --cad_path "$CAD_PATH" \
#       --output_dir "$OUTPUT_RENDER_DIR"
# → "$OUTPUT_RENDER_DIR/templates/" 에 rgb_*.png / mask_*.png / xyz_*.npy 생성됨
export TEMPLATE_DIR="D:/LDH_ws/sam_6d/SAM-6D/Data/Example/outputs/templates"

# ── 결과 저장 위치 ─────────────────────────────────────────
export OUTPUT_DIR="D:/LDH_ws/sam_6d/SAM-6D/Data/Example/outputs_batch_$SCENE_ID"

export SEGMENTOR_MODEL="sam"

echo "[config_batch_lmo.sh] Variables loaded:"
echo "  RGB_DIR            = $RGB_DIR"
echo "  DEPTH_DIR          = $DEPTH_DIR"
echo "  SCENE_CAMERA_PATH  = $SCENE_CAMERA_PATH"
echo "  CAD_PATH           = $CAD_PATH"
echo "  TEMPLATE_DIR       = $TEMPLATE_DIR"
echo "  OUTPUT_DIR         = $OUTPUT_DIR"
echo ""
echo "실행 명령:"
echo "  conda activate sam_6d"
echo "  cd D:/LDH_ws/sam_6d/SAM-6D"
echo "  python run_batch_inference.py \\"
echo "    --rgb_dir \$RGB_DIR \\"
echo "    --depth_dir \$DEPTH_DIR \\"
echo "    --scene_camera_path \$SCENE_CAMERA_PATH \\"
echo "    --cad_path \$CAD_PATH \\"
echo "    --template_dir \$TEMPLATE_DIR \\"
echo "    --output_dir \$OUTPUT_DIR \\"
echo "    --segmentor_model \$SEGMENTOR_MODEL"
