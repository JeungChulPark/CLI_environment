# SAM-6D Pipeline Configuration
# Usage (Git Bash / Linux / macOS):
#   source config.sh

export CAD_PATH="D:/LDH_ws/sam_6d/SAM-6D/Data/Example/obj_000005.ply"
export RGB_PATH="D:/LDH_ws/sam_6d/SAM-6D/Data/Example/rgb.png"
export DEPTH_PATH="D:/LDH_ws/sam_6d/SAM-6D/Data/Example/depth.png"
export CAMERA_PATH="D:/LDH_ws/sam_6d/SAM-6D/Data/Example/camera.json"
export OUTPUT_DIR="D:/LDH_ws/sam_6d/SAM-6D/Data/Example/outputs"
export SEGMENTOR_MODEL="sam"
export SEG_PATH="$OUTPUT_DIR/sam6d_results/detection_ism.json"

echo "[config.sh] Variables loaded:"
echo "  CAD_PATH     = $CAD_PATH"
echo "  RGB_PATH     = $RGB_PATH"
echo "  DEPTH_PATH   = $DEPTH_PATH"
echo "  CAMERA_PATH  = $CAMERA_PATH"
echo "  OUTPUT_DIR   = $OUTPUT_DIR"
echo "  SEGMENTOR    = $SEGMENTOR_MODEL"
