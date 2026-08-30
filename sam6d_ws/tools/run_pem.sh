#!/usr/bin/env bash
# Run SAM-6D PEM over every (frame,object) bundle built by build_pem_inputs.py.
# Reads outputs/pem_inputs/<bag>/manifest.csv and writes, per bundle,
#   <frame_dir>/pem_<object>/sam6d_results/{detection_pem.json, vis_pem.png}
# Runs in the sam6d_ros_humble env (gorilla + pointnet2 built).
set -u
BAG="${1:-two_table_diagonal1}"
FILTER="${2:-}"            # optional substring filter on "frame:object"
ROOT="/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
PYH="/home/ldh9501/miniconda3/envs/sam6d_ros_humble/bin/python"
PEM="$ROOT/sam6d_master/SAM-6D/Pose_Estimation_Model"
MAN="$ROOT/outputs/pem_inputs/$BAG/manifest.csv"
[ -f "$MAN" ] || { echo "no manifest: $MAN"; exit 1; }
cd "$PEM"
n=0; ok=0
while IFS=, read -r fdir obj cad tdir seg; do
  [ "$fdir" = "frame_dir" ] && continue
  seg="${seg%$'\r'}"; tdir="${tdir%$'\r'}"   # strip CR from CRLF manifest
  tag="$(basename "$fdir"):$obj"
  [ -n "$FILTER" ] && [[ "$tag" != *"$FILTER"* ]] && continue
  n=$((n+1))
  echo "===== [$n] $tag ====="
  if $PYH run_inference_custom.py \
        --output_dir "$fdir/pem_$obj" --cad_path "$cad" \
        --rgb_path "$fdir/rgb.png" --depth_path "$fdir/depth.png" \
        --cam_path "$fdir/camera.json" --seg_path "$seg" \
        --template_dir "$tdir" --config config/base.yaml --det_score_thresh 0.2 \
        > "$fdir/pem_$obj.log" 2>&1; then
    pose=$(grep -o '"score": [0-9.]*' "$fdir/pem_$obj/sam6d_results/detection_pem.json" 2>/dev/null | head -1)
    echo "  OK  $pose"; ok=$((ok+1))
  else
    echo "  FAIL (see $fdir/pem_$obj.log)"; tail -3 "$fdir/pem_$obj.log"
  fi
done < "$MAN"
echo "===== PEM batch done: $ok/$n succeeded ====="
