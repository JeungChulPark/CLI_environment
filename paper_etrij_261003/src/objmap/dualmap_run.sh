#!/bin/bash
# Run DualMap (dataset mode, local/concrete mapping, repo defaults) on prepared YCB-V scenes,
# then export each concrete map to out/<method>/ycbv/<scene>.
# Usage: bash dualmap_run.sh 52 [48 49 ...]   (prepare first: python dualmap_prep.py)
# Deviations from the repo defaults: class list = 21 YCB-V prompts + 'unknown', use_rerun=False.
# DUALMAP_METHOD=dualmap_bgr also feeds YOLO-World/MobileSAM/FastSAM BGR images
# (DualMap passes RGB arrays, which ultralytics reads as BGR); default 'dualmap' = as published.
# timeout: an exception in the main thread leaves DualMap's worker threads alive (process hangs).
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
DM=${DUALMAP_REPO:-$HOME/DeepLearning/baselines/DualMap}
DATA=${DUALMAP_DATA:-$HOME/DeepLearning/Dataset/bop/ycbv_work/dualmap}
METHOD=${DUALMAP_METHOD:-dualmap}
RUNS=${DUALMAP_RUNS:-$DM/output/ycbv_runs_$METHOD}
LOCK=${GPU_LOCK:-}
[ "$METHOD" = dualmap_bgr ] && export DUALMAP_ULTRALYTICS_BGR=1
mkdir -p "$RUNS"
source ~/anaconda3/etc/profile.d/conda.sh
conda activate dualmap
cd "$DM"
for s in "$@"; do
  sid=$(printf "ycbv%06d" "$s")
  echo "== $METHOD $sid $(date +%T)"
  ${LOCK:+flock $LOCK} timeout 1500 python -m applications.runner_dataset \
    dataset_name=ycbv scene_id=$sid dataset_path=$DATA/Replica \
    dataset_conf_path=$DATA/configs/$sid.yaml output_path=$RUNS/$sid \
    yolo.given_classes_path=$DATA/ycbv_prompts.txt use_rerun=False \
    > "$RUNS/$sid.log" 2>&1 || { echo "FAILED, see $RUNS/$sid.log"; tail -20 "$RUNS/$sid.log"; continue; }
  python "$HERE/dualmap_export.py" --scene "$s" --method "$METHOD" --map "$RUNS/$sid/ycbv_$sid/map"
done
