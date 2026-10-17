# Environment for src/bop_more on the RTX 5090 Laptop (jucpark-device), HOPE-Video (doc 27).
# usage: source env_bop_more_laptop5090.sh
YW=$HOME/DeepLearning/Datasets/paper_benchmarks/ycbv_work_paper
source $YW/env.sh                                   # FASTSAM_X, DINOV2_VITL_DIR, PREDEF_POSES (laptop paths)
unset YCBV_OUT YCBV_RUN_LABEL YCBV_DIR YCBV_WORK
export BOP_DATASET=${BOP_DATASET:-hope}
export BOP_DS_DIR=$HOME/DeepLearning/Datasets/paper_benchmarks/HOPE/video_bop    # test_targets_bop19.json from make_hope_targets.py
export BOP_WORK=$HOME/DeepLearning/Datasets/paper_benchmarks/${BOP_DATASET}_work
export BOP_OUT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/out_5090/$BOP_DATASET
export PREDEF_POSES=$YW/predefined_poses
export BOP_RUN_LABEL="$BOP_DATASET-video run, RTX 5090 Laptop, $(date +%Y-%m-%d)"
export BOP_BANK_CACHE=${BOP_BANK_CACHE:-4}      # bounded per-image bank cache (run_orig.select_bank)
export SAM6D_PY=$HOME/anaconda3/envs/sam6d/bin/python
export BPROC=$YW/bproc_venv/bin/blenderproc
export BLENDER_DIR=$YW/blender
