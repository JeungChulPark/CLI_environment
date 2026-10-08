# Environment for src/bop_more on the 4x RTX 4090 PC (GPU0/1 are busy: use CUDA_VISIBLE_DEVICES=2 or 3).
# usage: export BOP_DATASET=lmo; source env_bop_more.sh
source $HOME/Dataset/bop/ycbv_work/env.sh          # FASTSAM_X, DINOV2_VITL_DIR (vitl + vits symlinks), YCBV_* (unused here)
export OBJPOSE_REPO=$HOME/DeepLearning/CLI_env_paper
export SAM6D_ORIG=$HOME/DeepLearning/CLI_env_paper/sam6d_ws/sam6d_master/SAM-6D
export BOP_DATASET=${BOP_DATASET:-lmo}
export BOP_DS_DIR=$HOME/Dataset/bop/$BOP_DATASET
export BOP_WORK=$HOME/Dataset/bop/${BOP_DATASET}_work
export BOP_OUT=$HOME/DeepLearning/server/CLI_environment/paper_etrij_261003/src/bop_more/out_4090/$BOP_DATASET
export PREDEF_POSES=$HOME/Dataset/bop/ycbv_work/predefined_poses
export BOP_TOOLKIT=$HOME/Dataset/bop/ycbv_work/bop_toolkit
export BOP_PYTHON=$HOME/Dataset/bop/ycbv_work/bop_venv/bin/python
export BOP_RUN_LABEL="$BOP_DATASET run, RTX 4090 (GPU2/3), $(date +%Y-%m-%d)"
unset CUDA_VISIBLE_DEVICES
export SAM6D_PY=$HOME/anaconda3/envs/sam6d/bin/python
export BPROC=$HOME/Dataset/bop/bproc_venv/bin/blenderproc
