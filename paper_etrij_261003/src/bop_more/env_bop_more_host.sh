# Environment for src/bop_more on the server .41 (4x RTX PRO 6000 Blackwell; GPU0 is shared: use GPU1, one job at a time).
# usage: export BOP_DATASET=lmo; source env_bop_more_host.sh
source $HOME/Dataset/bop/ycbv_work/env.sh          # OBJPOSE_REPO, SAM6D_ORIG, FASTSAM_X, DINOV2_VITL_DIR, PEM_CKPT (host paths)
export BOP_DATASET=${BOP_DATASET:-lmo}
export BOP_DS_DIR=$HOME/Dataset/bop/$BOP_DATASET
export BOP_WORK=$HOME/Dataset/bop/${BOP_DATASET}_work
export BOP_OUT=$HOME/Dataset/bop/bop_more_out/$BOP_DATASET          # pulled to the 4090 PC: src/bop_more/out_pro6000/<ds>
export PREDEF_POSES=$HOME/Dataset/bop/ycbv_work/predefined_poses
export BOP_TOOLKIT=$HOME/Dataset/bop/ycbv_work/bop_toolkit
export BOP_PYTHON=$HOME/Dataset/bop/ycbv_work/bop_venv/bin/python
export BOP_RUN_LABEL="$BOP_DATASET run, RTX PRO 6000 Blackwell (GPU1), $(date +%Y-%m-%d)"
unset CUDA_VISIBLE_DEVICES YCBV_OUT
export SAM6D_PY=$HOME/anaconda3/envs/sam6d/bin/python
export BPROC=$HOME/Dataset/bop/bproc_venv/bin/blenderproc
export BLENDER_DIR=$HOME/Dataset/bop/ycbv_work/blender
