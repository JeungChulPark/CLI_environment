# paper_etrij_261003/src/ycbv 를 이 PC(RTX 4090 ×4, GPU0 만 사용)에서 돌리기 위한 경로
export OBJPOSE_REPO=$HOME/DeepLearning/server/CLI_environment
export SAM6D_ORIG=$HOME/DeepLearning/server/CLI_environment/sam6d_ws/sam6d_master/SAM-6D
export YCBV_DIR=$HOME/Dataset/bop/ycbv
export YCBV_WORK=$HOME/Dataset/bop/ycbv_work
# (old/ copy deleted 2026-10-08; DINOV2_VITL_DIR set below)
export YCBV_RUN_LABEL="verification run, RTX 4090 (GPU0), 2026-10-06"
export CUDA_VISIBLE_DEVICES=0
export FASTSAM_X=/home/jucpark/Dataset/bop/ycbv_work/weights_dl/FastSAM-x.pt
export DINOV2_VITL_DIR=/home/jucpark/Dataset/bop/ycbv_work/dinov2_ckpt
