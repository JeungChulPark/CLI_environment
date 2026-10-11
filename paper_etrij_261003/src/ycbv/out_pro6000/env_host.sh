# paper_etrij_261003/src/ycbv on this host (.41: RTX PRO 6000 Blackwell x4; use GPU2, GPU0 is shared). 2026-10-08
export OBJPOSE_REPO=$HOME/DeepLearning/CLI_env_paper
export SAM6D_ORIG=$OBJPOSE_REPO/sam6d_ws/sam6d_master/SAM-6D
export YCBV_DIR=$HOME/Dataset/bop/ycbv
export YCBV_WORK=$HOME/Dataset/bop/ycbv_work
export YCBV_OUT=$HOME/Dataset/bop/ycbv_work/out_pro6000
export FASTSAM_X=$YCBV_WORK/weights_dl/FastSAM-x.pt
export DINOV2_VITL_DIR=$YCBV_WORK/dinov2_ckpt
export PEM_CKPT=$OBJPOSE_REPO/sam6d_realtime/sam6d_master/SAM-6D/Pose_Estimation_Model/checkpoints/sam-6d-pem-base.pth
export YCBV_RUN_LABEL="RTX PRO 6000 Blackwell, 2026-10-08"
export CUDA_VISIBLE_DEVICES=2
