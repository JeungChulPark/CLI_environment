#!/bin/bash
# Baseline environments for the object-map comparison on this PC (RTX 4090, driver 575 → CUDA ≤ 12.9), 2026-10-12.
# Three sections, run by hand once: (1) copy code+patches+weights from .41, (2) dualmap env, (3) fmfusion env.
# Code lives in ~/DeepLearning/baselines (same path as the laptop and .41, hard-coded in fmfusion_prep.py / dualmap_run.sh).
# FM-Fusion build on this PC: cmake -DOpen3D_DIR=<open3d-devel>/lib/cmake/Open3D with libc++ (copied from .41's conda pkgs,
#   dropped into the Open3D lib dir) linked via -DCMAKE_EXE_LINKER_FLAGS="-lc++ -lc++abi"; DualMap also needs scikit-learn.
exit 0  # documentation of what was run; remove this line to re-run a section

########## copy_baselines_from41.sh ##########
# 2026-10-12: copy the baseline code (with the laptop's local patches), weights and Open3D devel from .41 (RTX PRO 6000 host)
B=$HOME/DeepLearning/baselines; SSH="ssh -p 10022 -o ConnectTimeout=10"; H=jucpark@10.254.176.41
for d in FM-Fusion gsam_fmfusion DualMap weights open3d-devel-linux-x86_64-cxx11-abi-0.18.0; do
  echo "[$(date +%T)] $d"; rsync -a --exclude '__pycache__' -e "$SSH" $H:DeepLearning/baselines/$d/ $B/$d/ || echo "FAIL $d"
done
rsync -a -e "$SSH" $H:DeepLearning/baselines/open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz $B/
echo "[$(date +%T)] END"; du -sh $B/*

########## make_dualmap_env.sh ##########
# 2026-10-12: dualmap conda env for this PC (driver 575 → CUDA ≤ 12.9; .41's env is torch cu130 and cannot be copied).
# Clone the local sam6d env (no download) and add DualMap's pip requirements (environment.yml, torch kept as is).
source ~/anaconda3/etc/profile.d/conda.sh
echo "[$(date +%T)] clone sam6d -> dualmap"
conda create -y -n dualmap --clone sam6d --offline || { echo "CLONE FAILED"; exit 1; }
conda activate dualmap
echo "[$(date +%T)] pip install DualMap requirements"
pip install "numpy<2.0" "ultralytics==8.3.103" "supervision==0.25.1" faiss-cpu==1.9.0 tyro open_clip_torch wandb h5py openai hydra-core distinctipy dill open3d imageio natsort kornia "rerun-sdk==0.22.1" "record3d==1.4.1" pyliblzfse pypng tabulate pympler plyfile 2>&1 | tail -15
pip install git+https://github.com/ultralytics/CLIP.git 2>&1 | tail -3
echo "[$(date +%T)] check"
python -c "import torch, ultralytics, faiss, open3d, hydra, clip, numpy; print('ok torch', torch.__version__, 'cuda', torch.cuda.is_available(), 'ultralytics', ultralytics.__version__, 'numpy', numpy.__version__, 'open3d', open3d.__version__)"
echo "[$(date +%T)] ENV DONE"

########## make_fmfusion_env.sh ##########
# 2026-10-12: fmfusion conda env for this PC (GroundingDINO + SAM for fmfusion_detect.py). .41's env (python 3.10, torch cu128)
# has a GroundingDINO CUDA op built for Blackwell; here we clone the local sam6d env (python 3.12, torch 2.7.1+cu12.9)
# and build the op for the 4090 from the copied source (no code download).
source ~/anaconda3/etc/profile.d/conda.sh
G=$HOME/DeepLearning/baselines/gsam_fmfusion
echo "[$(date +%T)] clone sam6d -> fmfusion"
conda create -y -n fmfusion --clone sam6d --offline || { echo "CLONE FAILED"; exit 1; }
conda activate fmfusion
export CUDA_HOME=/usr/local/cuda-12.9 PATH=/usr/local/cuda-12.9/bin:$PATH TORCH_CUDA_ARCH_LIST="8.9" MAX_JOBS=8
rm -f $G/GroundingDINO/groundingdino/_C*.so
echo "[$(date +%T)] pip: deps + GroundingDINO (builds _C) + segment_anything"
nice -n 19 pip install addict yapf pycocotools "supervision==0.30.9" open3d transformers 2>&1 | tail -3
nice -n 19 pip install --no-build-isolation -e $G/GroundingDINO 2>&1 | tail -5
pip install -e $G/segment_anything 2>&1 | tail -2
echo "[$(date +%T)] check"
python -c "import groundingdino._C, segment_anything, supervision, open3d, transformers, torch; print('ok _C, torch', torch.__version__, 'cuda', torch.cuda.is_available())"
echo "[$(date +%T)] ENV DONE"
