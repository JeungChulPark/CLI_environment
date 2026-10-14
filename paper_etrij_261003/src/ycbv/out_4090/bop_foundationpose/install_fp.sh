#!/bin/bash
# FoundationPose conda install on host .3 (no docker). Internet via reverse ssh tunnel proxy on 127.0.0.1:3128.
set -x
export https_proxy=http://127.0.0.1:3128 http_proxy=http://127.0.0.1:3128 HTTPS_PROXY=http://127.0.0.1:3128 HTTP_PROXY=http://127.0.0.1:3128
source ~/anaconda3/etc/profile.d/conda.sh
cd ~/foundationpose/FoundationPose
conda env create -f environment.yml -n foundationpose || exit 1
conda activate foundationpose
python --version; which python
export CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH TORCH_CUDA_ARCH_LIST="8.9"
pip install torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cu124 || exit 1
python -c "import torch;print(torch.__version__, torch.cuda.is_available(), torch.version.cuda)"
pip install ninja
pip install --no-build-isolation "git+https://github.com/facebookresearch/pytorch3d.git" 2>&1 | tail -5
pip install --no-build-isolation "git+https://github.com/NVlabs/nvdiffrast.git" 2>&1 | tail -3
pip install -r requirements.txt 2>&1 | tail -3
bash build_all_conda.sh 2>&1 | tail -5
python check_env.py
rm -rf ~/.cache/pip
echo INSTALL_DONE
