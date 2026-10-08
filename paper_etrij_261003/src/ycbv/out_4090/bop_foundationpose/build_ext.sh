#!/bin/bash
# Rebuild CUDA extensions with the system gcc 11.4 (conda cxx-compiler = gcc 14, rejected by nvcc 12.4)
set -x
export https_proxy=http://127.0.0.1:3128 http_proxy=http://127.0.0.1:3128
source ~/anaconda3/etc/profile.d/conda.sh; conda activate foundationpose
export CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH TORCH_CUDA_ARCH_LIST="8.9" MAX_JOBS=16
export CC=/usr/bin/gcc CXX=/usr/bin/g++ CUDAHOSTCXX=/usr/bin/g++ NVCC_PREPEND_FLAGS="-ccbin /usr/bin/g++"
cd /tmp && rm -rf nvd && git clone -q https://github.com/NVlabs/nvdiffrast.git nvd && cd nvd && git log -1 --format=%h && pip install --no-build-isolation . 2>&1 | tail -3
cd /tmp && rm -rf p3d && git clone -q https://github.com/facebookresearch/pytorch3d.git p3d && cd p3d && git log -1 --format=%h && pip install --no-build-isolation . 2>&1 | tail -3
pip install pycocotools 2>&1 | tail -1
cd ~/foundationpose/FoundationPose && python check_env.py
rm -rf /tmp/nvd /tmp/p3d ~/.cache/pip
echo BUILD_DONE
