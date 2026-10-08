#!/bin/bash
# usage: run_fp.sh <out_dir> [extra args for run_fp_bop_ycbv.py]
source ~/anaconda3/etc/profile.d/conda.sh; conda activate foundationpose
export CUDA_HOME=/usr/local/cuda PATH=/usr/local/cuda/bin:$PATH TORCH_CUDA_ARCH_LIST="8.9"
export CC=/usr/bin/gcc CXX=/usr/bin/g++ CUDAHOSTCXX=/usr/bin/g++ NVCC_PREPEND_FLAGS="-ccbin /usr/bin/g++"
export PYOPENGL_PLATFORM=egl
OUT=$1; shift
mkdir -p $OUT
python ~/foundationpose/run_fp_bop_ycbv.py --out $OUT "$@"
echo RUN_EXIT=$?
