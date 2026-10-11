#!/bin/bash
set -x
source ~/anaconda3/etc/profile.d/conda.sh
conda activate gigapose
pip install -q "numpy<2" setuptools_scm
pip install -q torch==2.0.1 torchvision==0.15.2 --index-url https://download.pytorch.org/whl/cu118
pip install -q pytorch-lightning==1.8.1 torchmetrics==0.10.3 hydra-core hydra-colorlog omegaconf pandas webdataset einops roma opencv-python pycocotools scipy ruamel.yaml distinctipy iopath fvcore tqdm wandb transforms3d pypng scikit-image simplejson joblib seaborn bokeh pyrr xatlas imageio
pip install -q open3d panda3d pin
pip install -q git+https://github.com/thodan/bop_toolkit.git
cd ~/gigapose && pip install -q -e .
python -c "import torch,pytorch_lightning as pl,webdataset,pinocchio,panda3d,bop_toolkit_lib;print('OK torch',torch.__version__,torch.cuda.is_available(),'pl',pl.__version__)"
echo INSTALL_DONE
