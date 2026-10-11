#!/bin/bash
set -x
source ~/anaconda3/etc/profile.d/conda.sh
conda activate gigapose
pip install -q torchmetrics==0.10.3 hydra-core hydra-colorlog omegaconf pandas webdataset einops roma opencv-python pycocotools scipy ruamel.yaml distinctipy iopath fvcore tqdm wandb transforms3d pypng scikit-image simplejson joblib seaborn bokeh pyrr xatlas imageio "numpy<2"
pip install -q open3d panda3d
pip install -q "pin==2.6.20" || pip install -q "pin==2.6.0"
pip install git+https://github.com/thodan/bop_toolkit.git 2>&1 | tail -3
cd ~/gigapose && pip install -q -e . 2>&1 | tail -3
python - <<'PY'
import importlib
for m in ["torch","pytorch_lightning","torchmetrics","webdataset","pinocchio","panda3d","open3d","bop_toolkit_lib","roma","einops","hydra","omegaconf","pandas","cv2","pycocotools","distinctipy","iopath","fvcore","transforms3d","skimage","wandb","megapose"]:
    try: importlib.import_module(m); print("ok ",m)
    except Exception as e: print("FAIL",m,e)
import torch; print("cuda",torch.cuda.is_available())
PY
echo INSTALL_DONE
