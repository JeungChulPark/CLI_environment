#!/bin/bash
# mask-source study with the original object decision 0.5, YCB-V 900 images: ViT-S (개선안 2/3) then ViT-L (원본 0.5)
cd "$(dirname "$0")/.."; PY=$HOME/anaconda3/envs/sam6d/bin/python
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
$PY det_study/mask_recog.py --desc dinov2_vits14 --out det_study/out/mask_recog_900_vits.json > det_study/out/mask_recog_900_vits.log 2>&1 || echo "FAIL vits"
echo VITS_DONE
$PY det_study/mask_recog.py --desc dinov2_vitl14 --out det_study/out/mask_recog_900_vitl.json > det_study/out/mask_recog_900_vitl.log 2>&1 || echo "FAIL vitl"
echo RECOG_DONE
