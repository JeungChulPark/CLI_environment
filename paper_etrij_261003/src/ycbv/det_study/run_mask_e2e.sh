#!/bin/bash
# mask-source study, stage 2: 36 images end to end with the 개선안 3 back end (ViT-S, original decision 0.5,
# our PEM + pose verification); only the mask source changes (run_ours.py --orig-seg)
cd "$(dirname "$0")/.."; PY=$HOME/anaconda3/envs/sam6d/bin/python
export YCBV_SUBSET=25 YCBV_OUT=$PWD/out/quick FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
until grep -q "^-> " det_study/out/mask_study_900.log 2>/dev/null; do sleep 20; done
for seg in fastsam_full fastsam_msam text_msam; do
  $PY run_ours.py --mode text --warmup 2 --orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg $seg \
    --out $YCBV_OUT/pred_mask_$seg.json > det_study/out/mask_e2e_$seg.log 2>&1 || echo "FAIL $seg"
  echo "done $seg"
done
cd ladder          # quick_compare.py resolves its paths from the working directory when exec'd from stdin
$PY - <<'PY'
import json
from pathlib import Path
exec(open("quick_compare.py").read().split("levels = ")[0])
out = {}
for k in ["fastsam_full", "fastsam_msam", "text_msam"]:
    _, m = judge(json.load(open(Q / f"pred_mask_{k}.json"))); out[k] = m; print(k, m)
json.dump(out, open("../det_study/out/mask_e2e.json", "w"), indent=1)
PY
echo MASK_E2E_DONE
