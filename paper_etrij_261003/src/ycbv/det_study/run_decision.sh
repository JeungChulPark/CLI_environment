#!/bin/bash
# decision study on YCB-V, same front end (YOLO-World text boxes, method-1 prompts, top-3 + MobileSAM + DINOv2 ViT-S):
#   A   original SAM-6D decision (semantic + appearance + geometric score, top-1 per object, > 0.5)
#   B   our decision: gates semantic 0.25 / appearance 0.605 / HSV colour 0.06 + exclusive box assignment
#   B1  B + assignment fallback (the decision of 개선안 1)
# stage 1: recognition only on the 900 images (run_ours.py --ism-only); stage 2: 36 images with PEM + verification
cd "$(dirname "$0")/.."; PY=$HOME/anaconda3/envs/sam6d/bin/python
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
PR="text:yolov8m-worldv2.pt:$PWD/det_study/out/prompts_best_m.json"
A="--orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg text_msam"
B="--proposer $PR --gate-sem 0.25 --gate-appe 0.605 --gate-hsv 0.06"
O=det_study/out
$PY run_ours.py --mode text --warmup 3 $A --ism-only --out $O/decision_900_A.json > $O/decision_900_A.log 2>&1 || echo "FAIL A900"
echo "done 900 A"
$PY run_ours.py --mode text --warmup 3 $B --ism-only --out $O/decision_900_B.json > $O/decision_900_B.log 2>&1 || echo "FAIL B900"
echo "done 900 B"
$PY run_ours.py --mode text --warmup 3 $B --assign-fallback --ism-only --out $O/decision_900_B1.json > $O/decision_900_B1.log 2>&1 || echo "FAIL B1900"
echo "done 900 B1"
export YCBV_SUBSET=25
$PY run_ours.py --mode text --warmup 2 $A --out out/quick/pred_dec_A.json > $O/decision_e2e_A.log 2>&1 || echo "FAIL Ae2e"
$PY run_ours.py --mode text --warmup 2 $B --out out/quick/pred_dec_B.json > $O/decision_e2e_B.log 2>&1 || echo "FAIL Be2e"
$PY run_ours.py --mode text --warmup 2 $B --assign-fallback --out out/quick/pred_dec_B1.json > $O/decision_e2e_B1.log 2>&1 || echo "FAIL B1e2e"
cd ladder
$PY - <<'PY'
import json
from pathlib import Path
exec(open("quick_compare.py").read().split("levels = ")[0])
out = {}
for k in ["A", "B", "B1"]:
    _, m = judge(json.load(open(Q / f"pred_dec_{k}.json"))); out[k] = m; print(k, m)
json.dump(out, open("../det_study/out/decision_e2e.json", "w"), indent=1)
PY
echo DECISION_DONE
