#!/bin/bash
# decision study, A with the original descriptor ViT-L (as 원본 0.5): YOLO-World text boxes + MobileSAM + ViT-L + original decision 0.5
cd "$(dirname "$0")/.."; PY=$HOME/anaconda3/envs/sam6d/bin/python
export FASTSAM_X=$HOME/DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt
AL="--orig-ism 0.5 --orig-desc dinov2_vitl14 --orig-seg text_msam"
O=det_study/out
$PY run_ours.py --mode text --warmup 3 $AL --ism-only --out $O/decision_900_AL.json > $O/decision_900_AL.log 2>&1 || echo "FAIL AL900"
echo "done 900 AL"
YCBV_SUBSET=25 $PY run_ours.py --mode text --warmup 2 $AL --out out/quick/pred_dec_AL.json > $O/decision_e2e_AL.log 2>&1 || echo "FAIL ALe2e"
cd ladder
$PY - <<'PY'
import json
exec(open("quick_compare.py").read().split("levels = ")[0])
out = json.load(open("../det_study/out/decision_e2e.json"))
for k in ["AL", "combo_L05_verify"]:
    f = f"pred_dec_{k}.json" if k == "AL" else f"pred_{k}.json"
    _, m = judge(json.load(open(Q / f))); out[k] = m; print(k, m)
json.dump(out, open("../det_study/out/decision_e2e.json", "w"), indent=1)
PY
echo DECISION_L_DONE
