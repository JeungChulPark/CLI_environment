#!/bin/bash
# 2부: 구성 1 (FastSAM-x 전체 분할 -> ViT-L -> 원본 판정 0.5) 를 같은 PC 에서. 1부(run_decision_vitl_4090.sh) 가 끝난 뒤 실행.
set -u
W=$HOME/Dataset/bop/ycbv_work; source $W/env.sh
export OBJPOSE_REPO=$HOME/DeepLearning/server/CLI_environment
S=$OBJPOSE_REPO/paper_etrij_261003/src/ycbv
O=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003/src/ycbv/det_study/out_4090
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/decision_vitl_4090.log
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
until grep -q DECISION_VITL_4090_DONE $LOG; do sleep 30; done
cd $S
OL="--orig-ism 0.5 --orig-desc dinov2_vitl14 --orig-seg fastsam_full"
say "start decision_900_O_L"
$PY run_ours.py --mode text --warmup 3 $OL --ism-only --out $O/decision_900_O_L.json > $O/decision_900_O_L.log 2>&1 && say "done decision_900_O_L: $(grep -o "{'targets'.*" $O/decision_900_O_L.log | head -1 | cut -c1-200)" || say "FAIL decision_900_O_L"
say "start e2e_O_L"
YCBV_SUBSET=25 $PY run_ours.py --mode text --warmup 2 $OL --out $O/quick/pred_dec_O_L.json > $O/e2e_O_L.log 2>&1 && say "done e2e_O_L" || say "FAIL e2e_O_L"
cd ladder
$PY - "$O" <<'PY' >> $LOG 2>&1
import json, sys
from pathlib import Path
O = Path(sys.argv[1])
exec(open("quick_compare.py").read().split("levels = ")[0])
out = {}
for k in ["O_L", "A_L", "B_L", "B1_L", "A_S", "B_S", "B1_S"]:
    f = O / "quick" / f"pred_dec_{k}.json"
    if not f.exists(): print("missing", k); continue
    _, m = judge(json.load(open(f))); out[k] = m; print(k, m)
json.dump(out, open(O / "decision_e2e_4090.json", "w"), indent=1)
PY
say "DECISION_VITL_4090_PART2_DONE"
