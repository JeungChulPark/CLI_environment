#!/bin/bash
# 3부: 구성 1 의 ViT-S 판 (FastSAM-x 전체 분할 -> ViT-S -> 원본 판정 0.5) 같은 PC 에서.
set -u
W=$HOME/Dataset/bop/ycbv_work; source $W/env.sh
export OBJPOSE_REPO=$HOME/DeepLearning/server/CLI_environment
S=$OBJPOSE_REPO/paper_etrij_261003/src/ycbv
O=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003/src/ycbv/det_study/out_4090
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/decision_vitl_4090.log
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
cd $S
OS="--orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg fastsam_full"
say "start decision_900_O_S"
$PY run_ours.py --mode text --warmup 3 $OS --ism-only --out $O/decision_900_O_S.json > $O/decision_900_O_S.log 2>&1 && say "done decision_900_O_S: $(grep -o "{'targets'.*" $O/decision_900_O_S.log | head -1 | cut -c1-200)" || say "FAIL decision_900_O_S"
say "start e2e_O_S"
YCBV_SUBSET=25 $PY run_ours.py --mode text --warmup 2 $OS --out $O/quick/pred_dec_O_S.json > $O/e2e_O_S.log 2>&1 && say "done e2e_O_S" || say "FAIL e2e_O_S"
cd ladder
$PY - "$O" <<'PY' >> $LOG 2>&1
import json, sys
from pathlib import Path
O = Path(sys.argv[1])
exec(open("quick_compare.py").read().split("levels = ")[0])
out = {}
for k in ["O_S", "O_L", "A_S", "A_L", "B_S", "B_L", "B1_S", "B1_L"]:
    f = O / "quick" / f"pred_dec_{k}.json"
    if not f.exists(): print("missing", k); continue
    _, m = judge(json.load(open(f))); out[k] = m; print(k, m)
json.dump(out, open(O / "decision_e2e_4090.json", "w"), indent=1)
PY
say "DECISION_VITL_4090_PART3_DONE"
