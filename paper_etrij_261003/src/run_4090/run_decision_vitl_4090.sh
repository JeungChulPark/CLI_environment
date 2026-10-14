#!/bin/bash
# 판정 방식 비교(07 §3.12)의 ViT-L 판 — 이 PC(RTX 4090, GPU0). 2026-10-07
#   B  = 우리 관문(0.25/0.605/0.06) + 상자 독점 배정 (1순위)      B1 = B + 배정 대안
#   A  = 같은 앞단(문구 상자 + MobileSAM) + 원본 판정 0.5
#   _S = DINOv2 ViT-S (배포)   _L = DINOv2 ViT-L (ycbv_objects_vitl.yaml / --orig-desc dinov2_vitl14)
# 1단계: 900장 인식까지(--ism-only)  2단계: 36장(YCBV_SUBSET=25) 자세까지(PEM + 자세 검증)
set -u
W=$HOME/Dataset/bop/ycbv_work; source $W/env.sh
export OBJPOSE_REPO=$HOME/DeepLearning/server/CLI_environment          # 개선안 옵션(배정 대안)이 있는 코드
S=$OBJPOSE_REPO/paper_etrij_261003/src/ycbv
O=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003/src/ycbv/det_study/out_4090
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/decision_vitl_4090.log
mkdir -p $O/quick $W/logs; cd $S
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
CFG_S=$W/ycbv_objects.yaml; CFG_L=$W/ycbv_objects_vitl.yaml
PR="text:yolov8m-worldv2.pt:$S/det_study/out/prompts_best_m.json"
B="--proposer $PR --gate-sem 0.25 --gate-appe 0.605 --gate-hsv 0.06"
AS="--orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg text_msam"
AL="--orig-ism 0.5 --orig-desc dinov2_vitl14 --orig-seg text_msam"
run(){ # name cfg args...
  local name=$1 cfg=$2; shift 2
  say "start $name"
  YCBV_OURS_CFG=$cfg $PY run_ours.py --mode text "$@" > $O/$name.log 2>&1 && say "done $name: $(grep -o "{'targets'.*" $O/$name.log | head -1 | cut -c1-200)" || say "FAIL $name"
}
# --- 1단계 900장, 새 실험(ViT-L) 먼저
run decision_900_B_L  $CFG_L --warmup 3 $B --ism-only --out $O/decision_900_B_L.json
run decision_900_B1_L $CFG_L --warmup 3 $B --assign-fallback --ism-only --out $O/decision_900_B1_L.json
# --- 2단계 36장, ViT-L
export YCBV_SUBSET=25
run e2e_B_L  $CFG_L --warmup 2 $B --out $O/quick/pred_dec_B_L.json
run e2e_B1_L $CFG_L --warmup 2 $B --assign-fallback --out $O/quick/pred_dec_B1_L.json
# --- 같은 PC 기준값: ViT-S (B, B1) 와 원본 판정 0.5 (A_S, A_L)
unset YCBV_SUBSET
run decision_900_B_S  $CFG_S --warmup 3 $B --ism-only --out $O/decision_900_B_S.json
run decision_900_B1_S $CFG_S --warmup 3 $B --assign-fallback --ism-only --out $O/decision_900_B1_S.json
run decision_900_A_L  $CFG_S --warmup 3 $AL --ism-only --out $O/decision_900_A_L.json
run decision_900_A_S  $CFG_S --warmup 3 $AS --ism-only --out $O/decision_900_A_S.json
export YCBV_SUBSET=25
run e2e_B_S  $CFG_S --warmup 2 $B --out $O/quick/pred_dec_B_S.json
run e2e_B1_S $CFG_S --warmup 2 $B --assign-fallback --out $O/quick/pred_dec_B1_S.json
run e2e_A_L  $CFG_S --warmup 2 $AL --out $O/quick/pred_dec_A_L.json
run e2e_A_S  $CFG_S --warmup 2 $AS --out $O/quick/pred_dec_A_S.json
unset YCBV_SUBSET
# --- 36장 채점 (ladder/quick_compare.py 의 judge)
cd ladder
$PY - "$O" <<'PY' >> $LOG 2>&1
import json, sys
from pathlib import Path
O = Path(sys.argv[1])
exec(open("quick_compare.py").read().split("levels = ")[0])
out = {}
for k in ["B_L", "B1_L", "B_S", "B1_S", "A_L", "A_S"]:
    f = O / "quick" / f"pred_dec_{k}.json"
    if not f.exists(): print("missing", k); continue
    _, m = judge(json.load(open(f))); out[k] = m; print(k, m)
json.dump(out, open(O / "decision_e2e_4090.json", "w"), indent=1)
PY
say "DECISION_VITL_4090_DONE"
