#!/bin/bash
# pose verification cost study, part 2: --verify-candidates really limits the candidates measured
# (verify.candidate_topk; --verify-topk in run_verify_cost.sh is ignored while verification is on),
# and half precision with the verification kept in fp32
cd "$(dirname "$0")/.."; PY=$HOME/anaconda3/envs/sam6d/bin/python
export YCBV_SUBSET=25 YCBV_OUT=$PWD/out/quick
G="--mode text --warmup 2 --assign-fallback --size-gate 1.3 --gate-sem 0.25 --gate-appe 0.605 --gate-hsv 0.06 --proposer text:yolov8m-worldv2.pt:$PWD/det_study/out/prompts_best_m.json"
# smoke: a PEM error is caught and logged, not raised, so check the log as well as the exit code
for o in "--verify-candidates 30" "--precision fp16"; do
  $PY run_ours.py $G --limit 2 $o --out /tmp/claude-1000/vc_smoke.json > /tmp/claude-1000/vc_smoke.log 2>&1 \
    && ! grep -q "PEM 건너뜀" /tmp/claude-1000/vc_smoke.log || { echo "SMOKE_FAIL $o"; exit 1; }
done
for k in 100 50 30 10; do $PY run_ours.py $G --verify-candidates $k --out $YCBV_OUT/pred_vk_cand$k.json || echo "FAIL cand$k"; done
$PY run_ours.py --mode text --warmup 2 --verify-candidates 30 --out $YCBV_OUT/pred_vk_L4_cand30.json || echo "FAIL L4cand30"
# fp16/bf16 again: part 1 ran them before the verification steps were kept in fp32 (every PEM call failed)
for p in fp16 bf16; do $PY run_ours.py $G --precision $p --out $YCBV_OUT/pred_vk_$p.json || echo "FAIL $p"; done
echo VK_DONE
