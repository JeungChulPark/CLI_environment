#!/bin/bash
# gate study: 1 ownership fallback, 2 size gate, 3 thresholds re-tuned on held-out frames; 36-image checks
set -u
cd "$(dirname "$0")/.."; PY=$HOME/anaconda3/envs/sam6d/bin/python
until grep -q VITS_DONE ../ycbv_live/vits.log; do sleep 30; done
$PY det_study/tune_gates.py || echo "FAIL tune"
T=det_study/out/tune_gates.json
read S3 A3 H3 <<< $($PY -c "import json;c=json.load(open('$T'))['chosen_no_fallback'];print(c['sem'],c['appe'],c['hsv'])")
read S4 A4 H4 <<< $($PY -c "import json;c=json.load(open('$T'))['chosen'];print(c['sem'],c['appe'],c['hsv'])")
echo "tuned (no fallback): $S3 $A3 $H3 | tuned (with fallback): $S4 $A4 $H4"
export YCBV_SUBSET=25 YCBV_OUT=$PWD/out/quick
$PY run_ours.py --mode text --warmup 2 --assign-fallback --out $YCBV_OUT/pred_gate_G1.json || echo "FAIL G1"
$PY run_ours.py --mode text --warmup 2 --size-gate 1.3 --out $YCBV_OUT/pred_gate_G2.json || echo "FAIL G2"
$PY run_ours.py --mode text --warmup 2 --gate-sem $S3 --gate-appe $A3 --gate-hsv $H3 --out $YCBV_OUT/pred_gate_G3.json || echo "FAIL G3"
$PY run_ours.py --mode text --warmup 2 --assign-fallback --size-gate 1.3 --gate-sem $S4 --gate-appe $A4 --gate-hsv $H4 --out $YCBV_OUT/pred_gate_G123.json || echo "FAIL G123"
$PY run_ours.py --mode text --warmup 2 --assign-fallback --size-gate 1.3 --gate-sem $S4 --gate-appe $A4 --gate-hsv $H4 --proposer "text:yolov8m-worldv2.pt:$PWD/det_study/out/prompts_best_m.json" --out $YCBV_OUT/pred_gate_G123_p.json || echo "FAIL G123p"
echo GATE_DONE
