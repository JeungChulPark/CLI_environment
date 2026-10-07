#!/bin/bash
# detector study: validation recall of the chosen configurations, then the 900-image runs + evaluation
set -u
cd "$(dirname "$0")"; PY=$HOME/anaconda3/envs/sam6d/bin/python; O=/home/jucpark/DeepLearning/CLI_environment/paper_etrij_261003/src/ycbv/det_study/out
B=text:yolov8m-worldv2.pt:$O/prompts_best_m.json
E=text:yolov8m-worldv2.pt:$O/prompts_ensemble_m.json
X=text:yolov8x-worldv2.pt
V=visual:yoloe-11l-seg.pt:12
C=text:yolov8x-worldv2.pt:$O/prompts_best_m.json+visual:yoloe-11l-seg.pt:12
$PY det_recall.py --split val --out $O/val_chosen.json --spec $B --spec $E --spec text:yolov8x-worldv2.pt:$O/prompts_best_m.json --spec $C
$PY det_recall.py --split test --out $O/test_detector.json --spec text:yolov8m-worldv2.pt --spec $B --spec $E --spec $X --spec $V --spec $C
cd ..
for kv in "m1_best|$B" "m2_ens|$E" "m3_xworld|$X" "m4_visual|$V" "m1234_combo|$C"; do
  tag=${kv%%|*}; spec=${kv#*|}
  [ -f out/pred_ours_text_$tag.json ] || $PY run_ours.py --mode text --proposer "$spec" --tag $tag
done
$PY evaluate.py --results det_study/results_det.json --methods ours_text,ours_text_m1_best,ours_text_m2_ens,ours_text_m3_xworld,ours_text_m4_visual,ours_text_m1234_combo
echo QUEUE_DONE
