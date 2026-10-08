#!/bin/bash
# This PC (CPU only; the NVIDIA driver is wedged): fetch host .3 outputs, convert, score with our protocol, merge BOP AR.
set -e
D=/home/jucpark/DeepLearning/server/CLI_environment/paper_etrij_261003/src/ycbv/out_4090/bop_foundationpose
O=/home/jucpark/DeepLearning/server/CLI_environment/paper_etrij_261003/src/ycbv/out_4090
H=jucpark@192.168.20.3
scp -q -P 10022 $H:foundationpose/out_full/foundationpose_ycbv-test.csv $H:foundationpose/out_full/foundationpose_ycbv-test_timing.json $H:foundationpose/out_full/results.jsonl $H:foundationpose/full.log $D/
scp -q -P 10022 $H:foundationpose/bop_results/eval/foundationpose_ycbv-test/scores_bop19.json $D/scores_bop19_foundationpose.json
scp -q -P 10022 $H:foundationpose/bop_results/eval_log_foundationpose.txt $H:foundationpose/eval_bop_host.sh $D/
mkdir -p $D/eval && scp -q -r -P 10022 $H:foundationpose/bop_results/eval/foundationpose_ycbv-test $D/eval/
python3 -I $D/csv_to_pred.py $D/foundationpose_ycbv-test.csv $O/pred_ladder_foundationpose.json foundationpose_modelbased \
  "FoundationPose model-based (CNOS-FastSAM masks, hypotheses+refiner x5+scorer), RTX 4090 host .3, 2026-10-08" \
  "NVlabs/FoundationPose a1b694b, weights 2023-10-28-18-33-37 (refiner) + 2024-01-11-20-02-45 (scorer), est.register iteration=5, zfar 1.5 m" \
  $D/foundationpose_ycbv-test_timing.json
export YCBV_DIR=$HOME/Dataset/bop/ycbv YCBV_WORK=$HOME/Dataset/bop/ycbv_work YCBV_OUT=$O OBJPOSE_REPO=$HOME/DeepLearning/CLI_env_paper
export YCBV_RUN_LABEL="FoundationPose model-based, RTX 4090 (.3), 2026-10-08" CUDA_VISIBLE_DEVICES=""
cd $HOME/DeepLearning/CLI_env_paper/paper_etrij_261003/src/ycbv
timeout 1200 $HOME/anaconda3/envs/sam6d/bin/python evaluate.py --skip_bop --methods ladder_foundationpose --results $D/results_foundationpose_4090.json 2>&1 | tail -2
python3 -I - <<PY
import json
D="$D"
res=json.load(open(f"{D}/results_foundationpose_4090.json"))
s=json.load(open(f"{D}/scores_bop19_foundationpose.json"))
m=res["methods"]["ladder_foundationpose"]
m["bop_ar"]={k: round(100.0*v,1) for k,v in s.items() if k.startswith("bop19_average_recall")}
m["bop_ar"]["raw"]=s
m["bop_ar"]["note"]="official bop_toolkit cea62d6 scripts/eval_bop19_pose.py --renderer_type=vispy, run on host .3 (this PC's GPU driver was wedged)"
t=json.load(open(f"{D}/foundationpose_ycbv-test_timing.json"))
m["timing"]={k:v for k,v in t.items() if k!="images_detail"}
m["timing"]["cnos_detection_s_per_image"]=0.19
m["timing"]["note2"]="CSV time excludes CNOS detection (~0.19 s/img on this GPU, from the BOP default detection file)"
res["methods"]["ladder_foundationpose"]=m
json.dump(res, open(f"{D}/results_foundationpose_4090.json","w"), indent=1, default=float)
print("BOP AR", {k:v for k,v in m["bop_ar"].items() if k!="raw"})
print({k:m[k] for k in ("images","visible_gt_targets","answers","correct_answers","found_pct","answers_correct_pct","wrong_per_image","adds_auc","adds_auc_mean_over_objects")})
print("time_ms", m["time_ms"])
PY
cp $O/pred_ladder_foundationpose.json $D/
ls -la $D
