#!/bin/bash
W=$HOME/Dataset/bop/ycbv_work; R=$HOME/DeepLearning/server/CLI_environment; S=$R/paper_etrij_261003/src/ycbv
O=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003/src/ycbv/out_4090; PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/main900_gpu1.log
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
source $W/env.sh; export OBJPOSE_REPO=$R CUDA_VISIBLE_DEVICES=1 YCBV_OUT=$O; cd $S
until [ -f $O/pred_hybS05_900.json ]; do sleep 20; done; say "hybS05_900 done: $(grep -o "{'median'.*" $O/hybS05_900.log | head -1)"
say "start hybS05_cf_900"; $PY run_ours.py --mode text --warmup 3 --orig-ism 0.5 --orig-desc dinov2_vits14 --orig-seg fastsam_full --verify-cluster-first --out $O/pred_hybS05_cf_900.json > $O/hybS05_cf_900.log 2>&1 && say "done hybS05_cf_900: $(grep -o "{'median'.*" $O/hybS05_cf_900.log | head -1)" || say "FAIL hybS05_cf_900"
say "evaluate"; CUDA_VISIBLE_DEVICES="" $PY evaluate.py --skip_bop --methods ladder_L0,ladder_L1_th05,ladder_L4,hybS05_900,hybS05_cf_900 --results $O/results_main_4090.json > $W/logs/eval_main900.log 2>&1 && say "eval ok" || say "FAIL eval"
say "same-object ADD-S"; CUDA_VISIBLE_DEVICES="" $PY ladder/same_object_adds.py --preds pred_ladder_L0.json,pred_ladder_L1_th05.json,pred_hybS05_900.json,pred_hybS05_cf_900.json,pred_ladder_L4.json --names L0_원본0.2,개선안2_ViTS0.5_검증끔,개선안3,개선안3_군집먼저,L4_배포 --out $O/same_object_adds_4090.json > $W/logs/same_object.log 2>&1 && say "same-object ok" || say "FAIL same-object"
say "MAIN900 DONE"
