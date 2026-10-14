#!/bin/bash
# 06 단계별 분해 실험(사다리) 900장 전체를 이 PC(GPU0 단독)에서: 가중치 내려받기 대기 → L0,L1,L3,L5,B,A → 채점(BOP AR 포함)
W=$HOME/Dataset/bop/ycbv_work; S=$HOME/DeepLearning/old/CLI_environment/paper_etrij_261003/src/ycbv
PY=$HOME/anaconda3/envs/sam6d/bin/python; LOG=$W/logs/ladder_chain.log
say(){ echo "[$(date +%T)] $*" >> $LOG; }
say "wait download"; until grep -q DL_OK $W/logs/download.log; do sleep 20; done
sz1=$(stat -c %s $W/weights_dl/FastSAM-x.pt); sz2=$(stat -c %s $W/weights_dl/dinov2_vitl14_pretrain.pth)
[ "$sz1" = 144972346 ] && [ "$sz2" = 1217586395 ] || { say "FAIL size $sz1 $sz2"; exit 1; }
say "download ok, fastsam md5 $(md5sum $W/weights_dl/FastSAM-x.pt | cut -c1-8) (노트북 공식본 f1dc5b65)"
mkdir -p $W/dinov2_ckpt; ln -sf $W/weights_dl/dinov2_vitl14_pretrain.pth $W/dinov2_ckpt/
ln -sf $HOME/DeepLearning/old/CLI_environment/sam6d_realtime/sam6d_master/SAM-6D/Instance_Segmentation_Model/checkpoints/dinov2/dinov2_vits14_pretrain.pth $W/dinov2_ckpt/
grep -q FASTSAM_X $W/env.sh || cat >> $W/env.sh <<EOT
export FASTSAM_X=$W/weights_dl/FastSAM-x.pt
export DINOV2_VITL_DIR=$W/dinov2_ckpt
EOT
source $W/env.sh; export YCBV_OUT=$S/out_4090; cd $S
say "smoke L0"; $PY ladder/run_ladder.py --level L0 --limit 3 --warmup 1 > $W/logs/smoke_L0.log 2>&1 || { say "FAIL smoke L0"; exit 1; }
rm -f $YCBV_OUT/pred_ladder_L0.json
run(){ n=$1; shift; say "start $n"; "$@" > $W/logs/ladder_$n.log 2>&1 || say "FAIL $n"; say "end $n"; }
run L0  $PY ladder/run_ladder.py --level L0
run L1  $PY ladder/run_ladder.py --level L1
run L3  $PY run_ours.py --mode text --no-verify --out $YCBV_OUT/pred_ladder_L3.json
ln -sf $YCBV_OUT/pred_ours_text.json $YCBV_OUT/pred_ladder_L4.json   # L4 = run_ours.py --mode text, 11:33 실행분 재사용
run L5  $PY run_ours.py --mode text --proposer "text:yolov8m-worldv2.pt:$S/det_study/out/prompts_best_m.json" --out $YCBV_OUT/pred_ladder_L5.json
run B04 $PY ladder/run_ladder.py --level L1 --thresh 0.4
run B05 $PY ladder/run_ladder.py --level L1 --thresh 0.5
run A0  $PY run_ours.py --mode text --proposer fastsam --top-k 200 --no-verify --out $YCBV_OUT/pred_ladder_A_noverify.json
run A   $PY run_ours.py --mode text --proposer fastsam --top-k 200 --out $YCBV_OUT/pred_ladder_A.json
say "evaluate"; $PY evaluate.py --results $S/ladder/results_ladder_4090.json --methods ladder_L0,ladder_L1,ladder_L2,ladder_L3,ladder_L4,ladder_L5,ladder_L1_th04,ladder_L1_th05,ladder_A_noverify,ladder_A > $W/logs/ladder_evaluate.log 2>&1 || say "FAIL evaluate"
say "ALL DONE"
