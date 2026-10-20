# Shared environment of the .41 ObChange-room chains (sourced by run_obchange_rooms_41b.sh / run_obchange_bigroom_41.sh).
C=/home/jucpark/DeepLearning/CLI_environment; S=$C/paper_etrij_261003/src/objmap; W=$HOME/Dataset/bop/ycbv_work
LOG=$W/logs/obchange_rooms_41.log; mkdir -p $W/logs
say(){ echo "[$(date +%T)] $*" | tee -a $LOG; }
export OBJPOSE_REPO=$C SAM6D_ORIG=$C/sam6d_ws/sam6d_master/SAM-6D SAM6D_PY=$HOME/anaconda3/envs/sam6d/bin/python
export YCBV_DIR=$HOME/DeepLearning/Dataset/bop/ycbv YCBV_WORK=$W FASTSAM_X=$W/weights_dl/FastSAM-x.pt
export FMFUSION_ROOT=$HOME/DeepLearning/baselines/FM-Fusion GSAM_FMFUSION=$HOME/DeepLearning/baselines/gsam_fmfusion
export DUALMAP_REPO=$HOME/DeepLearning/baselines/DualMap BASELINE_WEIGHTS=$HOME/DeepLearning/baselines/weights
export LD_LIBRARY_PATH=$HOME/anaconda3/envs/fmfbuild/lib:$HOME/DeepLearning/baselines/open3d-devel-linux-x86_64-cxx11-abi-0.18.0/lib
export FMF=$HOME/anaconda3/envs/fmfusion/bin/python
[ -e $HOME/DeepLearning/Datasets ] || ln -s $HOME/DeepLearning/Dataset $HOME/DeepLearning/Datasets
W30=$HOME/DeepLearning/Datasets/objmap_obchange30; mkdir -p $W30/fmfusion $W30/dualmap $W30/bags
[ -e $W30/fmfusion/likelihood_ycbv.txt ] || cp $W/fmfusion/likelihood_*.txt $W30/fmfusion/
[ -d $W30/fmfusion/splits ] || cp -r $W/fmfusion/splits $W30/fmfusion/
# wait_convert ROOM: 0 when the room's 30 fps conversion finished, 1 when it failed
wait_convert(){ r=$(echo $1 | tr A-Z a-z)
  until grep -q "CONVERT_DONE $1 rc=0" $S/out30/logs_$r/convert.log 2>/dev/null; do
    grep -qE "MISMATCH|CONVERT_DONE $1 rc=[1-9]" $S/out30/logs_$r/convert.log 2>/dev/null && return 1; sleep 120; done; return 0; }
report(){ r=$(echo $1 | tr A-Z a-z); grep -E "^== " $S/out30/logs_$r/rt_chain.log | tail -12 | tee -a $LOG; tail -25 $S/out30/logs_$r/score.log 2>/dev/null | tee -a $LOG; }
