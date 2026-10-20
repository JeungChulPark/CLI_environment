#!/bin/bash
# One ObChange room end to end (doc 21's Kitchen pipeline, for BigRoom / LivingArea / Office / SmallRoom from the BOP
# frames at 10 fps): GT maps, sequence layout, bags, baseline inputs, ours (GPU, real time), baselines, scores.
# Usage: bash run_obchange_room.sh ROOM LAST      e.g.  bash run_obchange_room.sh Office 6   (BigRoom: 7)
# Logs: out/obchange_logs_<room>/ (Kitchen: out/obchange_logs/). Kitchen's frames come from obchange_prep.py (rosbags),
# so for Kitchen the obchange_bop_seq.py step is skipped. GPU_LOCK / DUALMAP_TIMEOUT pass through.
set -u
cd "$(dirname "$0")"
ROOM=${1:?ROOM}; LAST=${2:?LAST}
source ./obchange_env.sh
PY=${SAM6D_PY:-$HOME/anaconda3/envs/sam6d/bin/python}
D=obchange_$R
VIS=$(seq -s ' ' 2 "$LAST")
mkdir -p $L
step() { echo "== $* $(date +%T)"; }
set -e
if [ -z "${SKIP_PREP:-}" ]; then    # SKIP_PREP=1: the CPU steps were already run (PREP_ONLY=1)
step gt;       $PY obchange_gt.py --room "$ROOM" > $L/gt.log 2>&1
if [ "$R" != kitchen ]; then
  step seq;    $PY obchange_bop_seq.py --room "$ROOM" > $L/seq.log 2>&1
fi
step bags;     { $PY seq_bags.py --dataset ${D}_all --scenes $LAST; $PY seq_bags.py --dataset $D --scenes $VIS; } > $L/bags.log 2>&1
step dm_prep;  { $PY dualmap_prep.py --dataset ${D}_all --scenes $VIS; $PY dualmap_prep.py --dataset $D --scenes $VIS; } > $L/dm_prep.log 2>&1
step fmf_prep; { $PY fmfusion_prep.py --dataset ${D}_all --scenes $VIS; $PY fmfusion_prep.py --dataset $D --scenes $VIS; } > $L/fmf_prep.log 2>&1
fi
set +e
[ -n "${PREP_ONLY:-}" ] && { step prep_done; exit 0; }     # PREP_ONLY=1: CPU steps only (no GPU runs)
step ours;      bash run_obchange.sh "$ROOM" "$LAST" > $L/chain_ours.log 2>&1
step baselines; bash run_obchange_baselines.sh "$ROOM" "$LAST" > $L/chain_baselines.log 2>&1
step score;     $PY obchange_score.py --room "$ROOM" > $L/score.log 2>&1
step done;      tail -25 $L/score.log
