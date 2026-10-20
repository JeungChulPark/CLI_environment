#!/bin/bash
# GPU 2 chain (continues run_obchange_rooms_41.sh, whose parent was stopped at 17:00 so BigRoom could go to GPU 3):
# wait for the running Office run_obchange_rt.sh to finish, then LivingArea, then SmallRoom.
set -u
source $HOME/Dataset/bop/ycbv_work/obchange_rooms_env_41.sh
export CUDA_VISIBLE_DEVICES=2
cd $S
while pgrep -f "run_obchange_rt.sh Offic[e]" >/dev/null; do sleep 60; done
say "DONE Office (started 16:49 by the first chain)"; report Office
for RL in "LivingArea 6" "SmallRoom 6"; do
  set -- $RL; ROOM=$1; LAST=$2; r=$(echo $ROOM | tr A-Z a-z)
  wait_convert $ROOM || { say "SKIP $ROOM: conversion failed"; continue; }
  say "START $ROOM (last visit $LAST, GPU $CUDA_VISIBLE_DEVICES)"
  bash run_obchange_rt.sh $ROOM $LAST > out30/logs_$r/rt_chain.log 2>&1; rc=$?
  say "DONE $ROOM rc=$rc"; report $ROOM
done
say "ROOMS_GPU2_DONE"
