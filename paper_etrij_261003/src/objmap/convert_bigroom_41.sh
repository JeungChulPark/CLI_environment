#!/bin/bash
# BigRoom on this machine (.41): wait for raw/BigRoom.zip to finish downloading and verify (pdl_bigroom.log),
# then convert the 7 visit bags to 30 fps frames with obchange_raw_seq.py (doc 29 layout; same trees as
# run_obchange_rt.sh: gt30/ + the frame store below). GT visit maps come from gt/obchange_bigroom (obchange_gt.py).
set -u
cd "$(dirname "$0")"
RAW=/mnt/nvme1/Dataset/obchange/raw
export OBJMAP_GT=$PWD/gt30 OBJMAP_OBCHANGE_FRAMES=/mnt/nvme1/Dataset/obchange/work30
PY=$HOME/anaconda3/envs/sam6d/bin/python
echo "wait for download $(date +%T)"
until grep -q '^finished' $RAW/pdl_bigroom.log 2>/dev/null; do
  pgrep -f pdl_bigroom.sh >/dev/null || { echo "download script exited without finishing"; exit 1; }
  sleep 30
done
want=$(cut -d' ' -f2 $RAW/BigRoom.meta.txt | sed 's/^md5://'); got=$(grep '^md5 ' $RAW/pdl_bigroom.log | cut -d' ' -f2)
echo "md5 want $want got $got"
[ "$want" = "$got" ] || { echo "MD5 MISMATCH, not converting"; exit 2; }
echo "zip entries:"; unzip -l $RAW/BigRoom.zip | tail -12
echo "convert start $(date +%T)"
$PY obchange_raw_seq.py --room BigRoom --zip $RAW/BigRoom.zip --tmp $RAW/_tmp --gt-from $PWD/gt
rc=$?
echo "convert exit $rc $(date +%T)"
du -sh $OBJMAP_OBCHANGE_FRAMES/bigroom 2>/dev/null
echo "CONVERT_DONE rc=$rc"
