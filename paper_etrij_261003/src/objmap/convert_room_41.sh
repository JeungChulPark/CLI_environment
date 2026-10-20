#!/bin/bash
# ObChange room on this machine (.41): verify the room zip (md5 from the TU Wien record, zip/record_checksums.txt),
# then convert its visit bags to 30 fps frames with obchange_raw_seq.py (doc 29 layout; gt30/ + the frame store
# below, as run_obchange_rt.sh expects). GT visit maps must already be in gt/obchange_<room> (obchange_gt.py).
#   bash convert_room_41.sh Office /mnt/nvme1/Dataset/obchange/zip/Office.zip
set -u
cd "$(dirname "$0")"
ROOM=${1:?ROOM}; ZIP=${2:?ZIP}; r=$(echo "$ROOM" | tr 'A-Z' 'a-z')
OB=/mnt/nvme1/Dataset/obchange
export OBJMAP_GT=$PWD/gt30 OBJMAP_OBCHANGE_FRAMES=$OB/work30
PY=$HOME/anaconda3/envs/sam6d/bin/python
want=$(grep "^$ROOM.zip " $OB/zip/record_checksums.txt | awk '{print $3}' | sed 's/^md5://')
wsize=$(grep "^$ROOM.zip " $OB/zip/record_checksums.txt | awk '{print $2}')
size=$(stat -c %s "$ZIP")
echo "$ROOM: zip $ZIP size $size (want $wsize) $(date +%T)"
[ "$size" = "$wsize" ] || { echo "SIZE MISMATCH, not converting"; exit 2; }
if [ -f "$ZIP.md5ok" ] && [ "$(cat "$ZIP.md5ok")" = "$want" ]; then echo "md5 already verified"; else
  got=$(md5sum "$ZIP" | cut -d' ' -f1); echo "md5 want $want got $got $(date +%T)"
  [ "$got" = "$want" ] || { echo "MD5 MISMATCH, not converting"; exit 2; }
  echo "$got" > "$ZIP.md5ok"
fi
echo "zip entries:"; unzip -l "$ZIP" | grep -E '\.bag$'
echo "convert start $(date +%T)"
$PY obchange_raw_seq.py --room "$ROOM" --zip "$ZIP" --tmp $OB/raw/_tmp_$r --gt-from $PWD/gt
rc=$?
echo "convert exit $rc $(date +%T)"
du -sh $OBJMAP_OBCHANGE_FRAMES/$r 2>/dev/null
echo "CONVERT_DONE $ROOM rc=$rc"
