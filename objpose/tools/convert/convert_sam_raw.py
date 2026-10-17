#!/usr/bin/env python3
"""convert_sam_raw.py — run sam6d_realtime/data/convert_recording.py on a newer rosbag2 (2026-09-29).

converting_launch/convert_all.py (which produced data_slam_converted/) is not on this PC; its SAM stage is
convert_recording.py --time-source color_global --offset-ns <-info.clock.offset_ns>. convert_recording.py was written
for Humble, where TopicMetadata takes no `id`; the `realsense` env's rosbag2_py requires one. This shim adds the id and
then runs the unchanged converter, so the pixels, stamps and topics stay exactly those of convert_recording.py.
  conda activate realsense
  python objpose/tools/convert/convert_sam_raw.py <raw SAM dir> <out bag dir> --time-source color_global --offset-ns N
"""
import itertools
import runpy
import sys
from pathlib import Path

import rosbag2_py

_Orig = rosbag2_py.TopicMetadata
_ids = itertools.count()


def _topic_metadata(*args, **kw):
    try:
        return _Orig(*args, **kw)
    except TypeError:
        return _Orig(next(_ids), *args, **kw)


rosbag2_py.TopicMetadata = _topic_metadata
CONV = Path("/home/jucpark/DeepLearning/CLI_environment_server/sam6d_realtime/data/convert_recording.py")
sys.argv = [str(CONV)] + sys.argv[1:]
runpy.run_path(str(CONV), run_name="__main__")
