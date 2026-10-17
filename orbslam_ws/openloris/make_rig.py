#!/usr/bin/env python3
"""Gyro rig json for openloris_rgbd --gyro, from one sequence's trans_matrix.yaml.

R_cam_imu = rotation of T(d400_color_optical_frame -> d400_gyroscope): w_cam = R_cam_imu * w_gyro.
The D435i gyro is hardware-synchronised with the colour stream, so the time offset is 0; the start bias is 0
and the runner's online zero-rate update (objpose/src/gyro_aid.h) estimates it.

    make_rig.py <sequence dir> <out.json>
"""
import json
import sys

import cv2
import numpy as np

seq, out = sys.argv[1], sys.argv[2]
fs = cv2.FileStorage(f"{seq}/trans_matrix.yaml", cv2.FILE_STORAGE_READ)
node = fs.getNode("trans_matrix")
T = None
for i in range(node.size()):
    n = node.at(i)
    if n.getNode("parent_frame").string() == "d400_color_optical_frame" and n.getNode("child_frame").string() == "d400_gyroscope":
        T = n.getNode("matrix").mat()
if T is None:
    sys.exit("no d400_color_optical_frame -> d400_gyroscope in trans_matrix.yaml")
R = T[:3, :3]
u, _, vt = np.linalg.svd(R)
R = u @ vt
json.dump({"R_cam_imu": R.tolist(), "time": {"imu_minus_cam_s": 0.0}, "gyro_bias_rad_s": [0.0, 0.0, 0.0],
           "source": f"{seq}/trans_matrix.yaml"}, open(out, "w"), indent=1)
print(f"wrote {out}: rotation {np.degrees(np.arccos((np.trace(R) - 1) / 2)):.3f} deg from identity")
