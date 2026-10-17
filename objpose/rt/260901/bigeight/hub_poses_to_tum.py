#!/usr/bin/env python3
"""hub_poses_to_tum.py — objpose hub output (slam_poses.jsonl of one run) -> TUM trajectory of the live poses (state OK).
    python3 hub_poses_to_tum.py <hub out dir> <out.txt>   (T_wc = pose of the SLAM sensor in its world: camera or velodyne)"""
import json, sys
import numpy as np
from scipy.spatial.transform import Rotation
src, dst = sys.argv[1], sys.argv[2]
n = 0
with open(dst, "w") as f:
    for line in open(f"{src}/slam_poses.jsonl"):
        d = json.loads(line)
        if d.get("type") != "pose" or d.get("state") != "OK" or not d.get("T_wc"):
            continue
        T = np.array(d["T_wc"], float).reshape(4, 4)
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        f.write(f"{d['t_ns']/1e9:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
        n += 1
print(f"{dst}: {n} poses")
