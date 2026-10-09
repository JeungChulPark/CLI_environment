"""Convert a DualMap concrete map (map/*.pkl) to the common map format.

Runs in the `dualmap` conda env (the pickles need DualMap's object classes). Every saved
object becomes one map object: label = YCB-V name of its class id (ids 0..20 follow
ycbv_prompts.txt), 'unknown' for FastSAM segments; points = its point cloud (world frame of
cameras.json, metres - DualMap builds points with the absolute poses of traj.txt).

Usage: python dualmap_export.py --scene 52 --map <runs>/ycbv000052/ycbv_ycbv000052/map
"""
import argparse
import glob
import os
import pickle
import sys
from pathlib import Path

import numpy as np

from common import OUT, YCB_NAMES, save_map

DM = Path(os.environ.get("DUALMAP_REPO", Path.home() / "DeepLearning/baselines/DualMap"))
sys.path.insert(0, str(DM))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", type=int, required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--method", default="dualmap")
    a = ap.parse_args()
    objs = []
    for f in sorted(glob.glob(f"{a.map}/*.pkl")):
        with open(f, "rb") as fh:
            o = pickle.load(fh)
        P = np.asarray(o.pcd.points)
        cid = o.class_id
        label = YCB_NAMES[cid] if cid is not None and 0 <= cid < len(YCB_NAMES) else "unknown"
        objs.append({"label": label, "points": P, "class_id": cid})
    out = OUT / a.method / "ycbv" / f"{a.scene:06d}"
    save_map(out, objs, {"dataset": "ycbv", "scene": a.scene, "kind": a.method, "source": str(a.map)})
    named = sorted(o["label"] for o in objs if o["label"] != "unknown")
    print(f"{a.scene:06d}: {len(objs)} objects ({len(named)} YCB, {len(objs) - len(named)} unknown) -> {out}")
    print("   ", named)


if __name__ == "__main__":
    main()
