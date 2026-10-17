#!/usr/bin/env python3
"""set_object_frame.py — put an object's model into the common object frame (2026-09-29).

Convention used by every object (and by the viewer's axes): origin at the centre of the model's bounding box,
+Z = up (blue), +Y = back (green), +X = right when looking at the object's front (red). The front is the face a
camera sees when the object stands on a shelf as recorded.

SAM-6D's pose is expressed in the frame of the model it was given, so only two files depend on the frame:
  assets/model_points/<name>.npy      model points, mm         (PEM geometry, viewer box size)
  assets/pem_templates/<name>.pt      "tp" template points, m  (PEM template matching)
Both are transformed by the same rigid map p_new = R (p_old - c); poses then change as T_cam_new = T_cam_old @ T_old_new
(the physical pose is identical). Originals are kept as <file>.pre_frame (never overwritten).

    # say where the front and the top of the CURRENT model point, e.g. from objpose/tools/object_axes_check.py
    python tools/set_object_frame.py --object milk --front +X --up +Z --center           # dry run
    python tools/set_object_frame.py --object milk --front +X --up +Z --center --apply
"""
import argparse
import json
import shutil
import time
from pathlib import Path

import numpy as np
import torch

REPO = Path(__file__).resolve().parents[1]
AX = {"+X": [1, 0, 0], "-X": [-1, 0, 0], "+Y": [0, 1, 0], "-Y": [0, -1, 0], "+Z": [0, 0, 1], "-Z": [0, 0, -1]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", required=True)
    ap.add_argument("--front", required=True, choices=list(AX), help="current model axis pointing out of the front face")
    ap.add_argument("--up", required=True, choices=list(AX), help="current model axis pointing up")
    ap.add_argument("--center", action="store_true", help="also move the origin to the bounding-box centre")
    ap.add_argument("--apply", action="store_true", help="write the files (default: dry run)")
    a = ap.parse_args()
    front, up = np.array(AX[a.front], float), np.array(AX[a.up], float)
    if abs(front @ up) > 0.5:
        raise SystemExit("--front and --up must be different axes")
    z = up; y = -front; x = np.cross(y, z)
    R = np.stack([x, y, z])                       # rows = new axes in old coordinates; p_new = R (p_old - c)
    p_pts = REPO / "assets" / "model_points" / f"{a.object}.npy"
    p_tem = REPO / "assets" / "pem_templates" / f"{a.object}.pt"
    pts = np.load(p_pts).astype(np.float64)
    c = (pts.min(0) + pts.max(0)) / 2 if a.center else np.zeros(3)
    new = (pts - c) @ R.T
    blob = torch.load(p_tem, map_location="cpu")
    tp = blob["tp"].double()
    tp_new = ((tp - torch.tensor(c / 1000.0)) @ torch.tensor(R.T)).float()
    T_old_new = np.eye(4); T_old_new[:3, :3] = R.T; T_old_new[:3, 3] = c     # p_old = R^T p_new + c
    print(f"{a.object}: front {a.front} up {a.up} centre {np.round(c, 2)} mm")
    print(f"  model points: min {np.round(pts.min(0), 1)} max {np.round(pts.max(0), 1)}  ->  min {np.round(new.min(0), 1)} max {np.round(new.max(0), 1)}")
    print(f"  template tp : min {np.round(tp.reshape(-1, 3).min(0).values.numpy(), 4)} -> {np.round(tp_new.reshape(-1, 3).min(0).values.numpy(), 4)} m")
    print(f"  T_old_new (pose conversion T_cam_new = T_cam_old @ T_old_new):\n{np.round(T_old_new, 4)}")
    if not a.apply:
        print("dry run — add --apply to write"); return
    for p in (p_pts, p_tem):
        b = p.with_name(p.name + ".pre_frame")
        if not b.exists():
            shutil.copy2(p, b)
    np.save(p_pts, new.astype(np.load(p_pts).dtype))
    blob["tp"] = tp_new
    torch.save(blob, p_tem)
    rec_p = REPO / "assets" / "object_frames.json"
    rec = json.loads(rec_p.read_text()) if rec_p.exists() else {}
    rec.setdefault(a.object, []).append({"date": time.strftime("%Y-%m-%d %H:%M"), "front": a.front, "up": a.up,
                                         "center_mm": [round(float(v), 3) for v in c], "T_old_new": np.round(T_old_new, 6).tolist()})
    rec_p.write_text(json.dumps(rec, indent=1))
    print(f"written {p_pts.name}, {p_tem.name} (originals *.pre_frame), record {rec_p.name}")


if __name__ == "__main__":
    main()
