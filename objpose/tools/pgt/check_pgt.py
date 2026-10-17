#!/usr/bin/env python3
"""check_pgt.py — draw the pseudo ground truth (model points + visible box) on eval frames for a visual check."""
import json, sys
from pathlib import Path
import cv2, numpy as np, trimesh
ROOT = Path(sys.argv[1]); out = Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
T = json.loads((Path(__file__).resolve().parents[2] / "rt/bop/real_objects.json").read_text())
sc = ROOT / "eval/000001"
gt = json.loads((sc / "scene_gt.json").read_text()); gi = json.loads((sc / "scene_gt_info.json").read_text())
K = np.asarray(next(iter(json.loads((sc / "scene_camera.json").read_text()).values()))["cam_K"]).reshape(3, 3)
pts = {}
col = [(230, 60, 60), (60, 160, 60), (60, 60, 230), (0, 180, 230), (200, 0, 200), (0, 140, 255), (0, 230, 230), (140, 90, 40), (60, 220, 120)]
ids = sorted(gt, key=int)
sel = [ids[i] for i in np.linspace(0, len(ids) - 1, int(sys.argv[3]) if len(sys.argv) > 3 else 12).astype(int)]
for fid in sel:
    img = cv2.imread(str(sc / "rgb" / f"{int(fid):06d}.png"))
    for r, i in zip(gt[fid], gi[fid]):
        oid = r["obj_id"]
        if oid not in pts:
            m = trimesh.load(Path(T["models_dir"]) / f"obj_{oid:06d}.ply", process=False)
            pts[oid] = trimesh.sample.sample_surface(m, 3000, seed=0)[0] if len(getattr(m, "faces", [])) else np.asarray(m.vertices)[::50]
        R = np.asarray(r["cam_R_m2c"]).reshape(3, 3); t = np.asarray(r["cam_t_m2c"])
        X = pts[oid] @ R.T + t; X = X[X[:, 2] > 50]
        uv = (X[:, :2] / X[:, 2:3]) * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
        c = col[(oid - 1) % len(col)]
        if i["visib_fract"] >= 0.1:
            for u, v in uv.astype(int)[::3]:
                if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
                    img[v, u] = c
            x, y, w, h = i["bbox_visib"]
            cv2.rectangle(img, (x, y), (x + w, y + h), c, 1)
            name = next(o["name"] for o in T["objects"] if o["obj_id"] == oid)
            cv2.putText(img, f"{name} {i['visib_fract']:.2f}", (x, max(y - 3, 10)), 0, 0.4, c, 1)
    cv2.putText(img, fid, (5, 15), 0, 0.5, (255, 255, 255), 1)
    cv2.imwrite(str(out / f"{int(fid):06d}.jpg"), img)
