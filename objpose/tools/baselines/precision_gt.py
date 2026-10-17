#!/usr/bin/env python3
"""precision_gt.py — for every BOP19 run in $CMP_OUT: how many returned poses are wrong.

Each estimate is matched to the GT instance of the same object in its image (YCB-V / LM-O: one
instance per object) and judged with ADD-S < 0.1 d (d = model diameter, models_eval/models_info.json).
  answered  = GT instances that got an estimate / all GT instances (visib >= 0.1)
  wrong     = estimates failing ADD-S (or with no such object in the image) / all estimates
  correct   = GT instances with a passing estimate / all GT instances
"""
import json, os
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
OUT = Path(os.environ["CMP_OUT"])
DS = {"ycbv": ("objpose/rt/bop/ycbv_objects.json", "/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video"),
      "lmo": ("objpose/rt/bop/lmo_objects.json", "/home/jucpark/DeepLearning/Datasets/paper_benchmarks/bop/lmo")}
PTS = REPO / "sam6d_realtime/assets/model_points"
res = {}
for ds, (tab, root) in DS.items():
    table = json.loads((REPO / tab).read_text()); name = {o["obj_id"]: o["name"] for o in table["objects"]}
    info = json.loads((Path(root) / "models_eval/models_info.json").read_text())
    objs = {}
    for oid, nm in name.items():
        p = np.load(PTS / f"{nm}.npy").astype(float)
        if len(p) > 2000: p = p[np.random.default_rng(0).choice(len(p), 2000, replace=False)]
        objs[nm] = (p, cKDTree(p), 0.1 * info[str(oid)]["diameter"])
    for run in sorted(OUT.glob(f"*_{ds}_bop19")):
        est = gt_n = ans = ok = wrong = 0
        for sc in sorted(p for p in run.iterdir() if p.is_dir()):
            scene = Path(root) / "test" / sc.name
            gt = json.loads((scene / "scene_gt.json").read_text()); gi = json.loads((scene / "scene_gt_info.json").read_text())
            for l in (sc / "sam6d_frames.jsonl").read_text().splitlines():
                if not l.strip(): continue
                f = json.loads(l); k = str(f["im_id"])
                G = {}
                for r, i in zip(gt[k], gi[k]):
                    if i.get("visib_fract", 1) >= 0.1 and r["obj_id"] in name:
                        T = np.eye(4); T[:3, :3] = np.reshape(r["cam_R_m2c"], (3, 3)); T[:3, 3] = r["cam_t_m2c"]; G[name[r["obj_id"]]] = T
                gt_n += len(G)
                best = {}
                for d in f["dets"]:
                    if d["object"] not in best or d["score"] > best[d["object"]]["score"]: best[d["object"]] = d
                for nm, d in best.items():
                    est += 1
                    if nm not in G: wrong += 1; continue
                    ans += 1
                    p, tree, thr = objs[nm]
                    E = np.eye(4); E[:3, :3] = np.reshape(d["R"], (3, 3)); E[:3, 3] = d["t_mm"]
                    D = np.linalg.inv(G[nm]) @ E
                    e = tree.query(p @ D[:3, :3].T + D[:3, 3])[0].mean()
                    if e < thr: ok += 1
                    else: wrong += 1
        res[run.name] = {"answered": ans / gt_n, "wrong_rate": wrong / max(est, 1), "correct": ok / gt_n, "estimates": est, "gt": gt_n}
        print(run.name, {k: round(v, 4) if isinstance(v, float) else v for k, v in res[run.name].items()})
(OUT / "precision_gt.json").write_text(json.dumps(res, indent=1))
