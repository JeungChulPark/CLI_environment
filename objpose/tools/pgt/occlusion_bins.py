#!/usr/bin/env python3
"""occlusion_bins.py — found-rate by visible fraction (BOP visib_fract) for the estimator comparison runs."""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import score_estcmp as S
BINS = [(0.1, 0.3), (0.3, 0.5), (0.5, 0.7), (0.7, 0.9), (0.9, 1.01)]
out = {}
for ds, cfg in S.RUNS.items():
    table = json.loads((S.REPO / "objpose/rt/bop" / cfg["table"]).read_text())
    n2i = {o["name"]: o["obj_id"] for o in table["objects"]}
    diam = {int(k): v["diameter"] for k, v in json.loads(Path(cfg["info"]).read_text()).items()}
    out[ds] = {}
    for m, root in cfg["runs"].items():
        cnt = np.zeros((len(BINS), 2), int)
        for sc in sorted(p for p in root.iterdir() if (p / "sam6d_frames.jsonl").exists()):
            gt = json.loads((cfg["scenes"] / sc.name / "scene_gt.json").read_text())
            gi = json.loads((cfg["scenes"] / sc.name / "scene_gt_info.json").read_text())
            for l in (sc / "sam6d_frames.jsonl").read_text().splitlines():
                f = json.loads(l); fid = str(f["im_id"])
                best = {}
                for d in f["dets"]:
                    if d["object"] not in best or d["score"] > best[d["object"]]["score"]:
                        best[d["object"]] = d
                for r, i in zip(gt.get(fid, []), gi.get(fid, [])):
                    vf = i["visib_fract"]; oid = r["obj_id"]
                    b = next((k for k, (a, z) in enumerate(BINS) if a <= vf < z), None)
                    if b is None or oid not in diam:
                        continue
                    cnt[b, 0] += 1
                    d = next((d for n, d in best.items() if n2i[n] == oid), None)
                    if d is not None:
                        e = S.adds(S.model_pts(table, oid), np.asarray(d["R"]).reshape(3, 3), np.asarray(d["t_mm"]).reshape(3),
                                   np.asarray(r["cam_R_m2c"]).reshape(3, 3), np.asarray(r["cam_t_m2c"]).reshape(3))
                        cnt[b, 1] += e < 0.1 * diam[oid]
        out[ds][m] = cnt.tolist()
        print(ds, m, [f"{c[1] / max(c[0], 1):.2f}({c[0]})" for c in cnt], flush=True)
(S.OUT / f"occlusion_bins{S.SUF}.json").write_text(json.dumps({"bins": BINS, "counts": out}, indent=1))
