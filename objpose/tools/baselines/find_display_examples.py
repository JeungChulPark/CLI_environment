#!/usr/bin/env python3
"""find_display_examples.py — frames where the Object Pose Map built from the ORIGINAL SAM-6D draws an object wrong
(off the ground truth or doubled) while the map built from the PROPOSED pipeline draws the same object right.
Same replay as display_eval.py (map state available at k/30 s, drawn with the SLAM pose of frame k).
    python find_display_examples.py  -> objpose/output/cmp_260924_gt/display/examples.json
"""
import json, os, sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parent))
import display_eval as DE                                          # noqa: E402

BM = DE.BM
REPO = DE.REPO
RUNS = {"orig": Path(os.environ.get("ORIG_RUN", REPO / "objpose/output/cmp_260924/orig_ycbv_realtime_top1")),
        "prop": Path(os.environ.get("PROP_RUN", REPO / "objpose/output/cmp_260924_gt/proposed_ycbv_realtime"))}


def scene_examples(scene):
    table = json.loads(BM.TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    info = json.loads((BM.YCBV.parent / "models_eval/models_info.json").read_text())
    name_id = {v: k for k, v in by_id.items()}
    pts = {}
    for nm in by_id.values():
        p = np.load(BM.PTS / f"{nm}.npy").astype(float)
        if len(p) > 1500: p = p[np.random.default_rng(0).choice(len(p), 1500, replace=False)]
        pts[nm] = (p, cKDTree(p), 0.1 * info[str(name_id[nm])]["diameter"])
    rep = {k: DE.replay(scene, v, by_id) for k, v in RUNS.items()}
    ids, slam, K, _ = rep["orig"]
    gt = json.loads((BM.YCBV / scene / "scene_gt.json").read_text())
    gi = json.loads((BM.YCBV / scene / "scene_gt_info.json").read_text())
    out = []
    for k in range(0, len(ids), 10):
        if k not in slam:
            continue
        fid = ids[k]
        G = {}
        for r, i in zip(gt[fid], gi[fid]):
            if r["obj_id"] in by_id and i.get("visib_fract", 1) >= 0.6:
                T = np.eye(4); T[:3, :3] = np.reshape(r["cam_R_m2c"], (3, 3)); T[:3, 3] = np.asarray(r["cam_t_m2c"]) / 1000.0
                G[by_id[r["obj_id"]]] = T
        res = {}
        for key, (ids_, slam_, K_, snaps) in rep.items():
            done = [s[0] for s in snaps]
            j = int(np.searchsorted(done, k / DE.FPS, side="right")) - 1
            if j < 0:
                res[key] = None; continue
            Tcw = np.linalg.inv(slam_[k])
            per = {}
            for nm, Tw in snaps[j][1]:
                if nm not in G:
                    continue
                p, tree, thr = pts[nm]
                Tc = Tcw @ Tw
                D = np.linalg.inv(G[nm]) @ Tc
                e = tree.query(p @ D[:3, :3].T + D[:3, 3] * 1000.0)[0].mean()
                dpos = float(np.linalg.norm(Tc[:3, 3] - G[nm][:3, 3]) * 1000)
                per.setdefault(nm, []).append((float(e), dpos, float(thr)))
            res[key] = per
        if res["orig"] is None or res["prop"] is None:
            continue
        for nm in G:
            o, q = res["orig"].get(nm, []), res["prop"].get(nm, [])
            if len(q) != 1 or q[0][0] > 0.5 * q[0][2] or not o:
                continue
            bad_o = [x for x in o if x[0] > x[2]]
            if not bad_o and len(o) == 1:
                continue
            worst = max(x[1] for x in o)
            if worst < 30:                                # must be visibly off
                continue
            out.append({"scene": scene, "k": k, "object": nm, "orig_boxes": len(o), "orig_worst_mm": worst,
                        "prop_mm": q[0][1], "n_visible": len(G)})
    return out


def main():
    scenes = sorted(p.name for p in BM.YCBV.iterdir())
    with ProcessPoolExecutor(6) as ex:
        allx = [x for r in ex.map(scene_examples, scenes) for x in r]
    allx.sort(key=lambda x: (-x["orig_worst_mm"] + 10 * x["n_visible"]))
    (DE.OUT / "examples.json").write_text(json.dumps(allx, indent=1))
    for x in allx[:40]:
        print(x)
    print(len(allx), "candidates")


if __name__ == "__main__":
    main()
