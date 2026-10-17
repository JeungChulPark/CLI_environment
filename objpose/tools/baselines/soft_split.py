#!/usr/bin/env python3
"""soft_split.py — counts for the verification figure: per step, correct / wrong / not answered
(GT instances, visib >= 0.1), and for the score-based variant, split by verified vs fallback.
Also lists example (scene, im_id, object) cases per category for the image slide."""
import json, os
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
OUT = Path(os.environ["CMP_OUT"])
DS = {"ycbv": ("objpose/rt/bop/ycbv_objects.json", "/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video"),
      "lmo": ("objpose/rt/bop/lmo_objects.json", "/home/jucpark/DeepLearning/Datasets/paper_benchmarks/bop/lmo")}
PTS = REPO / "sam6d_realtime/assets/model_points"
steps = ["orig", "proposednoverify", "proposednogate", "proposedsoft", "proposed", "proposedscore"]
res = {}
for ds, (tab, root) in DS.items():
    table = json.loads((REPO / tab).read_text()); name = {o["obj_id"]: o["name"] for o in table["objects"]}
    info = json.loads((Path(root) / "models_eval/models_info.json").read_text())
    objs = {}
    for oid, nm in name.items():
        p = np.load(PTS / f"{nm}.npy").astype(float)
        if len(p) > 2000: p = p[np.random.default_rng(0).choice(len(p), 2000, replace=False)]
        objs[nm] = (p, cKDTree(p), 0.1 * info[str(oid)]["diameter"])
    judged = {}
    for st in steps:
        run = OUT / f"{st}_{ds}_bop19"
        c = {"correct": 0, "wrong": 0, "none": 0, "correct_low": 0, "wrong_low": 0}
        J = {}
        for sc in sorted(p for p in run.iterdir() if p.is_dir()):
            scene = Path(root) / "test" / sc.name
            gt = json.loads((scene / "scene_gt.json").read_text()); gi = json.loads((scene / "scene_gt_info.json").read_text())
            F = {json.loads(l)["im_id"]: json.loads(l) for l in (sc / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()}
            for im, f in F.items():
                k = str(im)
                best = {}
                for d in f["dets"]:
                    if d["object"] not in best or d["score"] > best[d["object"]]["score"]: best[d["object"]] = d
                for r, i in zip(gt[k], gi[k]):
                    if i.get("visib_fract", 1) < 0.1 or r["obj_id"] not in name: continue
                    nm = name[r["obj_id"]]; d = best.get(nm)
                    if d is None:
                        c["none"] += 1; J[(sc.name, im, nm)] = "none"; continue
                    T = np.eye(4); T[:3, :3] = np.reshape(r["cam_R_m2c"], (3, 3)); T[:3, 3] = r["cam_t_m2c"]
                    E = np.eye(4); E[:3, :3] = np.reshape(d["R"], (3, 3)); E[:3, 3] = d["t_mm"]
                    p, tree, thr = objs[nm]; D = np.linalg.inv(T) @ E
                    ok = tree.query(p @ D[:3, :3].T + D[:3, 3])[0].mean() < thr
                    low = (d.get("score", 0) < 3.0) if "gate_pass" in d else (d.get("verified") is False)
                    key = ("correct" if ok else "wrong") + ("_low" if low else "")
                    c[key] += 1; J[(sc.name, im, nm)] = key + ("|%.3f" % i.get("visib_fract", 1))
        res[f"{st}_{ds}"] = c; judged[st] = J
        print(ds, st, c)
    # examples: rejected by pose verification (answered by noverify, none in nogate)
    ex = {"rejected_correct": [], "rejected_wrong": [], "kept_correct": []}
    for key, jn in judged["proposednoverify"].items():
        jv = judged["proposednogate"].get(key)
        cat = None
        if jv == "none" and jn.startswith("correct"): cat = "rejected_correct"
        elif jv == "none" and jn.startswith("wrong"): cat = "rejected_wrong"
        elif jv and jv.startswith("correct") and jn.startswith("correct"): cat = "kept_correct"
        if cat: ex[cat].append([*key, float(jn.split("|")[1]) if "|" in jn else None])
    # visibility of rejected-correct vs kept-correct
    vis = {k: float(np.median([e[3] for e in v])) if v else None for k, v in ex.items()}
    print(ds, "counts", {k: len(v) for k, v in ex.items()}, "median visib", vis)
    res[f"examples_{ds}"] = {k: v[:400] for k, v in ex.items()}
    res[f"visib_{ds}"] = vis
(OUT / "soft_split.json").write_text(json.dumps(res, indent=1))
