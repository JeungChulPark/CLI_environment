#!/usr/bin/env python3
"""fp_all21.py — object-level false detections when every YCB-V object (21) is searched.

Per BOP-19 test image: predicted object set = objects with at least one estimate (each method's
own output threshold), GT set = objects annotated in the image with visib_fract >= 0.1.
Reports absent-object detections per image, object precision and object recall."""
import json
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[3]
OUT = REPO / "objpose/output/cmp_260924"
Y = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video/test")
res = {}
for m in ("proposed", "orig"):
    fp = tp = fn = n = 0; fp_img = []
    for sc in sorted(p.name for p in Y.iterdir()):
        gt = json.loads((Y / sc / "scene_gt.json").read_text()); info = json.loads((Y / sc / "scene_gt_info.json").read_text())
        fr = [json.loads(l) for l in (OUT / f"{m}_ycbv_bop19_all21" / sc / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()]
        names = {o["obj_id"]: o["name"] for o in json.loads((REPO / "objpose/rt/bop/ycbv_objects.json").read_text())["objects"]}
        for f in fr:
            k = str(f["im_id"])
            g = {names[r["obj_id"]] for r, i in zip(gt[k], info[k]) if i.get("visib_fract", 1) >= 0.1}
            p = {d["object"] for d in f["dets"]}
            fp += len(p - g); tp += len(p & g); fn += len(g - p); n += 1; fp_img.append(len(p - g))
    res[m] = {"images": n, "absent_det_per_img": fp / n, "images_with_absent_det": float(np.mean(np.array(fp_img) > 0)),
              "obj_precision": tp / max(tp + fp, 1), "obj_recall": tp / max(tp + fn, 1)}
    print(m, res[m])
(OUT / "fp_all21.json").write_text(json.dumps(res, indent=1))

# threshold sweep for the original: keep an object if its best estimate score >= tau
def sweep(m):
    rows = []
    items = []   # (score, is_present) per (image, object) with any estimate
    ngt = 0
    names = {o["obj_id"]: o["name"] for o in json.loads((REPO / "objpose/rt/bop/ycbv_objects.json").read_text())["objects"]}
    for sc in sorted(p.name for p in Y.iterdir()):
        gt = json.loads((Y / sc / "scene_gt.json").read_text()); info = json.loads((Y / sc / "scene_gt_info.json").read_text())
        for l in (OUT / f"{m}_ycbv_bop19_all21" / sc / "sam6d_frames.jsonl").read_text().splitlines():
            if not l.strip():
                continue
            f = json.loads(l); k = str(f["im_id"])
            g = {names[r["obj_id"]] for r, i in zip(gt[k], info[k]) if i.get("visib_fract", 1) >= 0.1}
            ngt += len(g)
            best = {}
            for d in f["dets"]:
                best[d["object"]] = max(best.get(d["object"], -1), d["score"])
            items += [(s, o in g) for o, s in best.items()]
    s = np.array([x[0] for x in items]); t = np.array([x[1] for x in items])
    for tau in np.linspace(0, 0.9, 91):
        k = s >= tau; tp = int((k & t).sum()); fp = int((k & ~t).sum())
        rows.append({"tau": round(float(tau), 3), "precision": tp / max(tp + fp, 1), "recall": tp / ngt, "absent_per_img": fp / 900})
    return rows
sw = sweep("orig")
(OUT / "fp_all21_orig_sweep.json").write_text(json.dumps(sw, indent=1))
f1 = max(sw, key=lambda r: 2 * r["precision"] * r["recall"] / max(r["precision"] + r["recall"], 1e-9))
print("orig best-F1", f1)
p = res["proposed"]
same_p = min((r for r in sw if r["precision"] >= p["obj_precision"]), key=lambda r: r["tau"], default=None)
print("orig at proposed precision", same_p)
same_r = max((r for r in sw if r["recall"] >= p["obj_recall"]), key=lambda r: r["tau"], default=None)
print("orig at proposed recall", same_r)
