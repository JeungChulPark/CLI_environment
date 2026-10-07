"""Detector recall on YCB-V frames, per object, with no gates: is the GT object among the proposals
our ISM stage would look at (the object's top_k = 3 boxes, score >= 0.02)?

A GT instance (visib_fract >= 0.1) counts as proposed when one of its object's top-3 boxes has
IoU >= 0.5 with the GT visible box or the GT full box.

  --split val   held-out frames of the 12 YCB-V test videos: not in test_targets_bop19 and at
                least 5 frames away from every listed frame, every 15th, at most 50 per video.
                Used to choose prompts (never the 900 evaluation images).
  --split test  the 900 evaluation images (report only).

Each image runs over its own GT objects (val) or its target objects (test), as in run_ours.py.

    python det_recall.py --spec text:yolov8m-worldv2.pt --split val --out out/x.json
    python det_recall.py --candidates prompt_candidates.json --split val --out out/cand.json
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import paths as P  # noqa: E402
import data as D  # noqa: E402
import proposers as PR  # noqa: E402

FULL = P.YCBV / "full" / "test"
TOPK, THR, IOU = 3, 0.02, 0.5


def val_frames(step=15, gap=5, per_scene=50):
    listed = {}
    for s, i, _ in D.load_targets():
        listed.setdefault(s, []).append(i)
    out = []
    for s in sorted(listed):
        gt = json.load(open(FULL / f"{s:06d}" / "scene_gt.json"))
        ids = sorted(int(k) for k in gt)
        L = np.array(sorted(listed[s]))
        cand = [i for i in ids[::step] if np.min(np.abs(L - i)) >= gap]
        sel = [cand[k] for k in np.linspace(0, len(cand) - 1, min(per_scene, len(cand))).round().astype(int)]
        out += [(s, i) for i in sel]
    return out


_sc = {}


def frame(s, i, split):
    root = (FULL if split == "val" else P.YCBV / "test") / f"{s:06d}"
    if (root, s) not in _sc:
        _sc[(root, s)] = {k: json.load(open(root / f"{k}.json")) for k in ("scene_gt", "scene_gt_info")}
    sc = _sc[(root, s)]
    bgr = cv2.imread(str(root / "rgb" / f"{i:06d}.png"))
    gts = [(g["obj_id"], gi["bbox_visib"], gi["bbox_obj"], gi["visib_fract"])
           for g, gi in zip(sc["scene_gt"][str(i)], sc["scene_gt_info"][str(i)])]
    return bgr, gts


def xyxy(b):
    x, y, w, h = b
    return [x, y, x + w, y + h]


def score(prop, items, split):
    per = {o: [0, 0] for o in range(1, 22)}
    nbox = []
    t = []
    for s, i, oids in items:
        bgr, gts = frame(s, i, split)
        oids = oids or sorted({g[0] for g in gts})
        prop.activate(oids)
        t0 = time.perf_counter()
        dets = prop.predict(bgr, THR)
        t.append(1e3 * (time.perf_counter() - t0))
        nbox.append(len(dets))
        top = {}
        for d in sorted(dets, key=lambda d: -d[2]):
            if len(top.setdefault(d[1], [])) < TOPK:
                top[d[1]].append(d[0])
        for o, bv, bo, vf in gts:
            if o not in oids or vf < 0.1:
                continue
            per[o][1] += 1
            if any(max(PR._iou(b, xyxy(bv)), PR._iou(b, xyxy(bo))) >= IOU for b in top.get(o, [])):
                per[o][0] += 1
    hit = sum(v[0] for v in per.values()); n = sum(v[1] for v in per.values())
    return {"recall": round(100 * hit / max(n, 1), 1), "hit": hit, "n": n,
            "boxes_per_image": round(float(np.mean(nbox)), 2), "det_ms_median": round(float(np.median(t)), 1),
            "per_object": {P.YCB_NAMES[o - 1]: {"recall": round(100 * v[0] / v[1], 1) if v[1] else None, "n": v[1]}
                           for o, v in per.items()}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spec", action="append", default=[])
    ap.add_argument("--candidates", default="", help="json obj_id -> [prompts]: recall of every prompt alone")
    ap.add_argument("--weights", default="yolov8m-worldv2.pt")
    ap.add_argument("--split", choices=["val", "test"], default="val")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    items = ([(s, i, None) for s, i in val_frames()] if a.split == "val" else D.load_targets())
    print(f"{a.split}: {len(items)} images", flush=True)
    res = {"split": a.split, "images": len(items), "rule": f"top-{TOPK} boxes per object, score >= {THR}, IoU >= {IOU}",
           "label": P.LABEL, "results": {}}
    for spec in a.spec:
        prop = PR.build(spec)
        res["results"][spec] = r = score(prop, items, a.split)
        print(spec, r["recall"], r["boxes_per_image"], r["det_ms_median"], flush=True)
        del prop
    if a.candidates:
        cand = {int(k): v for k, v in json.load(open(a.candidates)).items()}
        n_max = max(len(v) for v in cand.values())
        res["candidates"] = {}
        for j in range(n_max):      # round j: every object uses its j-th candidate (or its first)
            pm = {o: [v[j] if j < len(v) else v[0]] for o, v in cand.items()}
            prop = PR.TextProposer(a.weights, pm)
            r = score(prop, items, a.split)
            for o, v in cand.items():
                if j < len(v):
                    res["candidates"].setdefault(str(o), {})[v[j]] = r["per_object"][P.YCB_NAMES[o - 1]]["recall"]
            print(f"round {j}: {r['recall']}", flush=True)
            del prop
    json.dump(res, open(a.out, "w"), indent=1)
    print("->", a.out)


if __name__ == "__main__":
    main()
