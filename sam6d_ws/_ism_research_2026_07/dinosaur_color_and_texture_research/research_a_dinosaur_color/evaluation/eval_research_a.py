#!/usr/bin/env python3
"""eval_research_a.py — Dinosaur-ONLY template color correction. READ-ONLY, offline.

Only Dinosaur's 42-view reference is corrected; the other 9 objects' proto is untouched, so
their accept decisions are IDENTICAL by construction -> total delta = Dinosaur delta only.

Correction is a single fixed transform on the Dinosaur render pixels BEFORE feat_rgb
(which still applies the shared -3 OpenCV hue + x1.3 sat). Runtime HSV algorithm/threshold
unchanged (t=0.12140). Variants:
  A0 current ; A1 hue shift d ; A2 hue d + sat s ; A4 hue histogram-match to real calib
Calibration = choose the transform on 5 datasets (valid visible Dino), test held-out (LODO).
"""
import csv, os, sys
from collections import defaultdict
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
RA = os.path.dirname(HERE); ROOT = os.path.dirname(RA)
RSRCH = os.path.dirname(ROOT); REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import ism_hsv                       # noqa: E402
import yolo_ism_object_n as o_n      # noqa: E402

GT = os.path.join(RSRCH, "gt_input")
PIPE = os.path.join(RSRCH, "yolo_localization_research", "pipeline", "cur")
OUT = os.path.join(RA, "results"); os.makedirs(OUT, exist_ok=True)
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
MASKED = slice(0, 224); HS = slice(96, 224); TOPK = 3; T = 0.1214

defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
tdir = {o["name"]: o["template_dir"] for o in objs}
BASE = {}
for o in objs:
    BASE[o["name"]] = ism_hsv.load_cache(
        os.path.join(os.path.dirname(o["cls_cache"]), f"{o['name']}_hsv.npz"), o["template_dir"])[0]


def dino_pixels():
    px = []
    for i in range(42):
        rp = os.path.join(tdir["Dinosaur"], f"rgb_{i}.png")
        mp = os.path.join(tdir["Dinosaur"], f"mask_{i}.png")
        xp = os.path.join(tdir["Dinosaur"], f"xyz_{i}.npy")
        if not all(os.path.isfile(x) for x in (rp, mp, xp)):
            continue
        m = cv2.imread(mp, cv2.IMREAD_GRAYSCALE) > 0
        bgr = cv2.imread(rp)
        if bgr is None or m.sum() < ism_hsv.MIN_MASK_PX:
            continue
        px.append(ism_hsv._sub(bgr[m][:, ::-1], seed=i))
    return px


DPX = dino_pixels()


def transform(rgb, dh=0, sat=1.0):
    hsv = cv2.cvtColor(rgb.reshape(-1, 1, 3)[:, :, ::-1], cv2.COLOR_BGR2HSV).astype(np.int16)
    hsv[..., 0] = (hsv[..., 0] + dh) % 180
    hsv[..., 1] = np.clip(hsv[..., 1] * sat, 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR).reshape(-1, 3)[:, ::-1]


def dino_proto(dh=0, sat=1.0):
    return np.stack([ism_hsv.feat_rgb(transform(p, dh, sat)) for p in DPX])


# ---- GT + candidates (only need Dino candidates + all-object accept for totals) ----
gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(
            t.strip() for t in r["visible_objects"].split(";") if t.strip())

rec = {}                    # (ds,f,obj) -> dict(accept_b1,q,vis)
for ds in DATASETS:
    z = np.load(os.path.join(PIPE, f"{ds}_hsv.npz"))
    hq = {str(u): h[MASKED][HS].astype(np.float64) for u, h in zip(z["uid"], z["hist"])}
    rows = defaultdict(list)
    for r in csv.DictReader(open(os.path.join(PIPE, f"{ds}_pairs.csv"))):
        if (ds, int(r["frame_id"])) not in gt:
            continue
        rows[(ds, int(r["frame_id"]), r["object"])].append(r)
    for k, cs in rows.items():
        v = [c for c in cs if int(c["routed"]) == 1 and float(c["yolo_conf"]) >= 0.02]
        v.sort(key=lambda c: -float(c["yolo_conf"])); v = v[:TOPK]
        if not v:
            continue
        b = max(v, key=lambda c: float(c["sem_top5"]))
        acc = (float(b["sem_top5"]) >= float(b["sim_thr"])) and (float(b["appe11_clstop1"]) >= float(b["appe_gate"]))
        rec[k] = dict(accept_b1=acc, q=hq.get(b["uid"]), vis=(k[2] in gt[(ds, k[1])]))


def accept(o, r, proto):
    if not r["accept_b1"]:
        return False
    if r["q"] is None:
        return True
    return ism_hsv.similarity(r["q"], proto) >= T


def score(dino_p, frames=None):
    frames = frames or list(gt)
    fset = set(frames)
    TP = FP = FN = 0; d = [0, 0, 0]
    for (ds, f), vis in gt.items():
        if (ds, f) not in fset:
            continue
        for o in OBJECTS:
            r = rec.get((ds, f, o))
            proto = dino_p if o == "Dinosaur" else BASE[o]
            a = accept(o, r, proto) if r else False
            v = o in vis
            if v and a:
                TP += 1
                if o == "Dinosaur": d[0] += 1
            elif a:
                FP += 1
                if o == "Dinosaur": d[1] += 1
            elif v:
                FN += 1
                if o == "Dinosaur": d[2] += 1
    return TP, FP, FN, d


# ---- real Dino hue stats (calibration) ----
def dino_real_hue(frames):
    hs = []
    hcent = (np.arange(16) + 0.5) * (180 / 16)
    for (ds, f, o), r in rec.items():
        if o != "Dinosaur" or not r["vis"] or not r["accept_b1"] or r["q"] is None:
            continue
        if (ds, f) not in frames:
            continue
        hm = r["q"].reshape(16, 8).sum(1)
        if hm.sum() > 0:
            hs.append(float((hcent * hm).sum() / hm.sum()))
    return hs


# baseline (A0) on all
tp0, fp0, fn0, d0 = score(BASE["Dinosaur"])
print(f"A0 baseline: TP {tp0} FP {fp0} FN {fn0} | Dino TP/FP/FN {d0}")

# ---- LODO sweeps ----
def lodo(kind):
    """kind in {'A1','A2','A4'}. Returns pooled held-out totals + Dino."""
    TP = FP = FN = 0; d = [0, 0, 0]; picks = []
    for held in DATASETS:
        trf = set(x for x in gt if x[0] != held)
        tef = [x for x in gt if x[0] == held]
        # choose transform on calibration (train frames), objective = Dino F1 there
        best, bp = -1, (0, 1.0)
        if kind == "A1":
            grid = [(dh, 1.0) for dh in range(-2, 20)]
        elif kind == "A2":
            grid = [(dh, s) for dh in range(0, 18, 2) for s in (0.9, 1.0, 1.1, 1.3)]
        else:  # A4 hue histogram match: shift = real_mean - template_mean (learned on calib)
            real = dino_real_hue(trf)
            tmpl = []
            hcent = (np.arange(16) + 0.5) * (180 / 16)
            for p in DPX:
                h = ism_hsv.feat_rgb(p).reshape(16, 8).sum(1)
                tmpl.append(float((hcent * h).sum() / max(h.sum(), 1e-9)))
            dh = int(round(np.mean(real) - np.mean(tmpl))) if real else 0
            grid = [(dh, 1.0)]
        for dh, s in grid:
            _, _, _, dd = score(dino_proto(dh, s), trf)
            tpc = dd[0]; f1 = 2 * tpc / max(2 * tpc + dd[1] + dd[2], 1)
            if f1 > best:
                best, bp = f1, (dh, s)
        t, f, n, dd = score(dino_proto(*bp), tef)
        TP += t; FP += f; FN += n
        for i in range(3): d[i] += dd[i]
        picks.append((held, bp))
    return TP, FP, FN, d, picks


rows_out = [dict(variant="A0", TP=tp0, FP=fp0, FN=fn0, DinoTP=d0[0], DinoFP=d0[1], DinoFN=d0[2],
                 f1=round(2 * tp0 / max(2 * tp0 + fp0 + fn0, 1), 4), pick="-")]
for kind in ("A1", "A2", "A4"):
    TP, FP, FN, d, picks = lodo(kind)
    rows_out.append(dict(variant=kind, TP=TP, FP=FP, FN=FN, DinoTP=d[0], DinoFP=d[1], DinoFN=d[2],
                         f1=round(2 * TP / max(2 * TP + FP + FN, 1), 4),
                         pick=";".join(f"{h[3:]}:{p}" for h, p in picks)))

with open(os.path.join(OUT, "dinosaur_color_final_comparison.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows_out[0].keys())); w.writeheader(); w.writerows(rows_out)

print("\n=== Research A: Dino-only template correction (LODO held-out pooled, t=0.1214) ===")
print(f"{'var':4s} {'TP':>4} {'FP':>4} {'FN':>4} {'F1':>7} | {'DinoTP':>6} {'DinoFP':>6} {'DinoFN':>6}")
for r in rows_out:
    print(f"{r['variant']:4s} {r['TP']:>4} {r['FP']:>4} {r['FN']:>4} {r['f1']:>7.4f} | "
          f"{r['DinoTP']:>6} {r['DinoFP']:>6} {r['DinoFN']:>6}")
print("picks:", {r['variant']: r['pick'] for r in rows_out if r['variant'] != 'A0'})
# real hue stats
allhs = dino_real_hue(set(gt))
print(f"\nreal Dino hue (ocv) mean {np.mean(allhs):.1f} median {np.median(allhs):.1f} "
      f"p10 {np.percentile(allhs,10):.1f} p90 {np.percentile(allhs,90):.1f} n {len(allhs)}")
with open(os.path.join(OUT, "dinosaur_real_hsv_statistics.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["metric", "value"])
    for k, v in [("mean_H_ocv", np.mean(allhs)), ("median", np.median(allhs)),
                 ("p10", np.percentile(allhs, 10)), ("p90", np.percentile(allhs, 90)), ("n", len(allhs))]:
        w.writerow([k, round(float(v), 2)])
print(f"-> {OUT}")
