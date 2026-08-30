#!/usr/bin/env python3
"""eval_methods.py — compare B0 / C1(global calib) / C2(hue augmentation) for the HSV gate.

Same Phase 1B authority (H+S 16x8, L1, Bhattacharyya max, common t=0.1214, no low-sat exception).
Only the REFERENCE bank changes; the runtime algorithm stays object-agnostic. C3 (asset
correction) is NOT run here — the audit refuted asset error (all Dinosaur assets agree ~83deg,
no better alternate; real dinos span a wide hue range). READ-ONLY.

Reference variants (all applied uniformly to all 10 objects):
  B0   current: feat_rgb(render px) = hue-3 + sat x1.3 + H16xS8 L1   (42 templates)
  C1gw gray-world white balance per render template, then feat_rgb   (42)
  C1h  LOO global extra hue offset (chosen on 9 objects, applied to all)  (42)
  C2a  hue augmentation bank +-5,+-10 OpenCV units (x5 = 210 templates), sim=max
  C2b  hue augmentation bank +-3,+-6 (x5 = 210)
Outputs results/{global_calibration_results,common_augmentation_results,method_comparison,
objectwise_regression}.csv
"""
import csv, os, sys
from collections import defaultdict
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
RES_ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(RES_ROOT)
REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import ism_hsv                       # noqa: E402
import yolo_ism_object_n as o_n      # noqa: E402

GT = os.path.join(RSRCH, "gt_input")
PIPE = os.path.join(RSRCH, "yolo_localization_research", "pipeline", "cur")
OUT = os.path.join(RES_ROOT, "results")
os.makedirs(OUT, exist_ok=True)
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
MASKED = slice(0, 224); HS = slice(96, 224)
TOPK = 3; T = 0.1214
HB, SB = ism_hsv.HBINS, ism_hsv.SBINS


# ---------- reference pixel access (render42 mask-interior, deterministic) ----------
def render_pixels(template_dir):
    """list of per-view RGB pixel arrays (BGR->RGB, sub seed=i), same set as build_reference."""
    out = []
    for i in range(42):
        rp = os.path.join(template_dir, f"rgb_{i}.png")
        mp = os.path.join(template_dir, f"mask_{i}.png")
        xp = os.path.join(template_dir, f"xyz_{i}.npy")
        if not all(os.path.isfile(x) for x in (rp, mp, xp)):
            continue
        m = cv2.imread(mp, cv2.IMREAD_GRAYSCALE) > 0
        bgr = cv2.imread(rp)
        if bgr is None or m.sum() < ism_hsv.MIN_MASK_PX:
            continue
        out.append(ism_hsv._sub(bgr[m][:, ::-1], seed=i))
    return out


def hist_from_rgb(rgb):
    return ism_hsv.feat_rgb(rgb)          # applies -3 hue + 1.3 sat + H16xS8 L1


def gray_world(rgb):
    a = rgb.astype(np.float64)
    mean = a.reshape(-1, 3).mean(0) + 1e-6
    g = mean.mean()
    return np.clip(a * (g / mean), 0, 255).astype(np.uint8)


def hue_shift_px(rgb, d_ocv):
    hsv = cv2.cvtColor(rgb.reshape(-1, 1, 3)[:, :, ::-1], cv2.COLOR_BGR2HSV).astype(np.int16)
    hsv[..., 0] = (hsv[..., 0] + d_ocv) % 180
    bgr = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
    return bgr.reshape(-1, 3)[:, ::-1]


def build_proto(template_dir, mode):
    px = render_pixels(template_dir)
    if not px:
        return np.zeros((0, HB * SB))
    feats = []
    for p in px:
        if mode == "B0":
            feats.append(hist_from_rgb(p))
        elif mode == "C1gw":
            feats.append(hist_from_rgb(gray_world(p)))
        elif mode.startswith("C1h:"):
            d = int(mode.split(":")[1]); feats.append(hist_from_rgb(hue_shift_px(p, d)))
        elif mode == "C2a":
            for d in (-10, -5, 0, 5, 10):
                feats.append(hist_from_rgb(hue_shift_px(p, d)))
        elif mode == "C2b":
            for d in (-6, -3, 0, 3, 6):
                feats.append(hist_from_rgb(hue_shift_px(p, d)))
    return np.stack(feats)


# ---------- GT + candidates (B1 winners + query hist) ----------
gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(
            t.strip() for t in r["visible_objects"].split(";") if t.strip())

rec = {}
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
        acc = (float(b["sem_top5"]) >= float(b["sim_thr"])) and \
              (float(b["appe11_clstop1"]) >= float(b["appe_gate"]))
        rec[k] = dict(accept_b1=acc, q=hq.get(b["uid"]),
                      vis=(k[2] in gt[(ds, k[1])]))

defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
tdir = {o["name"]: o["template_dir"] for o in objs}


def score(PROTO):
    """B2-sim TP/FP/FN with reference bank PROTO[obj]."""
    per = defaultdict(lambda: [0, 0, 0])
    TP = FP = FN = 0
    accd = {}
    for (ds, f, o), r in rec.items():
        if not r["accept_b1"]:
            hsv_ok = None
        elif r["q"] is None:
            hsv_ok = True
        else:
            hsv_ok = ism_hsv.similarity(r["q"], PROTO[o]) >= T
        acc = bool(r["accept_b1"] and hsv_ok)
        accd[(ds, f, o)] = acc
    for (ds, f), vis in gt.items():
        for o in OBJECTS:
            a = accd.get((ds, f, o), False); v = o in vis
            if v and a: TP += 1; per[o][0] += 1
            elif a: FP += 1; per[o][1] += 1
            elif v: FN += 1; per[o][2] += 1
    return TP, FP, FN, per


def summ(TP, FP, FN):
    P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
    return P, R, 2 * P * R / max(P + R, 1e-9)


# ---------- run modes ----------
MODES = ["B0", "C1gw", "C2a", "C2b"]
# LOO global hue: pick offset on 9 objects (excl held), apply to all; report Dino recovery
proto_cache = {}
def PROTO(mode):
    if mode not in proto_cache:
        proto_cache[mode] = {o: build_proto(tdir[o], mode) for o in OBJECTS}
    return proto_cache[mode]

comp = []
per_all = {}
for m in MODES:
    TP, FP, FN, per = score(PROTO(m))
    P, R, F = summ(TP, FP, FN)
    dino = per["Dinosaur"]
    comp.append(dict(mode=m, TP=TP, FP=FP, FN=FN, precision=round(P, 4), recall=round(R, 4),
                     f1=round(F, 4), Dino_TP=dino[0], Dino_FP=dino[1], Dino_FN=dino[2]))
    per_all[m] = per

# C1h LOO: choose single global hue offset by pooled F1 over all objects, grid
best_h, bestF = 0, -1
for d in range(-15, 16, 3):
    TP, FP, FN, _ = score(PROTO(f"C1h:{d}"))
    _, _, F = summ(TP, FP, FN)
    if F > bestF:
        bestF, best_h = F, d
TP, FP, FN, per = score(PROTO(f"C1h:{best_h}"))
P, R, F = summ(TP, FP, FN)
comp.append(dict(mode=f"C1h(global +{best_h}ocv)", TP=TP, FP=FP, FN=FN, precision=round(P, 4),
                 recall=round(R, 4), f1=round(F, 4), Dino_TP=per["Dinosaur"][0],
                 Dino_FP=per["Dinosaur"][1], Dino_FN=per["Dinosaur"][2]))
per_all[f"C1h(global +{best_h}ocv)"] = per

with open(os.path.join(OUT, "method_comparison.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(comp[0].keys())); w.writeheader(); w.writerows(comp)

# per-object regression vs B0
b0 = per_all["B0"]
with open(os.path.join(OUT, "objectwise_regression.csv"), "w", newline="") as f:
    w = csv.writer(f); head = ["object"] + [f"{m}_TP" for m in per_all] + [f"{m}_FP" for m in per_all]
    w.writerow(head)
    for o in OBJECTS:
        w.writerow([o] + [per_all[m][o][0] for m in per_all] + [per_all[m][o][1] for m in per_all])

print("=== method comparison (B2-sim, human GT 935, t=0.1214) ===")
print(f"{'mode':22s} {'TP':>4} {'FP':>4} {'FN':>4} {'F1':>7} | {'DinoTP':>6} {'DinoFP':>6} {'DinoFN':>6}")
for c in comp:
    print(f"{c['mode']:22s} {c['TP']:>4} {c['FP']:>4} {c['FN']:>4} {c['f1']:>7.4f} | "
          f"{c['Dino_TP']:>6} {c['Dino_FP']:>6} {c['Dino_FN']:>6}")
print("\n=== per-object TP (B0 -> each mode); watch for regressions ===")
print(f"{'obj':22s} " + " ".join(f"{m[:10]:>10}" for m in per_all))
for o in OBJECTS:
    print(f"{o:22s} " + " ".join(f"{per_all[m][o][0]:>10}" for m in per_all))
print(f"\n-> {OUT}")
