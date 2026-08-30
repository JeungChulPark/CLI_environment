#!/usr/bin/env python3
"""audit_dinosaur_color.py — root-cause the Dinosaur HSV domain gap. READ-ONLY.

Compares the OpenCV-HSV Hue/Sat/Val of every Dinosaur color source:
  operational PLY vertex colors, alternate PLYs (high/middle/low), the texture PNG,
  the 42 render templates, and the REAL camera crops (passing vs HSV-rejected).
Decides root cause A-G and whether an alternate asset is "more correct".
"""
import csv, os, sys
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
RES_ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(RES_ROOT)
REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(RSRCH, "ism_fusion_research", "ply_hsv"))
from build_prototype_colors import load_ply     # noqa: E402
import ism_hsv                                   # noqa: E402

OUT = os.path.join(RES_ROOT, "results")


def hsv_stats(rgb):
    """rgb [N,3] uint8 -> OpenCV HSV mean/median (H 0-180, S/V 0-255) + hue in 0-360."""
    rgb = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)
    hsv = cv2.cvtColor(rgb[:, :, ::-1], cv2.COLOR_BGR2HSV).reshape(-1, 3).astype(np.float64)
    # circular mean hue (0-180)
    ang = hsv[:, 0] * 2 * np.pi / 180.0
    hmean = (np.arctan2(np.sin(ang).mean(), np.cos(ang).mean()) % (2 * np.pi)) * 180 / (2 * np.pi)
    return dict(H_ocv=round(float(hmean), 1), H_deg=round(float(hmean * 2), 1),
                H_med=round(float(np.median(hsv[:, 0])), 1),
                S=round(float(hsv[:, 1].mean()), 1), V=round(float(hsv[:, 2].mean()), 1),
                n=len(hsv))


rows = []
# --- 1. PLYs ---
PLYS = {
    "operational_PLY": "data/cad/Dinosaur/Dinosaur_color_with_normal_vertexcolor.ply",
    "alt_high": "data/ply_files_excluding_20260702_extracted/Dinosaur_high.ply",
    "alt_middle": "data/ply_files_excluding_20260702_extracted/Dinosaur_middle.ply",
    "alt_low": "data/ply_files_excluding_20260702_extracted/Dinosaur_low.ply",
    "operational_PLY_orig_meter": "data/cad/Dinosaur/Dinosaur_color_with_normal_vertexcolor.ply.orig_meter",
}
for name, rel in PLYS.items():
    p = os.path.join(REPO, rel)
    if not os.path.isfile(p):
        rows.append(dict(source=name, note="MISSING")); continue
    V, C = load_ply(p)
    st = hsv_stats(C)
    rows.append(dict(source=name, **st))

# --- 2. texture png ---
tex = os.path.join(REPO, "data/cad/Dinosaur/Dinosaur_color.png")
if os.path.isfile(tex):
    img = cv2.imread(tex)                       # BGR
    px = img.reshape(-1, 3)
    # drop near-white/near-black background of a texture atlas
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    keep = (hsv[:, 1] > 40) & (hsv[:, 2] > 40)
    rgb = px[keep][:, ::-1]
    rows.append(dict(source="texture_png(sat>40)", **hsv_stats(rgb)))

# --- 3. render42 templates (what the renderer produced) ---
PC = np.load(os.path.join(RSRCH, "ism_fusion_research", "ply_hsv", "prototype_colors.npz"))
rend = PC["Dinosaur__render42"].reshape(-1, 3)   # RGB
rows.append(dict(source="render42_templates", **hsv_stats(rend)))
plyview = PC["Dinosaur__ply_view42"].reshape(-1, 3)
rows.append(dict(source="ply_view42(visible verts)", **hsv_stats(plyview)))

# --- 4. REAL camera crops: passing vs rejected Dinosaur ---
# use the shadow decisions to split, and re-extract real crop HSV from pipeline query hists
DEC = os.path.join(RSRCH, "phase1b_validation", "shadow_decisions.csv")
PIPE = os.path.join(RSRCH, "yolo_localization_research", "pipeline", "cur")
MASKED = slice(0, 224); HS = slice(96, 224)
# H bins: 16 over [0,180] -> centers
hcent = (np.arange(16) + 0.5) * (180 / 16)
split = {"real_Dino_pass(TP,hsv>=t)": [], "real_Dino_reject(would_reject)": []}
uid_by = {}
for ds in ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]:
    z = np.load(os.path.join(PIPE, f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        uid_by[str(u)] = h[MASKED][HS].astype(np.float64)   # 128 = 16H x 8S
# map (ds,frame,box)->uid via pairs
uid_of = {}
for ds in ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]:
    for r in csv.DictReader(open(os.path.join(PIPE, f"{ds}_pairs.csv"))):
        if r["object"] == "Dinosaur":
            x = r["uid"].split("|")[2]
            uid_of[(ds, int(r["frame_id"]), x)] = r["uid"]
for r in csv.DictReader(open(DEC)):
    if r["object"] != "Dinosaur" or r["visible"] != "1":
        continue
    key = (r["dataset"], int(r["frame_id"]), f"{r['x1']}_{r['y1']}_{r['x2']}_{r['y2']}")
    uid = uid_of.get(key)
    if uid is None or uid not in uid_by:
        continue
    hist = uid_by[uid].reshape(16, 8)
    hmarg = hist.sum(1)                       # marginal over S
    if hmarg.sum() <= 0:
        continue
    hbar = float((hcent * hmarg).sum() / hmarg.sum())    # weighted mean H (0-180)
    tgt = "real_Dino_reject(would_reject)" if r["would_hsv_reject"] == "1" else "real_Dino_pass(TP,hsv>=t)"
    if r["b1_accept"] == "1":
        split[tgt].append(hbar)
for name, hs in split.items():
    if hs:
        rows.append(dict(source=name, H_ocv=round(float(np.mean(hs)), 1),
                         H_deg=round(float(np.mean(hs) * 2), 1), H_med=round(float(np.median(hs)), 1),
                         S="", V="", n=len(hs)))

os.makedirs(OUT, exist_ok=True)
cols = ["source", "H_ocv", "H_deg", "H_med", "S", "V", "n", "note"]
with open(os.path.join(OUT, "dinosaur_asset_audit.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=cols, restval=""); w.writeheader(); w.writerows(rows)

print("=== Dinosaur color audit (OpenCV Hue 0-180; H_deg = 0-360 conventional) ===")
print(f"{'source':34s} {'H_ocv':>6} {'H_deg':>7} {'H_med':>6} {'S':>6} {'V':>6} {'n':>8}")
for r in rows:
    print(f"{r['source']:34s} {str(r.get('H_ocv','')):>6} {str(r.get('H_deg','')):>7} "
          f"{str(r.get('H_med','')):>6} {str(r.get('S','')):>6} {str(r.get('V','')):>6} {str(r.get('n','')):>8}")
print(f"\n-> {OUT}/dinosaur_asset_audit.csv")
