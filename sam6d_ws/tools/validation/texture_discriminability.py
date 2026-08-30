#!/usr/bin/env python3
"""RGB Texture Discriminability Check (구조변경 전 마지막 검증). 읽기전용·코드 미수정.

질문: milk template의 빨강/파랑/텍스처 색신호를 쓰면 흰 박스/벽/모니터 FP를 가를 수 있나?
데이터: baseline(315 풀-GT 자기정합) frames/<fid>.png + _run/<fid>/detection_pem.json(bbox+RLE+score)
       + frame_results(decision_type) + 42 템플릿(rgb/mask).
"""
import csv, json, os, math
import numpy as np
import cv2
from pycocotools import mask as maskutil

BASE = "outputs/validation/SLAM_with_milk_nomilk_baseline_bak"
FRAMES = f"{BASE}/frames"
RUN = f"{BASE}/_run"
FR = f"{BASE}/frame_results.csv"
TPL = "sam6d_master/SAM-6D/Data/custom/Milk_scaled_195mm/templates"
VIS_OUT = "outputs/validation/texture_audit_vis"
os.makedirs(VIS_OUT, exist_ok=True)
np.random.seed(1)

H_BINS, S_BINS = 30, 32


# ---------- feature extraction on a BGR image + boolean mask ----------
def feats(bgr, m):
    if m.sum() < 20:
        return None
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    H = hsv[..., 0].astype(np.float32)        # 0-179
    S = hsv[..., 1].astype(np.float32) / 255   # 0-1
    V = hsv[..., 2].astype(np.float32) / 255
    fg = m > 0
    Hf, Sf, Vf = H[fg], S[fg], V[fg]
    # red: H near 0 or 179, colorful, bright
    red = ((((Hf < 12) | (Hf > 168)) & (Sf > 0.35) & (Vf > 0.25)).mean())
    # blue: H ~ 100-130
    blue = (((Hf >= 95) & (Hf <= 135) & (Sf > 0.30) & (Vf > 0.20)).mean())
    sat_mean, sat_std = float(Sf.mean()), float(Sf.std())
    # colorfulness (Hasler-Susstrunk) on fg pixels
    b, g, r = bgr[..., 0].astype(np.float32), bgr[..., 1].astype(np.float32), bgr[..., 2].astype(np.float32)
    rg = (r - g)[fg]; yb = (0.5 * (r + g) - b)[fg]
    colorful = float(math.sqrt(rg.std()**2 + yb.std()**2) + 0.3 * math.sqrt(rg.mean()**2 + yb.mean()**2))
    # edge density (Canny within mask)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150) > 0
    edge_density = float((edges & fg).sum() / max(1, fg.sum()))
    # HSV 2D hist (H,S) normalized
    hist = cv2.calcHist([hsv], [0, 1], m.astype(np.uint8), [H_BINS, S_BINS], [0, 180, 0, 256])
    hist = cv2.normalize(hist, hist).flatten()
    return dict(red=red, blue=blue, sat_mean=sat_mean, sat_std=sat_std,
                colorful=colorful, edge=edge_density, hist=hist,
                rb=red + blue)


# ---------- Task1: template audit → milk reference ----------
tpl_feats = []
tpl_hist_accum = None
for i in range(42):
    rp = f"{TPL}/rgb_{i}.png"; mp = f"{TPL}/mask_{i}.png"
    if not (os.path.isfile(rp) and os.path.isfile(mp)):
        continue
    bgr = cv2.imread(rp); m = cv2.imread(mp, cv2.IMREAD_GRAYSCALE)
    if bgr is None or m is None:
        continue
    fe = feats(bgr, m)
    if fe is None:
        continue
    tpl_feats.append(fe)
    tpl_hist_accum = fe["hist"] if tpl_hist_accum is None else tpl_hist_accum + fe["hist"]
ref_hist = cv2.normalize(tpl_hist_accum, tpl_hist_accum).flatten()
def agg(k): return np.array([t[k] for t in tpl_feats])
print(f"=== [Task1] TEMPLATE RGB TEXTURE AUDIT (n={len(tpl_feats)}) ===")
for k in ("red", "blue", "rb", "sat_mean", "colorful", "edge"):
    v = agg(k)
    print(f"  {k:9s}: mean={v.mean():.3f} std={v.std():.3f} [{v.min():.3f},{v.max():.3f}]")
print("  → 템플릿에 빨강/파랑/채도/엣지 신호가 존재하는지 위 분포로 확인.")


# ---------- load GT + crops ----------
gt = {}
for r in csv.DictReader(open(FR)):
    if r["manually_labeled_milk_visible"] in ("True", "False"):
        gt[r["frame_id"]] = dict(
            y=1 if r["manually_labeled_milk_visible"] == "True" else 0,
            dt=r["decision_type"],
            appe=float(r["mask_score"]) if r["mask_score"] else None,
            sem=float(r["similarity_score"]) if r["similarity_score"] else None,
            geo=float(r["bbox_score"]) if r["bbox_score"] else None)

def crop_feats_for_frame(fid):
    """best(det[0]) candidate 의 mask 영역 색/텍스처 feature."""
    img = f"{FRAMES}/{fid}.png"
    dj = f"{RUN}/{fid}/detection_pem.json"
    if not (os.path.isfile(img) and os.path.isfile(dj)):
        return None
    dets = json.load(open(dj))
    if not dets:
        return None
    det = max(dets, key=lambda d: d.get("score", -1))
    bgr = cv2.imread(img)
    if bgr is None:
        return None
    seg = det.get("segmentation")
    try:
        if isinstance(seg, dict):
            rle = dict(seg)
            if isinstance(rle.get("counts"), list):
                rle = maskutil.frPyObjects(rle, rle["size"][0], rle["size"][1])
            m = maskutil.decode(rle)
        else:
            return None
    except Exception:
        return None
    if m.shape[:2] != bgr.shape[:2]:
        return None
    fe = feats(bgr, m)
    if fe is None:
        return None
    fe["hist_sim"] = float(cv2.compareHist(fe["hist"].astype(np.float32),
                                            ref_hist.astype(np.float32), cv2.HISTCMP_CORREL))
    return fe, det, bgr, m

# build per-frame table
rows = []
for fid, g in gt.items():
    cf = crop_feats_for_frame(fid)
    if cf is None:
        continue
    fe = cf[0]
    rows.append(dict(fid=fid, **g,
                     red=fe["red"], blue=fe["blue"], rb=fe["rb"],
                     sat=fe["sat_mean"], colorful=fe["colorful"],
                     edge=fe["edge"], hist_sim=fe["hist_sim"]))
print(f"\n=== crops analyzed: {len(rows)} frames ===")
dtc = {}
for r in rows: dtc[r["dt"]] = dtc.get(r["dt"], 0) + 1
print("  decision_type:", dtc)


# ---------- AUC helpers ----------
def auc(score, lab):
    pos = [s for s, l in zip(score, lab) if l == 1]
    neg = [s for s, l in zip(score, lab) if l == 0]
    if not pos or not neg: return float("nan")
    sv = sorted(pos + neg); n = len(sv); v2r = {}; j = 0
    while j < n:
        k = j
        while k + 1 < n and sv[k + 1] == sv[j]: k += 1
        v2r[sv[j]] = (j + k) / 2 + 1; j = k + 1
    R = sum(v2r[s] for s in pos); U = R - len(pos) * (len(pos) + 1) / 2
    a = U / (len(pos) * len(neg))
    return max(a, 1 - a)  # direction-agnostic separation power

def pr_auc(score, lab, flip=False):
    s = [-x for x in score] if flip else list(score)
    order = sorted(range(len(s)), key=lambda i: -s[i])
    tp = fp = 0; Pt = sum(lab); area = 0; pr = 0
    for i in order:
        if lab[i] == 1: tp += 1
        else: fp += 1
        prec = tp / (tp + fp); rec = tp / Pt
        area += (rec - pr) * prec; pr = rec
    return area

y = [r["y"] for r in rows]                 # present=1
isfp = [1 if r["dt"] == "FP" else 0 for r in rows]
istp = [1 if r["dt"] == "TP" else 0 for r in rows]
# TP-vs-FP mask
tpfp_idx = [i for i, r in enumerate(rows) if r["dt"] in ("TP", "FP")]
tpfp_lab = [1 if rows[i]["dt"] == "TP" else 0 for i in tpfp_idx]

print("\n=== [Task3] TEXTURE SCORE SEPARATION (AUC, direction-agnostic) ===")
print(f"  {'score':12s} | present-vs-absent | TP-vs-FP")
texture_scores = ["red", "blue", "rb", "sat", "colorful", "edge", "hist_sim"]
for k in texture_scores + ["appe", "sem", "geo"]:
    sc = [r[k] for r in rows]
    a_pa = auc(sc, y)
    a_tf = auc([rows[i][k] for i in tpfp_idx], tpfp_lab)
    print(f"  {k:12s} | {a_pa:.3f}             | {a_tf:.3f}")


# ---------- Task4: complementarity (correlation + combined logreg AUC) ----------
def zscore(a):
    a = np.array(a, float); return (a - a.mean()) / (a.std() + 1e-9)

print("\n=== [Task4] COMPLEMENTARITY vs appearance ===")
appe = np.array([r["appe"] for r in rows], float)
for k in ["rb", "colorful", "edge", "hist_sim", "sat"]:
    sc = np.array([r[k] for r in rows], float)
    c = float(np.corrcoef(appe, sc)[0, 1])
    print(f"  corr(appe, {k:9s}) = {c:+.3f}")

# combined logreg (in-sample AUC, 참고) : appe vs appe+best_texture
from scipy.optimize import minimize
def logreg_auc(Xcols, lab):
    X = np.column_stack([zscore([r[c] for r in rows]) for c in Xcols])
    Xb = np.hstack([np.ones((len(X), 1)), X]); yv = np.array(lab, float)
    def nll(w):
        z = Xb @ w; return -np.sum(yv * z - np.logaddexp(0, z)) + 1.0 * np.sum(w[1:]**2)
    w = minimize(nll, np.zeros(Xb.shape[1]), method="L-BFGS-B").x
    p = 1 / (1 + np.exp(-(Xb @ w)))
    return auc(list(p), lab), dict(zip(["int"] + Xcols, w.round(3)))
best_tex = max(["rb", "colorful", "edge", "hist_sim"], key=lambda k: auc([r[k] for r in rows], y))
print(f"  best texture score (present-vs-absent) = {best_tex}")
a_app, _ = logreg_auc(["appe"], y)
a_tex, _ = logreg_auc([best_tex], y)
a_both, w_both = logreg_auc(["appe", best_tex], y)
a_eqtex, _ = logreg_auc(["sem", "appe", "geo", best_tex], y)
print(f"  appe-only AUC={a_app:.3f} | {best_tex}-only AUC={a_tex:.3f} | appe+{best_tex} AUC={a_both:.3f}  coef={w_both}")
print(f"  [sem,appe,geo]+{best_tex} AUC={a_eqtex:.3f}  (cf. Eq.4/logreg3 ≈ 0.91)")


# ---------- Task5: FP visual audit (high appearance-score FP) ----------
fps = sorted([r for r in rows if r["dt"] == "FP"], key=lambda r: -(r["appe"] or 0))[:6]
saved = 0
for r in fps:
    cf = crop_feats_for_frame(r["fid"])
    if cf is None: continue
    _, det, bgr, m = cf
    x, yy, w, h = [int(v) for v in det["bbox"]]
    crop = bgr[max(0,yy):yy+h, max(0,x):x+w].copy()
    # nearest template by hist (color)
    over = bgr.copy(); over[m > 0] = (0.5*over[m>0] + 0.5*np.array([0,0,255])).astype(np.uint8)
    tpl = cv2.imread(f"{TPL}/rgb_0.png")
    def fit(im,hh=160):
        if im is None or im.size==0: return np.zeros((hh,hh,3),np.uint8)
        sc=hh/max(1,im.shape[0]); return cv2.resize(im,(max(1,int(im.shape[1]*sc)),hh))
    panel = cv2.hconcat([fit(bgr), fit(over), fit(crop), fit(tpl)])
    cv2.putText(panel, f"FP {r['fid']} appe={r['appe']:.2f} rb={r['rb']:.2f} colorful={r['colorful']:.1f}",
                (5,15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1)
    cv2.imwrite(f"{VIS_OUT}/fp_{r['fid']}.png", panel)
    saved += 1
print(f"\n=== [Task5] FP visual audit saved {saved} panels → {VIS_OUT}/ ===")
print("  high-appe FP 샘플 색특성:")
for r in fps[:6]:
    print(f"   {r['fid']}: appe={r['appe']:.3f} rb={r['rb']:.3f} colorful={r['colorful']:.1f} edge={r['edge']:.3f} hist_sim={r['hist_sim']:.3f}")
# TP reference
tps = [r for r in rows if r["dt"] == "TP"]
import statistics as st
print("  (참고) TP 평균 rb=%.3f colorful=%.1f edge=%.3f hist_sim=%.3f | FP 평균 rb=%.3f colorful=%.1f edge=%.3f hist_sim=%.3f"
      % (st.mean([r['rb'] for r in tps]), st.mean([r['colorful'] for r in tps]), st.mean([r['edge'] for r in tps]), st.mean([r['hist_sim'] for r in tps]),
         st.mean([r['rb'] for r in rows if r['dt']=='FP']), st.mean([r['colorful'] for r in rows if r['dt']=='FP']), st.mean([r['edge'] for r in rows if r['dt']=='FP']), st.mean([r['hist_sim'] for r in rows if r['dt']=='FP'])))
print("\n=== DONE ===")
