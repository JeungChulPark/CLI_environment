#!/usr/bin/env python3
"""analyze_hsv.py — 검증 2: HSV histogram 변형 스윕 + 객체별 일반성 (READ-ONLY).

저장된 히스토그램 원본(H32/S32/V32/H16xS8 × masked/bbox/background)에서
bin 수(32→16→8 합산), 색공간 조합, 유사도 5종을 오프라인으로 재계산한다.
라벨(labels/box_labels_merged.csv)이 있을 때만 AUROC/PR-AUC 를 산출한다.
"""
import csv, glob, json, os, sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES = os.path.join(ROOT, "results")
HF = os.path.join(ROOT, "hsv_features")

# ---- 히스토그램 슬라이스 (per region 224: H32|S32|V32|HS128) × 3 region ----
REG = {"masked": 0, "bbox": 224, "background": 448}
SL = {"H": (0, 32), "S": (32, 64), "V": (64, 96), "HS": (96, 224)}


def rebin(v, nbin):
    n = v.shape[-1]
    if nbin == n:
        return v
    f = n // nbin
    return v.reshape(*v.shape[:-1], nbin, f).sum(-1)


def get(hist, region, space, nbin):
    o = REG[region]
    parts = []
    for ch in space:
        if ch == "2":       # HS 2D
            a = hist[..., o+96:o+224]
            parts.append(a)
            continue
        s, e = SL[ch]
        parts.append(rebin(hist[..., o+s:o+e], nbin))
    v = np.concatenate(parts, -1)
    s = v.sum(-1, keepdims=True)
    return v / np.maximum(s, 1e-12)


def sim(A, b, metric):
    """A[N,D] 레퍼런스, b[D] 쿼리 → 유사도(클수록 유사) 최대값."""
    if metric == "intersection":
        return float(np.minimum(A, b[None]).sum(1).max())
    if metric == "cosine":
        na = np.linalg.norm(A, axis=1) + 1e-12
        return float(((A @ b) / (na * (np.linalg.norm(b) + 1e-12))).max())
    if metric == "correlation":
        Am = A - A.mean(1, keepdims=True); bm = b - b.mean()
        d = (np.linalg.norm(Am, axis=1) + 1e-12) * (np.linalg.norm(bm) + 1e-12)
        return float(((Am @ bm) / d).max())
    if metric == "bhattacharyya":       # 계수(클수록 유사)
        return float(np.sqrt(np.maximum(A, 0) * np.maximum(b, 0)[None]).sum(1).max())
    if metric == "chi2":                # 거리 → 음수화
        d = ((A - b[None]) ** 2 / (A + b[None] + 1e-12)).sum(1)
        return float(-d.min())
    raise ValueError(metric)


def auroc(pos, neg):
    if not pos or not neg:
        return None
    a = sorted([(v, 1) for v in pos] + [(v, 0) for v in neg])
    n = len(a); r = [0.0]*n; i = 0
    while i < n:
        j = i
        while j+1 < n and a[j+1][0] == a[i][0]:
            j += 1
        avg = (i+j)/2.0 + 1
        for k in range(i, j+1):
            r[k] = avg
        i = j+1
    rp = sum(r[k] for k in range(n) if a[k][1] == 1)
    return (rp - len(pos)*(len(pos)+1)/2) / (len(pos)*len(neg))


def pr_auc(pos, neg):
    if not pos or not neg:
        return None
    a = sorted([(v, 1) for v in pos] + [(v, 0) for v in neg], reverse=True)
    tp = fp = 0; P = len(pos); prev_r = 0.0; area = 0.0
    for v, y in a:
        tp += y; fp += (1-y)
        rec = tp/P; prec = tp/(tp+fp)
        area += (rec - prev_r) * prec
        prev_r = rec
    return area


def main():
    # 데이터 적재
    hist, uids = [], []
    for f in sorted(glob.glob(os.path.join(HF, "sam_*_hsv.npz"))):
        z = np.load(f, allow_pickle=True)
        uids += list(z["uid"]); hist.append(z["hist"])
    hist = np.concatenate(hist, 0)
    UI = {u: i for i, u in enumerate(uids)}
    tpl = dict(np.load(os.path.join(HF, "template_render_hsv.npz"), allow_pickle=True))
    print(f"boxes with hist = {len(uids)}, template objects = {len(tpl)}")

    labf = os.path.join(ROOT, "labels", "box_labels_merged.csv")
    if not os.path.isfile(labf):
        print(f"[warn] 라벨 없음 ({labf}) — 변형 스윕만 수행하고 AUROC 는 생략")
        return
    labels = {r["uid"]: r["true_class"] for r in csv.DictReader(open(labf))}
    ds_of = {u: u.split("|")[0] for u in uids}
    OBJS = sorted(tpl.keys())
    lab_uids = [u for u in uids if labels.get(u) not in (None, "unclear")]
    print(f"labeled boxes = {len(lab_uids)}")

    # ---------------- 변형 스윕 (렌더 템플릿 레퍼런스) ----------------
    sweep = []
    for region in ("masked", "bbox"):
        for space, sname in (("H", "H"), ("S", "S"), ("V", "V"), ("HS", "H+S"),
                             ("HSV", "H+S+V"), ("2", "H16xS8(2D)")):
            for nbin in ((8, 16, 32) if space != "2" else (0,)):
                for metric in ("intersection", "cosine", "correlation", "bhattacharyya", "chi2"):
                    aucs = {}
                    for o in OBJS:
                        A = get(tpl[o], "masked", space, nbin or 32)
                        pos, neg = [], []
                        for u in lab_uids:
                            v = sim(A, get(hist[UI[u]], region, space, nbin or 32), metric)
                            (pos if labels[u] == o else neg).append(v)
                        a = auroc(pos, neg)
                        if a is not None:
                            aucs[o] = round(a, 3)
                    if aucs:
                        sweep.append({"region": region, "space": sname, "bins": nbin or "16x8",
                                      "metric": metric,
                                      "mean_auroc": round(float(np.mean(list(aucs.values()))), 3),
                                      "min_auroc": min(aucs.values()),
                                      **{f"auc_{k}": v for k, v in aucs.items()}})
                    print(f"  {region:10s} {sname:11s} bins={nbin or '16x8':<5} {metric:14s} "
                          f"mean={sweep[-1]['mean_auroc'] if sweep else '-'}", flush=True)
    sweep.sort(key=lambda r: -r["mean_auroc"])
    with open(os.path.join(RES, "hsv_variant_sweep.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(sweep[0].keys())); w.writeheader(); w.writerows(sweep)
    print("\n=== 상위 10 변형 ===")
    for r in sweep[:10]:
        print(f"  {r['region']:10s} {r['space']:11s} bins={r['bins']:<6} {r['metric']:14s} "
              f"mean={r['mean_auroc']} min={r['min_auroc']}")

    # ---------------- 실사 crop prototype (dataset 분리) ----------------
    best = sweep[0]
    nb = 32 if best["bins"] == "16x8" else int(best["bins"])
    space_map = {"H": "H", "S": "S", "V": "V", "H+S": "HS", "H+S+V": "HSV", "H16xS8(2D)": "2"}
    sp = space_map[best["space"]]
    dsets = sorted({ds_of[u] for u in lab_uids})
    rows = []
    for held in dsets:                       # leave-one-dataset-out
        for o in OBJS:
            ref = [u for u in lab_uids if labels[u] == o and ds_of[u] != held]
            if len(ref) < 3:
                continue
            A = np.stack([get(hist[UI[u]], "masked", sp, nb) for u in ref])
            ev = [u for u in lab_uids if ds_of[u] == held]
            pos = [sim(A, get(hist[UI[u]], "masked", sp, nb), best["metric"])
                   for u in ev if labels[u] == o]
            neg = [sim(A, get(hist[UI[u]], "masked", sp, nb), best["metric"])
                   for u in ev if labels[u] != o]
            a = auroc(pos, neg)
            if a is None:
                continue
            rows.append({"held_out_dataset": held, "object": o, "n_ref_crops": len(ref),
                         "n_pos": len(pos), "n_neg": len(neg),
                         "auroc_real_prototype": round(a, 3),
                         "pr_auc": round(pr_auc(pos, neg), 3)})
    if rows:
        with open(os.path.join(RES, "hsv_real_prototype_lodo.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
        print(f"\n=== 실사 prototype LODO ({best['space']}/{best['bins']}/{best['metric']}) ===")
        byo = defaultdict(list)
        for r in rows:
            byo[r["object"]].append(r["auroc_real_prototype"])
        for o, v in sorted(byo.items()):
            print(f"  {o:22s} n_folds={len(v)} mean={np.mean(v):.3f} min={min(v):.3f} max={max(v):.3f}")


if __name__ == "__main__":
    main()
