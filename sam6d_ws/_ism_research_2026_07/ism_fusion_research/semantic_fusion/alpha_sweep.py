#!/usr/bin/env python3
"""alpha_sweep.py — S_fusion 의 α 를 촘촘히 훑어 최적 지점을 찾는다 (READ-ONLY).

    S_fusion = α · z(S_top5) + (1-α) · z(S_mean)

정규화 z(·) 의 평균·표준편차와 판정 threshold 는 **calibration fold 에서만** 추정하고
held-out fold 에서만 채점한다. α=1 이면 Top-5 단독, α=0 이면 42장 평균 단독이므로
지시된 5개 값(0, .25, .5, .75, 1)은 이 곡선의 부분집합이다.

산출: results/semantic_alpha_sweep.csv
"""
import csv, os
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
ALPHAS = [round(a, 2) for a in np.arange(0, 1.001, 0.05)]

Z = np.load(os.path.join(HERE, "box_cls.npz"))
TCLS = {k[len("__tcls__"):]: Z[k] for k in Z.files if k.startswith("__tcls__")}
labels = {r["uid"]: r["true_class"]
          for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}

pairs = []
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        pairs.append({"uid": r["uid"], "dataset": ds, "frame_id": int(r["frame_id"]),
                      "object": r["object"], "appe": float(r["appe11_clstop1"]),
                      "appe_gate": float(r["appe_gate"])})
S = {}
for p in pairs:
    S[(p["uid"], p["object"])] = np.sort(TCLS[p["object"]] @ Z[p["uid"]])[::-1]

groups = defaultdict(list)
for p in pairs:
    groups[(p["dataset"], p["frame_id"], p["object"])].append(p)
full = {k: v for k, v in groups.items() if all(x["uid"] in labels for x in v)}
lab_pairs = [p for p in pairs if p["uid"] in labels]


def auroc(pos, neg):
    if len(pos) == 0 or len(neg) == 0:
        return None
    a, b = np.asarray(pos, float), np.asarray(neg, float)
    return float(((a[:, None] > b[None, :]).sum() + 0.5 * (a[:, None] == b[None, :]).sum())
                 / (len(a) * len(b)))


def mat(ps):
    return np.stack([S[(p["uid"], p["object"])] for p in ps])


rows = []
for a in ALPHAS:
    tot = defaultdict(int)
    aucs = defaultdict(list)
    for held in DATASETS:
        cal_all = [p for p in pairs if p["dataset"] != held]
        M = mat(cal_all)
        t5, tm = M[:, :5].mean(1), M.mean(1)
        m5, s5, mm, sm = t5.mean(), t5.std() + 1e-9, tm.mean(), tm.std() + 1e-9

        def sc(ps):
            X = mat(ps)
            return a * ((X[:, :5].mean(1) - m5) / s5) + (1 - a) * ((X.mean(1) - mm) / sm)

        # 객체별 AUROC (held-out)
        te = [p for p in lab_pairs if p["dataset"] == held]
        if te:
            v = sc(te)
            for obj in TCLS:
                idx = [i for i, p in enumerate(te) if p["object"] == obj]
                if len(idx) < 4:
                    continue
                y = np.array([1 if labels[te[i]["uid"]] == obj else 0 for i in idx])
                if y.sum() < 2 or (1 - y).sum() < 2:
                    continue
                aucs[obj].append(auroc(v[idx][y == 1], v[idx][y == 0]))

        # 결정 단위
        cal_g = {k: v for k, v in full.items() if k[0] != held}
        te_g = {k: v for k, v in full.items() if k[0] == held}
        if not cal_g or not te_g:
            continue

        def decide(gs, thr):
            flat, own = [], []
            for k, v in gs.items():
                for p in v:
                    flat.append(p); own.append(k)
            v = sc(flat)
            best = {}
            for i, k in enumerate(own):
                if k not in best or v[i] > best[k][0]:
                    best[k] = (v[i], flat[i])
            TP = FP = FN = TN = 0
            for k, (s, p) in best.items():
                obj = k[2]
                has = any(labels[x["uid"]] == obj for x in gs[k])
                acc = (s >= thr) and (p["appe"] >= p["appe_gate"])
                ok = labels[p["uid"]] == obj
                TP, FP, FN, TN = (TP + (acc and ok), FP + (acc and not ok),
                                  FN + ((not acc) and has), TN + ((not acc) and not has))
            return TP, FP, FN, TN

        cflat = [p for v in cal_g.values() for p in v]
        grid = np.quantile(sc(cflat), np.linspace(0.02, 0.98, 97))
        bt, bf = grid[0], -1
        for t in grid:
            TP, FP, FN, TN = decide(cal_g, t)
            f1 = 2 * TP / max(2 * TP + FP + FN, 1)
            if f1 > bf:
                bf, bt = f1, t
        TP, FP, FN, TN = decide(te_g, bt)
        for k, v in (("TP", TP), ("FP", FP), ("FN", FN), ("TN", TN)):
            tot[k] += v

    P = tot["TP"] / max(tot["TP"] + tot["FP"], 1)
    R = tot["TP"] / max(tot["TP"] + tot["FN"], 1)
    rows.append({"alpha": a, "TP": tot["TP"], "FP": tot["FP"], "FN": tot["FN"], "TN": tot["TN"],
                 "precision": round(P, 4), "recall": round(R, 4),
                 "f1": round(2 * P * R / max(P + R, 1e-9), 4),
                 "mean_auroc": round(float(np.mean([np.mean(v) for v in aucs.values()])), 4),
                 "min_object_auroc": round(float(min(np.mean(v) for v in aucs.values())), 4),
                 **{f"auroc_{o}": round(float(np.mean(v)), 4) for o, v in sorted(aucs.items())}})

with open(os.path.join(RES, "semantic_alpha_sweep.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

b5 = [r for r in rows if r["alpha"] == 1.0][0]
print(f"{'α':>5} {'TP':>4} {'FP':>4} {'FN':>4} {'Prec':>7} {'Rec':>7} {'F1':>7} "
      f"{'평균AUROC':>9} {'최저객체':>9}  vs Top5(α=1)")
for r in rows:
    d = f"ΔFP {r['FP']-b5['FP']:+3d}  ΔFN {r['FN']-b5['FN']:+3d}"
    star = " ★" if (r["FP"] <= b5["FP"] and r["FN"] <= b5["FN"]) else ""
    print(f"{r['alpha']:>5.2f} {r['TP']:>4} {r['FP']:>4} {r['FN']:>4} {r['precision']:>7.4f} "
          f"{r['recall']:>7.4f} {r['f1']:>7.4f} {r['mean_auroc']:>9.4f} "
          f"{r['min_object_auroc']:>9.4f}  {d}{star}")
print(f"\n-> {os.path.join(RES,'semantic_alpha_sweep.csv')}")
