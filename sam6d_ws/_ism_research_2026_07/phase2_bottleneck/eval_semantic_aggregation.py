#!/usr/bin/env python3
"""eval_semantic_aggregation.py — Workstream B: Top-5 vs 전체평균 결합 검증.

운영 semantic = mean(top-5 of 42 view cosines), HARD gate >= sim_thr(0.35).
"Top-5 × 전체평균"의 의미가 문서에 없어 곱(S5)·기하평균(S6)·가중합(S7)을 모두 비교.

두 렌즈: (1) 단독 판별력 AUROC/AUPRC, (2) 파이프라인 기여(semantic 게이트만 교체,
selection은 sem_top5 고정, appe11 게이트 0.55 + HSV Phase1C 고정, LODO threshold/α 선택).

산출: csv/semantic_formula_sweep.csv, semantic_weight_threshold_sweep.csv,
      semantic_class_metrics.csv, metrics/semantic_metrics.json
"""
import csv, json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase2_common as pc

T_HSV = 0.1214; APPE_FIXED = None    # appe_gate는 후보별 b["appe_gate"] 사용(0.55)
OUTC = os.path.join(pc.OUT, "csv"); OUTM = os.path.join(pc.OUT, "metrics")

BASE_FORMULAS = {
    "S1_top1": lambda b, a: b["sem_top1"],
    "S2_top3": lambda b, a: b["sem_top3"],
    "S3_top5": lambda b, a: b["sem_top5"],          # 운영
    "S4_allmean": lambda b, a: b["sem_all_mean"],
    "S5_product": lambda b, a: b["sem_top5"] * b["sem_all_mean"],
    "S6_geomean": lambda b, a: float(np.sqrt(max(0.0, b["sem_top5"]) * max(0.0, b["sem_all_mean"]))),
    "S7_alpha": lambda b, a: a * b["sem_top5"] + (1 - a) * b["sem_all_mean"],
}


def main():
    gt = pc.load_gt(); rows = pc.load_candidates(); sel = pc.build_selected(rows, gt)
    pop = [(k, b) for k, b in sel.items() if b is not None]
    labels = np.array([1 if k[2] in gt[(k[0], k[1])] else 0 for k, _ in pop])

    # ---------- (1) 단독 판별력 ----------
    disc = {}
    for name, fn in BASE_FORMULAS.items():
        a = 0.5 if name == "S7_alpha" else None
        s = np.array([fn(b, a) for _, b in pop])
        disc[name] = {"AUROC": round(pc.auroc(s[labels == 1], s[labels == 0]), 4),
                      "AUPRC": round(pc.auprc(s, labels), 4)}
    # top5 vs allmean 상관, product/geomean 관계
    top5 = np.array([b["sem_top5"] for _, b in pop]); allm = np.array([b["sem_all_mean"] for _, b in pop])
    corr = {"top5_allmean": round(float(np.corrcoef(top5, allm)[0, 1]), 4)}

    class_auroc = {}
    objs = np.array([k[2] for k, _ in pop])
    for o in pc.OBJECTS:
        m = objs == o; y = labels[m]
        if y.sum() == 0 or (y == 0).sum() == 0:
            class_auroc[o] = None; continue
        class_auroc[o] = {name: round(pc.auroc(
            np.array([fn(b, 0.5) for (k, b) in pop if k[2] == o])[y == 1],
            np.array([fn(b, 0.5) for (k, b) in pop if k[2] == o])[y == 0]), 4)
            for name, fn in BASE_FORMULAS.items()}

    # ---------- (2) 파이프라인 기여 (LODO) ----------
    def grid_accept(score_fn, thr, a):
        def acc(ds, fr, o):
            b = sel.get((ds, fr, o))
            if not b: return False
            if score_fn(b, a) < thr: return False           # semantic 게이트 교체
            if b["appe11_clstop1"] < b["appe_gate"]: return False
            return b["hsv_score"] >= T_HSV
        return acc

    def train_scores(score_fn, a, train_ds):
        s, y = [], []
        for (ds, fr, o), b in pop:
            if ds not in train_ds: continue
            s.append(score_fn(b, a)); y.append(1 if o in gt[(ds, fr)] else 0)
        return np.array(s), np.array(y)

    def lodo(name, score_fn, alphas=None):
        agg = {"TP": 0, "FP": 0, "FN": 0, "TN": 0}; per = {o: [0, 0, 0, 0] for o in pc.OBJECTS}
        folds = []
        for test_ds, train_ds in pc.lodo_folds():
            best = None
            for a in (alphas if alphas is not None else [None]):
                s, y = train_scores(score_fn, a, train_ds)
                thr, f1 = pc.best_f1_threshold(s, y)
                if best is None or f1 > best[0]:
                    best = (f1, thr, a)
            f1t, thr, a = best
            folds.append({"held_out": test_ds, "alpha": a, "threshold": round(float(thr), 4),
                          "train_f1": round(f1t, 4)})
            acc = grid_accept(score_fn, thr, a)
            for (ds, fr), vis in gt.items():
                if ds != test_ds: continue
                for o in pc.OBJECTS:
                    aok = acc(ds, fr, o); v = o in vis
                    idx = 0 if (v and aok) else 1 if aok else 2 if v else 3
                    agg[["TP", "FP", "FN", "TN"][idx]] += 1; per[o][idx] += 1
        TP, FP, FN = agg["TP"], agg["FP"], agg["FN"]
        P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
        return {"name": name, "TP": TP, "FP": FP, "FN": FN, "TN": agg["TN"],
                "precision": round(P, 4), "recall": round(R, 4),
                "f1": round(2 * P * R / max(P + R, 1e-9), 4), "per_class": per, "folds": folds}

    # S0 = 운영(sem_top5 게이트 고정 0.35)
    def acc_S0(ds, fr, o):
        b = sel.get((ds, fr, o))
        if not b: return False
        if b["sem_top5"] < b["sim_thr"]: return False
        if b["appe11_clstop1"] < b["appe_gate"]: return False
        return b["hsv_score"] >= T_HSV
    m0 = pc.grid_metrics(gt, acc_S0)
    S0 = {"name": "S0_operational_top5_gate0.35", "TP": m0["TP"], "FP": m0["FP"], "FN": m0["FN"],
          "TN": m0["TN"], "precision": m0["precision"], "recall": m0["recall"], "f1": m0["f1"],
          "per_class": m0["per_class"]}

    configs = [S0]
    for name, fn in BASE_FORMULAS.items():
        alphas = [round(x, 2) for x in np.arange(0, 1.01, 0.1)] if name == "S7_alpha" else None
        configs.append(lodo(name + "_LODOthr", fn, alphas))

    # ---------- 저장 ----------
    os.makedirs(OUTC, exist_ok=True); os.makedirs(OUTM, exist_ok=True)
    with open(os.path.join(OUTC, "semantic_formula_sweep.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["config", "AUROC", "TP", "FP", "FN", "F1", "precision", "recall"])
        for c in configs:
            nm = c["name"].replace("_LODOthr", "")
            au = disc.get(nm, {}).get("AUROC", "")
            w.writerow([c["name"], au, c["TP"], c["FP"], c["FN"], c["f1"], c["precision"], c["recall"]])

    with open(os.path.join(OUTC, "semantic_weight_threshold_sweep.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["config", "held_out", "alpha", "threshold", "train_f1"])
        for c in configs:
            for fo in c.get("folds", []):
                w.writerow([c["name"], fo["held_out"], fo["alpha"], fo["threshold"], fo["train_f1"]])

    with open(os.path.join(OUTC, "semantic_class_metrics.csv"), "w", newline="") as f:
        w = csv.writer(f); head = ["class"] + list(BASE_FORMULAS.keys())
        w.writerow([h + "_AUROC" for h in head])
        for o in pc.OBJECTS:
            ca = class_auroc[o]
            w.writerow([o] + ([ca[k] for k in BASE_FORMULAS] if ca else [""] * len(BASE_FORMULAS)))

    out = {"discriminability": disc, "correlation": corr, "class_auroc": class_auroc,
           "pipeline_configs": [{k: v for k, v in c.items() if k != "per_class"} for c in configs],
           "notes": {"selection": "held fixed at sem_top5 (Phase 1C canonical); only gate score swapped",
                     "meaning_top5xall": "문서 정의 없음 → S5 곱/S6 기하평균/S7 가중합 모두 비교",
                     "hsv": "self-consistent phase2 hsv_score"}}
    json.dump(out, open(os.path.join(OUTM, "semantic_metrics.json"), "w"), indent=2, ensure_ascii=False)

    print("=== Workstream B: 단독 판별력 (AUROC) ===")
    for n, d in disc.items():
        print(f"  {n:14s} AUROC {d['AUROC']}  AUPRC {d['AUPRC']}")
    print("  corr top5~allmean:", corr["top5_allmean"])
    print("=== Workstream B: 파이프라인 기여 (LODO) ===")
    for c in configs:
        extra = ""
        if c["name"].startswith("S7"):
            al = [fo["alpha"] for fo in c["folds"]]; extra = f" alpha_folds={al}"
        print(f"  {c['name']:26s} TP {c['TP']} FP {c['FP']} FN {c['FN']} F1 {c['f1']}{extra}")
    print(f"-> {OUTM}/semantic_metrics.json")


if __name__ == "__main__":
    main()
