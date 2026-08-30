#!/usr/bin/env python3
"""eval_appearance_blocks.py — Workstream A: DINOv2 block2 vs block9 (vs 운영 block11).

두 렌즈:
  (1) 단독 판별력: 선택된 best 후보 population에서 appe_blockX 가 visible(정답객체) vs
      non-visible(FP원천) 을 얼마나 잘 가르는가 — AUROC/AUPRC, 전체·클래스별.
  (2) 파이프라인 기여: semantic(0.35)+HSV(Phase1C) 고정, appe 게이트만 교체(A0~A4).
      block별 score range가 달라 게이트 threshold(및 fusion weight)는 dataset LODO로 선택.

산출: csv/appearance_weight_sweep.csv, appearance_block_scores_eval.csv,
      appearance_class_metrics.csv, metrics/appearance_metrics.json
"""
import csv, json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase2_common as pc

T_HSV = 0.1214
OUTC = os.path.join(pc.OUT, "csv"); OUTM = os.path.join(pc.OUT, "metrics")


def main():
    gt = pc.load_gt(); rows = pc.load_candidates(); sel = pc.build_selected(rows, gt)
    # 선택 후보 population (candidate 있는 cell)
    pop = [(k, b) for k, b in sel.items() if b is not None]
    labels = np.array([1 if k[2] in gt[(k[0], k[1])] else 0 for k, _ in pop])

    SCORES = {"block2": "appe2_clstop1", "block9": "appe9_clstop1", "block11": "appe11_clstop1"}
    arr = {n: np.array([b[c] for _, b in pop]) for n, c in SCORES.items()}

    # ---------- (1) 단독 판별력 ----------
    disc = {}
    for n, s in arr.items():
        disc[n] = {"AUROC": round(pc.auroc(s[labels == 1], s[labels == 0]), 4),
                   "AUPRC": round(pc.auprc(s, labels), 4),
                   "n_pos": int(labels.sum()), "n_neg": int((labels == 0).sum())}
    # block 상관 + 일치/불일치
    corr = {"b2_b9": round(float(np.corrcoef(arr["block2"], arr["block9"])[0, 1]), 4),
            "b2_b11": round(float(np.corrcoef(arr["block2"], arr["block11"])[0, 1]), 4),
            "b9_b11": round(float(np.corrcoef(arr["block9"], arr["block11"])[0, 1]), 4)}
    # semantic/hsv 상관
    sem = np.array([b["sem_top5"] for _, b in pop]); hsv = np.array([b["hsv_score"] for _, b in pop])
    corr["b9_sem"] = round(float(np.corrcoef(arr["block9"], sem)[0, 1]), 4)
    corr["b9_hsv"] = round(float(np.corrcoef(arr["block9"], hsv)[0, 1]), 4)
    corr["b2_hsv"] = round(float(np.corrcoef(arr["block2"], hsv)[0, 1]), 4)

    # 클래스별 AUROC
    class_auroc = {}
    objs = np.array([k[2] for k, _ in pop])
    for o in pc.OBJECTS:
        m = objs == o; y = labels[m]
        if y.sum() == 0 or (y == 0).sum() == 0:
            class_auroc[o] = {n: None for n in arr}; continue
        class_auroc[o] = {n: round(pc.auroc(arr[n][m][y == 1], arr[n][m][y == 0]), 4) for n in arr}

    # ---------- (2) 파이프라인 기여 (LODO) ----------
    # score 함수: cand -> appe 대체 점수
    def sc_block(col):
        return lambda b: b[col]

    def sc_mean(c1, c2):
        return lambda b: 0.5 * (b[c1] + b[c2])

    def sc_wsum(c1, c2, w, mu, sd):
        # z-normalize (train 통계 mu/sd) 후 가중합
        return lambda b: w * ((b[c1] - mu[0]) / sd[0]) + (1 - w) * ((b[c2] - mu[1]) / sd[1])

    def grid_accept_factory(score_fn, thr):
        def acc(ds, fr, o):
            b = sel.get((ds, fr, o))
            if not b: return False
            if b["sem_top5"] < b["sim_thr"]: return False
            if score_fn(b) < thr: return False
            return b["hsv_score"] >= T_HSV
        return acc

    def pop_scores_labels(score_fn, train_ds):
        s, y = [], []
        for (ds, fr, o), b in pop:
            if ds not in train_ds: continue
            if b["sem_top5"] < b["sim_thr"]: continue     # semantic 통과분에서 threshold 선택
            s.append(score_fn(b)); y.append(1 if o in gt[(ds, fr)] else 0)
        return np.array(s), np.array(y)

    def lodo_eval(name, make_score_fn, wsweep=None):
        """make_score_fn(train_ds)->(score_fn, meta). wsweep: fusion weight grid or None."""
        # pooled LODO: 각 held-out에서 train으로 (w,thr) 선택 → held-out 적용
        agg = {"TP": 0, "FP": 0, "FN": 0, "TN": 0}
        per = {o: [0, 0, 0, 0] for o in pc.OBJECTS}
        chosen = []
        for test_ds, train_ds in pc.lodo_folds():
            best = None
            wcands = wsweep if wsweep is not None else [None]
            for w in wcands:
                score_fn, meta = make_score_fn(train_ds, w)
                s, y = pop_scores_labels(score_fn, train_ds)
                thr, f1 = pc.best_f1_threshold(s, y)
                if best is None or f1 > best[0]:
                    best = (f1, thr, w, score_fn, meta)
            _, thr, w, score_fn, meta = best
            chosen.append({"held_out": test_ds, "weight": w, "threshold": round(float(thr), 4),
                           "train_f1": round(best[0], 4), **meta})
            acc = grid_accept_factory(score_fn, thr)
            for (ds, fr), vis in gt.items():
                if ds != test_ds: continue
                for o in pc.OBJECTS:
                    a = acc(ds, fr, o); v = o in vis
                    idx = 0 if (v and a) else 1 if a else 2 if v else 3
                    key = ["TP", "FP", "FN", "TN"][idx]; agg[key] += 1; per[o][idx] += 1
        TP, FP, FN, TN = agg["TP"], agg["FP"], agg["FN"], agg["TN"]
        P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
        f1 = 2 * P * R / max(P + R, 1e-9)
        return {"name": name, "TP": TP, "FP": FP, "FN": FN, "TN": TN,
                "precision": round(P, 4), "recall": round(R, 4), "f1": round(f1, 4),
                "per_class": per, "folds": chosen}

    # A0: 운영(block11, 고정 게이트 0.55 — LODO 없이 실제 운영값)
    def acc_A0(ds, fr, o):
        b = sel.get((ds, fr, o))
        if not b: return False
        if b["sem_top5"] < b["sim_thr"]: return False
        if b["appe11_clstop1"] < b["appe_gate"]: return False
        return b["hsv_score"] >= T_HSV
    m = pc.grid_metrics(gt, acc_A0)
    A0 = {"name": "A0_current_block11_gate0.55", "TP": m["TP"], "FP": m["FP"], "FN": m["FN"],
          "TN": m["TN"], "precision": m["precision"], "recall": m["recall"], "f1": m["f1"],
          "per_class": m["per_class"]}

    # 정규화 통계(z): fusion 용, train별로 재계산되므로 여기선 lambda 안에서 처리
    def make_block(col):
        return lambda train, w: (lambda b: b[col], {"variant": col})

    def make_mean():
        return lambda train, w: (lambda b: 0.5 * (b["appe2_clstop1"] + b["appe9_clstop1"]),
                                 {"variant": "mean(b2,b9)"})

    def make_wsum():
        def f(train, w):
            s2 = np.array([b["appe2_clstop1"] for (ds, _, _), b in pop if ds in train])
            s9 = np.array([b["appe9_clstop1"] for (ds, _, _), b in pop if ds in train])
            mu = (s2.mean(), s9.mean()); sd = (s2.std() + 1e-6, s9.std() + 1e-6)
            fn = lambda b: w * ((b["appe2_clstop1"] - mu[0]) / sd[0]) + \
                           (1 - w) * ((b["appe9_clstop1"] - mu[1]) / sd[1])
            return fn, {"variant": "zwsum(b2,b9)", "w_block2": round(w, 2)}
        return f

    A1 = lodo_eval("A1_block2_LODOthr", make_block("appe2_clstop1"))
    A2 = lodo_eval("A2_block9_LODOthr", make_block("appe9_clstop1"))
    A3 = lodo_eval("A3_mean_b2b9_LODOthr", make_mean())
    WGRID = [round(x, 2) for x in np.arange(0, 1.01, 0.1)]
    A4 = lodo_eval("A4_zwsum_b2b9_LODO_w_thr", make_wsum(), wsweep=WGRID)
    # 보충: block9 게이트를 LODO로 (block11 대비 정당비교), block11 LODO
    A2b = lodo_eval("Ax_block11_LODOthr", make_block("appe11_clstop1"))

    configs = [A0, A1, A2, A2b, A3, A4]

    # ---------- 저장 ----------
    os.makedirs(OUTC, exist_ok=True); os.makedirs(OUTM, exist_ok=True)
    with open(os.path.join(OUTC, "appearance_weight_sweep.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["config", "TP", "FP", "FN", "F1", "precision", "recall", "note"])
        for c in configs:
            note = ""
            if "folds" in c:
                note = "LODO thr=" + ";".join(f"{x['held_out']}:{x['threshold']}" +
                        (f"(w{x.get('w_block2')})" if x.get('weight') is not None else "") for x in c["folds"])
            w.writerow([c["name"], c["TP"], c["FP"], c["FN"], c["f1"], c["precision"], c["recall"], note])

    with open(os.path.join(OUTC, "appearance_block_scores_eval.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["block", "AUROC", "AUPRC", "n_pos", "n_neg"])
        for n, d in disc.items():
            w.writerow([n, d["AUROC"], d["AUPRC"], d["n_pos"], d["n_neg"]])

    with open(os.path.join(OUTC, "appearance_class_metrics.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["class", "AUROC_b2", "AUROC_b9", "AUROC_b11",
                                       "A0_TP", "A0_FP", "A0_FN", "A2_block9_TP", "A2_block9_FP", "A2_block9_FN"])
        for o in pc.OBJECTS:
            ca = class_auroc[o]
            a0 = A0["per_class"].get(o, [0, 0, 0, 0]); a2 = A2["per_class"].get(o, [0, 0, 0, 0])
            w.writerow([o, ca["block2"], ca["block9"], ca["block11"],
                        a0[0], a0[1], a0[2], a2[0], a2[1], a2[2]])

    out = {"discriminability": disc, "block_correlation": corr, "class_auroc": class_auroc,
           "pipeline_configs": [{k: v for k, v in c.items() if k != "per_class"} for c in configs],
           "notes": {"population": "selected-best candidate per (frame,object) with a routed candidate",
                     "label": "gt_visible (object present in frame)",
                     "hsv": "self-consistent phase2 hsv_score (A0 here TP=%d)" % A0["TP"],
                     "score_variant": "clstop1 (operational appe view = semantically-best template)"}}
    json.dump(out, open(os.path.join(OUTM, "appearance_metrics.json"), "w"), indent=2, ensure_ascii=False)

    print("=== Workstream A: 단독 판별력 (AUROC/AUPRC) ===")
    for n, d in disc.items():
        print(f"  {n:8s} AUROC {d['AUROC']}  AUPRC {d['AUPRC']}  (pos {d['n_pos']} / neg {d['n_neg']})")
    print("  corr:", corr)
    print("=== Workstream A: 파이프라인 기여 (LODO) ===")
    for c in configs:
        print(f"  {c['name']:32s} TP {c['TP']} FP {c['FP']} FN {c['FN']} F1 {c['f1']}")
    print(f"-> {OUTM}/appearance_metrics.json")


if __name__ == "__main__":
    main()
