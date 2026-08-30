#!/usr/bin/env python3
"""phase2_common.py — Phase 2 검증 공용 유틸 (READ-ONLY).

- GT 로드(frame-level visibility, user_reviewed==yes)
- 증강 후보표(phase2_candidates.csv) 로드
- Phase 1C 정본 선택 규칙(routed==1 & conf>=0.02, top3 by conf, best by sem_top5)
- per-(frame,object) grid TP/FP/FN/TN 지표(전체/클래스별/데이터셋별)
- AUROC = rank기반 Mann-Whitney (np.trapz/sklearn 미사용), AUPRC
- dataset 단위 LODO split
"""
import csv, os
from collections import defaultdict
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE)
REPO = os.path.dirname(RSRCH)
OUT = os.path.join(REPO, "outputs", "phase2_appearance_semantic_yolo_bbox")
GT_CSV = os.path.join(RSRCH, "gt_input", "user_visibility_gt.csv")
CUR = os.path.join(RSRCH, "yolo_localization_research", "pipeline", "cur")
CAND_CSV = os.path.join(OUT, "csv", "phase2_candidates.csv")

DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
SEED = 20260722

_FLOAT_COLS = {"yolo_conf", "sem_top1", "sem_top3", "sem_top5", "sem_all_mean", "sem_median",
               "sem_product", "sem_geomean", "appe2_clstop1", "appe9_clstop1", "appe11_clstop1",
               "hsv_score", "hsv_threshold", "sim_thr", "appe_gate"}
_INT_COLS = {"frame_id", "routed", "hue_correction_ocv"}


def load_gt():
    gt = {}
    with open(GT_CSV, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("user_reviewed", "").strip() != "yes":
                continue
            vo = (r.get("visible_objects") or "").replace(",", ";")
            gt[(r["dataset_name"], int(r["frame_id"]))] = {
                t.strip() for t in vo.split(";") if t.strip() and t.strip() != "none"}
    return gt


def load_candidates(path=CAND_CSV):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            d = dict(r)
            for c in _FLOAT_COLS:
                if c in d and d[c] != "":
                    d[c] = float(d[c])
            for c in _INT_COLS:
                if c in d and d[c] != "":
                    d[c] = int(d[c])
            d["gt_visible"] = int(d["gt_visible"]) if d.get("gt_visible", "") != "" else None
            rows.append(d)
    return rows


def group_by_cell(rows):
    """(ds,frame,object) -> [candidate rows]."""
    g = defaultdict(list)
    for r in rows:
        g[(r["dataset"], r["frame_id"], r["object"])].append(r)
    return g


def select_best(cands):
    """Phase 1C 정본 선택. 없으면 None."""
    v = [c for c in cands if int(c["routed"]) == 1 and float(c["yolo_conf"]) >= 0.02]
    if not v:
        return None
    v.sort(key=lambda c: -float(c["yolo_conf"]))
    v = v[:3]
    return max(v, key=lambda c: float(c["sem_top5"]))


def build_selected(rows, gt):
    """gt 프레임 한정, (ds,frame,object) -> best candidate (or None)."""
    g = group_by_cell(rows)
    sel = {}
    for (ds, fr, obj), cs in g.items():
        if (ds, fr) not in gt:
            continue
        sel[(ds, fr, obj)] = select_best(cs)
    return sel


def grid_metrics(gt, accept_fn, objects=OBJECTS):
    """accept_fn(ds,frame,object) -> bool. per-(frame,object) confusion."""
    TP = FP = FN = TN = 0
    per = defaultdict(lambda: [0, 0, 0, 0])       # obj -> [TP,FP,FN,TN]
    perds = defaultdict(lambda: [0, 0, 0, 0])
    for (ds, fr), vis in gt.items():
        for o in objects:
            acc = bool(accept_fn(ds, fr, o))
            v = o in vis
            idx = 0 if (v and acc) else 1 if acc else 2 if v else 3
            if idx == 0: TP += 1
            elif idx == 1: FP += 1
            elif idx == 2: FN += 1
            else: TN += 1
            per[o][idx] += 1
            perds[ds][idx] += 1
    return _summ(TP, FP, FN, TN, per, perds)


def _summ(TP, FP, FN, TN, per, perds):
    P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
    spec = TN / max(TN + FP, 1); f1 = 2 * P * R / max(P + R, 1e-9)
    return {"TP": TP, "FP": FP, "FN": FN, "TN": TN,
            "precision": round(P, 4), "recall": round(R, 4),
            "specificity": round(spec, 4), "f1": round(f1, 4),
            "balanced_accuracy": round((R + spec) / 2, 4),
            "per_class": {k: list(v) for k, v in per.items()},
            "per_dataset": {k: list(v) for k, v in perds.items()}}


def class_f1(per):
    out = {}
    for o, (tp, fp, fn, tn) in per.items():
        p = tp / max(tp + fp, 1); r = tp / max(tp + fn, 1)
        out[o] = {"TP": tp, "FP": fp, "FN": fn, "GT_visible": tp + fn,
                  "precision": round(p, 3), "recall": round(r, 3),
                  "f1": round(2 * p * r / max(p + r, 1e-9), 3)}
    return out


def auroc(pos, neg):
    """rank기반 Mann-Whitney U AUROC (tie-aware). 한쪽 비면 NaN."""
    pos = np.asarray(pos, float); neg = np.asarray(neg, float)
    if len(pos) == 0 or len(neg) == 0:
        return float("nan")
    allv = np.concatenate([pos, neg])
    order = np.argsort(allv, kind="mergesort")
    ranks = np.empty(len(allv), float)
    ranks[order] = np.arange(1, len(allv) + 1)
    # tie 평균 순위
    _, inv, cnt = np.unique(allv, return_inverse=True, return_counts=True)
    csum = np.cumsum(cnt); starts = csum - cnt
    avg = (starts + csum + 1) / 2.0
    ranks = avg[inv]
    P = len(pos)
    rsum = ranks[:P].sum()
    return float((rsum - P * (P + 1) / 2.0) / (P * len(neg)))


def auprc(scores, labels):
    """average precision (step). labels 1=pos."""
    scores = np.asarray(scores, float); labels = np.asarray(labels, int)
    if labels.sum() == 0:
        return float("nan")
    order = np.argsort(-scores, kind="mergesort")
    labels = labels[order]
    tp = np.cumsum(labels); fp = np.cumsum(1 - labels)
    prec = tp / np.maximum(tp + fp, 1); rec = tp / labels.sum()
    ap = 0.0; prev_r = 0.0
    for p, r in zip(prec, rec):
        ap += p * (r - prev_r); prev_r = r
    return float(ap)


def best_f1_threshold(scores, labels, grid=None):
    """단일 score gate의 최적 F1 threshold (>=). 반환 (thr, f1)."""
    scores = np.asarray(scores, float); labels = np.asarray(labels, int)
    if grid is None:
        grid = np.unique(np.concatenate([[-1e9], np.sort(scores), [1e9]]))
    best = (grid[0], -1.0)
    Ptot = labels.sum()
    for t in grid:
        acc = scores >= t
        tp = int((acc & (labels == 1)).sum()); fp = int((acc & (labels == 0)).sum())
        fn = int(Ptot - tp)
        f1 = 2 * tp / max(2 * tp + fp + fn, 1)
        if f1 > best[1]:
            best = (float(t), f1)
    return best


def lodo_folds(datasets=DATASETS):
    """[(test_ds, [train_ds...]), ...]"""
    return [(d, [x for x in datasets if x != d]) for d in datasets]
