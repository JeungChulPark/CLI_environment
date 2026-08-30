#!/usr/bin/env python3
"""eval_fusion.py — 모든 객체에 공통 적용할 semantic score 1개를 결정한다 (READ-ONLY).

설계 원칙
  1) 객체별로 다른 방식을 쓰는 안은 제외한다(지시). 후보는 전부 **전 객체 공통 규칙**이다.
  2) 새 점수는 스케일이 달라지므로 기존 threshold 를 그대로 쓰면 비교가 성립하지 않는다.
     → LODO 로 5개 데이터셋에서 정규화 통계와 threshold 를 정하고, 남은 1개에서만 채점한다.
  3) 순위 지표(AUROC/PR-AUC)뿐 아니라 **결정 단위 TP/FP/FN** 까지 본다.
     운영 판정은 "후보 박스 중 semantic 최대인 것을 고르고 두 게이트를 통과시키는" 구조라,
     점수가 바뀌면 **선택 자체가 바뀐다**. 순위 지표만으로는 이 효과가 보이지 않는다.

결정 단위 정의 (라벨이 완비된 (frame, object) 그룹만 사용)
  TP : 수락했고, 선택된 박스의 true_class == 대상 객체
  FP : 수락했으나, 선택된 박스의 true_class != 대상 객체
  FN : 수락하지 않았으나, 그룹 안에 true_class == 대상 객체인 박스가 존재
  TN : 수락하지 않았고, 그룹 안에 해당 객체가 없음

산출
  results/semantic_scheme_auroc.csv        LODO 객체별 AUROC/PR-AUC
  results/semantic_scheme_decision.csv     LODO 결정 단위 TP/FP/FN/P/R/F1
  results/semantic_alpha_sweep.csv         α 스윕
  results/semantic_error_delta.csv         Top-5 대비 신규/수정 FP·FN 건별
"""
import csv, json, os, sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
REPO = os.path.dirname(RSRCH)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RES = os.path.join(ROOT, "results")
os.makedirs(RES, exist_ok=True)
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
NONOBJ = {"carton", "other", "unclear"}          # 템플릿이 없는 라벨 (negative 로만 쓰임)

# ---------------------------------------------------------------- 적재
Z = np.load(os.path.join(HERE, "box_cls.npz"))
TCLS = {k[len("__tcls__"):]: Z[k] for k in Z.files if k.startswith("__tcls__")}
OBJS = sorted(TCLS)
print("객체", len(OBJS), "| 템플릿", {k: v.shape for k, v in list(TCLS.items())[:1]})

labels = {r["uid"]: r["true_class"]
          for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}

# 후보 pair (운영 baseline 과 동일 조건) + appe/threshold
pairs = []
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        pairs.append({"uid": r["uid"], "dataset": ds, "frame_id": int(r["frame_id"]),
                      "object": r["object"],
                      "appe": float(r["appe11_clstop1"]), "appe_gate": float(r["appe_gate"]),
                      "sim_thr": float(r["sim_thr"]),
                      "sem_top5_ref": float(r["sem_top5"])})
print(f"후보 pair {len(pairs):,}")

# 42-view 유사도 복원: sims[uid, object] = TCLS[object] @ cls[uid]
sims = {}
miss = 0
for p in pairs:
    c = Z.get(p["uid"])
    if c is None:
        miss += 1; continue
    sims[(p["uid"], p["object"])] = TCLS[p["object"]] @ c
print(f"유사도 복원 {len(sims):,} (CLS 없음 {miss})")

# 재현 검증: 복원한 top5 가 덤프의 sem_top5 와 일치하는가
chk = [(np.sort(sims[(p['uid'], p['object'])])[::-1][:5].mean(), p["sem_top5_ref"])
       for p in pairs[:5000] if (p["uid"], p["object"]) in sims]
d = np.abs(np.array([a for a, _ in chk]) - np.array([b for _, b in chk]))
print(f"재현 검증: |복원 top5 - 덤프 sem_top5| 최대 {d.max():.5f} 중앙 {np.median(d):.5f} "
      f"(덤프는 소수 4자리 반올림)")

# ---------------------------------------------------------------- 공통 aggregation 정의
def agg_funcs():
    F = {}
    for k in (1, 3, 5, 10, 20, 42):
        F[f"top{k}"] = (lambda k: (lambda S: S[:, :k].mean(1)))(k)
    F["min1"] = lambda S: S[:, -1]
    F["bottom5"] = lambda S: S[:, -5:].mean(1)
    F["harmonic"] = lambda S: S.shape[1] / np.maximum(1.0 / np.maximum(S, 1e-6), 0).sum(1)
    F["geometric"] = lambda S: np.exp(np.log(np.maximum(S, 1e-6)).mean(1))
    F["mean_minus_std"] = lambda S: S.mean(1) - S.std(1)
    F["trim0_5"] = lambda S: S[:, :-5].mean(1)
    return F


AGG = agg_funcs()
ALPHAS = [0.0, 0.25, 0.5, 0.75, 1.0]


def sorted_sims(uid_obj_list):
    return np.stack([np.sort(sims[k])[::-1] for k in uid_obj_list])


# ---------------------------------------------------------------- 라벨 완비 그룹 구성
groups = defaultdict(list)          # (ds, frame, object) -> [pair]
for p in pairs:
    if (p["uid"], p["object"]) in sims:
        groups[(p["dataset"], p["frame_id"], p["object"])].append(p)

full = {k: v for k, v in groups.items() if all(x["uid"] in labels for x in v)}
print(f"후보 그룹 {len(groups):,} / 라벨 완비 그룹 {len(full):,} "
      f"(박스 {sum(len(v) for v in full.values()):,})")

# 순위 평가용 pair (라벨된 박스만)
lab_pairs = [p for p in pairs if p["uid"] in labels and (p["uid"], p["object"]) in sims]
print(f"라벨된 후보 pair {len(lab_pairs):,}")


def auroc(pos, neg):
    if len(pos) == 0 or len(neg) == 0:
        return None
    p, n = np.asarray(pos, float), np.asarray(neg, float)
    return float(((p[:, None] > n[None, :]).sum() + 0.5 * (p[:, None] == n[None, :]).sum())
                 / (len(p) * len(n)))


def pr_auc(sc, y):
    o = np.argsort(-np.asarray(sc, float)); yy = np.asarray(y)[o]
    tp = np.cumsum(yy); fp = np.cumsum(1 - yy)
    prec = tp / np.maximum(tp + fp, 1); rec = tp / max(yy.sum(), 1)
    return float(np.sum(np.diff(np.concatenate([[0.0], rec])) * prec))


# ---------------------------------------------------------------- 점수 생성기
def make_scheme(name):
    """name -> (pair 리스트 -> 점수 배열). fusion 은 정규화 통계가 필요해 별도 처리."""
    if name.startswith("fusion"):
        return None
    f = AGG[name]
    return lambda ps: f(sorted_sims([(p["uid"], p["object"]) for p in ps]))


def zstats(ps, fn):
    """정규화 통계(평균·표준편차). 라벨이 필요 없으므로 calibration 의 **전 후보 pair**로 추정한다."""
    v = fn(sorted_sims([(p["uid"], p["object"]) for p in ps]))
    return float(v.mean()), float(v.std() + 1e-9)


SCHEMES = list(AGG) + [f"fusion_a{a}" for a in ALPHAS]

# ---------------------------------------------------------------- LODO
auc_rows, dec_rows, err_rows = [], [], []
for held in DATASETS:
    calib_p = [p for p in lab_pairs if p["dataset"] != held]
    test_p = [p for p in lab_pairs if p["dataset"] == held]
    calib_g = {k: v for k, v in full.items() if k[0] != held}
    test_g = {k: v for k, v in full.items() if k[0] == held}
    if not test_p or not test_g:
        continue

    # fusion 정규화 통계는 calibration 에서만 추정 (라벨 불필요 → 전 후보 pair 사용)
    calib_all = [p for p in pairs if p["dataset"] != held and (p["uid"], p["object"]) in sims]
    m5, s5 = zstats(calib_all, AGG["top5"])
    mm, sm = zstats(calib_all, AGG["top42"])

    def score(ps, name):
        if name.startswith("fusion"):
            a = float(name.split("_a")[1])
            S = sorted_sims([(p["uid"], p["object"]) for p in ps])
            return a * ((AGG["top5"](S) - m5) / s5) + (1 - a) * ((AGG["top42"](S) - mm) / sm)
        return AGG[name](sorted_sims([(p["uid"], p["object"]) for p in ps]))

    for name in SCHEMES:
        # --- 순위 지표 (객체별)
        sc_t = score(test_p, name)
        for obj in OBJS:
            idx = [i for i, p in enumerate(test_p) if p["object"] == obj]
            if len(idx) < 4:
                continue
            y = np.array([1 if labels[test_p[i]["uid"]] == obj else 0 for i in idx])
            if y.sum() < 2 or (1 - y).sum() < 2:
                continue
            s = sc_t[idx]
            auc_rows.append({"held_out": held, "scheme": name, "object": obj,
                             "n_pos": int(y.sum()), "n_neg": int((1 - y).sum()),
                             "auroc": round(auroc(s[y == 1], s[y == 0]), 4),
                             "pr_auc": round(pr_auc(s, y), 4)})

        # --- 결정 단위: threshold 를 calibration 그룹에서 F1 최대로 결정
        def decide(gs, thr, nm):
            ps_flat, owner = [], []
            for k, v in gs.items():
                for p in v:
                    ps_flat.append(p); owner.append(k)
            if not ps_flat:
                return None
            sc = score(ps_flat, nm)
            best = {}
            for i, k in enumerate(owner):
                if k not in best or sc[i] > best[k][0]:
                    best[k] = (sc[i], ps_flat[i])
            TP = FP = FN = TN = 0
            detail = []
            for k, (s, p) in best.items():
                obj = k[2]
                has = any(labels[x["uid"]] == obj for x in gs[k])
                acc = (s >= thr) and (p["appe"] >= p["appe_gate"])
                ok = labels[p["uid"]] == obj
                if acc and ok:
                    TP += 1; lab = "TP"
                elif acc and not ok:
                    FP += 1; lab = "FP"
                elif (not acc) and has:
                    FN += 1; lab = "FN"
                else:
                    TN += 1; lab = "TN"
                detail.append((k, lab, p["uid"], round(float(s), 4)))
            return TP, FP, FN, TN, detail

        # calibration 에서 threshold 탐색
        cps, cow = [], []
        for k, v in calib_g.items():
            for p in v:
                cps.append(p); cow.append(k)
        csc = score(cps, name)
        grid = np.quantile(csc, np.linspace(0.02, 0.98, 97))
        bestthr, bestf1 = grid[0], -1
        for t in grid:
            r = decide(calib_g, t, name)
            if r is None:
                continue
            TP, FP, FN, TN, _ = r
            f1 = 2 * TP / max(2 * TP + FP + FN, 1)
            if f1 > bestf1:
                bestf1, bestthr = f1, t
        r = decide(test_g, bestthr, name)
        if r is None:
            continue
        TP, FP, FN, TN, detail = r
        P = TP / max(TP + FP, 1); R = TP / max(TP + FN, 1)
        dec_rows.append({"held_out": held, "scheme": name,
                         "threshold": round(float(bestthr), 5),
                         "calib_f1": round(bestf1, 4),
                         "n_group": TP + FP + FN + TN,
                         "TP": TP, "FP": FP, "FN": FN, "TN": TN,
                         "precision": round(P, 4), "recall": round(R, 4),
                         "f1": round(2 * P * R / max(P + R, 1e-9), 4)})
        if True:   # 전 방식의 그룹별 결과를 남긴다 (paired_compare.py 가 사용)
            for k, lab, uid, s in detail:
                err_rows.append({"held_out": held, "scheme": name, "dataset": k[0],
                                 "frame_id": k[1], "object": k[2], "outcome": lab,
                                 "selected_uid": uid, "score": s,
                                 "selected_true_class": labels.get(uid, "")})

for p, rows in ((os.path.join(RES, "semantic_scheme_auroc.csv"), auc_rows),
                (os.path.join(RES, "semantic_scheme_decision.csv"), dec_rows),
                (os.path.join(RES, "semantic_error_delta.csv"), err_rows)):
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

# ---------------------------------------------------------------- 요약
print("\n=== LODO 합산 결정 단위 (held-out 6 fold 합계)")
agg = defaultdict(lambda: defaultdict(int))
for r in dec_rows:
    for k in ("TP", "FP", "FN", "TN"):
        agg[r["scheme"]][k] += r[k]
print(f"{'scheme':16s} {'TP':>5} {'FP':>5} {'FN':>5} {'TN':>5} {'Prec':>7} {'Rec':>7} {'F1':>7}")
base = None
order = []
for s in SCHEMES:
    a = agg[s]
    P = a["TP"] / max(a["TP"] + a["FP"], 1); R = a["TP"] / max(a["TP"] + a["FN"], 1)
    F = 2 * P * R / max(P + R, 1e-9)
    order.append((s, a["TP"], a["FP"], a["FN"], a["TN"], P, R, F))
    if s == "top5":
        base = (a["FP"], a["FN"])
for s, TP, FP, FN, TN, P, R, F in order:
    mark = ""
    if base:
        mark = ("  ← FP,FN 둘 다 악화 없음" if FP <= base[0] and FN <= base[1] else "")
    print(f"{s:16s} {TP:>5} {FP:>5} {FN:>5} {TN:>5} {P:>7.4f} {R:>7.4f} {F:>7.4f}{mark}")

print("\n=== LODO 객체별 AUROC (held-out 평균)")
by = defaultdict(lambda: defaultdict(list))
for r in auc_rows:
    by[r["scheme"]][r["object"]].append(r["auroc"])
show = ["top1", "top5", "top42", "min1", "bottom5", "harmonic", "geometric",
        "mean_minus_std", "fusion_a0.25", "fusion_a0.5", "fusion_a0.75"]
print(f"{'객체':22s} " + " ".join(f"{s[:9]:>10s}" for s in show))
for o in OBJS:
    if not by["top5"].get(o):
        continue
    print(f"{o:22s} " + " ".join(
        f"{np.mean(by[s][o]):>10.4f}" if by[s].get(o) else f"{'-':>10s}" for s in show))
print(f"{'평균':22s} " + " ".join(
    f"{np.mean([np.mean(by[s][o]) for o in by[s]]):>10.4f}" for s in show))
