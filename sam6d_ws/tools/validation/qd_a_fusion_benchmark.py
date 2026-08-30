#!/usr/bin/env python3
"""QD-A: SAM-6D Eq.(4) vs 학습 결합 오프라인 벤치마크 (numpy/scipy, sklearn 불요).

목적: milk 데이터에서 Eq.(4)를 능가하는 결합이 hold-out에서 가능한지 GO/NO-GO 판정.
입력(읽기전용): baseline frame_results(315 풀-GT) + detection_ism.json(Eq.4 score 재현)
             + 계측 run debug(top1-top2 margin, Model D).
프로덕션 코드 미수정.
"""
import csv, json, os, math
import numpy as np
from scipy.optimize import minimize

np.random.seed(1)
ROOT = "outputs/validation"
BASE = f"{ROOT}/SLAM_with_milk_nomilk_baseline_bak"
FR = f"{BASE}/frame_results.csv"
RUN = f"{BASE}/_run"
INSTR_DBG = f"{ROOT}/SLAM_with_milk_nomilk/_run/debug/sam6d_debug.csv"


def f(x):
    try: return float(x)
    except: return None


# ---------- load 315 baseline ----------
rows = []
with open(FR) as fh:
    for r in csv.DictReader(fh):
        vis = r["manually_labeled_milk_visible"]
        if vis not in ("True", "False"):
            continue
        rows.append(dict(
            fid=r["frame_id"],
            y=1 if vis == "True" else 0,
            sem=f(r["similarity_score"]), appe=f(r["mask_score"]),
            geo=f(r["bbox_score"]), pose=f(r["pose_score"]),
            dt=r["decision_type"]))
rows = [r for r in rows if None not in (r["sem"], r["appe"], r["geo"])]
rows.sort(key=lambda r: r["fid"])   # temporal order
N = len(rows)
P = sum(r["y"] for r in rows)
print(f"=== DATA: {N} GT frames (present={P}, absent={N-P}) ===")

# ---------- Eq.(4) score 재현 (detection_ism.json best-candidate score) ----------
eq4_cov = 0
for r in rows:
    p = os.path.join(RUN, r["fid"], "detection_ism.json")
    sc = None
    if os.path.isfile(p):
        try:
            d = json.load(open(p))
            if d:
                sc = max(float(x.get("score", -1)) for x in d)
        except Exception:
            sc = None
    r["eq4"] = sc
    if sc is not None:
        eq4_cov += 1
eq4_fallback = eq4_cov < int(0.8 * N)
if eq4_fallback:
    # fallback: r 미저장 → 동일가중 근사 (sem+appe+geo)/3, 경고
    for r in rows:
        r["eq4"] = (r["sem"] + r["appe"] + r["geo"]) / 3.0
    print(f"[WARN] detection_ism.json 커버리지 {eq4_cov}/{N} <80% → Eq.4 fallback=(sem+appe+geo)/3")
else:
    # 결측 프레임만 fallback 보정
    for r in rows:
        if r["eq4"] is None:
            r["eq4"] = (r["sem"] + r["appe"] + r["geo"]) / 3.0
    print(f"[OK] Eq.4 score = detection_ism.json best score (커버리지 {eq4_cov}/{N})")

# Task1 audit: Eq.4 score가 sem/appe/geo와 단조 정합인지 (코드감사는 별도, 여기선 수치 sanity)
y = np.array([r["y"] for r in rows])
eq4 = np.array([r["eq4"] for r in rows])


# ---------- metrics ----------
def auc_roc(score, lab):
    pos = score[lab == 1]; neg = score[lab == 0]
    if len(pos) == 0 or len(neg) == 0: return float("nan")
    allv = np.concatenate([pos, neg])
    order = allv.argsort()
    ranks = np.empty(len(allv)); ranks[order] = np.arange(1, len(allv) + 1)
    # tie correction
    _, inv, cnt = np.unique(allv, return_inverse=True, return_counts=True)
    avg = {}
    srt = np.sort(allv)
    i = 0
    while i < len(srt):
        j = i
        while j + 1 < len(srt) and srt[j + 1] == srt[i]: j += 1
        avg[srt[i]] = (i + j) / 2 + 1; i = j + 1
    R = sum(avg[s] for s in pos)
    U = R - len(pos) * (len(pos) + 1) / 2
    return U / (len(pos) * len(neg))


def pr_auc(score, lab):
    order = np.argsort(-score)
    l = lab[order]; tp = 0; fp = 0; Ptot = lab.sum(); area = 0; pr = 0
    for li in l:
        if li == 1: tp += 1
        else: fp += 1
        prec = tp / (tp + fp); rec = tp / Ptot
        area += (rec - pr) * prec; pr = rec
    return area


def prec_at_recall(score, lab, target_rec):
    order = np.argsort(-score)
    l = lab[order]; tp = 0; fp = 0; Ptot = lab.sum()
    best = 0.0
    for li in l:
        if li == 1: tp += 1
        else: fp += 1
        rec = tp / Ptot
        if rec >= target_rec:
            return tp / (tp + fp), rec
    return best, 0.0


# ---------- logistic regression (scipy) ----------
def fit_logreg(X, yv, l2=1.0):
    Xb = np.hstack([np.ones((len(X), 1)), X])
    def nll(w):
        z = Xb @ w
        ll = np.sum(yv * z - np.logaddexp(0, z))
        return -ll + l2 * np.sum(w[1:] ** 2)
    w0 = np.zeros(Xb.shape[1])
    res = minimize(nll, w0, method="L-BFGS-B")
    return res.x


def predict_logreg(w, X):
    Xb = np.hstack([np.ones((len(X), 1)), X])
    z = Xb @ w
    return 1 / (1 + np.exp(-z))


def standardize(Xtr, Xte, mu=None, sd=None):
    if mu is None:
        mu = Xtr.mean(0); sd = Xtr.std(0) + 1e-9
    return (Xtr - mu) / sd, (Xte - mu) / sd, mu, sd


# feature matrices
F3 = np.array([[r["sem"], r["appe"], r["geo"]] for r in rows])   # Model B/C
names3 = ["sem", "appe", "geo"]

# ---------- Task2/3: full-data separability (ranking AUC; 학습은 hold-out에서) ----------
print("\n=== [Task2/3] present-vs-absent 분리 (full-data ranking AUC, 참고용) ===")
single = {"sem": F3[:, 0], "appe": F3[:, 1], "geo": F3[:, 2],
          "pose": np.array([r["pose"] for r in rows]), "Eq4(A)": eq4}
for k, s in single.items():
    print(f"  {k:8s} ROC-AUC={auc_roc(s, y):.3f}  PR-AUC={pr_auc(s, y):.3f}")
# in-sample logreg (상한, 과적합 미포함 참고치)
Xs_all = standardize(F3, F3)[0]
w_in = fit_logreg(Xs_all, y, l2=1.0)
p_in = predict_logreg(w_in, Xs_all)
print(f"  C(logreg in-sample) ROC-AUC={auc_roc(p_in, y):.3f} (상한, 과적합 미반영)")


# ---------- Task4: hold-out (5-fold stratified CV + temporal split) ----------
def strat_folds(yv, k=5, seed=1):
    rng = np.random.RandomState(seed)
    idx0 = np.where(yv == 0)[0]; idx1 = np.where(yv == 1)[0]
    rng.shuffle(idx0); rng.shuffle(idx1)
    folds = [[] for _ in range(k)]
    for i, ix in enumerate(idx0): folds[i % k].append(ix)
    for i, ix in enumerate(idx1): folds[i % k].append(ix)
    return [np.array(sorted(fo)) for fo in folds]


def eval_model_cv(Xfull, yv, target_rec=0.90, k=5):
    folds = strat_folds(yv, k)
    aucs, praucs, precs, tr_aucs = [], [], [], []
    for i in range(k):
        te = folds[i]; tr = np.concatenate([folds[j] for j in range(k) if j != i])
        Xtr, Xte, mu, sd = standardize(Xfull[tr], Xfull[te])
        w = fit_logreg(Xtr, yv[tr], l2=1.0)
        pte = predict_logreg(w, Xte); ptr = predict_logreg(w, Xtr)
        aucs.append(auc_roc(pte, yv[te])); tr_aucs.append(auc_roc(ptr, yv[tr]))
        praucs.append(pr_auc(pte, yv[te]))
        precs.append(prec_at_recall(pte, yv[te], target_rec)[0])
    return (np.mean(aucs), np.std(aucs), np.mean(praucs),
            np.mean(precs), np.mean(tr_aucs))


def eval_eq4_cv(score, yv, target_rec=0.90, k=5):
    # Eq.4 는 학습 없음 → fold test 에서 같은 score 의 AUC/precision
    folds = strat_folds(yv, k)
    aucs, praucs, precs = [], [], []
    for i in range(k):
        te = folds[i]
        aucs.append(auc_roc(score[te], yv[te]))
        praucs.append(pr_auc(score[te], yv[te]))
        precs.append(prec_at_recall(score[te], yv[te], target_rec)[0])
    return np.mean(aucs), np.std(aucs), np.mean(praucs), np.mean(precs)


print("\n=== [Task4] 5-fold CV hold-out (present=positive, precision@recall=0.90) ===")
for rec_t in (0.90, 0.95):
    a_auc, a_sd, a_pr, a_prec = eval_eq4_cv(eq4, y, rec_t)
    c_auc, c_sd, c_pr, c_prec, c_tr = eval_model_cv(F3, y, rec_t)
    print(f" [recall≥{rec_t}]")
    print(f"   A Eq.4         : ROC-AUC={a_auc:.3f}±{a_sd:.3f} PR-AUC={a_pr:.3f} P@R={a_prec:.3f}")
    print(f"   C logreg[3]    : ROC-AUC={c_auc:.3f}±{c_sd:.3f} PR-AUC={c_pr:.3f} P@R={c_prec:.3f}  (train-AUC={c_tr:.3f}, gap={c_tr-c_auc:+.3f})")
    print(f"   → ΔP@R(C-A)={c_prec-a_prec:+.3f}  ΔROC-AUC={c_auc-a_auc:+.3f}")

# temporal split (앞 60% train, 뒤 40% test)
print("\n=== [Task4] Temporal split (앞60% train / 뒤40% test) ===")
cut = int(0.6 * N)
tr = np.arange(cut); te = np.arange(cut, N)
print(f"  train n={len(tr)} (present {y[tr].sum()}), test n={len(te)} (present {y[te].sum()})")
Xtr, Xte, mu, sd = standardize(F3[tr], F3[te])
w = fit_logreg(Xtr, y[tr], l2=1.0)
pte = predict_logreg(w, Xte)
for rec_t in (0.90,):
    a_p = prec_at_recall(eq4[te], y[te], rec_t)[0]
    c_p = prec_at_recall(pte, y[te], rec_t)[0]
    print(f"  [recall≥{rec_t}] A Eq.4 ROC-AUC={auc_roc(eq4[te],y[te]):.3f} P@R={a_p:.3f} | "
          f"C logreg ROC-AUC={auc_roc(pte,y[te]):.3f} P@R={c_p:.3f} | ΔP@R={c_p-a_p:+.3f}")


# ---------- Task5: calibration (Platt on Eq.4) ----------
def ece(prob, lab, bins=10):
    e = 0
    for b in range(bins):
        lo, hi = b / bins, (b + 1) / bins
        m = (prob >= lo) & (prob < hi)
        if m.sum() == 0: continue
        e += abs(prob[m].mean() - lab[m].mean()) * m.sum() / len(lab)
    return e

# normalize eq4 to [0,1] as naive "prob"
eq4n = (eq4 - eq4.min()) / (eq4.max() - eq4.min() + 1e-9)
# Platt: logreg on raw eq4 (5-fold, report test ECE)
folds = strat_folds(y, 5)
ece_raw, ece_platt = [], []
for i in range(5):
    te = folds[i]; tr = np.concatenate([folds[j] for j in range(5) if j != i])
    w = fit_logreg(eq4[tr].reshape(-1, 1), y[tr], l2=0.1)
    pte = predict_logreg(w, eq4[te].reshape(-1, 1))
    ece_platt.append(ece(pte, y[te])); ece_raw.append(ece(eq4n[te], y[te]))
print("\n=== [Task5] Calibration (Eq.4 score) ===")
print(f"  ECE raw(minmax)={np.mean(ece_raw):.3f}  ECE Platt={np.mean(ece_platt):.3f}")
print(f"  → AUC는 Platt/temperature 로 불변(단조변환). calibration 은 임계해석만 개선, 분리력↑ 아님.")


# ---------- Task6: feature attribution (표준화 계수 + ablation) ----------
print("\n=== [Task6] Feature Attribution ===")
Xs, _, _, _ = standardize(F3, F3)
w_full = fit_logreg(Xs, y, l2=1.0)
print(f"  표준화 logreg 계수: intercept={w_full[0]:+.3f}  " +
      "  ".join(f"{names3[i]}={w_full[i+1]:+.3f}" for i in range(3)))
base_auc, *_ = eval_model_cv(F3, y, 0.90)
for i in range(3):
    cols = [j for j in range(3) if j != i]
    abl_auc, *_ = eval_model_cv(F3[:, cols], y, 0.90)
    print(f"  ablation drop {names3[i]:5s}: CV ROC-AUC {base_auc:.3f} → {abl_auc:.3f} (Δ={abl_auc-base_auc:+.3f})")


# ---------- Model D: + top1-top2 margin (계측 56프레임만) ----------
print("\n=== [Task3 Model D] +top1-top2 margin (계측 run, 저파워) ===")
mar = {}
try:
    import csv as _c
    best = {}
    for r in _c.DictReader(open(INSTR_DBG)):
        if str(r.get("is_best", "")).lower() == "true":
            best[r["frame_id"]] = r
    # need GT for these frames: use instrumented run frame_results
    instr_fr = f"{ROOT}/SLAM_with_milk_nomilk/frame_results.csv"
    gt = {r["frame_id"]: r["manually_labeled_milk_visible"]
          for r in _c.DictReader(open(instr_fr))}
    Xd, yd = [], []
    for fid, row in best.items():
        g = gt.get(fid)
        if g not in ("True", "False"): continue
        t5 = json.loads(row["top5_template_scores"]) if row.get("top5_template_scores") else []
        if len(t5) < 2: continue
        sem = f(row["similarity_score"]); appe = f(row["appearance_score"]); geo = f(row["geometric_score"])
        if None in (sem, appe, geo): continue
        Xd.append([sem, appe, geo, t5[0] - t5[1]]); yd.append(1 if g == "True" else 0)
    Xd = np.array(Xd); yd = np.array(yd)
    print(f"  Model D 표본: n={len(yd)} (present={yd.sum()}, absent={len(yd)-yd.sum()})")
    if len(yd) >= 30 and 5 <= yd.sum() <= len(yd) - 5:
        a3, sd3, *_ = eval_model_cv(Xd[:, :3], yd, 0.90, k=5)
        a4, sd4, *_ = eval_model_cv(Xd, yd, 0.90, k=5)
        print(f"  CV ROC-AUC [sem,appe,geo]={a3:.3f}±{sd3:.3f}  +margin={a4:.3f}±{sd4:.3f}  Δ={a4-a3:+.3f}")
        print("  ⚠️ n작음(저파워) — 참고용.")
    else:
        print("  ⚠️ 표본 부족 → Model D 생략(저파워).")
except Exception as e:
    print(f"  Model D 계산 실패: {e}")

print("\n=== DONE ===")
