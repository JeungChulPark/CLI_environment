#!/usr/bin/env python3
"""eval_fusion_modes.py — semantic+appearance 에 HSV 를 더하는 7가지 결합을 LODO 로 비교 (READ-ONLY).

고정 (색 효과만 분리하기 위해)
  YOLO raw/conf/NMS 결과 · 후보 BBox · top_k · MobileSAM mask · semantic · appearance
  → 전부 운영 덤프(ism_accuracy_observation)를 그대로 사용. 재계산하지 않는다.

결합 방식
  A  baseline           sem >= sim_thr AND appe >= appe_gate            (운영 그대로)
  B  hard gate          A AND hsv >= t_hsv                              (F1 최대로 t_hsv 보정)
  C  단순 평균          (z(sem)+z(appe)+z(hsv))/3 >= t                  선택도 이 점수로
  D  공통 가중 평균     w_s·z(sem)+w_a·z(appe)+w_h·z(hsv) >= t          w 는 공통(객체별 아님)
  E  rank fusion        세 점수를 calibration 분포의 백분위로 바꿔 평균
  F  uncertainty-only   baseline 이 확신하면 그대로, 애매할 때만 HSV 로 재판정
  G  negative rejection A AND hsv >= t_low, 단 t_low 는 **TP 손실 <= 1%** 제약 하에 FP 최소화

평가 GT
  사람 가시성 GT 339 프레임(935 가시) + triage 334 (검출가능 여부)
  TP = 보이는데 수락 / FP = 안 보이는데 수락 / FN = 보이는데 미수락
  ⚠ 프레임 단위라 수락 박스의 위치 정확성은 확인하지 않는다.
  ⚠ HSV 는 **없는 BBox 를 만들 수 없다**. 후보가 아예 없는 FN(구조적 하한)을 따로 집계한다.

산출: results/baseline_vs_hsv_{all,by_object,by_dataset}.csv, decision_outcomes.csv
"""
import csv, itertools, json, os
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
FUS = os.path.join(RSRCH, "ism_fusion_research")
GT = os.path.join(RSRCH, "gt_input")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
HUE_SHIFT, SAT_GAIN, LOWSAT_TH = -6, 1.3, 20.0
MASKED = slice(0, 224); SL_HS = slice(96, 224)
SL_S, SL_V = slice(32, 64), slice(64, 96)

# ---------------------------------------------------------------- GT
gt, detectable = {}, {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        toks = [t.strip() for t in r["visible_objects"].split(";") if t.strip()]
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(t for t in toks if t in OBJECTS)
for r in csv.DictReader(open(os.path.join(GT, "triage_answers.csv"), encoding="utf-8")):
    detectable[(r["dataset"], int(r["frame_id"]), r["object"])] = r["verdict"]
print(f"GT 프레임 {len(gt)} / 가시 {sum(len(v) for v in gt.values())} / triage {len(detectable)}")

# ---------------------------------------------------------------- 고정 후보 (운영 덤프)
hist = {}
for ds in DATASETS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        hist[str(u)] = h[MASKED].astype(np.float64)

cand = defaultdict(list)
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        k = (ds, int(r["frame_id"]), r["object"])
        if (ds, int(r["frame_id"])) not in gt:
            continue
        cand[k].append({"uid": r["uid"], "sem": float(r["sem_top5"]),
                        "appe": float(r["appe11_clstop1"]),
                        "sim_thr": float(r["sim_thr"]), "appe_gate": float(r["appe_gate"])})
print(f"고정 후보 그룹 {len(cand)} / 박스 {sum(len(v) for v in cand.values())}")


# ---------------------------------------------------------------- HSV reference (H2 채택 + H5 비교)
def feat_rgb(rgb, space, shift=0, gain=1.0):
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)
    if shift:
        hsv[..., 0] = (hsv[..., 0] + int(round(shift / 2))) % 180
    if gain != 1.0:
        hsv[..., 1] = np.clip(hsv[..., 1] * gain, 0, 255)
    hsv = hsv.astype(np.uint8)
    if space == "hs":
        x = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
        return (x / max(x.sum(), 1e-12)).astype(np.float64)
    a = cv2.calcHist([hsv], [1], None, [32], [0, 256]).flatten()
    b = cv2.calcHist([hsv], [2], None, [32], [0, 256]).flatten()
    return (np.concatenate([a / max(a.sum(), 1e-12), b / max(b.sum(), 1e-12)]) / 2).astype(np.float64)


def feat_hist(h224, space):
    x = h224[SL_HS] if space == "hs" else np.concatenate([h224[SL_S], h224[SL_V]])
    if space == "sv":
        a, b = np.split(x, 2)
        return np.concatenate([a / max(a.sum(), 1e-12), b / max(b.sum(), 1e-12)]) / 2
    return x / max(x.sum(), 1e-12)


PC = np.load(os.path.join(FUS, "ply_hsv", "prototype_colors.npz"))
SAT = {}
for o in OBJECTS:
    a = PC[f"{o}__render42"].reshape(-1, 1, 3)[:, :, ::-1]
    SAT[o] = float(cv2.cvtColor(a, cv2.COLOR_BGR2HSV)[..., 1].mean())
LOW = {o for o in OBJECTS if SAT[o] < LOWSAT_TH}


def build(ref, lowsat):
    P = {}
    for o in OBJECTS:
        sp, sh, gn = "hs", HUE_SHIFT, SAT_GAIN
        if lowsat == "svfall" and o in LOW:
            sp, sh, gn = "sv", 0, SAT_GAIN
        elif lowsat == "skip" and o in LOW:
            sh = 0
        blocks = [PC[f"{o}__render42"]]
        if ref == "H5":
            blocks.append(PC[f"{o}__ply_view42"])
        P[o] = (np.vstack([np.stack([feat_rgb(c, sp, sh, gn) for c in b]) for b in blocks]), sp)
    return P


def hsv_of(uid, obj, P):
    M, sp = P[obj]
    q = feat_hist(hist[uid], sp)
    bc = np.sqrt(np.maximum(q[None, :] * M, 0)).sum(1)
    return float((1.0 - np.sqrt(np.maximum(0.0, 1.0 - bc))).max())


REF, LOWSAT = os.environ.get("REF", "H2"), os.environ.get("LOWSAT", "svfall")
PROTO = build(REF, LOWSAT)
for k, v in cand.items():
    for c in v:
        c["hsv"] = hsv_of(c["uid"], k[2], PROTO)
print(f"HSV reference = {REF} / 저채도 처리 = {LOWSAT} / 저채도 객체 = {sorted(LOW)}")


# ---------------------------------------------------------------- 판정
def zs(keys):
    a = {k: np.array([c[k] for kk in keys for c in cand[kk]]) for k in ("sem", "appe", "hsv")}
    return {k: (v.mean(), v.std() + 1e-9) for k, v in a.items()}


def pct_maker(keys):
    ref = {k: np.sort(np.array([c[k] for kk in keys for c in cand[kk]]))
           for k in ("sem", "appe", "hsv")}
    return lambda k, v: float(np.searchsorted(ref[k], v) / max(len(ref[k]), 1))


def decide(keys, mode, par, Z, pct):
    """mode 별 (frame,object) 수락 여부 + 선택 박스."""
    out = {}
    for k in keys:
        v = cand[k]
        if mode in ("A", "B", "G"):
            b = max(v, key=lambda c: c["sem"])
            ok = (b["sem"] >= b["sim_thr"]) and (b["appe"] >= b["appe_gate"])
            if mode in ("B", "G") and ok:
                ok = b["hsv"] >= par["t_hsv"]
        elif mode in ("C", "D"):
            w = par["w"]
            for c in v:
                c["_s"] = sum(w[i] * ((c[nm] - Z[nm][0]) / Z[nm][1])
                              for i, nm in enumerate(("sem", "appe", "hsv")))
            b = max(v, key=lambda c: c["_s"])
            ok = b["_s"] >= par["t"]
        elif mode == "E":
            for c in v:
                c["_s"] = (pct("sem", c["sem"]) + pct("appe", c["appe"])
                           + pct("hsv", c["hsv"])) / 3.0
            b = max(v, key=lambda c: c["_s"])
            ok = b["_s"] >= par["t"]
        elif mode == "F":
            b = max(v, key=lambda c: c["sem"])
            base_ok = (b["sem"] >= b["sim_thr"]) and (b["appe"] >= b["appe_gate"])
            # 확신 구간: appe 가 게이트에서 충분히 떨어져 있으면 baseline 유지
            margin = abs(b["appe"] - b["appe_gate"])
            if margin >= par["m"]:
                ok = base_ok
            else:
                ok = base_ok and (b["hsv"] >= par["t_hsv"])
        else:
            raise ValueError(mode)
        out[k] = (ok, b)
    return out


def score(res, frames, use_detectable=False):
    TP = FP = FN = 0
    FN_nocand = 0
    per = defaultdict(lambda: [0, 0, 0])
    for (ds, f) in frames:
        vis = gt[(ds, f)]
        for o in OBJECTS:
            k = (ds, f, o)
            acc = res.get(k, (False, None))[0]
            v = o in vis
            if use_detectable and v and not acc:
                if detectable.get(k, "D") in ("B", "N"):
                    continue                       # 검출 불가/라벨오류는 분모에서 제외
            if v and acc:
                TP += 1; per[o][0] += 1
            elif acc:
                FP += 1; per[o][1] += 1
            elif v:
                FN += 1; per[o][2] += 1
                if k not in cand:
                    FN_nocand += 1
    return TP, FP, FN, FN_nocand, per


# 가중치 격자 (공통, 객체별 아님)
WGRID = [(a / 10, b / 10, c / 10) for a in range(0, 11) for b in range(0, 11)
         for c in range(0, 11) if a + b + c == 10]
MODES = ["A", "B", "C", "D", "E", "F", "G"]
MLABEL = {"A": "baseline (sem+appe)", "B": "HSV hard gate",
          "C": "단순 평균 (1/3,1/3,1/3)", "D": "공통 가중 평균",
          "E": "rank fusion (백분위 평균)", "F": "uncertainty-only HSV",
          "G": "negative rejection (TP손실≤1%)"}

agg = defaultdict(lambda: defaultdict(int))
per_obj = defaultdict(lambda: defaultdict(lambda: [0, 0, 0]))
per_ds = defaultdict(lambda: defaultdict(int))
outcomes = defaultdict(dict)
fold_rows, chosen = [], defaultdict(list)

for held in DATASETS:
    tr_f = [x for x in gt if x[0] != held]; te_f = [x for x in gt if x[0] == held]
    tr_k = [k for k in cand if k[0] != held]; te_k = [k for k in cand if k[0] == held]
    Z, pct = zs(tr_k), pct_maker(tr_k)

    for mode in MODES:
        par = {}
        if mode == "A":
            pass
        elif mode in ("B",):
            grid = np.quantile([c["hsv"] for k in tr_k for c in cand[k]],
                               np.linspace(0.0, 0.9, 46))
            best = -1
            for t in grid:
                TP, FP, FN, _, _ = score(decide(tr_k, mode, {"t_hsv": t}, Z, pct), tr_f)
                f1 = 2 * TP / max(2 * TP + FP + FN, 1)
                if f1 > best:
                    best, par = f1, {"t_hsv": float(t)}
        elif mode == "G":
            TP0, FP0, _, _, _ = score(decide(tr_k, "A", {}, Z, pct), tr_f)
            grid = np.quantile([c["hsv"] for k in tr_k for c in cand[k]],
                               np.linspace(0.0, 0.9, 46))
            bestfp, par = FP0, {"t_hsv": 0.0}
            for t in grid:
                TP, FP, FN, _, _ = score(decide(tr_k, "G", {"t_hsv": t}, Z, pct), tr_f)
                if TP >= TP0 * 0.99 and FP < bestfp:
                    bestfp, par = FP, {"t_hsv": float(t)}
        elif mode == "C":
            w = (1 / 3, 1 / 3, 1 / 3)
            sc = []
            for k in tr_k:
                for c in cand[k]:
                    sc.append(sum(w[i] * ((c[nm] - Z[nm][0]) / Z[nm][1])
                                  for i, nm in enumerate(("sem", "appe", "hsv"))))
            best = -1
            for t in np.quantile(sc, np.linspace(0.02, 0.98, 50)):
                TP, FP, FN, _, _ = score(decide(tr_k, mode, {"w": w, "t": t}, Z, pct), tr_f)
                f1 = 2 * TP / max(2 * TP + FP + FN, 1)
                if f1 > best:
                    best, par = f1, {"w": w, "t": float(t)}
        elif mode == "D":
            best = -1
            for w in WGRID:
                sc = [sum(w[i] * ((c[nm] - Z[nm][0]) / Z[nm][1])
                          for i, nm in enumerate(("sem", "appe", "hsv")))
                      for k in tr_k for c in cand[k]]
                for t in np.quantile(sc, np.linspace(0.05, 0.95, 19)):
                    TP, FP, FN, _, _ = score(decide(tr_k, mode, {"w": w, "t": t}, Z, pct), tr_f)
                    f1 = 2 * TP / max(2 * TP + FP + FN, 1)
                    if f1 > best:
                        best, par = f1, {"w": w, "t": float(t)}
        elif mode == "E":
            sc = [(pct("sem", c["sem"]) + pct("appe", c["appe"]) + pct("hsv", c["hsv"])) / 3
                  for k in tr_k for c in cand[k]]
            best = -1
            for t in np.quantile(sc, np.linspace(0.02, 0.98, 50)):
                TP, FP, FN, _, _ = score(decide(tr_k, mode, {"t": t}, Z, pct), tr_f)
                f1 = 2 * TP / max(2 * TP + FP + FN, 1)
                if f1 > best:
                    best, par = f1, {"t": float(t)}
        elif mode == "F":
            best = -1
            hg = np.quantile([c["hsv"] for k in tr_k for c in cand[k]], np.linspace(0.0, 0.9, 19))
            for m in (0.02, 0.05, 0.10, 0.15, 0.25, 0.40):
                for t in hg:
                    TP, FP, FN, _, _ = score(
                        decide(tr_k, mode, {"m": m, "t_hsv": t}, Z, pct), tr_f)
                    f1 = 2 * TP / max(2 * TP + FP + FN, 1)
                    if f1 > best:
                        best, par = f1, {"m": m, "t_hsv": float(t)}

        r = decide(te_k, mode, par, Z, pct)
        TP, FP, FN, FNnc, per = score(r, te_f)
        dTP, dFP, dFN, _, _ = score(r, te_f, use_detectable=True)
        for kk, vv in (("TP", TP), ("FP", FP), ("FN", FN), ("FN_nocand", FNnc),
                       ("dTP", dTP), ("dFP", dFP), ("dFN", dFN)):
            agg[mode][kk] += vv
            per_ds[(mode, held)][kk] += vv
        for o in OBJECTS:
            for i in range(3):
                per_obj[mode][o][i] += per[o][i]
        for (ds, f) in te_f:
            for o in OBJECTS:
                k = (ds, f, o)
                acc = r.get(k, (False, None))[0]
                sel = r.get(k, (False, None))[1]
                v = o in gt[(ds, f)]
                outcomes[mode][k] = ("TP" if v and acc else "FP" if acc
                                     else "FN" if v else "TN",
                                     sel["uid"] if sel else "", sel["hsv"] if sel else "")
        chosen[mode].append(par)
        fold_rows.append({"held_out": held, "mode": mode, "label": MLABEL[mode],
                          "params": json.dumps(par, ensure_ascii=False),
                          "TP": TP, "FP": FP, "FN": FN, "FN_no_candidate": FNnc})

# ---------------------------------------------------------------- 출력
with open(os.path.join(RES, "baseline_vs_hsv_all.csv"), "w", newline="") as f:
    ks = ["mode", "label", "TP", "FP", "FN", "FN_no_candidate", "precision", "recall", "f1",
          "TP_detectable", "FP_detectable", "FN_detectable", "recall_detectable"]
    w = csv.DictWriter(f, fieldnames=ks); w.writeheader()
    for m in MODES:
        x = agg[m]
        P = x["TP"] / max(x["TP"] + x["FP"], 1); R = x["TP"] / max(x["TP"] + x["FN"], 1)
        dR = x["dTP"] / max(x["dTP"] + x["dFN"], 1)
        w.writerow({"mode": m, "label": MLABEL[m], "TP": x["TP"], "FP": x["FP"], "FN": x["FN"],
                    "FN_no_candidate": x["FN_nocand"], "precision": round(P, 4),
                    "recall": round(R, 4), "f1": round(2 * P * R / max(P + R, 1e-9), 4),
                    "TP_detectable": x["dTP"], "FP_detectable": x["dFP"],
                    "FN_detectable": x["dFN"], "recall_detectable": round(dR, 4)})

print(f"\n=== LODO held-out 합산 (사람 GT 935 가시 / 2455 비가시)  ref={REF} lowsat={LOWSAT}")
print(f"{'':3s} {'설명':26s} {'TP':>5} {'FP':>5} {'FN':>5} {'(후보無FN)':>9} "
      f"{'Prec':>7} {'Rec':>7} {'F1':>7} {'Rec_검출가능':>11}")
A = agg["A"]
for m in MODES:
    x = agg[m]
    P = x["TP"] / max(x["TP"] + x["FP"], 1); R = x["TP"] / max(x["TP"] + x["FN"], 1)
    dR = x["dTP"] / max(x["dTP"] + x["dFN"], 1)
    mark = ""
    if m != "A":
        mark = ("  ★FP감소·FN증가없음" if x["FP"] < A["FP"] and x["FN"] <= A["FN"] else "")
    print(f"{m:3s} {MLABEL[m]:26s} {x['TP']:>5} {x['FP']:>5} {x['FN']:>5} "
          f"{x['FN_nocand']:>9} {P:>7.4f} {R:>7.4f} {2*P*R/max(P+R,1e-9):>7.4f} {dR:>11.4f}{mark}")

with open(os.path.join(RES, "baseline_vs_hsv_by_object.csv"), "w", newline="") as f:
    ks = ["object"] + [f"{m}_{s}" for m in MODES for s in ("TP", "FP", "FN")]
    w = csv.DictWriter(f, fieldnames=ks); w.writeheader()
    for o in OBJECTS:
        row = {"object": o}
        for m in MODES:
            row[f"{m}_TP"], row[f"{m}_FP"], row[f"{m}_FN"] = per_obj[m][o]
        w.writerow(row)
with open(os.path.join(RES, "baseline_vs_hsv_by_dataset.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["dataset", "mode", "TP", "FP", "FN"])
    for (m, ds), x in sorted(per_ds.items(), key=lambda t: (t[0][1], t[0][0])):
        w.writerow([ds, m, x["TP"], x["FP"], x["FN"]])
with open(os.path.join(RES, "fold_params.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(fold_rows[0].keys())); w.writeheader(); w.writerows(fold_rows)

np.save(os.path.join(RES, "_outcomes.npy"),
        np.array([{m: {f"{k[0]}|{k[1]}|{k[2]}": v for k, v in outcomes[m].items()}
                   for m in MODES}], dtype=object), allow_pickle=True)

print(f"\n=== 객체별 TP/FP/FN")
print(f"{'객체':22s} " + " ".join(f"{m:>14s}" for m in MODES))
for o in OBJECTS:
    print(f"{o:22s} " + " ".join(
        f"{per_obj[m][o][0]:>4}/{per_obj[m][o][1]:>3}/{per_obj[m][o][2]:>3} " for m in MODES))

print(f"\n=== fold 별 선택 파라미터 (공통 규칙인지 확인)")
for m in MODES:
    if m in ("A",):
        continue
    print(f"  {m}: {[json.dumps(p, ensure_ascii=False) for p in chosen[m]]}")
print(f"\n-> {RES}")
