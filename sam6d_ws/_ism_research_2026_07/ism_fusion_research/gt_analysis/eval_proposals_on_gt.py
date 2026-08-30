#!/usr/bin/env python3
"""eval_proposals_on_gt.py — 제안한 변경들을 **사람 가시성 GT** 로 검증한다 (READ-ONLY).

지금까지 semantic fusion(α=0.30)과 HSV 색보정은 347개 라벨완비 그룹(잠정 라벨)에서만
평가했다. 이제 사용자가 직접 만든 935건(frame×object) GT 가 있으므로 여기서 재검증한다.

비교 팔 (arm)
  A0   현행 그대로            sem_top5 + config threshold(0.35) + appe_gate(0.55)
  A0c  현행 점수 + 보정 thr   sem_top5 + LODO 보정 threshold        ← 공정 비교용 기준선
  A1   fusion α=0.30          0.3·z(top5)+0.7·z(mean) + LODO 보정 threshold
  A2   A0c + HSV 게이트       색보정 렌더 HSV 를 3번째 게이트로 추가
  A3   A1  + HSV 게이트
  A4   A0 + Rabbit 임계 완화  score_threshold 0.30 → 0.20 (proposal miss 의 53% 가 Rabbit)

공정성
  · A0 만 config 값을 쓰고 나머지는 LODO 로 보정한다. A0 와 A1 를 바로 비교하면
    aggregation 효과와 threshold 재보정 효과가 섞이므로 **A0c 를 기준선으로 읽어야** 한다.
  · threshold 는 held-out 데이터셋을 제외한 5개 GT 프레임에서만 F1 최대로 고른다.

GT 정의 (프레임 단위)
  TP = 보이는데 수락 / FP = 안 보이는데 수락 / FN = 보이는데 미수락
  ⚠ 프레임 단위 GT 라 **수락된 박스가 실제로 그 객체 위치인지는 확인하지 않는다**(BBox 부재).
  ⚠ 가시 기준이 관대하므로 recall 은 하한, FP 는 상한이다(라벨 작성자 진술).

산출: results/gt_arm_comparison.csv, results/gt_arm_per_object.csv, results/gt_arm_paired.csv
"""
import csv, math, os
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
REPO = os.path.dirname(RSRCH)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
GT = os.path.join(RSRCH, "gt_input")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
HS = slice(96, 224)          # 관찰 덤프 masked 구간 내 H16xS8
MASKED = slice(0, 224)
HUE_SHIFT, SAT_GAIN, LOWSAT_TH = -6, 1.3, 20.0

# ---------------------------------------------------------------- GT
gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        toks = [t.strip() for t in r["visible_objects"].split(";") if t.strip()]
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(t for t in toks if t in OBJECTS)
print(f"GT 프레임 {len(gt)}  가시 (frame×object) {sum(len(v) for v in gt.values())}")

# ---------------------------------------------------------------- 후보 + 점수 재료
Z = np.load(os.path.join(ROOT, "semantic_fusion", "box_cls.npz"))
TCLS = {k[len("__tcls__"):]: Z[k] for k in Z.files if k.startswith("__tcls__")}

hsvq = {}
for ds in DATASETS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        hsvq[str(u)] = h[MASKED][HS].astype(np.float64)

cand = defaultdict(list)
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_pairs.csv"))):
        if int(r["is_candidate"]) != 1:
            continue
        k = (ds, int(r["frame_id"]), r["object"])
        if (ds, int(r["frame_id"])) not in gt:
            continue
        u = r["uid"]
        s = np.sort(TCLS[r["object"]] @ Z[u])[::-1]
        cand[k].append({"uid": u, "S": s,
                        "top5": float(s[:5].mean()), "mean": float(s.mean()),
                        "appe": float(r["appe11_clstop1"]),
                        "appe_gate": float(r["appe_gate"]),
                        "sim_thr": float(r["sim_thr"]),
                        "yolo_conf": float(r["yolo_conf"]),
                        "score_threshold": float(r["sim_thr"])})
print(f"GT 프레임 내 후보 그룹 {len(cand)}")

# 객체별 YOLO score_threshold (config 값) — A4 용
YTH = {}
for ds in DATASETS:
    for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_yolo.csv"))):
        YTH.setdefault(r["object"], float(r["score_threshold"]))

# ---------------------------------------------------------------- HSV prototype (색보정)
def hist_hs(rgb, shift=0, gain=1.0):
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)
    if shift:
        hsv[..., 0] = (hsv[..., 0] + int(round(shift / 2))) % 180
    if gain != 1.0:
        hsv[..., 1] = np.clip(hsv[..., 1] * gain, 0, 255)
    hsv = hsv.astype(np.uint8)
    h = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
    return (h / max(h.sum(), 1e-12)).astype(np.float64)


PC = np.load(os.path.join(ROOT, "ply_hsv", "prototype_colors.npz"))
PROTO, SAT = {}, {}
for o in OBJECTS:
    k = f"{o}__render42"
    if k not in PC.files:
        continue
    cols = PC[k]
    a = cols.reshape(-1, 3)
    SAT[o] = float(cv2.cvtColor(a.reshape(-1, 1, 3)[:, :, ::-1], cv2.COLOR_BGR2HSV)[..., 1].mean())
    sh = 0 if SAT[o] < LOWSAT_TH else HUE_SHIFT          # 저채도 객체는 Hue 보정 제외
    PROTO[o] = np.stack([hist_hs(c, sh, SAT_GAIN) for c in cols])
print("HSV prototype:", {o: (round(SAT[o], 1), "hue보정" if SAT[o] >= LOWSAT_TH else "저채도예외")
                         for o in sorted(SAT)})


def hsv_sim(uid, obj):
    q = hsvq.get(uid)
    if q is None or obj not in PROTO:
        return 1.0                                      # 정보 없으면 게이트를 통과시킨다
    bc = np.sqrt(np.maximum(q[None, :] * PROTO[obj], 0)).sum(1)
    return float((1.0 - np.sqrt(np.maximum(0.0, 1.0 - bc))).max())


for k, v in cand.items():
    for c in v:
        c["hsv"] = hsv_sim(c["uid"], k[2])

# fusion 정규화 통계 (라벨 불필요 → 전체 후보에서, fold 별로 재계산)
def zstat(keys):
    t5 = np.array([c["top5"] for k in keys for c in cand[k]])
    mn = np.array([c["mean"] for k in keys for c in cand[k]])
    return t5.mean(), t5.std() + 1e-9, mn.mean(), mn.std() + 1e-9


# ---------------------------------------------------------------- 판정
def decide(keys, arm, sem_thr, hsv_thr, zs):
    """arm 규칙으로 (ds,frame,object) 별 수락 여부를 낸다."""
    m5, s5, mm, sm = zs
    res = {}
    for k in keys:
        v = cand[k]
        if arm == "A4":     # Rabbit 임계 완화: YOLO conf 가 0.20~0.30 인 후보도 허용
            th = 0.20 if k[2] == "Rabbit" else YTH.get(k[2], 0.02)
            v = [c for c in v if c["yolo_conf"] >= th] or v
        if not v:
            continue
        if arm in ("A1", "A3"):
            for c in v:
                c["_s"] = 0.3 * ((c["top5"] - m5) / s5) + 0.7 * ((c["mean"] - mm) / sm)
        else:
            for c in v:
                c["_s"] = c["top5"]
        b = max(v, key=lambda c: c["_s"])
        ok = (b["_s"] >= sem_thr) and (b["appe"] >= b["appe_gate"])
        if arm in ("A2", "A3"):
            ok = ok and (b["hsv"] >= hsv_thr)
        res[k] = (ok, b)
    return res


def score(res, keys_frames):
    TP = FP = FN = TN = 0
    per = defaultdict(lambda: [0, 0, 0])
    for (ds, f) in keys_frames:
        vis = gt[(ds, f)]
        for o in OBJECTS:
            k = (ds, f, o)
            acc = res.get(k, (False, None))[0]
            v = o in vis
            if v and acc:
                TP += 1; per[o][0] += 1
            elif (not v) and acc:
                FP += 1; per[o][1] += 1
            elif v and not acc:
                FN += 1; per[o][2] += 1
            else:
                TN += 1
    return TP, FP, FN, TN, per


ARMS = ["A0", "A0c", "A1", "A2", "A3", "A4"]
LABEL = {"A0": "현행 (config thr 0.35)", "A0c": "현행 점수 + LODO 보정 thr",
         "A1": "fusion α=0.30 + 보정 thr", "A2": "A0c + HSV 게이트",
         "A3": "A1 + HSV 게이트", "A4": "현행 + Rabbit YOLO 임계 0.30→0.20"}

rows, per_obj = [], defaultdict(lambda: defaultdict(lambda: [0, 0, 0]))
outcome = defaultdict(dict)        # arm -> (ds,f,o) -> 'TP'/'FP'/'FN'/'TN'
for held in DATASETS:
    tr_f = [kf for kf in gt if kf[0] != held]
    te_f = [kf for kf in gt if kf[0] == held]
    tr_k = [k for k in cand if k[0] != held]
    te_k = [k for k in cand if k[0] == held]
    if not te_f:
        continue
    zs = zstat(tr_k)

    for arm in ARMS:
        if arm == "A0" or arm == "A4":
            sem_thr, hsv_thr = 0.35, 0.0
        else:
            # calibration fold 에서 F1 최대 threshold 탐색
            base = decide(tr_k, arm, -1e9, -1e9, zs)
            ss = sorted(b["_s"] for _, b in base.values())
            grid = [ss[int(q * (len(ss) - 1))] for q in np.linspace(0.02, 0.98, 60)]
            hgrid = ([0.0] if arm in ("A0c", "A1")
                     else [np.quantile([b["hsv"] for _, b in base.values()], q)
                           for q in np.linspace(0.0, 0.6, 13)])
            best, sem_thr, hsv_thr = -1, grid[0], 0.0
            for t in grid:
                for h in hgrid:
                    r = decide(tr_k, arm, t, h, zs)
                    TP, FP, FN, _, _ = score(r, tr_f)
                    f1 = 2 * TP / max(2 * TP + FP + FN, 1)
                    if f1 > best:
                        best, sem_thr, hsv_thr = f1, t, h
        r = decide(te_k, arm, sem_thr, hsv_thr, zs)
        TP, FP, FN, TN, per = score(r, te_f)
        for o in OBJECTS:
            for i in range(3):
                per_obj[arm][o][i] += per[o][i]
        for (ds, f) in te_f:
            for o in OBJECTS:
                acc = r.get((ds, f, o), (False, None))[0]
                v = o in gt[(ds, f)]
                outcome[arm][(ds, f, o)] = ("TP" if v and acc else "FP" if acc
                                            else "FN" if v else "TN")
        rows.append({"held_out": held, "arm": arm, "label": LABEL[arm],
                     "sem_thr": round(float(sem_thr), 5), "hsv_thr": round(float(hsv_thr), 5),
                     "TP": TP, "FP": FP, "FN": FN, "TN": TN})

with open(os.path.join(RES, "gt_arm_comparison.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

# ---------------------------------------------------------------- 합산 출력
agg = defaultdict(lambda: defaultdict(int))
for r in rows:
    for k in ("TP", "FP", "FN", "TN"):
        agg[r["arm"]][k] += r[k]
print(f"\n=== LODO held-out 합산 (사람 GT 935 가시 / 2455 비가시)")
print(f"{'arm':5s} {'설명':30s} {'TP':>5} {'FP':>5} {'FN':>5} {'Prec':>7} {'Rec':>7} {'F1':>7}")
base = None
for a in ARMS:
    x = agg[a]
    P = x["TP"] / max(x["TP"] + x["FP"], 1); R = x["TP"] / max(x["TP"] + x["FN"], 1)
    F = 2 * P * R / max(P + R, 1e-9)
    if a == "A0c":
        base = (x["FP"], x["FN"])
    mark = ""
    if base and a not in ("A0", "A0c"):
        mark = "  ★FP·FN 둘 다 개선" if x["FP"] <= base[0] and x["FN"] <= base[1] else ""
    print(f"{a:5s} {LABEL[a]:30s} {x['TP']:>5} {x['FP']:>5} {x['FN']:>5} "
          f"{P:>7.4f} {R:>7.4f} {F:>7.4f}{mark}")

# 짝지은 비교 (A0c 대비)
def mcnemar(b, c):
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / (2 ** n))


print(f"\n=== A0c(현행 점수+보정 thr) 대비 짝지은 비교")
print(f"{'arm':5s} {'신규오류':>8} {'수정오류':>8} {'McNemar p':>10} {'ΔTP':>5} {'ΔFP':>5} {'ΔFN':>5}")
pr_rows = []
for a in ARMS:
    if a == "A0c":
        continue
    keys = [k for k in outcome["A0c"] if k in outcome[a]]
    okf = lambda o: o in ("TP", "TN")
    b = sum(1 for k in keys if okf(outcome["A0c"][k]) and not okf(outcome[a][k]))
    c = sum(1 for k in keys if not okf(outcome["A0c"][k]) and okf(outcome[a][k]))
    d = {x: agg[a][x] - agg["A0c"][x] for x in ("TP", "FP", "FN")}
    p = mcnemar(b, c)
    pr_rows.append({"arm": a, "label": LABEL[a], "new_errors": b, "fixed_errors": c,
                    "mcnemar_p": round(p, 5), **{f"d{x}": d[x] for x in d}})
    print(f"{a:5s} {b:>8} {c:>8} {p:>10.5f} {d['TP']:>+5} {d['FP']:>+5} {d['FN']:>+5}")
with open(os.path.join(RES, "gt_arm_paired.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(pr_rows[0].keys())); w.writeheader(); w.writerows(pr_rows)

# 객체별
po = []
print(f"\n=== 객체별 (TP/FP/FN)")
print(f"{'객체':22s} " + " ".join(f"{a:>16s}" for a in ARMS))
for o in OBJECTS:
    print(f"{o:22s} " + " ".join(
        f"{per_obj[a][o][0]:>4}/{per_obj[a][o][1]:>3}/{per_obj[a][o][2]:>3} " for a in ARMS))
    row = {"object": o}
    for a in ARMS:
        t, fp, fn = per_obj[a][o]
        row[f"{a}_TP"], row[f"{a}_FP"], row[f"{a}_FN"] = t, fp, fn
    po.append(row)
with open(os.path.join(RES, "gt_arm_per_object.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(po[0].keys())); w.writeheader(); w.writerows(po)
print(f"\n-> {RES}/gt_arm_comparison.csv, gt_arm_per_object.csv, gt_arm_paired.csv")
