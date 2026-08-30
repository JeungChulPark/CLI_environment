#!/usr/bin/env python3
"""eval_pipeline_end2end.py — 후보 생성 × 라우팅 정책의 최종 TP/FP/FN 비교 (READ-ONLY).

후단은 확정된 구성을 그대로 쓴다: semantic(sem_top5) + appearance(appe11) + HSV hard gate
(H2 = Hue 보정 렌더, -6°/채도 x1.3, H+S 16x8, Bhattacharyya, 42 중 max).

축 1 — 후보 집합
  cur         현행 YOLO-World 10 pass
  yoloe       YOLOE text prompt 1 pass
  union       둘의 합집합 (IoU 0.75 중복 제거)

축 2 — 라우팅 정책
  R0 현행     프롬프트→객체 고정 라우팅 + 객체별 score_threshold + top_k=3
  R1 제거     모든 박스를 모든 객체의 후보로 (score_threshold·라우팅 없음), 프레임당 상한 K
  R2 완화     라우팅은 유지하되 객체별 score_threshold 만 0.02 로 통일

HSV 임계는 LODO calibration fold 에서 F1 최대로 정한다. 평가는 사람 가시성 GT 935 기준.
⚠ 프레임 단위 GT 라 수락 박스의 위치 정확성은 확인하지 않는다(직전 연구와 동일 한계).

산출: results/end2end_comparison.csv, end2end_by_object.csv
"""
import csv, json, os
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
FUS = os.path.join(RSRCH, "ism_fusion_research")
GT = os.path.join(RSRCH, "gt_input")
PIPE = os.path.join(ROOT, "pipeline")
RES = os.path.join(ROOT, "results")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
OBJECTS = ["Bear", "Rabbit", "Dinosaur", "milk", "choco_hazelnut_high",
           "Febreze_high", "Mugcup_high", "saffron", "Sauce_high", "Sikhye_high"]
HS = slice(96, 224); MASKED = slice(0, 224)
HUE_SHIFT, SAT_GAIN = -6, 1.3
TOPK = 3
K_NOROUTE = int(os.environ.get("K_NOROUTE", "12"))     # 라우팅 제거 시 프레임당 후보 상한

# ---------------------------------------------------------------- GT
gt = {}
for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
    if r["user_reviewed"] == "yes":
        gt[(r["dataset_name"], int(r["frame_id"]))] = set(
            t.strip() for t in r["visible_objects"].split(";") if t.strip())
NVIS = sum(len([t for t in v if t in OBJECTS]) for v in gt.values())
print(f"GT 프레임 {len(gt)} / 가시 {NVIS}")

# ---------------------------------------------------------------- HSV prototype
def feat_rgb(rgb):
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.int16)
    hsv[..., 0] = (hsv[..., 0] + int(round(HUE_SHIFT / 2))) % 180
    hsv[..., 1] = np.clip(hsv[..., 1] * SAT_GAIN, 0, 255)
    x = cv2.calcHist([hsv.astype(np.uint8)], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
    return (x / max(x.sum(), 1e-12)).astype(np.float64)


PC = np.load(os.path.join(FUS, "ply_hsv", "prototype_colors.npz"))
PROTO = {o: np.stack([feat_rgb(c) for c in PC[f"{o}__render42"]]) for o in OBJECTS}


def load(var):
    d = os.path.join(PIPE, var)
    if not os.path.isdir(d):
        return None, None
    hist = {}
    for ds in DATASETS:
        z = np.load(os.path.join(d, f"{ds}_hsv.npz"))
        for u, h in zip(z["uid"], z["hist"]):
            hist[str(u)] = h[MASKED][HS].astype(np.float64)
    rows = defaultdict(list)          # (ds,frame,object) -> 후보들
    for ds in DATASETS:
        for r in csv.DictReader(open(os.path.join(d, f"{ds}_pairs.csv"))):
            k = (ds, int(r["frame_id"]), r["object"])
            if (ds, int(r["frame_id"])) not in gt:
                continue
            rows[k].append({"uid": r["uid"], "routed": int(r["routed"]),
                            "conf": float(r["yolo_conf"]),
                            "sem": float(r["sem_top5"]), "appe": float(r["appe11_clstop1"]),
                            "st": float(r["sim_thr"]), "ag": float(r["appe_gate"]),
                            "sth": float(r["score_threshold"])})
    return rows, hist


HSVC = {}
def hsv_of(uid, obj, hist):
    k = (uid, obj)
    if k not in HSVC:
        q = hist.get(uid)
        if q is None:
            HSVC[k] = 1.0
        else:
            bc = np.sqrt(np.maximum(q[None, :] * PROTO[obj], 0)).sum(1)
            HSVC[k] = float((1.0 - np.sqrt(np.maximum(0.0, 1.0 - bc))).max())
    return HSVC[k]


def candidates(rows, k, policy):
    v = rows.get(k, [])
    if policy == "R0":
        v = [c for c in v if c["routed"] and c["conf"] >= c["sth"]]
        v = sorted(v, key=lambda c: -c["conf"])[:TOPK]
    elif policy == "R2":
        v = [c for c in v if c["routed"]]
        v = sorted(v, key=lambda c: -c["conf"])[:TOPK]
    else:                                    # R1 라우팅 제거
        v = sorted(v, key=lambda c: -c["conf"])[:K_NOROUTE]
    return v


def run(rows, hist, policy, t_hsv, keys):
    res = {}
    for k in keys:
        v = candidates(rows, k, policy)
        if not v:
            continue
        b = max(v, key=lambda c: c["sem"])
        ok = (b["sem"] >= b["st"]) and (b["appe"] >= b["ag"]) \
            and (hsv_of(b["uid"], k[2], hist) >= t_hsv)
        res[k] = (ok, b["uid"])
    return res


def score(res, frames):
    TP = FP = FN = 0
    per = defaultdict(lambda: [0, 0, 0])
    for (ds, f) in frames:
        vis = gt[(ds, f)]
        for o in OBJECTS:
            acc = res.get((ds, f, o), (False, None))[0]
            v = o in vis
            if v and acc:
                TP += 1; per[o][0] += 1
            elif acc:
                FP += 1; per[o][1] += 1
            elif v:
                FN += 1; per[o][2] += 1
    return TP, FP, FN, per


VARS = ["cur", "yoloe_txt", "union"]
POLS = ["R0", "R1", "R2"]
VL = {"cur": "현행 YOLO-World", "yoloe_txt": "YOLOE 단독", "union": "현행+YOLOE union"}
PL = {"R0": "현행 라우팅(+객체별 thr, top_k3)", "R1": f"라우팅 제거(프레임당 {K_NOROUTE})",
      "R2": "라우팅 유지 + thr 통일 0.02"}

DATA = {}
for v in VARS:
    r, h = load(v)
    if r is not None:
        DATA[v] = (r, h)
        print(f"  {v}: 그룹 {len(r):,}")

agg = defaultdict(lambda: defaultdict(int))
per_obj = defaultdict(lambda: defaultdict(lambda: [0, 0, 0]))
fold_rows = []
for var in VARS:
    if var not in DATA:
        continue
    rows, hist = DATA[var]
    allk = list(rows)
    for pol in POLS:
        for held in DATASETS:
            trk = [k for k in allk if k[0] != held]
            tek = [k for k in allk if k[0] == held]
            trf = [x for x in gt if x[0] != held]
            tef = [x for x in gt if x[0] == held]
            hv = [hsv_of(c["uid"], k[2], hist) for k in trk for c in candidates(rows, k, pol)]
            if len(hv) < 30:
                continue
            grid = np.quantile(hv, np.linspace(0.0, 0.9, 40))
            best, bt = -1, 0.0
            for t in grid:
                TP, FP, FN, _ = score(run(rows, hist, pol, t, trk), trf)
                f1 = 2 * TP / max(2 * TP + FP + FN, 1)
                if f1 > best:
                    best, bt = f1, t
            TP, FP, FN, per = score(run(rows, hist, pol, bt, tek), tef)
            key = f"{var}|{pol}"
            for kk, vv in (("TP", TP), ("FP", FP), ("FN", FN)):
                agg[key][kk] += vv
            for o in OBJECTS:
                for i in range(3):
                    per_obj[key][o][i] += per[o][i]
            fold_rows.append({"variant": var, "policy": pol, "held_out": held,
                              "t_hsv": round(float(bt), 5), "TP": TP, "FP": FP, "FN": FN})

with open(os.path.join(RES, "end2end_folds.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(fold_rows[0].keys())); w.writeheader(); w.writerows(fold_rows)

out = []
for var in VARS:
    for pol in POLS:
        k = f"{var}|{pol}"
        if not agg[k]:
            continue
        x = agg[k]
        P = x["TP"] / max(x["TP"] + x["FP"], 1); R = x["TP"] / max(x["TP"] + x["FN"], 1)
        out.append({"variant": var, "policy": pol,
                    "label": f"{VL[var]} + {PL[pol]}",
                    "TP": x["TP"], "FP": x["FP"], "FN": x["FN"],
                    "precision": round(P, 4), "recall": round(R, 4),
                    "f1": round(2 * P * R / max(P + R, 1e-9), 4)})
with open(os.path.join(RES, "end2end_comparison.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(out[0].keys())); w.writeheader(); w.writerows(out)

base = next((r for r in out if r["variant"] == "cur" and r["policy"] == "R0"), None)
print(f"\n=== 최종 TP/FP/FN (사람 GT {NVIS} 가시, LODO, HSV 게이트 포함)  K_NOROUTE={K_NOROUTE}")
print(f"{'후보':16s} {'라우팅':6s} {'설명':44s} {'TP':>5} {'FP':>5} {'FN':>5} "
      f"{'Prec':>7} {'Rec':>7} {'F1':>7}")
for r in out:
    d = ""
    if base and not (r["variant"] == "cur" and r["policy"] == "R0"):
        d = f"  (ΔTP {r['TP']-base['TP']:+d} ΔFP {r['FP']-base['FP']:+d} ΔFN {r['FN']-base['FN']:+d})"
    print(f"{r['variant']:16s} {r['policy']:6s} {r['label']:44s} {r['TP']:>5} {r['FP']:>5} "
          f"{r['FN']:>5} {r['precision']:>7.4f} {r['recall']:>7.4f} {r['f1']:>7.4f}{d}")

with open(os.path.join(RES, "end2end_by_object.csv"), "w", newline="") as f:
    ks = ["object"] + [f"{v}|{p}_{s}" for v in VARS for p in POLS for s in ("TP", "FP", "FN")]
    w = csv.DictWriter(f, fieldnames=ks, restval=""); w.writeheader()
    for o in OBJECTS:
        row = {"object": o}
        for v in VARS:
            for p in POLS:
                k = f"{v}|{p}"
                if per_obj[k]:
                    row[f"{k}_TP"], row[f"{k}_FP"], row[f"{k}_FN"] = per_obj[k][o]
        w.writerow(row)

print(f"\n=== 객체별 TP/FP/FN (주요 3안)")
sel = ["cur|R0", "union|R0", "union|R1"]
print(f"{'객체':22s} " + " ".join(f"{s:>16s}" for s in sel))
for o in OBJECTS:
    print(f"{o:22s} " + " ".join(
        f"{per_obj[s][o][0]:>5}/{per_obj[s][o][1]:>4}/{per_obj[s][o][2]:>4}" for s in sel))
print(f"\n-> {RES}")
