#!/usr/bin/env python3
"""test_fusion.py — 실사 촬영 없이 렌더 HSV 를 얼마나 회복할 수 있는지 검증한다 (READ-ONLY).

compare_prototypes.py 에서 나온 사실:
  - *_high.ply 재렌더 개선량 ≈ 0 (-0.001 ~ +0.007)
  - 그런데 PLY 정점색을 렌더러 없이 직접 쓰면 Bear 0.519 → 0.955
    → 색 정보는 PLY 안에 있는데 렌더러가 그것을 잃는다

따라서 "새 실사 촬영 없이" 쓸 수 있는 후보는 다음이다.

  S1 render_old            현행
  S2 ply_color             PLY 정점색만
  S3 render+ply            두 prototype 을 합쳐 max 유사도
  S4 render_gray_norm      렌더 템플릿을 채도/명도 정규화 후 사용 (조명 보정 근사)
  S5 per_object_best       객체별로 S1/S2 중 나은 쪽 — 단, 선택은 학습 fold 에서만
  S6 real_lodo             실사 prototype (상한 기준, 참고용)

S5 는 객체별 선택이므로 반드시 LODO 학습 fold 에서 고르고 held-out 에서만 채점한다.

산출: results/hsv_fusion_lodo.csv
"""
import csv, os
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
REPO = os.path.dirname(RSRCH)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RES = os.path.join(ROOT, "results")

HS = slice(96, 224); MASKED = slice(0, 224)
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
EXCLUDE_LABEL = {"unclear"}
HIGH_MAP = {"Bear_high": "Bear", "saffron_high": "saffron",
            "Febreze_high": "Febreze_high", "Mugcup_high": "Mugcup_high"}
OBJ_PLY = {
    "Bear": "data/cad/Bear/bear_color_with_normal_vertexcolor.ply",
    "Dinosaur": "data/cad/Dinosaur/Dinosaur_color_with_normal_vertexcolor.ply",
    "Febreze_high": "data/cad/Febreze_high/Febreze_color_with_normal_vertexcolor.ply",
    "Mugcup_high": "data/cad/Mugcup_color_high/Mugcup_color_with_normal_vertexcolor.ply",
    "Rabbit": "data/cad/Rabbit/Rabbit_color_with_normal_vertexcolor.ply",
    "Sauce_high": "data/cad/Sauce_high/Sauce_color_with_normal_vertexcolor.ply",
    "Sikhye_high": "data/cad/Sikhye_high/Sikhye_color_with_normal_vertexcolor.ply",
    "choco_hazelnut_high": "data/cad/choco_hazelnut_color_high/choco_hazelnut_color_with_normal_vertexcolor.ply",
    "saffron": "data/cad/saffron/saffron_color_with_normal_vertexcolor.ply",
    "milk": "data/cad/milk/Milk.ply",
}

# compare_prototypes 를 그대로 import 하면 그쪽 평가가 다시 실행되므로,
# "자료 적재" 앞의 함수 정의 구간만 떼어내 실행한다 (bhatt_sim/auroc/pr_auc/ply_colors/hs_hist_*).
_src = open(os.path.join(HERE, "compare_prototypes.py")).read()
_head = _src.split("# ------------------------------------------------------------------ 자료 적재")[0]
_head = "\n".join(l for l in _head.splitlines()
                  if not l.startswith(("import importlib", "spec = ", "# triangulate")))
exec(compile(_head, "compare_prototypes_head", "exec"), globals())


def sat_norm_template(td):
    """렌더 템플릿의 조명 편향을 근사 보정: masked 영역의 S,V 를 평균 0.5 로 재조정."""
    hs = []
    for i in range(200):
        rp, mp = os.path.join(td, f"rgb_{i}.png"), os.path.join(td, f"mask_{i}.png")
        if not (os.path.isfile(rp) and os.path.isfile(mp)):
            continue
        bgr, m = cv2.imread(rp), cv2.imread(mp, cv2.IMREAD_GRAYSCALE)
        if bgr is None or m is None:
            continue
        mk = m > 0
        if mk.sum() < 50:
            continue
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
        for ch in (1, 2):
            mu = hsv[..., ch][mk].mean()
            if mu > 1:
                hsv[..., ch] = np.clip(hsv[..., ch] * (128.0 / mu), 0, 255)
        hsv = hsv.astype(np.uint8)
        h = cv2.calcHist([hsv], [0, 1], mk.astype(np.uint8) * 255,
                         [16, 8], [0, 180, 0, 256]).flatten()
        hs.append((h / max(h.sum(), 1e-12)).astype(np.float32))
    return np.stack(hs) if hs else None


# ------------------------------------------------------------------ 적재
labels = {r["uid"]: r["true_class"]
          for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
boxes = []
for ds in DATASETS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        c = labels.get(str(u))
        if c and c not in EXCLUDE_LABEL:
            boxes.append((str(u), ds, c, h[MASKED][HS].astype(np.float64)))

PR = defaultdict(dict)
z = np.load(os.path.join(OBS, "hsv_features", "template_render_hsv.npz"))
for k in z.files:
    PR[k]["render_old"] = z[k][:, HS].astype(np.float64)
for obj, rel in OBJ_PLY.items():
    p = os.path.join(REPO, rel)
    if os.path.isfile(p):
        c = ply_colors(p)
        if c is not None:
            PR[obj]["ply_color"] = hs_hist_from_rgb(c)[None, :].astype(np.float64)
for obj in list(PR):
    td = os.path.join(REPO, "template", {"Bear": "Bear", "Dinosaur": "Dinosaur",
                                         "Febreze_high": "Febreze_high",
                                         "Mugcup_high": "Mugcup_color_high",
                                         "Rabbit": "Rabbit", "Sauce_high": "Sauce_high",
                                         "Sikhye_high": "Sikhye_high",
                                         "choco_hazelnut_high": "choco_hazelnut_color_high",
                                         "saffron": "saffron", "milk": "milk"}[obj], "templates")
    sn = sat_norm_template(td)
    if sn is not None:
        PR[obj]["render_satnorm"] = sn.astype(np.float64)
    if "ply_color" in PR[obj]:
        PR[obj]["render_plus_ply"] = np.vstack([PR[obj]["render_old"], PR[obj]["ply_color"]])

SCHEMES = ["render_old", "ply_color", "render_plus_ply", "render_satnorm",
           "per_object_best", "real_lodo"]
rows, per = [], defaultdict(lambda: defaultdict(list))
for held in DATASETS:
    tr = [b for b in boxes if b[1] != held]
    te = [b for b in boxes if b[1] == held]
    for obj in sorted(PR):
        pos = [b for b in te if b[2] == obj]; neg = [b for b in te if b[2] != obj]
        trp = [b for b in tr if b[2] == obj]; trn = [b for b in tr if b[2] != obj]
        if len(pos) < 2 or len(neg) < 2 or len(trp) < 2:
            continue
        real = np.stack([b[3] for b in trp])

        def score(P):
            return ([bhatt_sim(b[3], P) for b in pos], [bhatt_sim(b[3], P) for b in neg])

        # per_object_best: 학습 fold 에서 render_old vs ply_color 중 나은 쪽 선택
        cand = {}
        for s in ("render_old", "ply_color"):
            if s in PR[obj]:
                a = auroc([bhatt_sim(b[3], PR[obj][s]) for b in trp],
                          [bhatt_sim(b[3], PR[obj][s]) for b in trn])
                cand[s] = a
        pick = max(cand, key=cand.get) if cand else None

        for s in SCHEMES:
            if s == "real_lodo":
                P = real
            elif s == "per_object_best":
                P = PR[obj].get(pick) if pick else None
            else:
                P = PR[obj].get(s)
            if P is None:
                continue
            sp, sn_ = score(P)
            a = auroc(sp, sn_)
            per[obj][s].append(a)
            rows.append({"held_out": held, "object": obj, "scheme": s,
                         "picked": pick if s == "per_object_best" else "",
                         "n_pos": len(pos), "n_neg": len(neg),
                         "auroc": round(a, 4),
                         "pr_auc": round(pr_auc(sp + sn_, [1]*len(sp) + [0]*len(sn_)), 4)})

with open(os.path.join(RES, "hsv_fusion_lodo.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)

print(f"{'객체':22s} " + " ".join(f"{s:>16s}" for s in SCHEMES))
for obj in sorted(per):
    print(f"{obj:22s} " + " ".join(
        f"{np.mean(per[obj][s]):>16.4f}" if per[obj][s] else f"{'-':>16s}" for s in SCHEMES))
print(f"{'평균':22s} " + " ".join(
    f"{np.mean([np.mean(per[o][s]) for o in per if per[o][s]]):>16.4f}" for s in SCHEMES))
picks = defaultdict(list)
for r in rows:
    if r["scheme"] == "per_object_best":
        picks[r["object"]].append(r["picked"])
print("\nper_object_best 선택 (fold별):")
for o in sorted(picks):
    c = picks[o]
    print(f"  {o:22s} {max(set(c), key=c.count)} ({c.count(max(set(c), key=c.count))}/{len(c)})")
print(f"\n-> {os.path.join(RES,'hsv_fusion_lodo.csv')}")
