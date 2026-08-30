#!/usr/bin/env python3
"""compare_prototypes.py — HSV prototype 4종을 같은 척도(AUROC)로 비교한다 (READ-ONLY).

거리(Bhattacharyya) 절대값은 해석 척도가 없다. 같은 객체라도 데이터셋이 다르면
거리가 벌어지기 때문이다. 그래서 여기서는 운영상 의미가 있는 질문으로 바꾼다:

  "이 prototype 으로 실사 박스를 채점하면, 그 객체의 박스를 다른 객체 박스와
   얼마나 잘 구분하는가?"  → 객체별 AUROC / PR-AUC

prototype 4종
  A. render_old   기존 운영 렌더 템플릿 42장 (masked)
  B. render_high  압축본 *_high.ply 로 새로 렌더한 42장 (masked)   — 있는 객체만
  P. ply_color    PLY 정점 색 자체 (렌더러를 거치지 않음)          — 진단용
  C. real_lodo    실사 crop (평가 데이터셋을 제외한 5개에서만)     — 상한 기준

점수 = max_{proto} (1 - Bhattacharyya(hist_box, proto))
평가 = leave-one-dataset-out. 모든 prototype 을 **같은 held-out 집합**에서 채점한다.
       (render/ply 는 누수가 없지만 비교 가능성을 위해 동일 분할을 쓴다.)

산출: results/old_vs_high_ply_hsv.csv, results/rendered_vs_real_hsv.csv
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

HS = slice(96, 224)                       # H16xS8
MASKED = slice(0, 224)
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
EXCLUDE_LABEL = {"unclear"}               # 판정 불가 박스는 pos/neg 어느 쪽도 아님

import importlib.util
spec = importlib.util.spec_from_file_location("tri", os.path.join(HERE, "triangulate_color.py"))
# triangulate_color 를 import 하면 본문이 실행되므로 필요한 함수만 복제한다.

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
# high 렌더 폴더명 -> ISM 객체명
HIGH_MAP = {"Bear_high": "Bear", "saffron_high": "saffron",
            "Febreze_high": "Febreze_high", "Mugcup_high": "Mugcup_high"}


def bhatt_sim(q, P):
    """q:[128], P:[n,128] -> 각 prototype 과의 유사도 (1 - Bhattacharyya) 중 최대."""
    bc = np.sqrt(np.maximum(q[None, :] * P, 0)).sum(1)
    return float((1.0 - np.sqrt(np.maximum(0.0, 1.0 - bc))).max())


def auroc(pos, neg):
    if len(pos) == 0 or len(neg) == 0:
        return None
    p, n = np.asarray(pos, float), np.asarray(neg, float)
    return float(((p[:, None] > n[None, :]).sum() + 0.5 * (p[:, None] == n[None, :]).sum())
                 / (len(p) * len(n)))


def pr_auc(scores, ys):
    o = np.argsort(-np.asarray(scores, float)); y = np.asarray(ys)[o]
    tp = np.cumsum(y); fp = np.cumsum(1 - y)
    prec = tp / np.maximum(tp + fp, 1); rec = tp / max(y.sum(), 1)
    return float(np.sum(np.diff(np.concatenate([[0.0], rec])) * prec))


def ply_colors(path, want=200000):
    import struct
    T = {"char": "b", "uchar": "B", "short": "h", "ushort": "H", "int": "i", "uint": "I",
         "float": "f", "double": "d", "int8": "b", "uint8": "B", "int16": "h",
         "uint16": "H", "int32": "i", "uint32": "I", "float32": "f", "float64": "d"}
    with open(path, "rb") as f:
        props, nv, fmt, elem = [], 0, "", None
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if not line:
                return None
            if line.startswith("format"):
                fmt = line.split()[1]
            elif line.startswith("element"):
                _, nm, n = line.split(); elem = nm
                if nm == "vertex":
                    nv = int(n)
            elif line.startswith("property") and elem == "vertex":
                props.append((line.split()[-1], line.split()[1]))
            elif line == "end_header":
                off = f.tell(); break
        names = [p[0] for p in props]
        if "red" not in names:
            return None
        out, step = [], max(1, nv // want)
        if fmt.startswith("ascii"):
            idx = {n: i for i, n in enumerate(names)}
            f.seek(off)
            for i in range(nv):
                line = f.readline()
                if not line:
                    break
                if i % step:
                    continue
                t = line.split()
                if len(t) >= len(names):
                    out.append((int(t[idx["red"]]), int(t[idx["green"]]), int(t[idx["blue"]])))
        else:
            en = "<" if "little" in fmt else ">"
            sf = en + "".join(T[t] for _, t in props); sz = struct.calcsize(sf)
            ri = names.index("red")
            for i in range(0, nv, step):
                f.seek(off + i * sz); buf = f.read(sz)
                if len(buf) < sz:
                    break
                v = struct.unpack(sf, buf); out.append((v[ri], v[ri + 1], v[ri + 2]))
    return np.array(out, np.uint8)


def hs_hist_from_rgb(rgb):
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
    return (h / max(h.sum(), 1e-12)).astype(np.float32)


def hs_hist_from_template(td):
    hs = []
    for i in range(200):
        rp, mp = os.path.join(td, f"rgb_{i}.png"), os.path.join(td, f"mask_{i}.png")
        if not (os.path.isfile(rp) and os.path.isfile(mp)):
            continue
        bgr, m = cv2.imread(rp), cv2.imread(mp, cv2.IMREAD_GRAYSCALE)
        if bgr is None or m is None:
            continue
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h = cv2.calcHist([hsv], [0, 1], (m > 0).astype(np.uint8) * 255,
                         [16, 8], [0, 180, 0, 256]).flatten()
        hs.append((h / max(h.sum(), 1e-12)).astype(np.float32))
    return np.stack(hs) if hs else None


# ------------------------------------------------------------------ 자료 적재
labels = {r["uid"]: r["true_class"]
          for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
boxes = []                                # (uid, dataset, true_class, hist)
for ds in DATASETS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    for u, h in zip(z["uid"], z["hist"]):
        c = labels.get(str(u))
        if c and c not in EXCLUDE_LABEL:
            boxes.append((str(u), ds, c, h[MASKED][HS].astype(np.float64)))
print(f"평가 박스 {len(boxes)} (라벨 클래스 {len(set(b[2] for b in boxes))}종)")

PROTO = {}
z = np.load(os.path.join(OBS, "hsv_features", "template_render_hsv.npz"))
for k in z.files:
    PROTO.setdefault(k, {})["render_old"] = z[k][:, HS].astype(np.float64)
for d, obj in HIGH_MAP.items():
    P = hs_hist_from_template(os.path.join(ROOT, "high_ply_templates", d, "templates"))
    if P is not None and P.shape[0] >= 42:
        PROTO.setdefault(obj, {})["render_high"] = P.astype(np.float64)
for obj, rel in OBJ_PLY.items():
    p = os.path.join(REPO, rel)
    if os.path.isfile(p):
        c = ply_colors(p)
        if c is not None:
            PROTO.setdefault(obj, {})["ply_color"] = hs_hist_from_rgb(c)[None, :].astype(np.float64)
print("prototype 보유:", {k: sorted(v) for k, v in sorted(PROTO.items())})

# ------------------------------------------------------------------ LODO 평가
SRC = ["render_old", "render_high", "ply_color", "real_lodo"]
per = defaultdict(lambda: defaultdict(list))
fold_rows = []
for held in DATASETS:
    tr = [b for b in boxes if b[1] != held]
    te = [b for b in boxes if b[1] == held]
    for obj in sorted(PROTO):
        pos = [b for b in te if b[2] == obj]
        neg = [b for b in te if b[2] != obj]
        if len(pos) < 2 or len(neg) < 2:
            continue
        real = np.stack([b[3] for b in tr if b[2] == obj]) if any(b[2] == obj for b in tr) else None
        for src in SRC:
            P = PROTO[obj].get(src) if src != "real_lodo" else real
            if P is None or len(P) < 1:
                continue
            sp = [bhatt_sim(b[3], P) for b in pos]
            sn = [bhatt_sim(b[3], P) for b in neg]
            a = auroc(sp, sn)
            pr = pr_auc(sp + sn, [1] * len(sp) + [0] * len(sn))
            per[obj][src].append(a)
            fold_rows.append({"held_out": held, "object": obj, "prototype": src,
                              "n_proto": len(P), "n_pos": len(pos), "n_neg": len(neg),
                              "auroc": round(a, 4), "pr_auc": round(pr, 4)})

with open(os.path.join(RES, "rendered_vs_real_hsv.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(fold_rows[0].keys())); w.writeheader(); w.writerows(fold_rows)

out = []
for obj in sorted(per):
    r = {"object": obj}
    for src in SRC:
        v = per[obj][src]
        r[f"auroc_{src}"] = round(float(np.mean(v)), 4) if v else ""
        r[f"n_fold_{src}"] = len(v)
    if r["auroc_render_high"] != "" and r["auroc_render_old"] != "":
        r["high_minus_old"] = round(r["auroc_render_high"] - r["auroc_render_old"], 4)
    if r["auroc_real_lodo"] != "" and r["auroc_render_old"] != "":
        r["real_minus_old"] = round(r["auroc_real_lodo"] - r["auroc_render_old"], 4)
    out.append(r)

with open(os.path.join(RES, "old_vs_high_ply_hsv.csv"), "w", newline="") as f:
    ks = ["object"] + [f"{p}_{s}" for s in SRC for p in ("auroc", "n_fold")] + \
         ["high_minus_old", "real_minus_old"]
    w = csv.DictWriter(f, fieldnames=ks, extrasaction="ignore"); w.writeheader(); w.writerows(out)

print(f"\n{'객체':22s} {'기존렌더':>9} {'high렌더':>9} {'PLY정점색':>10} {'실사LODO':>9} "
      f"{'high-기존':>10} {'실사-기존':>10}")
for r in out:
    print(f"{r['object']:22s} {str(r['auroc_render_old']):>9} {str(r['auroc_render_high']):>9} "
          f"{str(r['auroc_ply_color']):>10} {str(r['auroc_real_lodo']):>9} "
          f"{str(r.get('high_minus_old','-')):>10} {str(r.get('real_minus_old','-')):>10}")
for s in SRC:
    v = [r[f"auroc_{s}"] for r in out if r[f"auroc_{s}"] != ""]
    print(f"평균 {s:12s} = {np.mean(v):.4f}  (객체 {len(v)}종)")
print(f"\n-> {os.path.join(RES, 'old_vs_high_ply_hsv.csv')}")
