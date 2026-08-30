#!/usr/bin/env python3
"""triangulate_color.py — 렌더 색이 나쁜 원인을 PLY / 렌더러 로 분리한다 (READ-ONLY).

세 지점의 색 분포를 같은 방식(HSV 히스토그램)으로 재고 서로의 거리를 잰다.

  P (ply)    : PLY 정점 색 자체.        렌더러를 전혀 거치지 않은 "설계상의 색"
  A (render) : 기존 렌더 템플릿(masked). PLY → blenderproc 조명/셰이딩 → 이미지
  C (real)   : 6개 SAM bag 실사 crop(masked). 실제 카메라가 본 색

판정 논리
  d(P,C) 작고 d(A,C) 큼  →  PLY 색은 맞는데 렌더가 망침  → 렌더러/조명/감마 문제
  d(P,C) 큼   그리고 d(A,P) 작음 →  PLY 색 자체가 실물과 다름 → PLY(스캔) 문제
  둘 다 큼               →  두 원인 공존 또는 실사 crop/mask 문제

거리: Bhattacharyya (0=동일, 1=완전분리). H+S 결합 히스토그램(16x8) 사용.

산출: results/color_triangulation.csv
"""
import csv, os, sys
from collections import defaultdict

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
REPO = os.path.dirname(RSRCH)
OBS = os.path.join(RSRCH, "ism_accuracy_observation")
RES = os.path.join(ROOT, "results")
os.makedirs(RES, exist_ok=True)

# 관찰 덤프의 히스토그램 레이아웃 (dump_all_frames.hists 와 동일)
#   [0:32] H32  [32:64] S32  [64:96] V32  [96:224] H16xS8
# 박스당 3영역이 이어붙어 있음: masked(0:224) / bbox(224:448) / background(448:672)
HS = slice(96, 224)          # H16xS8 결합 히스토그램
MASKED = slice(0, 224)

DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]

# ISM 객체 -> 운영 PLY (verify_identity.py 의 PAIRS 와 동일)
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


def bhatt(a, b):
    """Bhattacharyya distance. 두 히스토그램은 합=1 로 정규화되어 있다고 가정."""
    a = a / max(a.sum(), 1e-12); b = b / max(b.sum(), 1e-12)
    bc = np.sqrt(a * b).sum()
    return float(np.sqrt(max(0.0, 1.0 - bc)))


def inter(a, b):
    a = a / max(a.sum(), 1e-12); b = b / max(b.sum(), 1e-12)
    return float(np.minimum(a, b).sum())


def rgb_to_hs_hist(rgb):
    """Nx3 uint8 RGB -> H16xS8 히스토그램 (렌더/실사와 동일 binning)."""
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]     # BGR
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
    return (h / max(h.sum(), 1e-12)).astype(np.float32)


def ply_colors(path, want=200000):
    """PLY 정점 색을 균등 표본으로 읽는다 (ascii/binary 모두)."""
    import struct
    T = {"char": ("b", 1), "uchar": ("B", 1), "short": ("h", 2), "ushort": ("H", 2),
         "int": ("i", 4), "uint": ("I", 4), "float": ("f", 4), "double": ("d", 8),
         "int8": ("b", 1), "uint8": ("B", 1), "int16": ("h", 2), "uint16": ("H", 2),
         "int32": ("i", 4), "uint32": ("I", 4), "float32": ("f", 4), "float64": ("d", 8)}
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
        out = []
        step = max(1, nv // want)
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
                if len(t) < len(names):
                    continue
                out.append((int(t[idx["red"]]), int(t[idx["green"]]), int(t[idx["blue"]])))
        else:
            en = "<" if "little" in fmt else ">"
            sf = en + "".join(T[t][0] for _, t in props)
            sz = struct.calcsize(sf); ri = names.index("red")
            for i in range(0, nv, step):
                f.seek(off + i * sz)
                buf = f.read(sz)
                if len(buf) < sz:
                    break
                v = struct.unpack(sf, buf)
                out.append((v[ri], v[ri + 1], v[ri + 2]))
    return np.array(out, np.uint8)


# ------------------------------------------------------------------ 자료 적재
labels = {r["uid"]: r["true_class"]
          for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv")))}
real = defaultdict(list)          # object -> [H16xS8]
real_ds = defaultdict(list)       # (object, dataset)
for ds in DATASETS:
    z = np.load(os.path.join(OBS, "hsv_features", f"{ds}_hsv.npz"))
    uids = list(z["uid"]); H = z["hist"]
    for i, u in enumerate(uids):
        c = labels.get(str(u))
        if c and c in OBJ_PLY:
            v = H[i][MASKED][HS]
            real[c].append(v); real_ds[(c, ds)].append(v)
print("실사 crop 수:", {k: len(v) for k, v in sorted(real.items())})

rend = {}
z = np.load(os.path.join(OBS, "hsv_features", "template_render_hsv.npz"))
for k in z.files:
    rend[k] = z[k][:, HS]                                  # (42, 128)
print("렌더 템플릿:", {k: v.shape[0] for k, v in sorted(rend.items())})

# 신규 high PLY 렌더가 있으면 함께 비교
HIGH = os.path.join(ROOT, "high_ply_templates")
high_rend = {}
NAME_MAP = {"Bear_high": "Bear", "saffron_high": "saffron",
            "Febreze_high": "Febreze_high", "Mugcup_high": "Mugcup_high"}
for d, obj in NAME_MAP.items():
    td = os.path.join(HIGH, d, "templates")
    hs = []
    for i in range(200):
        rp, mp = os.path.join(td, f"rgb_{i}.png"), os.path.join(td, f"mask_{i}.png")
        if not (os.path.isfile(rp) and os.path.isfile(mp)):
            continue
        bgr = cv2.imread(rp); m = cv2.imread(mp, cv2.IMREAD_GRAYSCALE)
        if bgr is None or m is None:
            continue
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h = cv2.calcHist([hsv], [0, 1], (m > 0).astype(np.uint8) * 255,
                         [16, 8], [0, 180, 0, 256]).flatten()
        hs.append((h / max(h.sum(), 1e-12)).astype(np.float32))
    if hs:
        high_rend[obj] = np.stack(hs)
print("신규 high 렌더:", {k: v.shape[0] for k, v in sorted(high_rend.items())})


def cen(a):
    """히스토그램 묶음의 평균(centroid)."""
    m = np.mean(a, 0)
    return m / max(m.sum(), 1e-12)


rows = []
for obj in sorted(OBJ_PLY):
    if obj not in real or len(real[obj]) < 3 or obj not in rend:
        print(f"[skip] {obj}: 실사 {len(real.get(obj,[]))} / 렌더 {obj in rend}")
        continue
    C = cen(np.stack(real[obj]))
    A = cen(rend[obj])
    p = os.path.join(REPO, OBJ_PLY[obj])
    cols = ply_colors(p) if os.path.isfile(p) else None
    P = rgb_to_hs_hist(cols) if cols is not None else None

    row = {"object": obj, "n_real_crop": len(real[obj]), "n_render_view": rend[obj].shape[0],
           "n_ply_vertex_sampled": 0 if cols is None else len(cols),
           "d_render_real": round(bhatt(A, C), 4),
           "i_render_real": round(inter(A, C), 4)}
    if P is not None:
        row.update({"d_ply_real": round(bhatt(P, C), 4),
                    "d_ply_render": round(bhatt(P, A), 4),
                    "i_ply_real": round(inter(P, C), 4)})
        # 판정
        dpc, dac, dap = bhatt(P, C), bhatt(A, C), bhatt(P, A)
        if dpc < dac - 0.05 and dap > 0.05:
            row["verdict"] = "렌더러/조명 문제 (PLY 색은 실사에 더 가까움)"
        elif dpc >= dac and dap < 0.05:
            row["verdict"] = "PLY 색 문제 (렌더는 PLY를 충실히 재현)"
        elif dpc > 0.5 and dac > 0.5:
            row["verdict"] = "PLY·렌더 모두 실사와 멂 (domain gap 또는 crop/mask)"
        else:
            row["verdict"] = "혼재 — 단일 원인으로 귀속 불가"
    if obj in high_rend:
        B = cen(high_rend[obj])
        row.update({"n_high_render_view": high_rend[obj].shape[0],
                    "d_highrender_real": round(bhatt(B, C), 4),
                    "d_highrender_oldrender": round(bhatt(B, A), 4),
                    "high_improvement": round(bhatt(A, C) - bhatt(B, C), 4)})
    # 데이터셋별 실사 편차 (실사 기준선이 안정적인지)
    dd = [bhatt(A, cen(np.stack(v))) for (o, ds), v in real_ds.items() if o == obj and len(v) >= 3]
    if dd:
        row["d_render_real_by_dataset_std"] = round(float(np.std(dd)), 4)
        row["d_render_real_by_dataset_range"] = f"{min(dd):.3f}~{max(dd):.3f}"
    rows.append(row)

ks = ["object", "n_real_crop", "n_render_view", "n_ply_vertex_sampled",
      "d_ply_real", "d_render_real", "d_ply_render", "i_ply_real", "i_render_real",
      "d_render_real_by_dataset_std", "d_render_real_by_dataset_range",
      "n_high_render_view", "d_highrender_real", "d_highrender_oldrender",
      "high_improvement", "verdict"]
with open(os.path.join(RES, "color_triangulation.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=ks, extrasaction="ignore"); w.writeheader(); w.writerows(rows)

print(f"\n{'객체':22s} {'실사n':>6} {'d(PLY,실사)':>11} {'d(렌더,실사)':>12} {'d(PLY,렌더)':>11}  판정")
for r in rows:
    print(f"{r['object']:22s} {r['n_real_crop']:>6} {r.get('d_ply_real','-'):>11} "
          f"{r['d_render_real']:>12} {r.get('d_ply_render','-'):>11}  {r.get('verdict','')}")
print(f"\n-> {os.path.join(RES, 'color_triangulation.csv')}")
