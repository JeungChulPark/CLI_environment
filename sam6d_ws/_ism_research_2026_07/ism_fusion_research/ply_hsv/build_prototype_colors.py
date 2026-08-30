#!/usr/bin/env python3
"""build_prototype_colors.py — 세 prototype 소스의 **원본 색상**을 저장한다 (READ-ONLY).

히스토그램을 저장하면 Hue 보정을 bin 단위(H16 이면 11.25°)로만 줄 수 있어 실험이 거칠어진다.
원본 색상을 남기면 임의 각도 보정 + 임의 binning 을 정확히 재현할 수 있다.

저장 소스
  render42   : 기존 렌더 템플릿 42장의 mask 내부 픽셀 색  (현행 방식)
  ply_all1   : PLY 전 vertex 색                            (방식 A)
  ply_view42 : 42 시점별 visible vertex 색                 (방식 B)
               가시성은 렌더가 남긴 xyz_<i>.npy(=PLY 좌표계 3D 위치)를 PLY vertex 에
               최근접 대응시켜 정한다. 픽셀 단위로 세므로 투영 면적 가중이 자동 적용된다.

각 view 당 최대 SUB 개로 균등 서브샘플(히스토그램 추정에는 충분, 용량 절감).

산출: ply_hsv/prototype_colors.npz
        <obj>__render42   uint8 [42, SUB, 3]
        <obj>__ply_view42 uint8 [42, SUB, 3]
        <obj>__ply_all1   uint8 [1, SUB, 3]
"""
import csv, os, time

import cv2
import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
REPO = os.path.dirname(RSRCH)
SUB = 20000
MAX_VERT = 1_200_000

OBJ = {
    "Bear": ("data/cad/Bear/bear_color_with_normal_vertexcolor.ply", "Bear"),
    "Dinosaur": ("data/cad/Dinosaur/Dinosaur_color_with_normal_vertexcolor.ply", "Dinosaur"),
    "Febreze_high": ("data/cad/Febreze_high/Febreze_color_with_normal_vertexcolor.ply", "Febreze_high"),
    "Mugcup_high": ("data/cad/Mugcup_color_high/Mugcup_color_with_normal_vertexcolor.ply", "Mugcup_color_high"),
    "Rabbit": ("data/cad/Rabbit/Rabbit_color_with_normal_vertexcolor.ply", "Rabbit"),
    "Sauce_high": ("data/cad/Sauce_high/Sauce_color_with_normal_vertexcolor.ply", "Sauce_high"),
    "Sikhye_high": ("data/cad/Sikhye_high/Sikhye_color_with_normal_vertexcolor.ply", "Sikhye_high"),
    "choco_hazelnut_high": ("data/cad/choco_hazelnut_color_high/choco_hazelnut_color_with_normal_vertexcolor.ply",
                            "choco_hazelnut_color_high"),
    "saffron": ("data/cad/saffron/saffron_color_with_normal_vertexcolor.ply", "saffron"),
    "milk": ("data/cad/milk/Milk.ply", "milk"),
}
PLY_T = {"char": ("b", 1), "uchar": ("B", 1), "short": ("h", 2), "ushort": ("H", 2),
         "int": ("i", 4), "uint": ("I", 4), "float": ("f", 4), "double": ("d", 8),
         "int8": ("b", 1), "uint8": ("B", 1), "int16": ("h", 2), "uint16": ("H", 2),
         "int32": ("i", 4), "uint32": ("I", 4), "float32": ("f", 4), "float64": ("d", 8)}


def load_ply(path, max_vert=MAX_VERT):
    with open(path, "rb") as f:
        props, nv, elem, fmt = [], 0, None, ""
        while True:
            line = f.readline().decode("ascii", "replace").strip()
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
    idx = {n: i for i, n in enumerate(names)}
    need = [idx[c] for c in ("x", "y", "z", "red", "green", "blue")]
    step = max(1, nv // max_vert)
    if not fmt.startswith("ascii"):
        endian = "<" if "little" in fmt else ">"
        dt = np.dtype([(n, endian + PLY_T[t][0]) for n, t in props])
        a = np.fromfile(path, dtype=dt, count=nv, offset=off)[::step]
        return (np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float32),
                np.stack([a["red"], a["green"], a["blue"]], 1).astype(np.uint8))
    rows = []
    with open(path, "r", errors="replace") as f:
        f.seek(off)
        for i in range(nv):
            line = f.readline()
            if not line:
                break
            if i % step:
                continue
            t = line.split()
            if len(t) >= len(names):
                rows.append([t[j] for j in need])
    a = np.array(rows, dtype=np.float64)
    return a[:, :3].astype(np.float32), a[:, 3:].astype(np.uint8)


def sub(a, n=SUB, seed=0):
    if len(a) == 0:
        return np.zeros((0, 3), np.uint8)
    if len(a) <= n:
        r = np.random.RandomState(seed).randint(0, len(a), n - len(a))
        return np.concatenate([a, a[r]]) if len(a) < n else a
    return a[np.random.RandomState(seed).choice(len(a), n, replace=False)]


out, log = {}, []
for obj, (rel, tdir) in OBJ.items():
    p, td = os.path.join(REPO, rel), os.path.join(REPO, "template", tdir, "templates")
    if not os.path.isfile(p):
        print(f"[skip] {obj}"); continue
    t0 = time.time()
    V, C = load_ply(p)
    tree = cKDTree(V)

    out[f"{obj}__ply_all1"] = sub(C)[None, ...]

    rend, plyv, dstat, npx = [], [], [], []
    for i in range(42):
        rp = os.path.join(td, f"rgb_{i}.png")
        mp = os.path.join(td, f"mask_{i}.png")
        xp = os.path.join(td, f"xyz_{i}.npy")
        if not all(os.path.isfile(x) for x in (rp, mp, xp)):
            continue
        m = cv2.imread(mp, cv2.IMREAD_GRAYSCALE) > 0
        bgr = cv2.imread(rp)
        if bgr is None or m.sum() < 50:
            continue
        rend.append(sub(bgr[m][:, ::-1], seed=i))                 # BGR->RGB
        pts = np.load(xp).astype(np.float32)[m]
        d, j = tree.query(pts, k=1, workers=-1)
        plyv.append(sub(C[j], seed=i))
        dstat.append(float(np.median(d))); npx.append(int(m.sum()))
    if not rend:
        continue
    out[f"{obj}__render42"] = np.stack(rend)
    out[f"{obj}__ply_view42"] = np.stack(plyv)

    def msat(a):
        return float(cv2.cvtColor(a.reshape(-1, 1, 3)[:, :, ::-1], cv2.COLOR_BGR2HSV)[..., 1].mean())

    log.append({"object": obj, "n_vertex": len(V), "n_view": len(rend),
                "mean_visible_px": round(float(np.mean(npx)), 0),
                "nn_dist_median_mm": round(float(np.median(dstat)), 3),
                "nn_dist_max_mm": round(float(np.max(dstat)), 3),
                "sat_render": round(msat(np.concatenate(rend)), 1),
                "sat_ply_view": round(msat(np.concatenate(plyv)), 1),
                "sat_ply_all": round(msat(C), 1),
                "ply_view42_reliable": int(np.median(dstat) < 3.0)})
    print(f"{obj:22s} v={len(V):>9,d} nn중앙 {np.median(dstat):>7.3f}mm "
          f"채도 렌더 {log[-1]['sat_render']:>5.1f} / PLY시점 {log[-1]['sat_ply_view']:>5.1f} "
          f"/ PLY전체 {log[-1]['sat_ply_all']:>5.1f}  ({time.time()-t0:.0f}s)")

np.savez_compressed(os.path.join(HERE, "prototype_colors.npz"), **out)
with open(os.path.join(ROOT, "results", "ply_prototype_build.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(log[0].keys())); w.writeheader(); w.writerows(log)
sz = os.path.getsize(os.path.join(HERE, "prototype_colors.npz")) / 1e6
print(f"\n-> prototype_colors.npz ({sz:.1f} MB)")
