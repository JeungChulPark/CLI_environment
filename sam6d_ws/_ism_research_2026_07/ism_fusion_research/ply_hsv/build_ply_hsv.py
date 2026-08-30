#!/usr/bin/env python3
"""build_ply_hsv.py — PLY 색상 prototype 을 두 방식으로 생성한다 (READ-ONLY).

방식 A (ply_all1) : PLY 의 **모든** vertex RGB → HSV → 객체당 히스토그램 **1개**
                    시점 정보가 없고 객체 전체의 색 분포만 표현한다.

방식 B (ply_view42): 42개 template camera pose 마다 **그 시점에서 보이는 vertex 만** 골라
                    히스토그램을 만들어 객체당 **42개**. 현재 RGB 템플릿과 같은 구조가 된다.

방식 B 의 가시성 판정 — 근사가 아니라 정확하다:
  기존 렌더가 저장한 `xyz_<i>.npy` 는 각 픽셀의 **PLY 좌표계 3D 위치**다
  (Bear 기준 범위 ±65 로 PLY 의 ±65 와 일치, 마스크 밖은 -1).
  따라서 mask 내부 픽셀의 xyz 를 PLY vertex 에 최근접 대응시키면
  "그 시점에 실제로 보인 표면" 의 색을 정확히 얻는다.
  Bear 검증: 최근접 거리 중앙값 0.664 mm (객체 대각 209 mm).
  픽셀 단위로 세므로 **투영 면적 가중**이 자동으로 걸린다(렌더와 동일한 성질).

산출: ply_hsv/ply_prototypes.npz
        <obj>__all1     [1, 128]     H16xS8
        <obj>__view42   [42, 128]
        <obj>__all1_h32 [1, 32]      Hue 전용 (보정 실험용)
        <obj>__view42_h32 [42, 32]
        <obj>__view42_sat [42]       시점별 평균 채도 (저채도 분석용)
"""
import os, sys, time

import cv2
import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
REPO = os.path.dirname(RSRCH)

# ISM 객체 -> (운영 PLY, 운영 템플릿 폴더)
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
MAX_VERT = 1_200_000        # KD-tree 상한. 0.66mm 간격 기준 서브샘플해도 sub-mm 정확도 유지


PLY_T = {"char": ("b", 1), "uchar": ("B", 1), "short": ("h", 2), "ushort": ("H", 2),
         "int": ("i", 4), "uint": ("I", 4), "float": ("f", 4), "double": ("d", 8),
         "int8": ("b", 1), "uint8": ("B", 1), "int16": ("h", 2), "uint16": ("H", 2),
         "int32": ("i", 4), "uint32": ("I", 4), "float32": ("f", 4), "float64": ("d", 8)}


def load_ply(path, max_vert=MAX_VERT):
    """PLY 의 xyz + rgb 를 읽는다 (ascii / binary 모두). 대형 파일은 균등 서브샘플."""
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
        # binary: numpy structured dtype 으로 한 번에 읽는다
        endian = "<" if "little" in fmt else ">"
        dt = np.dtype([(n, endian + PLY_T[t][0]) for n, t in props])
        a = np.fromfile(path, dtype=dt, count=nv, offset=off)[::step]
        V = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float32)
        C = np.stack([a["red"], a["green"], a["blue"]], 1).astype(np.uint8)
        return V, C

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
            if len(t) < len(names):
                continue
            rows.append([t[j] for j in need])
    a = np.array(rows, dtype=np.float64)
    return a[:, :3].astype(np.float32), a[:, 3:].astype(np.uint8)


def hs_hist(rgb):
    """Nx3 RGB -> H16xS8 정규화 히스토그램 (실사/렌더와 동일 binning)."""
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0, 1], None, [16, 8], [0, 180, 0, 256]).flatten()
    return (h / max(h.sum(), 1e-12)).astype(np.float32)


def h32_hist(rgb):
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = cv2.calcHist([hsv], [0], None, [32], [0, 180]).flatten()
    return (h / max(h.sum(), 1e-12)).astype(np.float32)


def mean_sat(rgb):
    img = np.asarray(rgb, np.uint8).reshape(-1, 1, 3)[:, :, ::-1]
    return float(cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[..., 1].mean())


out, log = {}, []
for obj, (rel, tdir) in OBJ.items():
    p = os.path.join(REPO, rel)
    if not os.path.isfile(p):
        print(f"[skip] {obj}: PLY 없음"); continue
    t0 = time.time()
    V, C = load_ply(p)
    tree = cKDTree(V)
    print(f"{obj:22s} vertex {len(V):>9,d} 적재 {time.time()-t0:>5.1f}s", end="", flush=True)

    # 방식 A
    out[f"{obj}__all1"] = hs_hist(C)[None, :]
    out[f"{obj}__all1_h32"] = h32_hist(C)[None, :]

    # 방식 B
    td = os.path.join(REPO, "template", tdir, "templates")
    hs, h32, sat, npix, dstat = [], [], [], [], []
    for i in range(42):
        xp, mp = os.path.join(td, f"xyz_{i}.npy"), os.path.join(td, f"mask_{i}.png")
        if not (os.path.isfile(xp) and os.path.isfile(mp)):
            continue
        xyz = np.load(xp).astype(np.float32)
        m = cv2.imread(mp, cv2.IMREAD_GRAYSCALE) > 0
        pts = xyz[m]
        if len(pts) < 50:
            continue
        d, j = tree.query(pts, k=1, workers=-1)
        col = C[j]
        hs.append(hs_hist(col)); h32.append(h32_hist(col))
        sat.append(mean_sat(col)); npix.append(len(pts)); dstat.append(float(np.median(d)))
    if hs:
        out[f"{obj}__view42"] = np.stack(hs)
        out[f"{obj}__view42_h32"] = np.stack(h32)
        out[f"{obj}__view42_sat"] = np.array(sat, np.float32)
        log.append({"object": obj, "n_vertex_used": len(V), "n_view": len(hs),
                    "mean_visible_px": float(np.mean(npix)),
                    "nn_dist_median_mm": float(np.median(dstat)),
                    "nn_dist_max_mm": float(np.max(dstat)),
                    "mean_saturation": float(np.mean(sat))})
        print(f" | view {len(hs)} 가시픽셀 평균 {np.mean(npix):>7.0f} "
              f"최근접 중앙 {np.median(dstat):.3f}mm 평균채도 {np.mean(sat):.1f} "
              f"({time.time()-t0:.0f}s)")

np.savez_compressed(os.path.join(HERE, "ply_prototypes.npz"), **out)
import csv
with open(os.path.join(ROOT, "results", "ply_prototype_build.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(log[0].keys())); w.writeheader(); w.writerows(log)
print(f"\n-> {os.path.join(HERE,'ply_prototypes.npz')}")
