#!/usr/bin/env python3
"""render_real_3d.py — 직접 녹화한 세션의 물체 지도를 3D 시점으로 나란히 그린다 (render_maps_3d.py 의 실제 주행 판).

  위  전체 지도: FM-Fusion (이름 + 점 뭉치) | Object Pose Map (이름 + 6D 자세 → CAD 모델과 물체 좌표축)
  아래 물체별 확대: 같은 가상 카메라로 본 FM-Fusion(윗줄)과 Object Pose Map(아랫줄)
  배경 = 최종 SLAM 자세로 쌓은 인식 카메라 깊이 점군, 파란 선 = 인식 카메라 경로.
  입력: make_real_scans.py 의 scans/<name> (color, depth, pose, meta.json, landmarks.json) 과 run_real.sh 의 FM-Fusion 출력.

    /home/jucpark/anaconda3/envs/sam6d/bin/python objpose/tools/baselines/fmfusion/render_real_3d.py \
        --data ~/DeepLearning/Datasets/fmfusion_real/scans --fmf objpose/output/fmf3d_real_261004/fmfusion --out objpose/output/fmf3d_real_261004/maps3d
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path

import cv2
import numpy as np
import trimesh
import yaml
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import render_maps_3d as R3  # noqa: E402
import eval_maps as EM  # noqa: E402

CAD_ROOT = Path.home() / "DeepLearning/CLI_environment_server/sam6d_ws"
COLORS = ["#1f6fb2", "#e08a1e", "#3a9d5d", "#b8342a", "#7b5ea7", "#8c564b", "#d4a017", "#17a2b8"]
OTHER = np.array([0.35, 0.35, 0.35])
AXC = R3.AXC
SHOW = {"choco_hazelnut_high": "choco box", "Febreze_high": "febreze", "Mugcup_high": "mug", "Sikhye_high": "sikhye can",
        "saffron": "jug", "milk": "milk", "Bear": "bear", "Dinosaur": "dinosaur"}


def show(nm):
    return SHOW.get(nm, nm)


def cad_samples(name, cfg, frames, n=200_000):
    """CAD 표면 점 [m]·색·법선을 자세 추정에 쓰는 물체 좌표계(model_points)로. 좌표계를 바꾼 물체는 object_frames.json 의 변환을 되돌린다."""
    m = trimesh.load(CAD_ROOT / cfg[name]["cad_ply"], process=False)
    p, fi, c = trimesh.sample.sample_surface(m, n, sample_color=True, seed=0)
    p, nr = np.asarray(p), np.asarray(m.face_normals)[fi]
    T = np.eye(4)
    for e in frames.get(name, []):
        T = T @ np.asarray(e["T_old_new"], float)
    T = np.linalg.inv(T)
    p, nr = p @ T[:3, :3].T + T[:3, 3], nr @ T[:3, :3].T
    mp = np.load(REPO / "sam6d_realtime/assets/model_points" / f"{name}.npy")
    err = float(np.percentile(cKDTree(mp).query(p[::20])[0], 99))
    assert err < 6.0, f"{name}: CAD does not match model_points (p99 {err:.1f} mm)"
    return p / 1000.0, np.asarray(c)[:, :3] / 255.0, nr


def cloud(dst: Path, meta, step, pix, voxel, zmax, near=None, near_r=0.0, min_hits=2):
    K = np.asarray(meta["K"])
    v, u = np.mgrid[0:meta["size"][1]:pix, 0:meta["size"][0]:pix]
    P, C = [], []
    for i, _, ok in meta["frames"][::step]:
        if not ok:
            continue
        f = f"frame-{i:06d}"
        d = cv2.imread(str(dst / "depth" / f"{f}.png"), cv2.IMREAD_UNCHANGED)[::pix, ::pix].astype(np.float32) * 1e-3
        rgb = cv2.imread(str(dst / "color" / f"{f}.png"))[::pix, ::pix, ::-1]
        T = np.loadtxt(dst / "pose" / f"{f}.txt")
        m = (d > 0.3) & (d < zmax)
        z = d[m]
        Pw = np.c_[(u[m] - K[0, 2]) * z / K[0, 0], (v[m] - K[1, 2]) * z / K[1, 1], z] @ T[:3, :3].T + T[:3, 3]
        Cw = rgb[m] / 255.0
        if near is not None:
            k = near.query(Pw, distance_upper_bound=near_r)[0] < near_r
            Pw, Cw = Pw[k], Cw[k]
        P.append(Pw); C.append(Cw)
    P, C = np.concatenate(P), np.concatenate(C)
    _, idx, cnt = np.unique(np.floor(P / voxel).astype(np.int64), axis=0, return_index=True, return_counts=True)
    idx = idx[cnt >= min_hits]                                                    # 한두 번만 찍힌 복셀은 깊이 잡음
    return P[idx], C[idx]


def fade(C, k=0.55):
    g = C.mean(1, keepdims=True)
    return (C * 0.5 + g * 0.5) * k + (1 - k) * 0.93


def without(P, C, obj_pts, gap):
    if not len(obj_pts):
        return P, C
    far = cKDTree(np.concatenate(obj_pts)).query(P, distance_upper_bound=gap)[0] > gap
    return P[far], C[far]


def clip_near(P, C, cam, near):
    k = (P - cam.eye) @ cam.R[2] > near                                          # 가상 카메라 바로 앞을 가리는 점은 뺀다
    return P[k], C[k]


def radius(voxel, cam, dist, k=0.75):
    return max(1, int(np.ceil(k * voxel * cam.f / dist)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--fmf", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--scenes", nargs="*", default=None)
    ap.add_argument("--elev", type=float, default=48.0)
    ap.add_argument("--azim", type=float, default=0.0, help="전체 지도 시점을 위 축 둘레로 돌린다 [deg]")
    ap.add_argument("--zoom-dist", type=float, default=0.85)
    ap.add_argument("--zoom-elev", type=float, default=25.0)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    cfg = {o["name"]: o for o in yaml.safe_load((REPO / "sam6d_realtime/configs/yolo_ism_objects.yaml").read_text())["objects"]}
    frames = json.loads((REPO / "sam6d_realtime/assets/object_frames.json").read_text())
    for s in a.scenes or sorted(p.name for p in a.fmf.iterdir() if (p / "instance_info.txt").exists()):
        dst = a.data / s
        meta = json.loads((dst / "meta.json").read_text())
        lms = sorted(json.loads((dst / "landmarks.json").read_text()), key=lambda m: m["name"])
        col = {m["name"]: np.array(to_rgb(COLORS[j % len(COLORS)])) for j, m in enumerate(lms)}
        cams = np.array([np.loadtxt(dst / "pose" / f"frame-{i:06d}.txt") for i, _, ok in meta["frames"] if ok])
        cpos = cams[:, :3, 3]
        up = np.linalg.svd(cpos - cpos.mean(0), full_matrices=False)[2][2]                 # 경로 평면의 법선
        if np.mean(cams[:, :3, 1] @ up) > 0:                                             # 카메라 y 축(영상 아래)의 반대가 위
            up = -up
        fmf = []
        for line in (a.fmf / s / "instance_info.txt").read_text().splitlines():
            if line.startswith("#") or not line.strip():
                continue
            f = line.split(";")
            fmf.append((f[1].split("(")[0], EM.read_ply_xyz(a.fmf / s / f"{int(f[0]):04d}.ply")))
        ours = []
        for m in lms:
            T = np.asarray(m["T_w_obj"], float)
            p, c, n = cad_samples(m["name"], cfg, frames)
            light = up * 0.8 + np.cross(up, [1, 0, 0]) * 0.4; light /= np.linalg.norm(light)
            sh = 0.55 + 0.45 * np.clip((n @ T[:3, :3].T) @ light, 0, 1)
            ours.append((m["name"], p @ T[:3, :3].T + T[:3, 3], np.clip(c * sh[:, None] * 1.1, 0, 1), T))
        octr = np.array([T[:3, 3] for *_, T in ours])
        cam_h = float(np.median(cpos @ up))

        def crop(P, C):                                                               # 천장·높은 벽은 뺀다
            h = P @ up - cam_h
            k = (h > -2.2) & (h < 0.25)
            return P[k], C[k]

        Pg, Cg = crop(*cloud(dst, meta, step=2, pix=3, voxel=0.012, zmax=3.0, near=cKDTree(octr), near_r=2.5, min_hits=3))
        Pf, Cf = crop(*cloud(dst, meta, step=1, pix=2, voxel=0.006, zmax=3.0, near=cKDTree(octr), near_r=0.8, min_hits=3))
        Cg, Cf = fade(Cg), fade(Cf)
        fmf_layers = lambda r: [(Pw, np.tile(col.get(nm, OTHER), (len(Pw), 1)), r) for nm, Pw in fmf]  # noqa: E731
        box = dict(boxstyle="round,pad=0.25", fc="white", alpha=0.92, lw=1.6)

        def axes3(ax, cam, T, L, lw):
            o = T[:3, 3]
            uv = cam.project(np.stack([o] + [o + T[:3, k] * L for k in range(3)]))[0] / R3.SS
            for k in range(3):
                ax.annotate("", uv[1 + k], xytext=uv[0], arrowprops=dict(arrowstyle="-|>", color="white", lw=lw + 2.6, mutation_scale=5 * lw, shrinkA=0, shrinkB=0))
                ax.annotate("", uv[1 + k], xytext=uv[0], arrowprops=dict(arrowstyle="-|>", color=AXC[k], lw=lw, mutation_scale=5 * lw, shrinkA=0, shrinkB=0))

        def labels(ax, cam, items, fs, dx, dy):
            tops = [cam.project(Pw[[np.argmax(Pw @ up)]])[0][0] / R3.SS for _, Pw in items]
            off = R3.label_offsets(tops, dx=dx, dy=dy, base=dy + 4) if tops else {}
            for j, (nm, _) in enumerate(items):
                c = col.get(nm, OTHER)
                if not (0 <= tops[j][0] <= R3.W and 0 <= tops[j][1] <= R3.H):
                    continue
                ax.annotate(show(nm), tops[j], xytext=(tops[j][0], max(tops[j][1] - off[j], dy * 0.6)), ha="center", va="center", fontsize=fs, color=c,
                            bbox=dict(box, ec=c), arrowprops=dict(arrowstyle="-", color=c, lw=1.2), annotation_clip=True)

        # ---- 전체 지도 ----
        R3.W, R3.H = 1500, 950
        ctr = (octr.min(0) + octr.max(0)) / 2
        def toward_path(o):                                                           # 물체 → 가장 가까운 경로 쪽 (수평)
            dv = cpos[np.argmin(np.linalg.norm(cpos - o, axis=1))] - o
            dv -= (dv @ up) * up
            return dv / np.linalg.norm(dv)
        d0 = np.mean([toward_path(o) for o in octr], 0)
        if np.linalg.norm(d0) < 0.3:                                                  # 물체가 사방을 향함 → 경로 중심 쪽
            d0 = cpos.mean(0) - octr.mean(0)
        d0 -= (d0 @ up) * up; d0 /= np.linalg.norm(d0)
        az = np.deg2rad(a.azim)
        d0 = np.cos(az) * d0 + np.sin(az) * np.cross(up, d0)
        ext = np.linalg.norm(octr - ctr, axis=1).max() + 0.45
        dist = ext / np.tan(np.deg2rad(19))
        el = np.deg2rad(a.elev)
        cam = R3.Cam(ctr + dist * (np.cos(el) * d0 + np.sin(el) * up), ctr, up, 0.95 * R3.W)
        path = np.concatenate([np.linspace(p, q, 12) for p, q in zip(cpos[:-1], cpos[1:])])
        tr = (path, np.tile(to_rgb("#3b6fd4"), (len(path), 1)), 3)
        rb = radius(0.012, cam, dist, 0.9)
        Pn, Cn = clip_near(Pg, Cg, cam, dist - ext - 0.3)
        left = R3.render(cam, [(*without(Pn, Cn, [Pw for _, Pw in fmf], 0.02), rb), tr] + fmf_layers(radius(0.01, cam, dist, 0.9)))
        right = R3.render(cam, [(*without(Pn, Cn, [Pw[::8] for _, Pw, _, _ in ours], 0.06), rb), tr] + [(Pw, Cw, 2) for _, Pw, Cw, _ in ours])

        nz = len(ours)
        fig = plt.figure(figsize=(30, 9.9 + 0.8 + 2 * 30 / nz * 0.8 + 0.5), dpi=100)
        gs = fig.add_gridspec(3, 2 * nz, height_ratios=[9.5, 30 / nz * 0.8, 30 / nz * 0.8], left=0.004, right=0.996, top=0.955, bottom=0.004, wspace=0.03, hspace=0.03)
        axL, axR = fig.add_subplot(gs[0, :nz]), fig.add_subplot(gs[0, nz:])
        for ax, im in ((axL, left), (axR, right)):
            ax.imshow(im); ax.set_xlim(0, R3.W); ax.set_ylim(R3.H, 0); ax.axis("off")
        labels(axL, cam, fmf, 13, 135, 30)
        labels(axR, cam, [(nm, Pw) for nm, Pw, _, _ in ours], 13, 135, 30)
        for *_, T in ours:
            axes3(axR, cam, T, 0.13, 2.8)
        axL.set_title(f"FM-Fusion — 물체마다 이름 + 점 뭉치 (방향 없음) · 지도 물체 {len(fmf)}개", fontsize=22, pad=10)
        axR.set_title(f"제안 — 물체마다 이름 + 6D 자세 (위치 3 + 방향 3) · 지도 물체 {len(ours)}개", fontsize=22, pad=10)

        # ---- 물체별 확대: 물체를 가장 가까이서 본 경로 쪽에서 ----
        R3.W = R3.H = 620
        for j, (nm, Pw, Cw, T) in enumerate(ours):
            o = T[:3, 3]
            dv = toward_path(o)
            ez = np.deg2rad(a.zoom_elev)
            zc = R3.Cam(o + a.zoom_dist * (np.cos(ez) * dv + np.sin(ez) * up), o, up, 1.15 * R3.W)
            nearf = [(n2, P2) for n2, P2 in fmf if np.linalg.norm(P2.mean(0) - o) < 0.45]
            rb = radius(0.006, zc, a.zoom_dist, 0.8)
            Pz, Cz = clip_near(Pf, Cf, zc, a.zoom_dist - 0.25)
            zl = R3.render(zc, [(*without(Pz, Cz, [P2 for _, P2 in nearf], 0.02), rb)] + [(P2, np.tile(col.get(n2, OTHER), (len(P2), 1)), radius(0.01, zc, a.zoom_dist, 0.8)) for n2, P2 in nearf])
            zr = R3.render(zc, [(*without(Pz, Cz, [P2[::6] for _, P2, _, _ in ours], 0.06), rb)] + [(P2, C2, 3) for _, P2, C2, _ in ours])
            a1, a2 = fig.add_subplot(gs[1, 2 * j:2 * j + 2]), fig.add_subplot(gs[2, 2 * j:2 * j + 2])
            for ax, im in ((a1, zl), (a2, zr)):
                ax.imshow(im); ax.set_xlim(0, R3.W); ax.set_ylim(R3.H, 0); ax.set_xticks([]); ax.set_yticks([])
                for sp in ax.spines.values():
                    sp.set_edgecolor(col[nm]); sp.set_linewidth(2.5)
            labels(a1, zc, nearf, 12, 210, 46)
            if not nearf:
                a1.text(0.5, 0.06, "지도에 없음", transform=a1.transAxes, ha="center", fontsize=14, color="#b8342a", bbox=dict(box, ec="#b8342a"))
            axes3(a2, zc, T, 0.09, 2.6)
            a2.text(0.5, 0.04, show(nm), transform=a2.transAxes, ha="center", fontsize=14, color=col[nm], bbox=dict(box, ec=col[nm]))
            if j == 0:
                a1.set_ylabel("FM-Fusion", fontsize=17); a2.set_ylabel("제안", fontsize=17)
        fig.savefig(a.out / f"map3d_{s}.png", dpi=100)
        plt.close(fig)
        print(s, len(fmf), len(ours), flush=True)


if __name__ == "__main__":
    main()
