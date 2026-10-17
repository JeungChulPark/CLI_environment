#!/usr/bin/env python3
"""render_maps_3d.py — 같은 장면의 물체 지도를 비스듬한 3D 시점으로 나란히 그린다 (render_maps.py 의 3D 판).

  왼쪽  FM-Fusion        물체 = 이름 + 복원 점 뭉치 (보인 면만, 방향 없음)
  오른쪽 Object Pose Map  물체 = 이름 + 6D 자세 → 그 자세로 놓은 CAD 모델과 물체 좌표축
  배경 = SLAM 위치(정렬 후)로 쌓은 깊이 점군. 두 칸은 같은 가상 카메라.
  --views front back : 촬영한 쪽 / 반대쪽(FM-Fusion 은 보지 못한 면이 비어 있음)

    BENCH_SLAM=objpose/output/fmf_cmp_260928/slam /home/jucpark/anaconda3/envs/sam6d/bin/python \
        objpose/tools/baselines/fmfusion/render_maps_3d.py --fmf objpose/output/fmf_cmp_260928/fmfusion_gtbox \
        --ours objpose/output/fmf_cmp_260928/proposed_gt --out objpose/output/fmf_cmp_260928/maps3d
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path

import cv2
import numpy as np
import trimesh
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_memory_maps as BM  # noqa: E402
import eval_maps as EM  # noqa: E402

COLORS = ["#1f6fb2", "#e08a1e", "#3a9d5d", "#b8342a", "#7b5ea7", "#8c564b", "#d4a017"]
AXC = ["#e02020", "#18a030", "#2050e0"]                        # 물체 좌표축 x, y, z
W, H, SS = 1500, 1050, 2                                       # 칸 크기 [px], 내부 배율(계단 줄이기)
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False})


def dense_cloud(scene, r, stride=6, pix=2, voxel=0.002):
    """BM.point_cloud 보다 촘촘한 배경 점군 (3D 시점에서 구멍이 덜 보이게)."""
    sc = BM.YCBV / scene
    K = r["K"]
    v, u = np.mgrid[0:480:pix, 0:640:pix]
    P, C = [], []
    for k in range(0, len(r["ids"]), stride):
        if k not in r["slam"]:
            continue
        d = cv2.imread(str(sc / "depth" / f"{k + 1:06d}.png"), cv2.IMREAD_UNCHANGED)[::pix, ::pix].astype(np.float32) * 1e-4
        rgb = cv2.imread(str(sc / "rgb" / f"{k + 1:06d}.png"))[::pix, ::pix, ::-1]
        m = (d > 0.2) & (d < 1.6)
        z = d[m]; x = (u[m] - K[0, 2]) * z / K[0, 0]; y = (v[m] - K[1, 2]) * z / K[1, 1]
        T = r["A"] @ r["slam"][k]
        P.append(np.c_[x, y, z] @ T[:3, :3].T + T[:3, 3]); C.append(rgb[m] / 255.0)
    P, C = np.concatenate(P), np.concatenate(C)
    _, idx = np.unique(np.floor(P / voxel).astype(np.int64), axis=0, return_index=True)
    return P[idx], C[idx]


def cad_samples(models_dir: Path, obj_id: int, n=250_000):
    """CAD 표면 점 [m], 색(텍스처), 법선 — 물체 좌표계."""
    m = trimesh.load(models_dir / f"obj_{obj_id:06d}.ply", process=False)
    p, fi, c = trimesh.sample.sample_surface(m, n, sample_color=True, seed=0)
    return np.asarray(p) / 1000.0, np.asarray(c)[:, :3] / 255.0, np.asarray(m.face_normals)[fi]


class Cam:
    def __init__(self, eye, target, up, f):
        z = target - eye; z /= np.linalg.norm(z)
        x = np.cross(z, up); x /= np.linalg.norm(x)
        self.R = np.stack([x, np.cross(z, x), z]); self.eye = eye; self.f = f * SS

    def project(self, P):
        q = (P - self.eye) @ self.R.T
        return np.c_[self.f * q[:, 0] / q[:, 2] + W * SS / 2, self.f * q[:, 1] / q[:, 2] + H * SS / 2], q[:, 2]


def splat(img, zbuf, cam, P, C, r):
    """점을 반지름 r(px, 내부 해상도) 원으로 찍는다. 깊이 버퍼로 가림 처리."""
    uv, z = cam.project(P)
    ok = (z > 0.05) & (uv[:, 0] > -r) & (uv[:, 0] < W * SS + r) & (uv[:, 1] > -r) & (uv[:, 1] < H * SS + r)
    u, v, z, C = np.rint(uv[ok, 0]).astype(int), np.rint(uv[ok, 1]).astype(int), z[ok].astype(np.float32), C[ok]
    offs = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1) if dx * dx + dy * dy <= r * r + 1]
    idx = []
    for dx, dy in offs:
        uu, vv = u + dx, v + dy
        m = (uu >= 0) & (uu < W * SS) & (vv >= 0) & (vv < H * SS)
        i = np.where(m, vv * W * SS + uu, -1)
        idx.append(i)
        np.minimum.at(zbuf, i[m], z[m])
    flat = img.reshape(-1, 3)
    for i in idx:
        m = i >= 0
        m[m] = z[m] <= zbuf[i[m]] + 1e-6
        flat[i[m]] = C[m]


def render(cam, layers):
    img = np.ones((H * SS, W * SS, 3), np.float32)
    zbuf = np.full(H * SS * W * SS, np.inf, np.float32)
    for P, C, r in layers:
        splat(img, zbuf, cam, P, C, r)
    return cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA)


def label_offsets(anchors, dx=150, dy=30, base=34):
    """겹치는 이름표를 위로 한 칸씩 올린다. anchors [px] → 각 이름표의 세로 띄움 [pt 아님, px]."""
    placed, out = [], {}
    for i in np.argsort([-a[1] for a in anchors]):
        y = anchors[i][1] - base
        while any(abs(anchors[i][0] - px) < dx and abs(y - py) < dy for px, py in placed):
            y -= dy
        placed.append((anchors[i][0], y)); out[i] = anchors[i][1] - y
    return out


def short(nm):
    return nm.replace("ycbv_", "").replace("_", " ")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fmf", type=Path, required=True)
    ap.add_argument("--ours", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--scenes", nargs="*", default=None)
    ap.add_argument("--views", nargs="*", default=["front"], choices=["front", "back"])
    ap.add_argument("--elev", type=float, default=38.0, help="가상 카메라 올려본 각 [deg]")
    ap.add_argument("--dist", type=float, default=0.72, help="물체 중심에서 가상 카메라까지 [m]")
    ap.add_argument("--axis", type=float, default=0.055, help="물체 좌표축 길이 [m]")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    tab = json.loads(BM.TABLE.read_text())
    table = tab["objects"]
    models_dir = Path(tab["models_dir"]).parent / "models"
    by_id = {o["obj_id"]: o["name"] for o in table}
    oid = {o["name"]: o["obj_id"] for o in table}
    by_label = {o["ycb"].split("_", 1)[1].replace("_", " "): o["name"] for o in table}
    scenes = a.scenes or sorted(p.name for p in a.fmf.iterdir() if p.is_dir() and (p / "instance_info.txt").exists())
    for s in scenes:
        BM.RUNS["rt"] = a.ours
        r = BM.run_memory(s, "rt", by_id)
        A = r["A"]
        col = {nm: np.array(to_rgb(COLORS[j % len(COLORS)])) for j, nm in enumerate(sorted(r["gobj"]))}
        P, C = dense_cloud(s, r)
        c0, B, up, th = BM.top_frame(r, P)
        hgt = (P - c0) @ up - th
        keep = (hgt > -0.02) & (np.linalg.norm((P - c0) @ B.T, axis=1) < 0.75)
        g = C[keep].mean(1, keepdims=True)
        Pb, Cb = P[keep], (C[keep] * 0.45 + g * 0.55) * 0.55 + 0.42              # 배경은 옅고 흐리게
        light = up * 0.8 + B[0] * 0.3 + B[1] * 0.5; light /= np.linalg.norm(light)

        fmf = []
        for line in (a.fmf / s / "instance_info.txt").read_text().splitlines():
            if line.startswith("#") or not line.strip():
                continue
            f = line.split(";")
            lab = f[1].split("(")[0]
            Pw = EM.read_ply_xyz(a.fmf / s / f"{int(f[0]):04d}.ply") @ A[:3, :3].T + A[:3, 3]
            fmf.append((by_label.get(lab, lab), Pw))
        ours = []
        for lm in r["lms"]:
            T = A @ np.asarray(lm["T_w_obj"], float)
            p, c, n = cad_samples(models_dir, oid[lm["name"]])
            sh = 0.55 + 0.45 * np.clip((n @ T[:3, :3].T) @ light, 0, 1)
            ours.append((lm["name"], p @ T[:3, :3].T + T[:3, 3], np.clip(c * sh[:, None] * 1.1, 0, 1), T))

        def bg_without(obj_pts, gap):                                           # 물체 표면에 겹친 배경 점은 뺀다 (물체가 얼룩지지 않게)
            far = cKDTree(np.concatenate(obj_pts)).query(Pb, distance_upper_bound=gap)[0] > gap
            return Pb[far], Cb[far], 4
        bg_l = bg_without([Pw for _, Pw in fmf], 0.006)
        bg_r = bg_without([Pw[::4] for _, Pw, _, _ in ours], 0.015)

        cams_h = r["Twc_gt"][:, :3, 3] - c0
        d = cams_h.mean(0); d = d - (d @ up) * up; d /= np.linalg.norm(d)           # 물체 → 촬영 쪽 (수평)
        target = c0 + up * (th + 0.04)
        el = np.deg2rad(a.elev)
        fig, axs = plt.subplots(len(a.views), 2, figsize=(2 * W / 100, len(a.views) * H / 100 + 0.9), dpi=100, squeeze=False)
        for row, view in zip(axs, a.views):
            sgn = 1.0 if view == "front" else -1.0
            cam = Cam(target + a.dist * (np.cos(el) * sgn * d + np.sin(el) * up), target, up, 0.95 * W)
            left = render(cam, [bg_l] + [(Pw, np.tile(col.get(nm, np.array([.45, .45, .45])), (len(Pw), 1)), 7) for nm, Pw in fmf])
            right = render(cam, [bg_r] + [(Pw, Cw, 3) for _, Pw, Cw, _ in ours])
            for ax, im in zip(row, (left, right)):
                ax.imshow(im); ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.axis("off")
            box = dict(boxstyle="round,pad=0.25", fc="white", alpha=0.92, lw=1.6)
            tops = [cam.project(Pw[[np.argmax(Pw @ up)]])[0][0] / SS for _, Pw in fmf]
            off = label_offsets(tops)
            for j, (nm, Pw) in enumerate(fmf):
                c = col.get(nm, (.45, .45, .45))
                row[0].annotate(short(nm), tops[j], xytext=(0, off[j]), textcoords="offset pixels", ha="center", fontsize=15, color=c,
                                bbox=dict(box, ec=c), arrowprops=dict(arrowstyle="-", color=c, lw=1.4))
            tops = [cam.project(Pw[[np.argmax(Pw @ up)]])[0][0] / SS for _, Pw, _, _ in ours]
            off = label_offsets(tops)
            for j, (nm, Pw, _, T) in enumerate(ours):
                o = T[:3, 3]
                uv, _ = cam.project(np.stack([o] + [o + T[:3, k] * a.axis for k in range(3)]))
                uv /= SS
                for k in range(3):
                    row[1].annotate("", uv[1 + k], xytext=uv[0], arrowprops=dict(arrowstyle="-|>", color="white", lw=6.5, mutation_scale=22, shrinkA=0, shrinkB=0))
                    row[1].annotate("", uv[1 + k], xytext=uv[0], arrowprops=dict(arrowstyle="-|>", color=AXC[k], lw=3.6, mutation_scale=20, shrinkA=0, shrinkB=0))
                c = col.get(nm, (.45, .45, .45))
                row[1].annotate(short(nm), tops[j], xytext=(0, off[j]), textcoords="offset pixels", ha="center", fontsize=15, color=c,
                                bbox=dict(box, ec=c), arrowprops=dict(arrowstyle="-", color=c, lw=1.4))
            if len(a.views) > 1:
                row[0].text(0.012, 0.03, {"front": "촬영한 쪽에서 본 모습", "back": "반대쪽에서 본 모습"}[view], transform=row[0].transAxes,
                            fontsize=17, color="#333", bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#999", alpha=0.9))
        axs[0, 0].set_title("FM-Fusion — 물체마다 이름 + 점 뭉치 (방향 없음)", fontsize=22, pad=12)
        axs[0, 1].set_title("제안 — 물체마다 이름 + 6D 자세 (위치 3 + 방향 3)", fontsize=22, pad=12)
        fig.subplots_adjust(left=0.005, right=0.995, top=1 - 0.75 / fig.get_figheight(), bottom=0.005, wspace=0.015, hspace=0.02)
        fig.savefig(a.out / f"map3d_{s}.png", dpi=100)
        plt.close(fig)
        print(s, len(fmf), len(ours), flush=True)


if __name__ == "__main__":
    main()
