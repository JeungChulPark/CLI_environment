#!/usr/bin/env python3
"""Render dense map (RGB) with the estimated trajectory overlaid, X-Z top view.

ORB uses orbslam3_dense_kf.pcd (optimized-KF dense), RTAB uses dense_map.pcd.
Writes per-output-dir `dense_map_trajectory.png` and a per-dataset side-by-side
`slam_comparison_report/dense_trajectory_<ds>.png`. Run in `conda activate rtabmap`.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import numpy as np

for _fp in ("/usr/share/fonts/truetype/nanum/NanumGothic.ttf",):
    if Path(_fp).exists():
        fm.fontManager.addfont(_fp)
        plt.rcParams["font.family"] = fm.FontProperties(fname=_fp).get_name()
        break
plt.rcParams["axes.unicode_minus"] = False

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "slam_comparison_report"
DATASETS = ["SLAM_forward_backward_repeat", "SLAM_one_lap",
            "SLAM_one_lap_back_and_forth", "SLAM_three_laps"]
SYSTEMS = [("ORB-SLAM3", ROOT / "orbslam_ws" / "output", "orbslam3_dense_kf.pcd"),
           ("RTAB-Map", ROOT / "rtabmap_ws" / "output", "dense_map.pcd")]
MAX_PLOT = 400000


def load_pcd(path: Path):
    xyz, rgb, started, fields = [], [], False, []
    if not path.exists():
        return np.empty((0, 3), np.float32), None
    for line in path.read_text(errors="replace").splitlines():
        if not started:
            t = line.split()
            if t and t[0].upper() == "FIELDS":
                fields = t[1:]
            elif t and t[0].upper() == "DATA":
                started = True
            continue
        p = line.split()
        if len(p) < 3:
            continue
        try:
            xyz.append((float(p[0]), float(p[1]), float(p[2])))
        except ValueError:
            continue
        if "rgb" in fields and len(p) > fields.index("rgb"):
            try:
                k = int(float(p[fields.index("rgb")]))
                rgb.append(((k >> 16) & 255, (k >> 8) & 255, k & 255))
            except ValueError:
                rgb.append((120, 120, 120))
    a = np.asarray(xyz, np.float32)
    c = (np.asarray(rgb, np.float32) / 255.0) if rgb and len(rgb) == len(xyz) else None
    m = np.isfinite(a).all(axis=1)
    return (a[m], (c[m] if c is not None else None)) if a.size else (a, None)


def load_traj(path: Path):
    r = []
    if path.exists():
        for l in path.read_text(errors="replace").splitlines():
            p = l.split()
            if len(p) >= 4:
                try:
                    r.append((float(p[1]), float(p[2]), float(p[3])))
                except ValueError:
                    pass
    return np.asarray(r, np.float32) if r else np.empty((0, 3), np.float32)


def draw(ax, title, xyz, rgb, traj):
    if len(xyz) > MAX_PLOT:
        idx = np.linspace(0, len(xyz) - 1, MAX_PLOT).astype(int)
        xyz = xyz[idx]
        rgb = rgb[idx] if rgb is not None else None
    if rgb is not None:
        ax.scatter(xyz[:, 0], xyz[:, 2], s=0.35, c=np.clip(rgb, 0, 1), linewidths=0, zorder=1)
    else:
        ax.scatter(xyz[:, 0], xyz[:, 2], s=0.35, c="#999999", linewidths=0, zorder=1)
    if len(traj) >= 2:
        ax.plot(traj[:, 0], traj[:, 2], "-", c="#ff7f0e", lw=1.8, alpha=0.95, zorder=3, label="추정 trajectory")
        ax.scatter([traj[0, 0]], [traj[0, 2]], c="#e8000b", s=150, edgecolors="white",
                   linewidths=1.6, zorder=5, label="Start")
        ax.scatter([traj[-1, 0]], [traj[-1, 2]], c="#1f77ff", s=150, edgecolors="white",
                   linewidths=1.6, zorder=5, label="End")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Z [m]")
    ax.set_title(title, fontsize=12)
    ax.grid(True, linewidth=0.4, color="#e6e6e6")
    ax.legend(loc="best", fontsize=9, framealpha=0.9)


def main() -> int:
    REPORT.mkdir(parents=True, exist_ok=True)
    n = 0
    for ds in DATASETS:
        loaded = {}
        for label, root, fname in SYSTEMS:
            xyz, rgb = load_pcd(root / ds / fname)
            traj = load_traj(root / ds / "trajectory.txt")
            loaded[label] = (xyz, rgb, traj, root)
            # individual
            fig, ax = plt.subplots(figsize=(9, 9), dpi=160)
            draw(ax, f"{label} dense + trajectory | {ds}", xyz, rgb, traj)
            fig.tight_layout()
            fig.savefig(root / ds / "dense_map_trajectory.png")
            plt.close(fig)
            n += 1
            print(f"  {label:10s} {ds:30s} pts={len(xyz):>8d} traj={len(traj):>5d}", flush=True)
        # side-by-side
        fig, axes = plt.subplots(1, 2, figsize=(17, 8.6), dpi=160)
        for ax, (label, _, _) in zip(axes, SYSTEMS):
            xyz, rgb, traj, _ = loaded[label]
            draw(ax, f"{label}", xyz, rgb, traj)
        fig.suptitle(f"Dense map + 추정 trajectory  |  {ds}  (Top-View X–Z)", fontsize=14)
        fig.tight_layout()
        fig.savefig(REPORT / f"dense_trajectory_{ds}.png")
        plt.close(fig)
    print(f"\nDone: {n} per-dir PNGs + {len(DATASETS)} side-by-side PNGs.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
