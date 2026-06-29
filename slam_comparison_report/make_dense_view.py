#!/usr/bin/env python3
"""Render colored Top-View (X-Z) of the DENSE maps for ORB-SLAM3 and RTAB-Map.

ORB-SLAM3's orbslam3_ros2 node already accumulates a dense RGB-D cloud
(orbslam3_dense_map.pcd, ~0.4-0.9M pts) — the earlier comparison only plotted
the SPARSE map_points.pcd, which is why ORB "looked" empty. This renders the
dense clouds (true RGB color) so ORB's dense map is visible and comparable to
RTAB-Map's dense_map.pcd. Run in `conda activate rtabmap` (matplotlib).
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DATASETS = ["SLAM_forward_backward_repeat", "SLAM_one_lap",
            "SLAM_one_lap_back_and_forth", "SLAM_three_laps"]
SYSTEMS = [
    ("ORB-SLAM3 (dense RGB-D)", ROOT / "orbslam_ws" / "output", "orbslam3_dense_map.pcd"),
    ("RTAB-Map (dense RGB-D)", ROOT / "rtabmap_ws" / "output", "dense_map.pcd"),
]
MAX_PLOT = 400000


def load_pcd_xyzrgb(path: Path):
    """Return (Nx3 xyz, Nx3 rgb floats 0..1 or None)."""
    if not path.exists() or path.stat().st_size == 0:
        return np.empty((0, 3), np.float32), None
    fields, started = [], False
    xyz, rgb = [], []
    with path.open("r", errors="replace") as f:
        for line in f:
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
                ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
                x, y, z = float(p[ix]), float(p[iy]), float(p[iz])
            except (ValueError, IndexError):
                continue
            xyz.append((x, y, z))
            if "rgb" in fields:
                try:
                    packed = int(float(p[fields.index("rgb")]))
                    rgb.append(((packed >> 16) & 255, (packed >> 8) & 255, packed & 255))
                except (ValueError, IndexError):
                    rgb.append((120, 120, 120))
    if not xyz:
        return np.empty((0, 3), np.float32), None
    a = np.asarray(xyz, np.float32)
    c = (np.asarray(rgb, np.float32) / 255.0) if rgb and len(rgb) == len(xyz) else None
    m = np.isfinite(a).all(axis=1)
    a = a[m]
    if c is not None:
        c = c[m]
    return a, c


def plot_dense(out_png: Path, title: str, xyz, rgb):
    if not len(xyz):
        return False
    if len(xyz) > MAX_PLOT:
        idx = np.linspace(0, len(xyz) - 1, MAX_PLOT).astype(int)
        xyz = xyz[idx]
        rgb = rgb[idx] if rgb is not None else None
    fig, ax = plt.subplots(figsize=(9, 9), dpi=160)
    if rgb is not None:
        ax.scatter(xyz[:, 0], xyz[:, 2], s=0.4, c=np.clip(rgb, 0, 1), linewidths=0)
    else:
        ax.scatter(xyz[:, 0], xyz[:, 2], s=0.4, c=xyz[:, 1], cmap="viridis", linewidths=0)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Z [m]")
    ax.set_title(f"{title}  ({len(xyz):,} pts shown)", fontsize=12)
    ax.grid(True, linewidth=0.3, color="#e8e8e8")
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)
    return True


def main() -> int:
    made = 0
    for ds in DATASETS:
        for label, root, fname in SYSTEMS:
            xyz, rgb = load_pcd_xyzrgb(root / ds / fname)
            out = root / ds / "dense_map_topview.png"
            ok = plot_dense(out, f"{label} | {ds}", xyz, rgb)
            print(f"  {label:26s} {ds:30s} pts={len(xyz):>8d} -> {'PNG' if ok else 'SKIP'}", flush=True)
            made += int(ok)
    print(f"\nDone: {made} dense top-view PNGs.")
    return 0 if made == 8 else 1


if __name__ == "__main__":
    raise SystemExit(main())
