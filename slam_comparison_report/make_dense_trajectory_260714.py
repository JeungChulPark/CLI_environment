#!/usr/bin/env python3
"""Render ORB-SLAM3 dense map (RGB) with the estimated trajectory overlaid, X-Z top view.

Targets the 260714_frame_data SLAM_data bags, whose ORB3 outputs live in
`orbslam_ws/output/260714/slam_<hhmmss>/` as `orbslam3_dense_map.pcd` +
`CameraTrajectory.txt` (note: different filenames than the sdk_bag_test runs that
`make_dense_trajectory.py` handles, and rgb is stored as a float bit-pattern).

Writes per-run `dense_map_trajectory.png` next to each trajectory, plus a 2x3 grid
`slam_comparison_report/dense_trajectory_260714_SLAM_grid.png`.
Run in `conda activate rtabmap`.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import numpy as np
import pandas as pd

for _fp in ("/usr/share/fonts/truetype/nanum/NanumGothic.ttf",):
    if Path(_fp).exists():
        fm.fontManager.addfont(_fp)
        plt.rcParams["font.family"] = fm.FontProperties(fname=_fp).get_name()
        break
plt.rcParams["axes.unicode_minus"] = False

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "slam_comparison_report"
OUT = ROOT / "orbslam_ws" / "output" / "260714"
RUNS = ["slam_105023", "slam_105319", "slam_105657",
        "slam_110109", "slam_110536", "slam_110637"]
DURATION_S = {"slam_105023": 133, "slam_105319": 99, "slam_105657": 183,
              "slam_110109": 182, "slam_110536": 51, "slam_110637": 118}
MAX_PLOT = 400000


def load_pcd(path: Path):
    """Return (xyz, rgb01). rgb is packed into a float32 field, so view its bits."""
    if not path.exists():
        return np.empty((0, 3), np.float32), None
    fields, skip = [], 0
    with path.open(errors="replace") as fh:
        for line in fh:
            skip += 1
            t = line.split()
            if t and t[0].upper() == "FIELDS":
                fields = t[1:]
            elif t and t[0].upper() == "DATA":
                break
    df = pd.read_csv(path, sep=r"\s+", skiprows=skip, header=None,
                     names=fields, engine="c", on_bad_lines="skip")
    xyz = df[["x", "y", "z"]].to_numpy(np.float32)
    rgb = None
    if "rgb" in df.columns:
        bits = df["rgb"].to_numpy(np.float32).view(np.uint32)
        rgb = np.stack([(bits >> 16) & 255, (bits >> 8) & 255, bits & 255], 1).astype(np.float32) / 255.0
    m = np.isfinite(xyz).all(axis=1)
    return xyz[m], (rgb[m] if rgb is not None else None)


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


def draw(ax, title, xyz, rgb, traj, legend=True):
    if len(xyz) > MAX_PLOT:
        idx = np.linspace(0, len(xyz) - 1, MAX_PLOT).astype(int)
        xyz = xyz[idx]
        rgb = rgb[idx] if rgb is not None else None
    if rgb is not None:
        ax.scatter(xyz[:, 0], xyz[:, 2], s=0.35, c=np.clip(rgb, 0, 1), linewidths=0, zorder=1)
    else:
        ax.scatter(xyz[:, 0], xyz[:, 2], s=0.35, c="#999999", linewidths=0, zorder=1)
    if len(traj) >= 2:
        ax.plot(traj[:, 0], traj[:, 2], "-", c="#ff7f0e", lw=1.8, alpha=0.95, zorder=3,
                label="추정 trajectory")
        ax.scatter([traj[0, 0]], [traj[0, 2]], c="#e8000b", s=150, edgecolors="white",
                   linewidths=1.6, zorder=5, label="Start")
        ax.scatter([traj[-1, 0]], [traj[-1, 2]], c="#1f77ff", s=150, edgecolors="white",
                   linewidths=1.6, zorder=5, label="End")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Z [m]")
    ax.set_title(title, fontsize=11)
    ax.grid(True, linewidth=0.4, color="#e6e6e6")
    if legend:
        ax.legend(loc="best", fontsize=8, framealpha=0.9)


def main() -> int:
    REPORT.mkdir(parents=True, exist_ok=True)
    loaded = {}
    for run in RUNS:
        d = OUT / run
        xyz, rgb = load_pcd(d / "orbslam3_dense_map.pcd")
        traj = load_traj(d / "CameraTrajectory.txt")
        loaded[run] = (xyz, rgb, traj)
        fig, ax = plt.subplots(figsize=(9, 9), dpi=160)
        draw(ax, f"ORB-SLAM3 dense + trajectory | {run} ({DURATION_S[run]}s)", xyz, rgb, traj)
        fig.tight_layout()
        fig.savefig(d / "dense_map_trajectory.png")
        plt.close(fig)
        print(f"  {run:14s} pts={len(xyz):>8d} traj={len(traj):>5d}", flush=True)

    fig, axes = plt.subplots(2, 3, figsize=(19, 12.5), dpi=150)
    for ax, run in zip(axes.ravel(), RUNS):
        xyz, rgb, traj = loaded[run]
        draw(ax, f"{run}  ({DURATION_S[run]}s, {len(traj)} poses)", xyz, rgb, traj, legend=False)
    h, l = axes.ravel()[0].get_legend_handles_labels()
    fig.legend(h, l, loc="lower center", ncol=3, fontsize=11, framealpha=0.9)
    fig.suptitle("260714 SLAM_data | ORB-SLAM3 dense map + 추정 trajectory (Top-View X–Z)",
                 fontsize=15)
    fig.tight_layout(rect=(0, 0.035, 1, 1))
    fig.savefig(REPORT / "dense_trajectory_260714_SLAM_grid.png")
    plt.close(fig)
    print(f"\nDone: {len(RUNS)} per-run PNGs + 1 grid PNG.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
