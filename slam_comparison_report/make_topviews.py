#!/usr/bin/env python3
"""Generate unified Top-View (X-Z) trajectory PNGs for ORB-SLAM3 and RTAB-Map.

Run in `conda activate rtabmap` (matplotlib). For each of the 4 datasets x 2
systems it reads the normalised outputs (map pcd, trajectory.txt, keyframes.txt)
and writes, into the SAME output dir, two PNGs with an identical visual style so
the two systems are directly comparable:
  - trajectory_topview.png            (map + trajectory + start/end + direction)
  - trajectory_topview_keyframes.png  (same + keyframe markers)

Convention (both systems use camera_color_optical_frame: X right, Y down,
Z forward), so the ground/top plane is X-Z. Start = red, End = blue,
trajectory = line, keyframes = star markers, arrows = travel direction.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATASETS = [
    "SLAM_forward_backward_repeat",
    "SLAM_one_lap",
    "SLAM_one_lap_back_and_forth",
    "SLAM_three_laps",
]
# (system label, output root, map pcd filename)
SYSTEMS = [
    ("ORB-SLAM3", PROJECT_ROOT / "orbslam_ws" / "output", "map_points.pcd"),
    ("RTAB-Map", PROJECT_ROOT / "rtabmap_ws" / "output", "dense_map.pcd"),
]

MAP_COLOR = "#c6c6c6"
TRAJ_COLOR = "#222222"
START_COLOR = "#d62728"   # red
END_COLOR = "#1f77b4"     # blue
KF_COLOR = "#ff7f0e"      # orange
MAX_MAP_PTS = 120000      # subsample dense maps for plotting


def load_pcd_xyz(path: Path) -> np.ndarray:
    if not path.exists() or path.stat().st_size == 0:
        return np.empty((0, 3), np.float32)
    fields, n_pts, data_started = [], 0, False
    rows = []
    with path.open("r", errors="replace") as f:
        for line in f:
            if not data_started:
                t = line.split()
                if not t:
                    continue
                key = t[0].upper()
                if key == "FIELDS":
                    fields = t[1:]
                elif key == "POINTS":
                    n_pts = int(t[1])
                elif key == "DATA":
                    data_started = True
                continue
            rows.append(line.split())
    if not rows:
        return np.empty((0, 3), np.float32)
    try:
        ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
    except ValueError:
        ix, iy, iz = 0, 1, 2
    out = []
    for r in rows:
        if len(r) > iz:
            try:
                out.append((float(r[ix]), float(r[iy]), float(r[iz])))
            except ValueError:
                pass
    arr = np.asarray(out, np.float32) if out else np.empty((0, 3), np.float32)
    if arr.size:
        arr = arr[np.isfinite(arr).all(axis=1)]
    return arr


def load_traj_xyz(path: Path) -> np.ndarray:
    if not path.exists() or path.stat().st_size == 0:
        return np.empty((0, 3), np.float32)
    rows = []
    with path.open("r", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) >= 4:
                try:
                    rows.append((float(p[1]), float(p[2]), float(p[3])))
                except ValueError:
                    pass
    arr = np.asarray(rows, np.float32) if rows else np.empty((0, 3), np.float32)
    if arr.size:
        arr = arr[np.isfinite(arr).all(axis=1)]
    return arr


def _plot(out_png: Path, title: str, map_xyz, traj_xyz, kf_xyz, with_kf: bool):
    fig, ax = plt.subplots(figsize=(9, 9), dpi=160)

    # map background (X-Z)
    if map_xyz.size:
        m = map_xyz
        if len(m) > MAX_MAP_PTS:
            m = m[:: max(1, len(m) // MAX_MAP_PTS)]
        ax.scatter(m[:, 0], m[:, 2], s=0.25, c=MAP_COLOR, alpha=0.55, linewidths=0, zorder=1)

    legend = [Line2D([0], [0], marker="o", linestyle="None", markerfacecolor=MAP_COLOR,
                     markeredgecolor="none", markersize=6, label="Map points")]

    if traj_xyz.shape[0] >= 2:
        tx, tz = traj_xyz[:, 0], traj_xyz[:, 2]
        ax.plot(tx, tz, "-", color=TRAJ_COLOR, linewidth=1.6, zorder=3, label="Trajectory")
        legend.append(Line2D([0], [0], color=TRAJ_COLOR, linewidth=1.6, label="Trajectory"))
        # direction arrows at ~12 evenly spaced segments
        n = len(tx)
        step = max(1, n // 12)
        for i in range(0, n - step, step):
            ax.annotate("", xy=(tx[i + step], tz[i + step]), xytext=(tx[i], tz[i]),
                        arrowprops=dict(arrowstyle="-|>", color=TRAJ_COLOR, lw=1.0, alpha=0.9),
                        zorder=4)
        ax.scatter([tx[0]], [tz[0]], s=130, c=START_COLOR, edgecolors="white",
                   linewidths=1.5, zorder=6, label="Start")
        ax.scatter([tx[-1]], [tz[-1]], s=130, c=END_COLOR, edgecolors="white",
                   linewidths=1.5, zorder=6, label="End")
        legend.append(Line2D([0], [0], marker="o", linestyle="None", markerfacecolor=START_COLOR,
                             markeredgecolor="white", markersize=11, label="Start"))
        legend.append(Line2D([0], [0], marker="o", linestyle="None", markerfacecolor=END_COLOR,
                             markeredgecolor="white", markersize=11, label="End"))

    if with_kf and kf_xyz.size:
        ax.scatter(kf_xyz[:, 0], kf_xyz[:, 2], s=46, marker="*", c=KF_COLOR,
                   edgecolors="#7a3d00", linewidths=0.4, zorder=5, label="KeyFrames")
        legend.append(Line2D([0], [0], marker="*", linestyle="None", markerfacecolor=KF_COLOR,
                             markeredgecolor="#7a3d00", markersize=13, label=f"KeyFrames ({len(kf_xyz)})"))

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Z [m]")
    ax.set_title(title, fontsize=12)
    ax.grid(True, linewidth=0.4, color="#e2e2e2")
    ax.legend(handles=legend, loc="best", fontsize=9, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_png)
    plt.close(fig)


def main() -> int:
    made = 0
    for ds in DATASETS:
        for label, root, mapname in SYSTEMS:
            out_dir = root / ds
            map_xyz = load_pcd_xyz(out_dir / mapname)
            traj = load_traj_xyz(out_dir / "trajectory.txt")
            kf = load_traj_xyz(out_dir / "keyframes.txt")
            title = f"{label}  |  {ds}  (Top-View X-Z)"
            if traj.shape[0] < 2:
                print(f"  WARN: {label}/{ds} trajectory has <2 poses; skipping", flush=True)
                continue
            _plot(out_dir / "trajectory_topview.png", title, map_xyz, traj, kf, with_kf=False)
            _plot(out_dir / "trajectory_topview_keyframes.png", title, map_xyz, traj, kf, with_kf=True)
            made += 2
            print(f"  {label:10s} {ds:30s} map={len(map_xyz):>8d} traj={len(traj):>5d} "
                  f"kf={len(kf):>4d} -> 2 PNG", flush=True)
    print(f"\nDone: {made} PNGs written.")
    return 0 if made == 16 else 1


if __name__ == "__main__":
    raise SystemExit(main())
