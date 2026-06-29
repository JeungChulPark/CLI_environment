#!/usr/bin/env python3
"""Render per-run SLAM result images (X-Z top view).

Usage: render_run.py <orb3|rtab> <dataset> <run_idx> <src_dir> <dest_dir>
Produces in dest_dir:
  trajectory_2d.png        : dense 2D map + estimated trajectory + start/end
  keyframes_loops_2d.png   : dense 2D map + keyframe positions + loop-closure edges
and copies the dense 3D map (map_3d.pcd).
ORB dense = orbslam3_dense_kf.pcd (Pattern-A reprojection); RTAB = dense_map.pcd.
Loop edges: ORB from loop_edges.txt; RTAB from rtabmap.db (Link type=1) + node poses.
"""
import sys, sqlite3, shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

system, ds, run_idx, src, dest = sys.argv[1], sys.argv[2], sys.argv[3], Path(sys.argv[4]), Path(sys.argv[5])
dest.mkdir(parents=True, exist_ok=True)
MAXPLOT = 350000


def load_pcd(path):
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
                k = int(float(p[fields.index("rgb")])); rgb.append(((k >> 16) & 255, (k >> 8) & 255, k & 255))
            except ValueError:
                rgb.append((130, 130, 130))
    a = np.asarray(xyz, np.float32)
    c = (np.asarray(rgb, np.float32) / 255.0) if rgb and len(rgb) == len(xyz) else None
    if not a.size:
        return a, None
    m = np.isfinite(a).all(axis=1)
    return a[m], (c[m] if c is not None else None)


def load_tum(path):
    r = []
    if path.exists():
        for l in path.read_text(errors="replace").splitlines():
            p = l.split()
            if len(p) >= 4 and not l.startswith("#"):
                try:
                    r.append((float(p[1]), float(p[2]), float(p[3])))
                except ValueError:
                    pass
    return np.asarray(r, np.float32) if r else np.empty((0, 3), np.float32)


def load_loop_edges():
    """Unified: loop_edges.txt lines 'c1x c1y c1z c2x c2y c2z' -> [((x1,z1),(x2,z2))]."""
    f = src / "loop_edges.txt"
    edges = []
    if f.exists():
        for l in f.read_text(errors="replace").splitlines():
            p = l.split()
            if len(p) >= 6:
                try:
                    a = [float(x) for x in p[:6]]; edges.append(((a[0], a[2]), (a[3], a[5])))
                except ValueError:
                    pass
    return edges


def subsample(xyz, rgb):
    if len(xyz) > MAXPLOT:
        idx = np.linspace(0, len(xyz) - 1, MAXPLOT).astype(int)
        return xyz[idx], (rgb[idx] if rgb is not None else None)
    return xyz, rgb


mapfile = "map_3d.pcd"
xyz, rgb = load_pcd(src / mapfile)
traj = load_tum(src / "trajectory.txt")
kf = load_tum(src / "keyframes.txt")
edges = load_loop_edges()

sysname = "ORB-SLAM3" if system == "orb3" else "RTAB-Map"
xs, rs = subsample(xyz, rgb)

# --- PNG 1: dense map + trajectory ---
fig, ax = plt.subplots(figsize=(9, 9), dpi=150)
if xs.size:
    ax.scatter(xs[:, 0], xs[:, 2], s=0.35, c=np.clip(rs, 0, 1) if rs is not None else "#999", linewidths=0, zorder=1)
if len(traj) >= 2:
    ax.plot(traj[:, 0], traj[:, 2], "-", c="#ff7f0e", lw=1.7, zorder=3, label="estimated trajectory")
    ax.scatter([traj[0, 0]], [traj[0, 2]], c="#e8000b", s=140, edgecolors="white", zorder=5, label="Start")
    ax.scatter([traj[-1, 0]], [traj[-1, 2]], c="#1f77ff", s=140, edgecolors="white", zorder=5, label="End")
ax.set_aspect("equal", adjustable="box"); ax.set_xlabel("X [m]"); ax.set_ylabel("Z [m]")
ax.set_title(f"{sysname} | {ds} | run{run_idx}\n2D dense map + estimated trajectory", fontsize=11)
ax.grid(True, lw=0.3, color="#e8e8e8"); ax.legend(loc="best", fontsize=9)
fig.tight_layout(); fig.savefig(dest / "trajectory_2d.png"); plt.close(fig)

# --- PNG 2: dense map (faint) + keyframes + loop edges ---
fig, ax = plt.subplots(figsize=(9, 9), dpi=150)
if xs.size:
    ax.scatter(xs[:, 0], xs[:, 2], s=0.25, c="#cfcfcf", alpha=0.5, linewidths=0, zorder=1)
if len(kf):
    ax.scatter(kf[:, 0], kf[:, 2], s=26, marker="o", c="#ff7f0e", edgecolors="#7a3d00",
               linewidths=0.3, zorder=3, label=f"KeyFrames ({len(kf)})")
for i, (a, b) in enumerate(edges):
    ax.plot([a[0], b[0]], [a[1], b[1]], "-", c="#d000d0", lw=1.3, alpha=0.9, zorder=4,
            label="loop-closure edge" if i == 0 else None)
ax.set_aspect("equal", adjustable="box"); ax.set_xlabel("X [m]"); ax.set_ylabel("Z [m]")
ax.set_title(f"{sysname} | {ds} | run{run_idx}\nKeyFrames + loop-closure edges ({len(edges)})", fontsize=11)
ax.grid(True, lw=0.3, color="#e8e8e8"); ax.legend(loc="best", fontsize=9)
fig.tight_layout(); fig.savefig(dest / "keyframes_loops_2d.png"); plt.close(fig)

print(f"{system} {ds} run{run_idx}: map_pts={len(xyz)} traj={len(traj)} kf={len(kf)} loop_edges={len(edges)}")
