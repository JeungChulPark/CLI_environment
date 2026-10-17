#!/usr/bin/env python3
"""live_result_figures.py — figures and a map-build video from a finished run's logs.

The hub does not store the viewer's rendered frames, only jsonl logs. Everything a slide needs
about a live run can still be rebuilt from them:

    slam_poses.jsonl      t_ns, state, T_wc[16], recv_lag_s      -> trajectory, arrival latency
    sam6d_estimates.jsonl t_ns, object, placed, T_w_obj[16],     -> object positions, spread,
                          result_age_s, ms{yolo,ism,pem,total}      result age
    sam6d/frames.jsonl    stamp_ns, n_accept, n_boxes, ms        -> per-frame cost split

Outputs (English labels on purpose: they go into a paper):
    <out>/<run>_overview.png    4 panels: top view, per-object spread, cost histogram, latency
    <out>/<run>_mapbuild.mp4    top-view animation of the map being built over time
    <out>/<run>_stats.json      the numbers printed on the slide

    python3 objpose/tools/live_result_figures.py <run dir> [more run dirs...] [--out DIR]
    python3 objpose/tools/live_result_figures.py --compare <run dir> <run dir> ... [--out DIR]

`--compare` adds one bar chart across runs (pose rate, arrival lag, SAM-6D rate, detections).
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter

plt.rcParams.update({"figure.dpi": 130, "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.25, "axes.spines.top": False, "axes.spines.right": False})

LANE_BLUE, LANE_ORANGE, ACCENT = "#56c2e0", "#f2a03d", "#c792ea"
OBJ_COLORS = ["#e6533c", "#3caae6", "#6ecf68", "#f0c419", "#c792ea", "#ff8fb1", "#4fd6c8", "#f2a03d", "#9aa7b4"]


# ── loading ──────────────────────────────────────────────────────────────────
def read_jsonl(path: Path):
    if not path.exists():
        return []
    out = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return out


def load_run(run: Path):
    poses = read_jsonl(run / "slam_poses.jsonl")
    ests = read_jsonl(run / "sam6d_estimates.jsonl")
    frames = read_jsonl(run / "sam6d" / "frames.jsonl")
    P = [(int(p["t_ns"]), np.asarray(p["T_wc"], float).reshape(4, 4), p.get("recv_lag_s"))
         for p in poses if p.get("type") == "pose" and p.get("T_wc")]
    P.sort(key=lambda r: r[0])
    return P, ests, frames


def ground_plane_axes(xyz: np.ndarray):
    """(e1, e2, origin): the two directions the platform actually moves in (PCA), so one
    function draws a top view for both camera worlds (y down) and LiDAR worlds (z up)."""
    c = xyz.mean(0)
    u, s, vt = np.linalg.svd(xyz - c, full_matrices=False)
    return vt[0], vt[1], c


# ── stats ────────────────────────────────────────────────────────────────────
def object_stats(ests):
    """{name: {'pts': Nx3 world positions, 'median': 3, 'spread_cm': (median, p90)}}"""
    by = {}
    for e in ests:
        if not e.get("placed") or not e.get("T_w_obj"):
            continue
        T = np.asarray(e["T_w_obj"], float).reshape(4, 4)
        by.setdefault(e["object"], []).append((int(e["t_ns"]), T[:3, 3]))
    out = {}
    for name, rows in by.items():
        rows.sort(key=lambda r: r[0])
        pts = np.array([p for _, p in rows])
        med = np.median(pts, axis=0)
        d = np.linalg.norm(pts - med, axis=1) * 100.0
        out[name] = {"t_ns": np.array([t for t, _ in rows]), "pts": pts, "median": med,
                     "spread_cm": (float(np.median(d)), float(np.percentile(d, 90))), "n": len(pts)}
    return dict(sorted(out.items(), key=lambda kv: -kv[1]["n"]))


def run_stats(run: Path, P, ests, frames, objs):
    lags = np.array([p[2] for p in P if p[2] is not None], float)
    ages = np.array([e["result_age_s"] for e in ests if e.get("result_age_s") is not None], float)
    tot = np.array([f["ms"]["total"] for f in frames if f.get("ms")], float)
    empty = np.array([f["ms"]["total"] for f in frames if f.get("ms") and not f.get("n_accept")], float)
    busy = np.array([f["ms"]["total"] for f in frames if f.get("ms") and f.get("n_accept")], float)
    dur = (P[-1][0] - P[0][0]) / 1e9 if len(P) > 1 else 0.0
    placed = sum(1 for e in ests if e.get("placed"))
    s = {
        "run": run.name,
        "duration_s": round(dur, 1),
        "slam_poses": len(P),
        "pose_rate_hz": round(len(P) / dur, 1) if dur else None,
        "pose_arrival_lag_ms": {"median": round(float(np.median(lags)) * 1e3, 1),
                                "p90": round(float(np.percentile(lags, 90)) * 1e3, 1)} if lags.size else None,
        "sam6d_frames": len(frames),
        "sam6d_rate_hz": round(len(frames) / dur, 2) if dur else None,
        "sam6d_ms": {"median": round(float(np.median(tot)), 1) if tot.size else None,
                     "empty_median": round(float(np.median(empty)), 1) if empty.size else None,
                     "with_detection_median": round(float(np.median(busy)), 1) if busy.size else None},
        "stage_share_pct": None,
        "result_age_s": {"median": round(float(np.median(ages)), 2),
                         "p90": round(float(np.percentile(ages, 90)), 2)} if ages.size else None,
        "estimates_total": len(ests), "estimates_placed": placed,
        "objects": {k: {"observations": v["n"],
                        "spread_median_cm": round(v["spread_cm"][0], 2),
                        "spread_p90_cm": round(v["spread_cm"][1], 2)} for k, v in objs.items()},
    }
    if frames:
        y = sum(f["ms"]["yolo"] for f in frames if f.get("ms"))
        i = sum(f["ms"]["ism"] for f in frames if f.get("ms"))
        p = sum(f["ms"]["pem"] for f in frames if f.get("ms"))
        tt = max(y + i + p, 1)
        s["stage_share_pct"] = {"yolo": round(100 * y / tt, 1), "ism": round(100 * i / tt, 1),
                                "pem": round(100 * p / tt, 1)}
    return s


# ── figures ──────────────────────────────────────────────────────────────────
def fig_overview(run: Path, P, ests, frames, objs, stats, out: Path):
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    traj = np.array([T[:3, 3] for _, T, _ in P])
    e1, e2, c = ground_plane_axes(traj) if len(traj) > 3 else (np.array([1., 0, 0]), np.array([0, 0, 1.]), np.zeros(3))
    proj = lambda X: np.column_stack([(X - c) @ e1, (X - c) @ e2])

    # (a) top view
    ax = axes[0, 0]
    tp = proj(traj)
    ax.plot(tp[:, 0], tp[:, 1], color=LANE_BLUE, lw=1.0, alpha=0.9, label="camera trajectory")
    # objects sit centimetres apart on one table, so their names go in the legend, not on the map
    for k, (name, o) in enumerate(objs.items()):
        col = OBJ_COLORS[k % len(OBJ_COLORS)]
        op = proj(o["pts"])
        ax.scatter(op[:, 0], op[:, 1], s=4, alpha=0.22, color=col, linewidths=0)
        m = proj(o["median"][None, :])[0]
        ax.scatter(*m, s=80, marker="*", color=col, edgecolor="white", linewidth=0.5, zorder=5,
                   label=f"{name.replace('_high','')} (n={o['n']})")
    ax.set_aspect("equal")
    ax.set_title("(a) Top view — trajectory and object observations", fontsize=10)
    ax.set_xlabel("m"); ax.set_ylabel("m")
    ax.legend(fontsize=6.5, loc="upper left", bbox_to_anchor=(1.005, 1.0), frameon=False,
              handletextpad=0.3, labelspacing=0.35)

    # (b) per-object spread
    ax = axes[0, 1]
    names = list(objs)
    if names:
        data = []
        for name in names:
            o = objs[name]
            data.append(np.linalg.norm(o["pts"] - o["median"], axis=1) * 100.0)
        bp = ax.boxplot(data, vert=True, showfliers=False, patch_artist=True, widths=0.6)
        for k, box in enumerate(bp["boxes"]):
            box.set(facecolor=OBJ_COLORS[k % len(OBJ_COLORS)], alpha=0.55, linewidth=0.8)
        for med in bp["medians"]:
            med.set(color="white", linewidth=1.2)
        ax.set_xticks(range(1, len(names) + 1))
        ax.set_xticklabels([n.replace("_high", "") for n in names], rotation=30, ha="right", fontsize=7)
    ax.set_ylabel("distance from per-object median [cm]")
    ax.set_title("(b) Observation spread of each object (static objects)", fontsize=10)

    # (c) per-frame cost
    ax = axes[1, 0]
    empty = [f["ms"]["total"] for f in frames if f.get("ms") and not f.get("n_accept")]
    busy = [f["ms"]["total"] for f in frames if f.get("ms") and f.get("n_accept")]
    bins = np.logspace(0.8, 3.8, 40)
    if empty:
        ax.hist(empty, bins=bins, color="#6ecf68", alpha=0.75, label=f"no detection (n={len(empty)})")
    if busy:
        ax.hist(busy, bins=bins, color=ACCENT, alpha=0.75, label=f"with detection (n={len(busy)})")
    ax.set_xscale("log"); ax.set_xlabel("SAM-6D time per frame [ms]"); ax.set_ylabel("frames")
    sh = stats.get("stage_share_pct") or {}
    ax.set_title(f"(c) Inference cost  ·  YOLO {sh.get('yolo','–')}% / ISM {sh.get('ism','–')}% / PEM {sh.get('pem','–')}%",
                 fontsize=10)
    ax.legend(fontsize=7)

    # (d) latency over time
    ax = axes[1, 1]
    t0 = P[0][0] if P else 0
    tp_ = np.array([(t - t0) / 1e9 for t, _, lag in P if lag is not None])
    lp = np.array([lag for _, _, lag in P if lag is not None], float)
    if tp_.size:
        ax.plot(tp_, lp * 1e3, color=LANE_BLUE, lw=0.6, alpha=0.8, label="SLAM pose arrival lag")
    te = np.array([(int(e["t_ns"]) - t0) / 1e9 for e in ests if e.get("result_age_s") is not None])
    ae = np.array([e["result_age_s"] for e in ests if e.get("result_age_s") is not None], float)
    if te.size:
        ax.scatter(te, ae * 1e3, s=3, color=LANE_ORANGE, alpha=0.5, label="SAM-6D result age")
    ax.axhline(300, color="#9aa7b4", ls="--", lw=0.8)
    ax.annotate("display delay 300 ms", (0.02, 320), xycoords=("axes fraction", "data"), fontsize=7, color="#9aa7b4")
    ax.set_yscale("log"); ax.set_xlabel("time [s]"); ax.set_ylabel("latency [ms]")
    ax.set_title("(d) Latency: pose arrival vs. inference result age", fontsize=10)
    ax.legend(fontsize=7, loc="upper right")

    ttl = (f"{run.name}  ·  {stats['duration_s']} s  ·  {stats['slam_poses']} poses "
           f"({stats['pose_rate_hz']} Hz)  ·  SAM-6D {stats['sam6d_frames']} frames "
           f"({stats['sam6d_rate_hz']} Hz)  ·  {stats['estimates_placed']} placed detections")
    fig.suptitle(ttl, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    path = out / f"{run.name}_overview.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def video_mapbuild(run: Path, P, objs, out: Path, seconds=20.0, fps=20):
    """Top-view animation: trajectory grows, object estimates appear and converge."""
    traj = np.array([T[:3, 3] for _, T, _ in P])
    if len(traj) < 10:
        return None
    e1, e2, c = ground_plane_axes(traj)
    proj = lambda X: np.column_stack([(np.atleast_2d(X) - c) @ e1, (np.atleast_2d(X) - c) @ e2])
    tp = proj(traj)
    t0, t1 = P[0][0], P[-1][0]
    n_frames = int(seconds * fps)
    times = np.linspace(t0, t1, n_frames)
    pose_t = np.array([t for t, _, _ in P])

    fig, ax = plt.subplots(figsize=(7.5, 6))
    pad = 0.6
    ax.set_xlim(tp[:, 0].min() - pad, tp[:, 0].max() + pad)
    ax.set_ylim(tp[:, 1].min() - pad, tp[:, 1].max() + pad)
    ax.set_aspect("equal"); ax.set_xlabel("m"); ax.set_ylabel("m")
    line, = ax.plot([], [], color=LANE_BLUE, lw=1.1, alpha=0.9)
    head = ax.scatter([], [], s=45, color="white", edgecolor=LANE_BLUE, zorder=6)
    obj_names = list(objs)
    scat, stars, labels = {}, {}, {}
    for k, name in enumerate(obj_names):
        col = OBJ_COLORS[k % len(OBJ_COLORS)]
        scat[name] = ax.scatter([], [], s=5, color=col, alpha=0.25, linewidths=0)
        stars[name] = ax.scatter([], [], s=90, marker="*", color=col, edgecolor="white", linewidth=0.5, zorder=5)
        labels[name] = ax.annotate("", (0, 0), fontsize=7, color=col,
                                   textcoords="offset points", xytext=(6, 5))
    title = ax.set_title("", fontsize=10)

    writer = FFMpegWriter(fps=fps, bitrate=2400, metadata={"title": run.name})
    path = out / f"{run.name}_mapbuild.mp4"
    with writer.saving(fig, str(path), dpi=130):
        for t in times:
            i = int(np.searchsorted(pose_t, t))
            line.set_data(tp[:i, 0], tp[:i, 1])
            if i:
                head.set_offsets(tp[i - 1:i])
            n_obj = 0
            for name in obj_names:
                o = objs[name]
                m = int(np.searchsorted(o["t_ns"], t))
                if m:
                    pts = proj(o["pts"][:m])
                    scat[name].set_offsets(pts)
                    med = np.median(o["pts"][:m], axis=0)
                    mp = proj(med)[0]
                    stars[name].set_offsets([mp])
                    labels[name].set_text(f"{name.replace('_high','')} ({m})")
                    labels[name].set_position((0, 0))
                    labels[name].xy = tuple(mp)
                    n_obj += 1
            title.set_text(f"{run.name}   t = {(t - t0)/1e9:6.1f} s   ·   objects placed: {n_obj}")
            writer.grab_frame()
    plt.close(fig)
    return path


def fig_compare(runs_stats, out: Path):
    ks = [("pose_rate_hz", "SLAM pose rate [Hz]", LANE_BLUE),
          ("pose_lag_ms", "pose arrival lag, median [ms]", LANE_ORANGE),
          ("sam6d_rate_hz", "SAM-6D processed [Hz]", ACCENT),
          ("result_age_ms", "result age, median [ms]", "#6ecf68")]
    fig, axes = plt.subplots(1, 4, figsize=(14, 3.6))
    names = [s["run"].replace("one_live_", "") for s in runs_stats]
    for ax, (key, label, col) in zip(axes, ks):
        vals = []
        for s in runs_stats:
            if key == "pose_lag_ms":
                vals.append((s["pose_arrival_lag_ms"] or {}).get("median", 0))
            elif key == "result_age_ms":
                vals.append(round((s["result_age_s"] or {}).get("median", 0) * 1000, 1))
            else:
                vals.append(s.get(key) or 0)
        ax.bar(range(len(vals)), vals, color=col, alpha=0.85)
        for i, v in enumerate(vals):
            ax.annotate(f"{v:g}", (i, v), ha="center", va="bottom", fontsize=7)
        ax.set_xticks(range(len(names)))
        ax.set_xticklabels(names, rotation=35, ha="right", fontsize=7)
        ax.set_title(label, fontsize=9)
    fig.suptitle("Live runs (2026-09-17, laptop sensors → workstation): backend comparison", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    path = out / "live_backend_compare.png"
    fig.savefig(path)
    plt.close(fig)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / "docs" / "figs")
    ap.add_argument("--compare", action="store_true", help="also draw one bar chart across the given runs")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--video-seconds", type=float, default=20.0)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)

    all_stats = []
    for run in a.runs:
        P, ests, frames = load_run(run)
        if not P:
            print(f"[skip] {run.name}: no poses")
            continue
        objs = object_stats(ests)
        st = run_stats(run, P, ests, frames, objs)
        all_stats.append(st)
        (a.out / f"{run.name}_stats.json").write_text(json.dumps(st, indent=1, ensure_ascii=False))
        print(f"[fig ] {fig_overview(run, P, ests, frames, objs, st, a.out)}")
        if not a.no_video and objs:
            v = video_mapbuild(run, P, objs, a.out, seconds=a.video_seconds)
            print(f"[vid ] {v}")
        print(f"[stat] {json.dumps(st, ensure_ascii=False)[:300]} ...")
    if a.compare and len(all_stats) > 1:
        print(f"[fig ] {fig_compare(all_stats, a.out)}")


if __name__ == "__main__":
    main()
