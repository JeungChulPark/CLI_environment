#!/usr/bin/env python3
"""compare_lanes.py — backend comparison from ONE multi-lane run (same SAM-6D detections).

A multi-lane run feeds the SAME detections to every SLAM lane, so the only difference between
the lanes is the SLAM itself. This reads that run's summary.json plus the per-lane logs and
produces the comparison figure and table used in the results section:

    (a) top view: every lane's SAM-camera trajectory and its object estimates, in the reference
        lane's world (each lane mapped through its frozen display alignment T_ref_lane)
    (b) per-object distance between the lanes' final object positions
    (c) per-lane observation spread (raw estimates around their own median)
    (d) per-lane runtime facts (poses, arrival lag, keyframes, memory state)

    python3 objpose/tools/compare_lanes.py <run dir> [--out DIR]
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams.update({"figure.dpi": 130, "font.size": 9, "axes.grid": True, "grid.alpha": 0.25,
                     "axes.spines.top": False, "axes.spines.right": False})


def read_jsonl(p: Path):
    if not p.exists():
        return []
    out = []
    for line in open(p, encoding="utf-8"):
        line = line.strip()
        if line:
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return out


def mat(v):
    return None if v is None else np.asarray(v, float).reshape(4, 4)


def apply(T, p):
    return (T[:3, :3] @ np.asarray(p, float)) + T[:3, 3]


def lane_assets(run: Path, lane, single: bool):
    sfx = "" if single else f"_{lane['name']}"
    poses = [p for p in read_jsonl(run / f"slam_poses{sfx}.jsonl") if p.get("T_wc")]
    ests = read_jsonl(run / f"sam6d_estimates{sfx}.jsonl")
    return poses, ests


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / "docs" / "figs")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    s = json.loads((a.run / "summary.json").read_text())
    lanes = s["lanes"]
    single = len(lanes) == 1
    ref_name = s.get("reference_lane")

    # world of the reference lane: each lane's own world is mapped through its display alignment
    A = {}
    for l in lanes:
        M = None if l["name"] == ref_name else mat(l.get("T_ref_lane"))
        A[l["name"]] = np.eye(4) if M is None else M

    colors = {l["name"]: l.get("color", "#56c2e0") for l in lanes}
    fig = plt.figure(figsize=(13, 8.5))
    ax_map = fig.add_subplot(2, 2, 1)
    ax_diff = fig.add_subplot(2, 2, 2)
    ax_spread = fig.add_subplot(2, 2, 3)
    ax_tbl = fig.add_subplot(2, 2, 4)
    ax_tbl.axis("off")

    # ── per-lane data ────────────────────────────────────────────────────────
    obj_world, spreads, facts = {}, {}, []
    all_xyz = []
    for l in lanes:
        name = l["name"]
        poses, ests = lane_assets(a.run, l, single)
        X = mat(l.get("extrinsic_matrix")) if l.get("extrinsic_matrix") else None  # not stored; use poses only
        traj = np.array([apply(A[name], mat(p["T_wc"])[:3, 3]) for p in poses]) if poses else np.zeros((0, 3))
        all_xyz.append(traj)
        # final object positions: object memory landmarks (active / remembered), else fusion objects
        lms = [m for m in (l.get("memory_landmarks") or []) if m.get("status") in ("active", "remembered")]
        pos = {}
        for m in lms:
            T = mat(m["T_w_obj"])
            if T is None:
                continue
            p = apply(A[name], T[:3, 3])
            # keep the instance with the most observations per class
            if m["name"] not in pos or m["n_obs"] > pos[m["name"]][1]:
                pos[m["name"]] = (p, m["n_obs"], m["status"])
        obj_world[name] = pos
        # raw observation spread
        sp = {}
        by = {}
        for e in ests:
            if e.get("placed") and e.get("T_w_obj"):
                by.setdefault(e["object"], []).append(mat(e["T_w_obj"])[:3, 3])
        for k, v in by.items():
            v = np.array(v)
            d = np.linalg.norm(v - np.median(v, axis=0), axis=1) * 100
            sp[k] = d
        spreads[name] = sp
        al = l.get("align_final") or {}
        facts.append({
            "lane": l["label"],
            "poses": f"{l['slam_ok']} / {l['slam_poses']}",
            "lag_s": l.get("slam_lag_s"),
            "track_ms": l.get("slam_track_ms"),
            "kf": f"{l.get('kf_count')} / {l.get('kf_updates')}",
            "maxcorr_cm": l.get("kf_max_correction_cm"),
            "memory": (l.get("memory") or {}).get("by_status"),
            "align": f"{al.get('mode')} rmse {al.get('rmse_cm')} cm" if name != ref_name else "reference",
        })
        if traj.size:
            ax_map.plot(traj[:, 0], traj[:, 1], lw=1.0, alpha=0.85, color=colors[name],
                        label=f"{l['label']} trajectory")
        for k, (p, n, st) in pos.items():
            ax_map.scatter(p[0], p[1], s=70, marker="*", color=colors[name], edgecolor="white", linewidth=0.5, zorder=5)

    ax_map.set_aspect("equal")
    ax_map.set_xlabel("m"); ax_map.set_ylabel("m")
    ax_map.set_title(f"(a) Both lanes in the {ref_name} world (stars = final object positions)", fontsize=10)
    ax_map.legend(fontsize=7, loc="lower right")

    # ── (b) per-object distance between lanes ───────────────────────────────
    if len(lanes) >= 2:
        n0, n1 = lanes[0]["name"], lanes[1]["name"]
        common = sorted(set(obj_world[n0]) & set(obj_world[n1]))
        d = [float(np.linalg.norm(obj_world[n0][k][0] - obj_world[n1][k][0]) * 100) for k in common]
        ax_diff.bar(range(len(common)), d, color="#c792ea", alpha=0.85)
        for i, v in enumerate(d):
            ax_diff.annotate(f"{v:.1f}", (i, v), ha="center", va="bottom", fontsize=7)
        ax_diff.set_xticks(range(len(common)))
        ax_diff.set_xticklabels([c.replace("_high", "") for c in common], rotation=35, ha="right", fontsize=7)
        ax_diff.set_ylabel("distance [cm]")
        ax_diff.set_title(f"(b) Same object, two SLAMs: |{n0} − {n1}| after alignment", fontsize=10)
        if d:
            ax_diff.axhline(float(np.median(d)), color="#9aa7b4", ls="--", lw=0.8)
            ax_diff.annotate(f"median {np.median(d):.1f} cm", (0.02, np.median(d)),
                             xycoords=("axes fraction", "data"), fontsize=7, color="#9aa7b4", va="bottom")

    # ── (c) spread per lane ─────────────────────────────────────────────────
    names = sorted({k for sp in spreads.values() for k in sp})
    width = 0.8 / max(len(lanes), 1)
    for j, l in enumerate(lanes):
        vals = [float(np.median(spreads[l["name"]][k])) if k in spreads[l["name"]] else 0.0 for k in names]
        ax_spread.bar(np.arange(len(names)) + j * width, vals, width=width, color=colors[l["name"]],
                      alpha=0.85, label=l["label"])
    ax_spread.set_xticks(np.arange(len(names)) + width * (len(lanes) - 1) / 2)
    ax_spread.set_xticklabels([n.replace("_high", "") for n in names], rotation=35, ha="right", fontsize=7)
    ax_spread.set_ylabel("observation spread, median [cm]")
    ax_spread.set_title("(c) Raw observation spread per object (before memory fusion)", fontsize=10)
    ax_spread.legend(fontsize=7)

    # ── (d) facts table ─────────────────────────────────────────────────────
    rows = [["lane", "poses ok/all", "arrival lag", "track", "kf / upd", "max corr", "memory"]]
    for f in facts:
        rows.append([f["lane"], f["poses"], f"{f['lag_s']} s" if f["lag_s"] is not None else "–",
                     f"{f['track_ms']} ms" if f["track_ms"] is not None else "–",
                     f["kf"], f"{f['maxcorr_cm']} cm",
                     ", ".join(f"{k} {v}" for k, v in (f["memory"] or {}).items())])
    t = ax_tbl.table(cellText=rows[1:], colLabels=rows[0], loc="center", cellLoc="left",
                     colWidths=[0.26, 0.15, 0.12, 0.09, 0.10, 0.10, 0.28])
    t.auto_set_font_size(False); t.set_fontsize(6.5); t.scale(1, 2.0)
    ax_tbl.set_title(f"(d) Runtime facts  ·  SAM-6D {s.get('sam6d_frames')} frames shared by both lanes",
                     fontsize=10)

    fig.suptitle(f"{a.run.name}  ·  same input, same detections, two SLAM backends", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = a.out / f"{a.run.name}_lane_compare.png"
    fig.savefig(out)
    plt.close(fig)

    stats = {"run": a.run.name, "reference_lane": ref_name, "sam6d_frames": s.get("sam6d_frames"),
             "lanes": facts,
             "object_distance_cm": ({k: round(float(np.linalg.norm(obj_world[lanes[0]['name']][k][0]
                                                                   - obj_world[lanes[1]['name']][k][0]) * 100), 1)
                                     for k in sorted(set(obj_world[lanes[0]['name']]) & set(obj_world[lanes[1]['name']]))}
                                    if len(lanes) >= 2 else {}),
             "spread_median_cm": {ln: {k: round(float(np.median(v)), 2) for k, v in sp.items()}
                                  for ln, sp in spreads.items()}}
    (a.out / f"{a.run.name}_lane_compare.json").write_text(json.dumps(stats, indent=1, ensure_ascii=False))
    print(f"[fig ] {out}")
    print(json.dumps(stats["object_distance_cm"], ensure_ascii=False))


if __name__ == "__main__":
    main()
