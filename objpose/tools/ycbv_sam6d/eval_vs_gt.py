#!/usr/bin/env python3
"""eval_vs_gt.py — SAM-6D 객체 포즈를 YCB-V GT 객체 포즈와 직접 비교한다 (모든 프레임 / 실시간 각각).

비교 대상 (셋 다 기준은 GT):
  all        모든 프레임 입력 — 각 프레임의 추정 vs 그 프레임의 GT
  rt         실시간 입력 — 결과가 나온 프레임의 추정 vs 그 프레임의 GT
  rt_screen  실시간 입력 — 프레임 k 시각에 이미 나와 있던 가장 최근 결과를 ORB-SLAM3 카메라 이동으로
             현재 프레임에 옮긴 것 vs 프레임 k 의 GT (화면에 실제로 보이는 것)

오차 (카메라 좌표계, 추정과 GT 가 같은 객체·같은 프레임일 때):
  위치 오차   |t_est - t_gt| [mm]  — 객체 모델 원점(YCB-V 모델은 대략 중심)의 3D 거리
  축별 오차   t_est - t_gt 의 x·y·z [mm] (z = 카메라에서 멀어지는 방향, 깊이)
  회전 오차   R_est 와 R_gt 사이 각도 [°]
  ADD-S       [mm]
대칭 물체는 BOP models_info.json 의 대칭 변환 S 중 T_gt·S 가 추정에 가장 가까운(ADD 최소) 것을 GT 로 쓴다
(연속 대칭은 5° 간격 72개로 샘플). 대칭이 적혀 있지 않은 객체는 S = I 하나뿐이다.

    python3 objpose/tools/ycbv_sam6d/eval_vs_gt.py \
        --all objpose/output/ycbv_sam6d_all_260921 --rt objpose/output/ycbv_sam6d_realtime_260921 \
        --slam objpose/output/ycbv_slam_orb_260921 --figs objpose/docs/figs_ycbv_sam6d_gt
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compare_modes import (FPS, PTS_DIR, TABLE, YCBV, load_frames, load_scene_gt, load_slam, se3)  # noqa: E402

C_GT = "#8a8a8a"
C_ALL = "#1f6fb2"
C_RT = "#e08a1e"
C_SCR = "#b8342a"
LABEL = {"all": "모든 프레임 입력", "rt": "실시간 입력 (결과가 나온 프레임)", "rt_screen": "실시간 · 화면 기준 (SLAM 보정)"}
COLOR = {"all": C_ALL, "rt": C_RT, "rt_screen": C_SCR}
plt.rcParams.update({
    "font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "axes.grid": True, "grid.alpha": 0.25,
    "font.size": 12, "axes.spines.top": False, "axes.spines.right": False,
})


def rot_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def sym_list(info):
    S = [np.eye(4)]
    for m in info.get("symmetries_discrete", []):
        S.append(np.asarray(m, float).reshape(4, 4))
    for c in info.get("symmetries_continuous", []):
        ax = np.asarray(c["axis"], float); ax /= np.linalg.norm(ax)
        off = np.asarray(c["offset"], float)
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        for a in np.radians(np.arange(5, 360, 5)):
            R = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
            T = np.eye(4); T[:3, :3] = R; T[:3, 3] = off - R @ off
            S.append(T)
    return S


class Obj:
    def __init__(self, name, info):
        from scipy.spatial import cKDTree
        p = np.load(PTS_DIR / f"{name}.npy").astype(np.float64)
        if len(p) > 1000:
            p = p[np.random.default_rng(0).choice(len(p), 1000, replace=False)]
        self.p, self.tree = p, cKDTree(p)
        self.thr = 0.1 * info["diameter"]
        self.S = sym_list(info)

    def err(self, T_est, T_gt):
        pe = self.p @ T_est[:3, :3].T + T_est[:3, 3]
        best = None
        for S in self.S:
            G = T_gt @ S
            pg = self.p @ G[:3, :3].T + G[:3, 3]
            add = np.linalg.norm(pe - pg, axis=1).mean()
            if best is None or add < best[0]:
                best = (add, G)
        G = best[1]
        D = np.linalg.inv(T_gt) @ T_est
        q = self.p @ D[:3, :3].T + D[:3, 3]
        adds = float(self.tree.query(q)[0].mean())
        dt = T_est[:3, 3] - G[:3, 3]
        return {"t_err": float(np.linalg.norm(dt)), "dx": float(dt[0]), "dy": float(dt[1]), "dz": float(dt[2]),
                "r_err": rot_deg(T_est[:3, :3], G[:3, :3]), "adds": adds, "add_sym": float(best[0]),
                "ok": adds < self.thr}


def eval_all(a, objs, by_id, scenes):
    rows, gt_count, series = [], {}, {}
    for s in scenes:
        sc = YCBV / "test" / s
        n, gt = load_scene_gt(sc, by_id)
        gt_count[s] = {"n_frames": n, "inst": len(gt), "by_obj": {}}
        for (_, nm) in gt:
            gt_count[s]["by_obj"][nm] = gt_count[s]["by_obj"].get(nm, 0) + 1
        slam_T = load_slam(a.slam / f"{s}_frames.txt", n)
        for mode, run in (("all", a.all / s), ("rt", a.rt / s)):
            frames = load_frames(run)
            for f in frames:
                i = f["idx"]
                for d in f["dets"]:
                    Tg, o = gt.get((i, d["object"])), objs.get(d["object"])
                    if Tg is None or o is None:
                        rows.append({"scene": s, "mode": mode, "k": i, "obj": d["object"], "matched": False})
                        continue
                    T = se3(d["R"], d["t_mm"])
                    rows.append({"scene": s, "mode": mode, "k": i, "src": i, "obj": d["object"], "matched": True,
                                 "gt_t": Tg[:3, 3].tolist(), "est_t": T[:3, 3].tolist(), **o.err(T, Tg)})
            if mode == "rt":
                done = sorted(frames, key=lambda f: f["done_s"])
                j = -1
                for k in range(n):
                    while j + 1 < len(done) and done[j + 1]["done_s"] <= k / FPS:
                        j += 1
                    if j < 0:
                        continue
                    src = done[j]
                    Ts, Tk = slam_T[src["idx"]], slam_T[k]
                    if Ts is None or Tk is None:
                        continue
                    C = np.linalg.inv(Tk) @ Ts
                    for d in src["dets"]:
                        Tg, o = gt.get((k, d["object"])), objs.get(d["object"])
                        if Tg is None or o is None:
                            continue
                        T = C @ se3(d["R"], d["t_mm"])
                        rows.append({"scene": s, "mode": "rt_screen", "k": k, "src": src["idx"], "obj": d["object"],
                                     "matched": True, "gt_t": Tg[:3, 3].tolist(), "est_t": T[:3, 3].tolist(),
                                     **o.err(T, Tg)})
        print(s, "scored", flush=True)
    return rows, gt_count


def stats(v):
    v = np.asarray(v, float)
    if not len(v):
        return {"n": 0}
    return {"n": int(len(v)), "median": float(np.median(v)), "mean": float(v.mean()),
            "p90": float(np.percentile(v, 90)), "rmse": float(np.sqrt((v ** 2).mean()))}


def summarize(rows, gt_count, filt=lambda r: True):
    out = {}
    inst = sum(g["inst"] for g in gt_count.values())
    for mode in ("all", "rt", "rt_screen"):
        m = [r for r in rows if r["mode"] == mode and r["matched"] and filt(r)]
        cov = {(r["scene"], r["k"], r["obj"]) for r in m}
        out[mode] = {"matched": len(m), "covered_inst": len(cov),
                     "hit_inst": len({(r["scene"], r["k"], r["obj"]) for r in m if r["ok"]}),
                     "t_err": stats([r["t_err"] for r in m]), "r_err": stats([r["r_err"] for r in m]),
                     "adds": stats([r["adds"] for r in m]),
                     "dz": stats([abs(r["dz"]) for r in m]), "dxy": stats([np.hypot(r["dx"], r["dy"]) for r in m]),
                     "pass": float(np.mean([r["ok"] for r in m])) if m else None,
                     "unmatched": sum(1 for r in rows if r["mode"] == mode and not r["matched"] and filt(r))}
    out["inst"] = inst
    return out


def f1(x, d=1):
    return "—" if x is None else f"{x:.{d}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--all", required=True, type=Path)
    ap.add_argument("--rt", required=True, type=Path)
    ap.add_argument("--slam", required=True, type=Path)
    ap.add_argument("--figs", required=True, type=Path)
    a = ap.parse_args()
    a.figs.mkdir(parents=True, exist_ok=True)
    table = json.loads(TABLE.read_text())["objects"]
    info = json.loads((YCBV / "models" / "models_info.json").read_text())
    by_id = {o["obj_id"]: o["name"] for o in table}
    objs = {o["name"]: Obj(o["name"], info[str(o["obj_id"])]) for o in table if (PTS_DIR / f"{o['name']}.npy").exists()}
    scenes = sorted(p.name for p in (YCBV / "test").iterdir()
                    if (a.all / p.name / "run.json").exists() and (a.rt / p.name / "run.json").exists())

    rows, gt_count = eval_all(a, objs, by_id, scenes)
    with open(a.figs / "errors_vs_gt.csv", "w", newline="") as fh:
        keys = ["scene", "mode", "k", "src", "obj", "matched", "t_err", "dx", "dy", "dz", "r_err", "adds", "ok"]
        w = csv.DictWriter(fh, keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    modes = ("all", "rt", "rt_screen")
    S = summarize(rows, gt_count)
    L = []
    L.append("### 전체 (12장면, GT 인스턴스 %d개)\n" % S["inst"])
    L.append("| GT 와 비교한 값 | " + " | ".join(LABEL[m] for m in modes) + " |")
    L.append("|---|---|---|---|")
    L.append("| 비교된 추정 수 | " + " | ".join(f"{S[m]['matched']:,}" for m in modes) + " |")
    L.append("| 추정을 받은 GT 인스턴스 | " + " | ".join(f"{S[m]['covered_inst']:,} ({S[m]['covered_inst'] / S['inst'] * 100:.1f}%)" for m in modes) + " |")
    for key, nm, u in (("t_err", "위치 오차", "mm"), ("dz", "  └ 깊이(z) 오차", "mm"), ("dxy", "  └ 화면 방향(xy) 오차", "mm"),
                       ("r_err", "회전 오차", "°"), ("adds", "ADD-S", "mm")):
        L.append(f"| {nm} 중앙값 [{u}] | " + " | ".join(f1(S[m][key].get('median')) for m in modes) + " |")
        if key in ("t_err", "r_err"):
            L.append(f"| {nm} 평균 [{u}] | " + " | ".join(f1(S[m][key].get('mean')) for m in modes) + " |")
            L.append(f"| {nm} 90% 지점 [{u}] | " + " | ".join(f1(S[m][key].get('p90')) for m in modes) + " |")
    L.append("| ADD-S < 0.1d 통과 (비교된 추정 중) | " + " | ".join(f"{S[m]['pass'] * 100:.1f}%" for m in modes) + " |")
    L.append("| GT 인스턴스 중 통과 (재현율) | " + " | ".join(f"{S[m]['hit_inst'] / S['inst'] * 100:.1f}%" for m in modes) + " |")
    L.append("| GT 에 없는 객체를 답한 추정 | " + " | ".join(f"{S[m]['unmatched']:,}" if m != 'rt_screen' else '—' for m in modes) + " |")

    L.append("\n### 장면별 — GT 대비 위치 오차 / 회전 오차 중앙값, GT 인스턴스 중 통과 비율\n")
    L.append("| 장면 | " + " | ".join(f"{LABEL[m]} 위치 [mm]" for m in modes) + " | " +
             " | ".join(f"{LABEL[m]} 회전 [°]" for m in modes) + " | " + " | ".join(f"{LABEL[m]} 재현율" for m in modes) + " |")
    L.append("|" + "---|" * 10)
    per_scene = {}
    for s in scenes:
        Ss = summarize(rows, {s: gt_count[s]}, lambda r, s=s: r["scene"] == s)
        per_scene[s] = Ss
        L.append(f"| {s} | " + " | ".join(f1(Ss[m]['t_err'].get('median')) for m in modes) + " | " +
                 " | ".join(f1(Ss[m]['r_err'].get('median'), 2) for m in modes) + " | " +
                 " | ".join(f"{Ss[m]['hit_inst'] / Ss['inst'] * 100:.1f}%" for m in modes) + " |")

    L.append("\n### 객체별 — GT 대비 위치 오차 / 회전 오차 중앙값 (12장면 합)\n")
    L.append("| 객체 | 대칭 | GT 인스턴스 | 추정 수 모든/실시간 | 위치 모든 [mm] | 위치 실시간 [mm] | 회전 모든 [°] | 회전 실시간 [°] |")
    L.append("|" + "---|" * 8)
    sym_txt = {o["name"]: ("연속" if info[str(o["obj_id"])].get("symmetries_continuous") else
                            ("이산" if info[str(o["obj_id"])].get("symmetries_discrete") else "")) for o in table}
    per_obj = {}
    for nm in sorted({n for g in gt_count.values() for n in g["by_obj"]}):
        g = sum(gc["by_obj"].get(nm, 0) for gc in gt_count.values())
        So = summarize(rows, gt_count, lambda r, nm=nm: r["obj"] == nm)
        per_obj[nm] = So
        L.append(f"| {nm.replace('ycbv_', '')} | {sym_txt.get(nm, '')} | {g} | {So['all']['matched']}/{So['rt']['matched']} | "
                 f"{f1(So['all']['t_err'].get('median'))} | {f1(So['rt']['t_err'].get('median'))} | "
                 f"{f1(So['all']['r_err'].get('median'), 2)} | {f1(So['rt']['r_err'].get('median'), 2)} |")
    (a.figs / "tables_vs_gt.md").write_text("\n".join(L) + "\n")
    (a.figs / "summary_vs_gt.json").write_text(json.dumps({"overall": S, "per_scene": per_scene}, indent=1))
    print("\n".join(L))

    # ── 그림 1: GT 대비 오차 누적분포 ──
    fig, axs = plt.subplots(1, 3, figsize=(17, 5))
    for ax, key, xl, lim in ((axs[0], "t_err", "GT 대비 위치 오차 [mm]", 40), (axs[1], "r_err", "GT 대비 회전 오차 [°]", 20),
                             (axs[2], "adds", "ADD-S [mm]", 30)):
        for m in modes:
            v = np.sort([r[key] for r in rows if r["mode"] == m and r["matched"]])
            ax.plot(v, np.arange(1, len(v) + 1) / len(v) * 100, color=COLOR[m], lw=2, label=LABEL[m])
            med = np.median(v)
            ax.plot([med], [50], "o", color=COLOR[m], ms=6)
        ax.axhline(50, color=C_GT, lw=0.8, ls=":")
        ax.set_xlim(0, lim); ax.set_ylim(0, 100)
        ax.set_xlabel(xl); ax.set_ylabel("이 값 이하인 추정의 비율 [%]")
    axs[0].legend(fontsize=11, loc="lower right")
    fig.suptitle("GT 객체 포즈와 비교한 오차의 누적분포 — 12장면 전체 (점 = 중앙값)", fontsize=15)
    fig.tight_layout(); fig.savefig(a.figs / "cdf_vs_gt.png", dpi=150); plt.close(fig)

    # ── 그림 2: 장면별 위치·회전 오차 중앙값 ──
    fig, axs = plt.subplots(1, 2, figsize=(17, 5))
    x = np.arange(len(scenes)); w = 0.27
    for ax, key, yl in ((axs[0], "t_err", "GT 대비 위치 오차 중앙값 [mm]"), (axs[1], "r_err", "GT 대비 회전 오차 중앙값 [°]")):
        for j, m in enumerate(modes):
            ax.bar(x + (j - 1) * w, [per_scene[s][m][key].get("median", np.nan) for s in scenes], w - 0.03,
                   color=COLOR[m], label=LABEL[m])
        ax.set_xticks(x, scenes, fontsize=10, rotation=45)
        ax.set_ylabel(yl)
    axs[0].legend(fontsize=10)
    fig.suptitle("장면별 GT 대비 오차 (중앙값)", fontsize=15)
    fig.tight_layout(); fig.savefig(a.figs / "scene_median_vs_gt.png", dpi=150); plt.close(fig)

    # ── 그림 3: GT 궤적 위에 추정 — 한 장면·한 객체의 카메라 좌표 x·y·z 와 회전 오차 ──
    s, nm = "000052", "ycbv_mustard_bottle"
    n, gt = load_scene_gt(YCBV / "test" / s, by_id)
    gk = sorted(k for (k, o) in gt if o == nm)
    G = np.array([gt[(k, nm)][:3, 3] for k in gk])
    tg = np.array(gk) / FPS
    fig, axs = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    for j, ax_name in enumerate(("x (화면 오른쪽)", "y (화면 아래)", "z (깊이)")):
        ax = axs[j]
        ax.plot(tg, G[:, j], color=C_GT, lw=4, alpha=0.6, label="GT", solid_capstyle="round")
        for m, ms, zo in (("all", 3, 3), ("rt_screen", 0, 2), ("rt", 7, 4)):
            rr = [r for r in rows if r["mode"] == m and r["matched"] and r["scene"] == s and r["obj"] == nm]
            rr.sort(key=lambda r: r["k"])
            t = np.array([r["k"] for r in rr]) / FPS
            v = np.array([r["est_t"][j] for r in rr])
            if m == "rt_screen":
                ax.plot(t, v, color=COLOR[m], lw=1.2, label=LABEL[m], zorder=zo)
            else:
                ax.plot(t, v, "o", color=COLOR[m], ms=ms, lw=0, label=LABEL[m], zorder=zo,
                        alpha=0.6 if m == "all" else 1)
        ax.set_ylabel(f"{ax_name} [mm]")
    axs[0].legend(fontsize=11, ncol=4, loc="upper left", bbox_to_anchor=(0, 1.32))
    ax = axs[3]
    for m in ("all", "rt_screen", "rt"):
        rr = sorted([r for r in rows if r["mode"] == m and r["matched"] and r["scene"] == s and r["obj"] == nm],
                    key=lambda r: r["k"])
        t = np.array([r["k"] for r in rr]) / FPS
        v = np.array([r["r_err"] for r in rr])
        if m == "rt_screen":
            ax.plot(t, v, color=COLOR[m], lw=1.2)
        else:
            ax.plot(t, v, "o", color=COLOR[m], ms=3 if m == "all" else 7, alpha=0.6 if m == "all" else 1)
    ax.set_ylabel("GT 대비 회전 오차 [°]"); ax.set_xlabel("시간 [s]"); ax.set_ylim(0, None)
    ax.set_xlim(0, n / FPS)
    fig.suptitle(f"장면 {s} · {nm.replace('ycbv_', '')} — 카메라 좌표계에서 본 객체 위치: GT(회색) 위에 추정", fontsize=15, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.97)); fig.savefig(a.figs / "track_vs_gt_000052_mustard.png", dpi=150); plt.close(fig)

    # ── 그림 4: 12장면 — 프레임별 GT 대비 위치 오차 시간축 ──
    fig, axs = plt.subplots(4, 3, figsize=(15, 13), sharey=True)
    for ax, s in zip(axs.flat, scenes):
        for m, sz, al in (("all", 2, 0.35), ("rt", 12, 1.0)):
            rr = [r for r in rows if r["mode"] == m and r["matched"] and r["scene"] == s]
            ax.scatter([r["k"] / FPS for r in rr], [max(r["t_err"], 0.3) for r in rr], s=sz, color=COLOR[m],
                       alpha=al, lw=0, label=LABEL[m])
        ax.set_yscale("log"); ax.set_ylim(0.3, 500)
        ax.set_title(f"장면 {s}", fontsize=13); ax.set_xlim(0, gt_count[s]["n_frames"] / FPS)
    for ax in axs[-1]:
        ax.set_xlabel("시간 [s]")
    for ax in axs[:, 0]:
        ax.set_ylabel("GT 대비 위치 오차 [mm]")
    h, l = axs.flat[0].get_legend_handles_labels()
    for hh in h:
        hh.set_sizes([40]); hh.set_alpha(1)
    fig.legend(h, l, loc="upper center", ncol=2, fontsize=13, frameon=False, bbox_to_anchor=(0.5, 0.97))
    fig.suptitle("프레임별 GT 대비 객체 위치 오차 (점 하나 = 추정 하나, 로그 축)", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(a.figs / "terr_time_vs_gt.png", dpi=150); plt.close(fig)


if __name__ == "__main__":
    main()
