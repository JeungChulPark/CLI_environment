#!/usr/bin/env python3
"""eval_ycbv_slam.py — ORB-SLAM3 RGB-D 추정 카메라 위치 vs YCB-Video GT 카메라 위치.

입력: ycbv_rgbd 가 장면마다 쓴 <scene>_frames.txt (추적 직후의 "현재 위치", camera->world TUM)
      와 <scene>_final.txt (종료 후 BA 반영본), BOP scene_camera.json (world->camera, mm).
SLAM 월드는 첫 카메라 좌표계라 GT 월드와 강체 변환만큼 다르다. 두 가지로 맞춘다.
  * SE3 정렬 — 전체 궤적에 Umeyama(스케일 없음, RGB-D). TUM 표준 ATE.
  * 첫 프레임 고정 — 첫 프레임에서만 맞추고 이후는 그대로. 실시간 누적 드리프트를 본다.

    python3 objpose/tools/ycbv_slam/eval_ycbv_slam.py \
        --run objpose/output/ycbv_slam_orb_260921 \
        --data /home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video/test \
        --figs objpose/docs/figs_ycbv_slam
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "slam_comparison_report" / "loc_eval"))
from metrics import quat_to_rot, rot_geodesic_deg, umeyama  # noqa: E402

FPS = 30.0
GT_C = "#8a8a8a"      # 정답 — 회색
EST_C = "#1f6fb2"     # SLAM — 파랑 (기존 그림의 CAM 색)
ACC = "#b8342a"       # 강조(최댓값 표시)
plt.rcParams.update({
    "font.family": "Noto Sans CJK JP", "axes.unicode_minus": False,
    "axes.grid": True, "grid.alpha": 0.25, "font.size": 12,
    "axes.spines.top": False, "axes.spines.right": False,
})


def load_gt(scene: Path):
    d = json.load(open(scene / "scene_camera.json"))
    ids = sorted(map(int, d))
    T = np.zeros((len(ids), 4, 4))
    for k, i in enumerate(ids):
        R = np.array(d[str(i)]["cam_R_w2c"]).reshape(3, 3)
        t = np.array(d[str(i)]["cam_t_w2c"]) / 1000.0
        T[k, :3, :3] = R.T
        T[k, :3, 3] = -R.T @ t
        T[k, 3, 3] = 1
    return np.array(ids), T                      # camera->world, m


def load_tum(p: Path):
    a = np.loadtxt(p, ndmin=2)
    idx = np.rint(a[:, 0] * FPS).astype(int) + 1  # 프레임 번호 (1부터)
    T = np.tile(np.eye(4), (len(a), 1, 1))
    T[:, :3, :3] = quat_to_rot(a[:, 4:8])
    T[:, :3, 3] = a[:, 1:4]
    return idx, T


def errors(Tg, Te, R, t):
    """Te 를 (R,t) 로 GT 월드에 옮긴 뒤 프레임별 위치 오차(m)·자세 오차(deg)."""
    pe = (R @ Te[:, :3, 3].T).T + t
    Re = np.matmul(R[None], Te[:, :3, :3])
    return pe, np.linalg.norm(pe - Tg[:, :3, 3], axis=1), rot_geodesic_deg(Tg[:, :3, :3], Re)


def rpe(Tg, Te, delta):
    """delta 프레임 간 상대 이동의 차이 — 위치(m), 자세(deg)."""
    inv = np.linalg.inv
    dt, dr = [], []
    for i in range(len(Tg) - delta):
        Eg = inv(Tg[i]) @ Tg[i + delta]
        Ee = inv(Te[i]) @ Te[i + delta]
        E = inv(Eg) @ Ee
        dt.append(np.linalg.norm(E[:3, 3]))
        dr.append(rot_geodesic_deg(E[None, :3, :3], np.eye(3)[None])[0])
    return np.array(dt), np.array(dr)


def rmse(a):
    return float(np.sqrt(np.mean(a ** 2)))


def eval_scene(run: Path, scene: Path):
    name = scene.name
    gid, Tg_all = load_gt(scene)
    out = {"scene": name, "n_frames": int(len(gid)), "duration_s": len(gid) / FPS}
    pos = Tg_all[:, :3, 3]
    out["gt_path_m"] = float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum())
    timing = np.loadtxt(run / f"{name}_timing.txt", ndmin=2)
    out["track_ms_median"] = float(np.median(timing[:, 1]))
    series = {}
    for kind in ("frames", "final"):
        eid, Te = load_tum(run / f"{name}_{kind}.txt")
        m = np.isin(gid, eid)
        Tg = Tg_all[m]
        Te = Te[np.searchsorted(eid, gid[m])]
        # SE3 정렬 (전체 궤적)
        _, R, t = umeyama(Te[:, :3, 3], Tg[:, :3, 3], with_scale=False)
        pe, et, er = errors(Tg, Te, R, t)
        # 첫 프레임 고정
        A = Tg[0] @ np.linalg.inv(Te[0])
        pa, ea, ra = errors(Tg, Te, A[:3, :3], A[:3, 3])
        dt, dr = rpe(Tg, Te, int(FPS))
        out[kind] = {
            "tracked": int(m.sum()),
            "ate_rmse_mm": rmse(et) * 1e3, "ate_mean_mm": float(et.mean() * 1e3),
            "ate_max_mm": float(et.max() * 1e3), "rot_mean_deg": float(er.mean()),
            "rot_max_deg": float(er.max()),
            "anchor_rmse_mm": rmse(ea) * 1e3, "anchor_end_mm": float(ea[-1] * 1e3),
            "anchor_max_mm": float(ea.max() * 1e3), "anchor_rot_end_deg": float(ra[-1]),
            "rpe1s_trans_mm": rmse(dt) * 1e3, "rpe1s_rot_deg": rmse(dr),
        }
        series[kind] = dict(t=(gid[m] - 1) / FPS, gt=Tg[:, :3, 3], est=pe, et=et, er=er,
                            ea=ea, anc=pa)
    return out, series


def top_plane(xyz):
    """GT 궤적이 가장 넓게 퍼진 평면(PCA 1·2축)으로 투영 — 탁상 주위를 도는 궤적의 위에서 본 모습."""
    c = xyz.mean(0)
    _, _, Vt = np.linalg.svd(xyz - c)
    return c, Vt[:2]


def fig_error_time(S, rows, path):
    fig, axs = plt.subplots(4, 3, figsize=(15, 13), sharey=True)
    ymax = max(s["frames"]["et"].max() for s in S.values()) * 1e3 * 1.08
    for ax, r in zip(axs.flat, rows):
        s = S[r["scene"]]["frames"]
        ax.plot(s["t"], s["et"] * 1e3, color=EST_C, lw=1.6)
        ax.axhline(r["frames"]["ate_rmse_mm"], color=GT_C, lw=1, ls="--")
        ax.text(0.98, 0.95, "RMSE %.1f mm" % r["frames"]["ate_rmse_mm"], transform=ax.transAxes,
                ha="right", va="top", fontsize=11, color="#333")
        ax.set_title("장면 %s  (%.0f s)" % (r["scene"], r["duration_s"]), fontsize=13)
        ax.set_ylim(0, ymax)
        ax.set_xlim(0, s["t"][-1])
    for ax in axs[-1]:
        ax.set_xlabel("시간 [s]")
    for ax in axs[:, 0]:
        ax.set_ylabel("위치 오차 [mm]")
    fig.suptitle("ORB-SLAM3 RGB-D 추정 위치와 정답 위치의 거리 — 프레임별 (SE3 정렬, 점선 = RMSE)",
                 fontsize=15)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_rot_time(S, rows, path):
    fig, axs = plt.subplots(4, 3, figsize=(15, 13), sharey=True)
    ymax = max(s["frames"]["er"].max() for s in S.values()) * 1.08
    for ax, r in zip(axs.flat, rows):
        s = S[r["scene"]]["frames"]
        ax.plot(s["t"], s["er"], color=EST_C, lw=1.6)
        ax.text(0.98, 0.95, "평균 %.2f°" % r["frames"]["rot_mean_deg"], transform=ax.transAxes,
                ha="right", va="top", fontsize=11, color="#333")
        ax.set_title("장면 %s" % r["scene"], fontsize=13)
        ax.set_ylim(0, ymax)
        ax.set_xlim(0, s["t"][-1])
    for ax in axs[-1]:
        ax.set_xlabel("시간 [s]")
    for ax in axs[:, 0]:
        ax.set_ylabel("자세 오차 [°]")
    fig.suptitle("ORB-SLAM3 RGB-D 추정 자세와 정답 자세의 각도 차 — 프레임별 (SE3 정렬)", fontsize=15)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_drift(S, rows, path):
    """첫 프레임에서만 맞췄을 때 오차가 시간에 따라 어떻게 쌓이는지 — 12장면을 한 축에."""
    fig, ax = plt.subplots(figsize=(11, 5.2))
    worst = max(rows, key=lambda r: r["frames"]["anchor_max_mm"])["scene"]
    for r in rows:
        s = S[r["scene"]]["frames"]
        hot = r["scene"] == worst
        ax.plot(s["t"], s["ea"] * 1e3, color=ACC if hot else EST_C, lw=2 if hot else 1.1,
                alpha=1 if hot else 0.45, zorder=3 if hot else 2)
        if hot:
            i = int(np.argmax(s["ea"]))
            ax.annotate("장면 %s  최대 %.0f mm" % (worst, s["ea"][i] * 1e3), (s["t"][i], s["ea"][i] * 1e3),
                        xytext=(10, 6), textcoords="offset points", fontsize=12, color="#333")
    ax.set_xlabel("시간 [s]")
    ax.set_ylabel("위치 오차 [mm]")
    ax.set_title("첫 프레임만 맞추고 그대로 둔 경우의 오차 — 12장면 (실시간 누적 드리프트)", fontsize=14)
    ax.set_xlim(0, None)
    ax.set_ylim(0, None)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_paths(S, rows, path):
    fig, axs = plt.subplots(3, 4, figsize=(18, 13.5))
    for ax, r in zip(axs.flat, rows):
        s = S[r["scene"]]["frames"]
        c, B = top_plane(s["gt"])
        g = (s["gt"] - c) @ B.T * 100
        e = (s["est"] - c) @ B.T * 100
        ax.plot(g[:, 0], g[:, 1], color=GT_C, lw=3.2, label="정답 (GT)", solid_capstyle="round")
        ax.plot(e[:, 0], e[:, 1], color=EST_C, lw=1.4, label="SLAM 추정")
        ax.plot(*g[0], "o", ms=8, color="#333", zorder=5)
        ax.plot(*g[-1], "s", ms=8, color="#333", zorder=5)
        ax.set_aspect("equal", "datalim")
        ax.set_title("장면 %s — RMSE %.1f mm" % (r["scene"], r["frames"]["ate_rmse_mm"]), fontsize=13)
        ax.set_xlabel("[cm]", fontsize=11)
        ax.tick_params(labelsize=10)
    h, l = axs.flat[0].get_legend_handles_labels()
    from matplotlib.lines import Line2D
    h += [Line2D([], [], marker="o", ls="", color="#333", ms=8), Line2D([], [], marker="s", ls="", color="#333", ms=8)]
    l += ["시작", "끝"]
    fig.legend(h, l, loc="upper center", ncol=4, fontsize=13, frameon=False, bbox_to_anchor=(0.5, 0.965))
    fig.suptitle("YCB-Video 12장면 카메라 경로 — 위에서 본 모습 (궤적이 가장 넓게 퍼진 평면)", fontsize=16)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_path3d(S, r, path):
    s = S[r["scene"]]["frames"]
    fig = plt.figure(figsize=(14, 6.4))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    g, e = s["gt"] * 100, s["est"] * 100
    ax.plot(*g.T, color=GT_C, lw=3, label="정답 (GT)")
    ax.plot(*e.T, color=EST_C, lw=1.3, label="SLAM 추정")
    ax.scatter(*g[0], color="#333", s=40, marker="o")
    ax.set_xlabel("x [cm]"); ax.set_ylabel("y [cm]"); ax.set_zlabel("z [cm]")
    ax.set_title("3차원 경로", fontsize=13)
    ax.legend(loc="upper left", fontsize=11)
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.plot(s["t"], (e - g)[:, 0] * 10, lw=1.3, color=EST_C, label="x")
    ax2.plot(s["t"], (e - g)[:, 1] * 10, lw=1.3, color="#e08a1e", label="y")
    ax2.plot(s["t"], (e - g)[:, 2] * 10, lw=1.3, color="#3a9d5d", label="z")
    ax2.axhline(0, color=GT_C, lw=1)
    ax2.set_xlabel("시간 [s]"); ax2.set_ylabel("축별 오차 [mm]")
    ax2.set_title("축별 위치 오차 (추정 − 정답)", fontsize=13)
    ax2.legend(fontsize=11, ncol=3)
    fig.suptitle("장면 %s — 오차가 가장 큰 장면 (SE3 정렬, RMSE %.1f mm)" % (r["scene"], r["frames"]["ate_rmse_mm"]),
                 fontsize=15)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, type=Path)
    ap.add_argument("--data", required=True, type=Path)
    ap.add_argument("--figs", required=True, type=Path)
    a = ap.parse_args()
    a.figs.mkdir(parents=True, exist_ok=True)
    rows, S = [], {}
    for scene in sorted(p for p in a.data.iterdir() if (p / "scene_camera.json").exists()):
        if not (a.run / f"{scene.name}_frames.txt").exists():
            continue
        r, s = eval_scene(a.run, scene)
        rows.append(r); S[scene.name] = s
    json.dump(rows, open(a.run / "eval_slam.json", "w"), indent=1)

    fr = [r["frames"] for r in rows]
    n = sum(r["n_frames"] for r in rows)
    print("| 장면 | 프레임 | 길이 s | GT 경로 m | 추적 | ATE RMSE mm | ATE 평균 | ATE 최대 | 자세 평균 ° | RPE 1s mm | RPE 1s ° | 첫프레임고정 끝 mm | BA후 ATE mm |")
    for r in rows:
        f, g = r["frames"], r["final"]
        print("| %s | %d | %.1f | %.2f | %.0f%% | %.1f | %.1f | %.1f | %.2f | %.1f | %.2f | %.1f | %.1f |" % (
            r["scene"], r["n_frames"], r["duration_s"], r["gt_path_m"], 100 * f["tracked"] / r["n_frames"],
            f["ate_rmse_mm"], f["ate_mean_mm"], f["ate_max_mm"], f["rot_mean_deg"], f["rpe1s_trans_mm"],
            f["rpe1s_rot_deg"], f["anchor_end_mm"], g["ate_rmse_mm"]))
    all_e = np.concatenate([S[r["scene"]]["frames"]["et"] for r in rows]) * 1e3
    all_r = np.concatenate([S[r["scene"]]["frames"]["er"] for r in rows])
    print("TOTAL frames %d tracked %d  ATE rmse(all frames) %.1f mm mean %.1f median %.1f max %.1f | rot mean %.2f"
          " | scene-rmse range %.1f-%.1f | median track ms %.1f | gt path %.1f m" % (
              n, sum(f["tracked"] for f in fr), np.sqrt((all_e ** 2).mean()), all_e.mean(), np.median(all_e),
              all_e.max(), all_r.mean(), min(f["ate_rmse_mm"] for f in fr), max(f["ate_rmse_mm"] for f in fr),
              np.median([r["track_ms_median"] for r in rows]), sum(r["gt_path_m"] for r in rows)))

    fig_error_time(S, rows, a.figs / "ycbv_slam_err_time.png")
    fig_rot_time(S, rows, a.figs / "ycbv_slam_rot_time.png")
    fig_drift(S, rows, a.figs / "ycbv_slam_drift.png")
    fig_paths(S, rows, a.figs / "ycbv_slam_paths.png")
    fig_path3d(S, max(rows, key=lambda r: r["frames"]["ate_rmse_mm"]), a.figs / "ycbv_slam_worst3d.png")


if __name__ == "__main__":
    main()
