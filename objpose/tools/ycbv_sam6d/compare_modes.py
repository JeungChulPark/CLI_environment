#!/usr/bin/env python3
"""compare_modes.py — YCB-V 에서 SAM-6D '모든 프레임' 실행과 '실시간' 실행을 같은 기준으로 채점한다.

지표 (임계값은 YCB-Video 관례대로 0.1 x 객체 지름):
  ADD / ADD-S          추정이 나온 경우만 센 조건부 정확도
  처리 프레임 재현율   처리한 프레임에 보이는 GT 인스턴스 중 ADD-S 를 통과한 비율 (eval_bop_run.py 와 같은 정의)
  영상 전체 재현율     영상의 **모든** 프레임에 보이는 GT 인스턴스 기준. 실시간 모드는 결과가 없는 프레임을 놓친 것으로 센다
  화면 기준 재현율     실시간 모드에서 각 프레임 k 의 시각(k/30 s)에 **이미 나와 있는** 가장 최근 결과를 그 프레임의
                       답으로 쓴다(사용자가 실제로 보는 것). 두 가지로 잰다
                         hold  마지막 결과를 카메라 좌표 그대로 들고 있음
                         slam  ORB-SLAM3 카메라 궤적으로 결과 프레임 → 현재 프레임 이동을 보정

    python3 objpose/tools/ycbv_sam6d/compare_modes.py \
        --all objpose/output/ycbv_sam6d_all_260921 --rt objpose/output/ycbv_sam6d_realtime_260921 \
        --slam objpose/output/ycbv_slam_orb_260921 --figs objpose/docs/figs_ycbv_sam6d
"""
from __future__ import annotations

import argparse
import json, os
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[3]
PTS_DIR = REPO / "sam6d_realtime" / "assets" / "model_points"
TABLE = Path(os.environ.get("BENCH_TABLE", REPO / "objpose" / "rt" / "bop" / "ycbv_objects.json"))
YCBV = Path(os.environ.get("BENCH_ROOT", "/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video"))
FPS = float(os.environ.get("BENCH_FPS", 30.0))
VISIB_MIN = 0.1

ALL_C = "#1f6fb2"     # 모든 프레임
RT_C = "#e08a1e"      # 실시간
GREY = "#8a8a8a"
plt.rcParams.update({
    "font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "axes.grid": True, "grid.alpha": 0.25,
    "font.size": 12, "axes.spines.top": False, "axes.spines.right": False,
})


def se3(R, t):
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float).reshape(3, 3)
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


class Obj:
    """모델점 + KD-트리. ADD-S 는 T_gt^-1 T_est 로 옮긴 점에서 모델점까지의 최근접 거리 평균 (강체라 동일)."""
    def __init__(self, name, diam):
        p = np.load(PTS_DIR / f"{name}.npy").astype(np.float64)
        if len(p) > 2000:
            p = p[np.random.default_rng(0).choice(len(p), 2000, replace=False)]
        self.p, self.tree, self.thr = p, cKDTree(p), 0.1 * diam

    def errs(self, T_est, T_gt):
        D = np.linalg.inv(T_gt) @ T_est
        q = self.p @ D[:3, :3].T + D[:3, 3]
        return float(np.linalg.norm(q - self.p, axis=1).mean()), float(self.tree.query(q)[0].mean())


def load_scene_gt(scene: Path, by_id):
    gt = json.loads((scene / "scene_gt.json").read_text())
    info = json.loads((scene / "scene_gt_info.json").read_text())
    ids = sorted(gt, key=int)
    out = {}                                   # (frame_idx, name) -> T_c_m
    for i, fid in enumerate(ids):
        for j, r in enumerate(gt[fid]):
            if info[fid][j].get("visib_fract", 1.0) < VISIB_MIN or r["obj_id"] not in by_id:
                continue
            out[(i, by_id[r["obj_id"]])] = se3(r["cam_R_m2c"], r["cam_t_m2c"])
    return len(ids), out


def load_frames(run: Path):
    return [json.loads(l) for l in (run / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()]


def load_slam(path: Path, n):
    """ORB-SLAM3 frames.txt (camera->world, m) → 프레임별 T_wc (mm)."""
    T = [None] * n
    for row in np.loadtxt(path, ndmin=2):
        i = int(round(row[0] * FPS))
        if 0 <= i < n:
            x, y, z, w = row[4:8]
            R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            T[i] = se3(R, row[1:4] * 1000.0)
    return T


def score_scene(scene: Path, run: Path, objs, by_id, slam_T=None, realtime=False):
    n, gt = load_scene_gt(scene, by_id)
    frames = load_frames(run)
    proc = {f["idx"] for f in frames}
    gt_names = sorted({k[1] for k in gt})
    res = {"frames": n, "processed": len(proc), "est": 0, "scored": 0, "add_ok": 0, "adds_ok": 0,
           "inst_all": len(gt), "inst_proc": sum(1 for (i, _) in gt if i in proc), "hit": 0,
           "latency": [f.get("latency_s") for f in frames], "ms": [f["ms"]["total"] for f in frames],
           "per_obj": {}, "series": []}
    for f in frames:
        i = f["idx"]
        for d in f["dets"]:
            res["est"] += 1
            o = objs.get(d["object"])
            Tg = gt.get((i, d["object"]))
            po = res["per_obj"].setdefault(d["object"], {"est": 0, "scored": 0, "adds_ok": 0, "hit": 0})
            po["est"] += 1
            if Tg is None or o is None:
                continue
            a, s = o.errs(se3(d["R"], d["t_mm"]), Tg)
            res["scored"] += 1; po["scored"] += 1
            res["add_ok"] += a < o.thr
            res["adds_ok"] += s < o.thr; po["adds_ok"] += s < o.thr
            res["hit"] += s < o.thr; po["hit"] += s < o.thr
            res["series"].append((i / FPS, d["object"], s))
    for nm in gt_names:
        po = res["per_obj"].setdefault(nm, {"est": 0, "scored": 0, "adds_ok": 0, "hit": 0})
        po["gt_all"] = sum(1 for (i, x) in gt if x == nm)
    if realtime:
        # 화면 기준: 프레임 k 의 시각에 이미 끝난 결과 중 가장 최근 것
        done = sorted(frames, key=lambda f: f["done_s"])
        for mode in ("hold", "slam"):
            hit = 0
            res[f"screen_{mode}_series"] = []
            j = -1
            for k in range(n):
                while j + 1 < len(done) and done[j + 1]["done_s"] <= k / FPS:
                    j += 1
                if j < 0:
                    continue
                src = done[j]
                C = np.eye(4)
                if mode == "slam":
                    Ts, Tk = slam_T[src["idx"]], slam_T[k]
                    if Ts is None or Tk is None:
                        continue
                    C = np.linalg.inv(Tk) @ Ts       # 결과 프레임 카메라 → 현재 카메라
                for d in src["dets"]:
                    Tg = gt.get((k, d["object"]))
                    o = objs.get(d["object"])
                    if Tg is None or o is None:
                        continue
                    e = o.errs(C @ se3(d["R"], d["t_mm"]), Tg)[1]
                    hit += e < o.thr
                    res[f"screen_{mode}_series"].append((k / FPS, d["object"], e))
            res[f"screen_{mode}_hit"] = hit
    return res


def fmt(x):
    return f"{100 * x:.1f}%"


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
    objs = {o["name"]: Obj(o["name"], info[str(o["obj_id"])]["diameter"]) for o in table
            if (PTS_DIR / f"{o['name']}.npy").exists()}

    scenes = sorted(p.name for p in (YCBV / "test").iterdir()
                    if (a.all / p.name / "run.json").exists() and (a.rt / p.name / "run.json").exists())
    R = {"all": {}, "rt": {}}
    for s in scenes:
        sc = YCBV / "test" / s
        R["all"][s] = score_scene(sc, a.all / s, objs, by_id)
        n = R["all"][s]["frames"]
        R["rt"][s] = score_scene(sc, a.rt / s, objs, by_id, load_slam(a.slam / f"{s}_frames.txt", n), realtime=True)
        print(s, "done", flush=True)

    # ── 장면별 표 ──
    lines = ["| 장면 | 프레임 | 처리(모든) | 처리(실시간) | ADD-S<0.1d 모든 | ADD-S<0.1d 실시간 | 영상 전체 재현율 모든 | "
             "영상 전체 재현율 실시간 | 화면 기준 hold | 화면 기준 SLAM 보정 | 지연 중앙 [s] |", "|" + "---|" * 11]
    tot = {k: 0 for k in ("frames", "pa", "pr", "sa", "sr", "oa", "or", "inst", "ha", "hr", "sh", "ss")}
    for s in scenes:
        A, T = R["all"][s], R["rt"][s]
        lat = np.median([x for x in T["latency"] if x is not None])
        lines.append(f"| {s} | {A['frames']} | {A['processed']} | {T['processed']} ({T['processed'] / T['frames'] * 100:.1f}%) | "
                     f"{fmt(A['adds_ok'] / max(A['scored'], 1))} | {fmt(T['adds_ok'] / max(T['scored'], 1))} | "
                     f"{fmt(A['hit'] / A['inst_all'])} | {fmt(T['hit'] / T['inst_all'])} | "
                     f"{fmt(T['screen_hold_hit'] / T['inst_all'])} | {fmt(T['screen_slam_hit'] / T['inst_all'])} | {lat:.2f} |")
        tot["frames"] += A["frames"]; tot["pa"] += A["processed"]; tot["pr"] += T["processed"]
        tot["sa"] += A["scored"]; tot["sr"] += T["scored"]; tot["oa"] += A["adds_ok"]; tot["or"] += T["adds_ok"]
        tot["inst"] += A["inst_all"]; tot["ha"] += A["hit"]; tot["hr"] += T["hit"]
        tot["sh"] += T["screen_hold_hit"]; tot["ss"] += T["screen_slam_hit"]
    lat_all = [x for s in scenes for x in R["rt"][s]["latency"] if x is not None]
    lines.append(f"| **전체** | **{tot['frames']}** | **{tot['pa']}** | **{tot['pr']} ({tot['pr'] / tot['frames'] * 100:.1f}%)** | "
                 f"**{fmt(tot['oa'] / tot['sa'])}** | **{fmt(tot['or'] / tot['sr'])}** | **{fmt(tot['ha'] / tot['inst'])}** | "
                 f"**{fmt(tot['hr'] / tot['inst'])}** | **{fmt(tot['sh'] / tot['inst'])}** | **{fmt(tot['ss'] / tot['inst'])}** | "
                 f"**{np.median(lat_all):.2f}** |")
    add_a = sum(R["all"][s]["add_ok"] for s in scenes) / tot["sa"]
    add_r = sum(R["rt"][s]["add_ok"] for s in scenes) / tot["sr"]
    ms_a = np.median([x for s in scenes for x in R["all"][s]["ms"]])
    ms_r = np.median([x for s in scenes for x in R["rt"][s]["ms"]])
    summary = {"frames": tot["frames"], "gt_instances": tot["inst"],
               "all": {"processed": tot["pa"], "scored": tot["sa"], "add": add_a, "adds": tot["oa"] / tot["sa"],
                       "recall_video": tot["ha"] / tot["inst"], "ms_median": float(ms_a),
                       "est": sum(R["all"][s]["est"] for s in scenes)},
               "rt": {"processed": tot["pr"], "scored": tot["sr"], "add": add_r, "adds": tot["or"] / tot["sr"],
                      "recall_processed": tot["hr"] / sum(R["rt"][s]["inst_proc"] for s in scenes),
                      "recall_video": tot["hr"] / tot["inst"], "screen_hold": tot["sh"] / tot["inst"],
                      "screen_slam": tot["ss"] / tot["inst"], "latency_median_s": float(np.median(lat_all)),
                      "ms_median": float(ms_r), "est": sum(R["rt"][s]["est"] for s in scenes)}}
    print("\n".join(lines))
    print(json.dumps(summary, indent=1))

    # ── 객체별 표 ──
    names = sorted({nm for s in scenes for nm in R["all"][s]["per_obj"]})
    ol = ["| 객체 | GT 인스턴스 | 추정 수 모든 | 추정 수 실시간 | ADD-S<0.1d 모든 | ADD-S<0.1d 실시간 | 재현율 모든 | 재현율 실시간 |",
          "|" + "---|" * 8]
    for nm in names:
        def agg(m, k):
            return sum(R[m][s]["per_obj"].get(nm, {}).get(k, 0) for s in scenes)
        g = agg("all", "gt_all")
        if g == 0:
            continue
        ea, er = agg("all", "est"), agg("rt", "est")
        sa, sr = agg("all", "scored"), agg("rt", "scored")
        ol.append(f"| {nm.replace('ycbv_', '')} | {g} | {ea} | {er} | "
                  f"{fmt(agg('all', 'adds_ok') / sa) if sa else '—'} | {fmt(agg('rt', 'adds_ok') / sr) if sr else '—'} | "
                  f"{fmt(agg('all', 'hit') / g)} | {fmt(agg('rt', 'hit') / g)} |")
    print("\n".join(ol))
    (a.figs / "tables.md").write_text("\n".join(lines) + "\n\n" + "\n".join(ol) + "\n")
    (a.figs / "summary.json").write_text(json.dumps(summary, indent=1))

    # ── 그림 1: 장면별 재현율 막대 ──
    x = np.arange(len(scenes))
    fig, ax = plt.subplots(figsize=(14, 5.2))
    va = [R["all"][s]["hit"] / R["all"][s]["inst_all"] * 100 for s in scenes]
    vr = [R["rt"][s]["hit"] / R["rt"][s]["inst_all"] * 100 for s in scenes]
    vs = [R["rt"][s]["screen_slam_hit"] / R["rt"][s]["inst_all"] * 100 for s in scenes]
    w = 0.27
    ax.bar(x - w, va, w - 0.02, color=ALL_C, label="모든 프레임 입력")
    ax.bar(x, vs, w - 0.02, color=RT_C, alpha=0.55, label="실시간 · 화면 기준 (마지막 결과 + SLAM 보정)")
    ax.bar(x + w, vr, w - 0.02, color=RT_C, label="실시간 · 새 결과가 나온 프레임만")
    ax.set_xticks(x, scenes, fontsize=11)
    ax.set_ylabel("정답 인스턴스 중 맞은 비율 [%]")
    ax.set_ylim(0, 105)
    ax.set_title("YCB-Video 12장면 — 영상의 모든 프레임을 분모로 센 재현율 (ADD-S < 0.1d)", fontsize=14)
    ax.legend(fontsize=11, loc="upper right", ncol=3, bbox_to_anchor=(1, 1.0))
    fig.tight_layout(); fig.savefig(a.figs / "recall_by_scene.png", dpi=150); plt.close(fig)

    # ── 그림 2: 시간축 — 장면별 ADD-S 오차 (두 모드) ──
    fig, axs = plt.subplots(4, 3, figsize=(15, 13), sharey=True)
    for ax, s in zip(axs.flat, scenes):
        sa = np.array([(t, e) for t, _, e in R["all"][s]["series"]])
        sr = np.array([(t, e) for t, _, e in R["rt"][s]["series"]])
        if len(sa):
            ax.scatter(sa[:, 0], sa[:, 1], s=2, color=ALL_C, alpha=0.35, lw=0, label="모든 프레임")
        if len(sr):
            ax.scatter(sr[:, 0], sr[:, 1], s=14, color=RT_C, lw=0, label="실시간")
        ax.set_yscale("log"); ax.set_ylim(0.5, 300)
        ax.axhspan(0.5, 10, color=GREY, alpha=0.08, lw=0)
        ax.set_title(f"장면 {s}", fontsize=13)
        ax.set_xlim(0, R["all"][s]["frames"] / FPS)
    for ax in axs[-1]:
        ax.set_xlabel("시간 [s]")
    for ax in axs[:, 0]:
        ax.set_ylabel("ADD-S [mm]")
    h, l = axs.flat[0].get_legend_handles_labels()
    for hh in h:
        hh.set_sizes([40]); hh.set_alpha(1)
    fig.legend(h, l, loc="upper center", ncol=2, fontsize=13, frameon=False, bbox_to_anchor=(0.5, 0.97))
    fig.suptitle("프레임별 객체 포즈 오차 (점 하나 = 추정 하나, 로그 축, 회색 띠 = 10 mm 이하)", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(a.figs / "adds_time.png", dpi=150); plt.close(fig)

    # ── 그림 3: 화면 기준 — 한 장면·한 객체의 프레임별 ADD-S (모든 프레임 / 실시간 hold / 실시간 SLAM 보정) ──
    s = max(scenes, key=lambda x: (R["rt"][x]["screen_slam_hit"] - R["rt"][x]["screen_hold_hit"]) / R["rt"][x]["inst_all"])
    T, A = R["rt"][s], R["all"][s]
    cnt = {}
    for _, nm, _ in T["screen_hold_series"]:
        cnt[nm] = cnt.get(nm, 0) + 1
    nm = max(cnt, key=cnt.get)
    thr = objs[nm].thr
    fig, ax = plt.subplots(figsize=(14, 5.4))
    def ser(lst):
        v = np.array([(t, e) for t, n_, e in lst if n_ == nm])
        return v[np.argsort(v[:, 0])] if len(v) else v
    va, vh, vs = ser(A["series"]), ser(T["screen_hold_series"]), ser(T["screen_slam_series"])
    ax.plot(va[:, 0], va[:, 1], color=ALL_C, lw=1.2, label="모든 프레임 입력 (그 프레임의 결과)")
    ax.plot(vh[:, 0], vh[:, 1], color=RT_C, lw=1.2, alpha=0.45, label="실시간 · 마지막 결과를 그대로 표시")
    ax.plot(vs[:, 0], vs[:, 1], color=RT_C, lw=1.6, label="실시간 · 마지막 결과 + SLAM 카메라 이동 보정")
    tr = np.array([f["arrive_s"] for f in load_frames(a.rt / s)])
    ax.plot(tr, np.full(len(tr), 0.6), "|", color="#333", ms=9, label="실시간 모드가 입력한 프레임")
    ax.axhline(thr, color=GREY, ls="--", lw=1)
    ax.text(0.3, thr * 1.08, f"합격선 0.1d = {thr:.1f} mm", fontsize=11, color="#333")
    ax.set_yscale("log"); ax.set_ylim(0.5, max(300, np.nanmax(vh[:, 1]) * 1.2 if len(vh) else 300))
    ax.set_xlim(0, A["frames"] / FPS)
    ax.set_xlabel("시간 [s]"); ax.set_ylabel("ADD-S [mm]")
    ax.set_title(f"장면 {s} · {nm.replace('ycbv_', '')} — 각 프레임에서 화면에 보이는 자세의 오차", fontsize=14)
    ax.legend(fontsize=11, loc="upper right", ncol=2)
    fig.tight_layout(); fig.savefig(a.figs / "screen_error_time.png", dpi=150); plt.close(fig)

    # ── 그림 4: 지연 분포 + 처리 시간 ──
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.6))
    axs[0].hist([x for s in scenes for x in R["all"][s]["ms"]], bins=60, range=(0, 1500), color=ALL_C, alpha=0.8,
                density=True, label="모든 프레임")
    axs[0].hist([x for s in scenes for x in R["rt"][s]["ms"]], bins=60, range=(0, 1500), color=RT_C, alpha=0.7,
                density=True, label="실시간")
    axs[0].set_xlabel("프레임당 추론 시간 [ms]"); axs[0].set_ylabel("밀도"); axs[0].legend()
    axs[0].set_title("추론 시간 분포", fontsize=13)
    axs[1].hist(np.array(lat_all) * 1000, bins=60, color=RT_C)
    axs[1].axvline(np.median(lat_all) * 1000, color="#333", lw=1, ls="--")
    axs[1].text(np.median(lat_all) * 1000, axs[1].get_ylim()[1] * 0.92, f" 중앙 {np.median(lat_all) * 1000:.0f} ms",
                fontsize=12)
    axs[1].set_xlabel("프레임 도착 → 결과까지 [ms]"); axs[1].set_ylabel("프레임 수")
    axs[1].set_title("실시간 모드 지연", fontsize=13)
    fig.tight_layout(); fig.savefig(a.figs / "timing.png", dpi=150); plt.close(fig)


if __name__ == "__main__":
    main()
