#!/usr/bin/env python3
"""build_memory_maps.py — YCB-V 장면마다 SLAM 현재 위치 + 실시간 SAM-6D 결과로 객체 메모리를 돌리고,
최종 맵(점군) · 등록된 객체 위치 · 카메라 추적 경로를 한 장에 그린다.

입력 (hub 와 같은 경로):
  * 카메라 위치: ORB-SLAM3 RGB-D 가 추적 직후 낸 프레임별 현재 위치 (output/ycbv_slam_orb_260921/<scene>_frames.txt)
    → objpose/pc/fusion.PoseBuffer
  * 객체 관측: 실시간 입력 SAM-6D 결과 (output/ycbv_sam6d_realtime_260921/<scene>/sam6d_frames.jsonl)
    → objpose/pc/memory.ObjectMemory (objectmemory_ws StreamingObjectMemory), hub 기본값:
      위치 연관 게이트 0.30 m · 회전 게이트 없음 · 깊이 검사(free 0.15 m, 3프레임, 0.6 m/s, 45 °/s)
  * --sam all 이면 모든 프레임 입력 결과를 넣는다(비교용).

평가용 정렬: SLAM 월드는 첫 카메라 좌표계라 GT 월드와 강체 변환만큼 다르다. 그림과 오차는 SLAM 궤적을 GT 궤적에
SE3 로 한 번 맞춘 좌표에서 낸다(위치와 자세를 함께 써서 맞춤 — 위치만 쓰면 회전이 2–3° 남아 물체가 수 cm 밀린다). 이 정렬은 채점용일 뿐
메모리 동작에는 쓰지 않는다.

    /home/jucpark/anaconda3/envs/sam6d/bin/python objpose/tools/ycbv_sam6d/build_memory_maps.py \
        --sam rt --out objpose/docs/figs_ycbv_memory
"""
from __future__ import annotations

import argparse
import json, os
import sys
from pathlib import Path

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose" / "pc"))
sys.path.insert(0, str(REPO / "slam_comparison_report" / "loc_eval"))
from fusion import Fusion, PoseBuffer                    # noqa: E402
from memory import ObjectMemory, DepthCheckConfig, map_occlusion_kwargs        # noqa: E402
from metrics import umeyama, quat_to_rot                 # noqa: E402

# BENCH_* override the dataset (default YCB-V): BENCH_ROOT (BOP root with test/, models_eval/), BENCH_TABLE, BENCH_SLAM, BENCH_FPS
YCBV = Path(os.environ.get("BENCH_ROOT", "/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video")) / "test"
PTS = REPO / "sam6d_realtime" / "assets" / "model_points"
TABLE = Path(os.environ.get("BENCH_TABLE", REPO / "objpose" / "rt" / "bop" / "ycbv_objects.json"))
SLAM = Path(os.environ.get("BENCH_SLAM", REPO / "objpose" / "output" / "ycbv_slam_orb_260921"))
RUNS = {"rt": REPO / "objpose" / "output" / "ycbv_sam6d_realtime_260921",
        "all": REPO / "objpose" / "output" / "ycbv_sam6d_all_260921"}
T0_NS, FPS = 1_600_000_000_000_000_000, float(os.environ.get("BENCH_FPS", 30.0))
STEP_NS = int(round(1e9 / FPS))
KEEP = ("active", "lost", "remembered")
COLORS = ["#1f6fb2", "#e08a1e", "#3a9d5d", "#b8342a", "#7b5ea7", "#8c564b", "#d4a017"]
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "font.size": 11})


def load_slam(scene, n):
    a = np.loadtxt(SLAM / f"{scene}_frames.txt", ndmin=2)
    out = {}
    for r in a:
        i = int(round(r[0] * FPS))
        T = np.eye(4); T[:3, :3] = quat_to_rot(r[4:8]); T[:3, 3] = r[1:4]
        if 0 <= i < n:
            out[i] = T
    return out


def gt_scene(sc: Path, by_id):
    cam = json.loads((sc / "scene_camera.json").read_text())
    gt = json.loads((sc / "scene_gt.json").read_text())
    ids = sorted(cam, key=int)
    Twc = []
    objs = {}
    for f in ids:
        R = np.array(cam[f]["cam_R_w2c"]).reshape(3, 3); t = np.array(cam[f]["cam_t_w2c"]) / 1000.0
        T = np.eye(4); T[:3, :3] = R.T; T[:3, 3] = -R.T @ t
        Twc.append(T)
        for r in gt[f]:
            Tco = np.eye(4); Tco[:3, :3] = np.array(r["cam_R_m2c"]).reshape(3, 3); Tco[:3, 3] = np.array(r["cam_t_m2c"]) / 1000.0
            objs.setdefault(by_id[r["obj_id"]], []).append(T @ Tco)
    K = np.array(cam[ids[0]]["cam_K"]).reshape(3, 3)
    # 물체는 장면 안에서 움직이지 않는다 → 위치는 중앙값, 자세는 중앙값 위치에 가장 가까운 프레임의 것
    gobj = {}
    for nm, Ts in objs.items():
        P = np.array([T[:3, 3] for T in Ts]); med = np.median(P, 0)
        gobj[nm] = Ts[int(np.argmin(np.linalg.norm(P - med, axis=1)))].copy(); gobj[nm][:3, 3] = med
    return ids, np.array(Twc), gobj, K


def run_memory(scene, sam, by_id):
    sc = YCBV / scene
    ids, Twc_gt, gobj, K = gt_scene(sc, by_id)
    n = len(ids)
    slam = load_slam(scene, n)
    poses = PoseBuffer(max_gap_s=0.25)
    for i in sorted(slam):
        poses.add(T0_NS + i * STEP_NS, "OK", slam[i], map_id=0)
    fusion = Fusion(np.eye(4), poses)
    mem = ObjectMemory(fusion, K, (640, 480),
                       depth_check=DepthCheckConfig(free_margin_m=0.15, confirm_frames=3, max_speed_mps=0.6, max_rot_dps=45.0),
                       assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None, **map_occlusion_kwargs(by_id.values()))
    frames = [json.loads(l) for l in (RUNS[sam] / scene / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()]
    frames.sort(key=lambda f: f.get("done_s", f["idx"]))
    history = []                                  # (영상 시각 s, 등록 객체 수)
    for f in frames:
        raw = cv2.imread(str(sc / "depth" / f"{f['idx'] + 1:06d}.png"), cv2.IMREAD_UNCHANGED)
        depth = np.rint(raw.astype(np.float32) * 0.1).astype(np.uint16)[::8, ::8]
        mem.add_frame(int(f["t_ns"]), [(d["object"], d["score"], d["R"], d["t_mm"]) for d in f["dets"]], depth=depth)
        mem.update()
        history.append((f.get("done_s", f["idx"] / FPS), sum(1 for m in mem.landmarks() if m["status"] in KEEP)))
    lms = [m for m in mem.landmarks() if m["status"] in KEEP]
    # 채점용 SE3 정렬: SLAM 궤적 → GT 궤적
    ks = sorted(slam)
    # 위치만으로 맞추면(Umeyama) 회전이 2–3° 남고, 0.9 m 앞의 물체가 그만큼(~40 mm) 한쪽으로 밀린다.
    # 그래서 자세까지 쓴다: R = SO(3) 투영(Σ R_gt R_slamᵀ), t = 평균(p_gt − R p_slam).
    M = sum(Twc_gt[k, :3, :3] @ slam[k][:3, :3].T for k in ks)
    U, _, Vt = np.linalg.svd(M)
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    t = np.mean([Twc_gt[k, :3, 3] - R @ slam[k][:3, 3] for k in ks], 0)
    A = np.eye(4); A[:3, :3] = R; A[:3, 3] = t
    return dict(ids=ids, K=K, Twc_gt=Twc_gt, gobj=gobj, slam=slam, A=A, lms=lms, frames=frames, history=history,
                stats=mem.stats())


def point_cloud(scene, r, stride=15, pix=6):
    """SLAM 위치(정렬 후)로 깊이 영상을 쌓은 컬러 점군 — 맵의 배경."""
    sc = YCBV / scene
    K = r["K"]
    v, u = np.mgrid[0:480:pix, 0:640:pix]
    P, C = [], []
    for k in range(0, len(r["ids"]), stride):
        if k not in r["slam"]:
            continue
        d = cv2.imread(str(sc / "depth" / f"{k + 1:06d}.png"), cv2.IMREAD_UNCHANGED)[::pix, ::pix].astype(np.float32) * 1e-4
        rgb = cv2.imread(str(sc / "rgb" / f"{k + 1:06d}.png"))[::pix, ::pix, ::-1]
        m = (d > 0.2) & (d < 1.6)
        z = d[m]; x = (u[m] - K[0, 2]) * z / K[0, 0]; y = (v[m] - K[1, 2]) * z / K[1, 1]
        T = r["A"] @ r["slam"][k]
        P.append(np.c_[x, y, z] @ T[:3, :3].T + T[:3, 3]); C.append(rgb[m] / 255.0)
    P, C = np.concatenate(P), np.concatenate(C)
    key = np.floor(P / 0.005).astype(np.int64)                        # 5 mm 복셀
    _, idx = np.unique(key, axis=0, return_index=True)
    return P[idx], C[idx]


def top_frame(r, P):
    """위에서 본 좌표계. '위' = 물체들이 놓인 테이블 평면의 법선(점군 RANSAC), 카메라 쪽을 향하게.
    가로축 = 그 평면에 투영한 GT 카메라 경로의 주성분."""
    obj_c = np.mean([T[:3, 3] for T in r["gobj"].values()], 0)
    near = P[np.linalg.norm(P - obj_c, axis=1) < 0.45]
    rng = np.random.default_rng(0)
    best, n_best = None, -1
    for _ in range(400):
        q = near[rng.choice(len(near), 3, replace=False)]
        n = np.cross(q[1] - q[0], q[2] - q[0])
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        cnt = int((np.abs((near - q[0]) @ n) < 0.006).sum())
        if cnt > n_best:
            best, n_best = (n, q[0]), cnt
    up, p0 = best
    cams = r["Twc_gt"][:, :3, 3]
    if np.mean((cams - p0) @ up) < 0:
        up = -up
    c = cams - obj_c
    c = c - np.outer(c @ up, up)
    e1 = np.linalg.svd(c, full_matrices=False)[2][0]
    e1 = e1 - (e1 @ up) * up; e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    table_h = float((p0 - obj_c) @ up)
    return obj_c, np.stack([e1, e2]), up, table_h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sam", choices=["rt", "all"], default="rt")
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--scenes", nargs="*", default=None)
    ap.add_argument("--run-dir", type=Path, default=None, help="use this run's sam6d_frames.jsonl instead of RUNS[--sam]")
    a = ap.parse_args()
    if a.run_dir is not None:
        RUNS[a.sam] = a.run_dir
    a.out.mkdir(parents=True, exist_ok=True)
    table = json.loads(TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    cor = {}
    for o in table:
        p = np.load(PTS / f"{o['name']}.npy") / 1000.0
        lo, hi = p.min(0), p.max(0)
        cor[o["name"]] = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    scenes = a.scenes or sorted(p.name for p in YCBV.iterdir())
    summary = []
    for s in scenes:
        r = run_memory(s, a.sam, by_id)
        A = r["A"]
        # 등록 객체 ↔ GT 객체 짝: 같은 이름 중 가장 가까운 것
        rows, used = [], set()
        for m in sorted(r["lms"], key=lambda m: -m["n_obs"]):
            Tw = A @ m["T_w_obj"]
            g = r["gobj"].get(m["name"])
            err = float(np.linalg.norm(Tw[:3, 3] - g[:3, 3]) * 1000) if g is not None else None
            dup = m["name"] in used
            used.add(m["name"])
            rows.append({"name": m["name"], "status": m["status"], "n_obs": m["n_obs"], "err_mm": err,
                         "duplicate": dup, "T": Tw})
        missing = [nm for nm in r["gobj"] if nm not in used]
        errs = [x["err_mm"] for x in rows if x["err_mm"] is not None and not x["duplicate"]]
        slam_err = np.linalg.norm(np.array([(A @ r["slam"][k])[:3, 3] - r["Twc_gt"][k, :3, 3] for k in r["slam"]]), axis=1) * 1000
        summary.append({"scene": s, "gt_objects": len(r["gobj"]), "registered": len(rows),
                        "matched": len(errs), "duplicates": sum(x["duplicate"] for x in rows),
                        "wrong_class": sum(1 for x in rows if x["err_mm"] is None), "missing": missing,
                        "err_median_mm": float(np.median(errs)) if errs else None, "err_max_mm": float(max(errs)) if errs else None,
                        "slam_ate_rmse_mm": float(np.sqrt((slam_err ** 2).mean())), "sam_frames": len(r["frames"]),
                        "objects": [{k: v for k, v in x.items() if k != "T"} for x in rows]})

        # ── 그림 ──
        P, C = point_cloud(s, r)
        c0, B, up, th = top_frame(r, P)
        h = (P - c0) @ up - th                                  # 테이블 면에서 위로의 높이
        uvall = (P - c0) @ B.T
        keep = (h > -0.01) & (h < 0.40) & (np.linalg.norm(uvall, axis=1) < 0.6)   # 테이블 위 물체 영역만
        uv = uvall[keep]
        order = np.argsort(h[keep])                             # 높은 점(물체 위)이 나중에 그려지도록
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch, Rectangle
        pg = (r["Twc_gt"][:, :3, 3] - c0) @ B.T
        ps = np.array([((A @ r["slam"][k])[:3, 3] - c0) @ B.T for k in sorted(r["slam"])])
        fi = sorted({f["idx"] for f in r["frames"]})
        pf = np.array([((A @ r["slam"][k])[:3, 3] - c0) @ B.T for k in fi if k in r["slam"]])
        gcs = np.array([(G[:3, 3] - c0) @ B.T for G in r["gobj"].values()])
        zc = gcs.mean(0); zh = max(0.16, np.abs(gcs - zc).max() + 0.12)
        fig, axs = plt.subplots(1, 2, figsize=(19, 9.6), gridspec_kw={"width_ratios": [1, 1]})
        for pi, ax in enumerate(axs):
            zoom = pi == 1
            ax.scatter(uv[order, 0], uv[order, 1], c=C[keep][order], s=4 if zoom else 1.2, lw=0, alpha=0.9, rasterized=True)
            ax.plot(pg[:, 0], pg[:, 1], color="white", lw=5, alpha=0.9, solid_capstyle="round")
            ax.plot(pg[:, 0], pg[:, 1], color="#555555", lw=3, label="카메라 경로 · GT")
            ax.plot(ps[:, 0], ps[:, 1], color="#1f6fb2", lw=1.6, label="카메라 경로 · SLAM 추정")
            ax.plot(*ps[0], "o", color="#1f6fb2", ms=9, mec="white", mew=1.5, label="시작")
            ax.plot(*ps[-1], "s", color="#1f6fb2", ms=9, mec="white", mew=1.5, label="끝")
            ax.plot(pf[:, 0], pf[:, 1], "|", color="#1f6fb2", ms=5, alpha=0.6)
            for j, (nm, G) in enumerate(sorted(r["gobj"].items())):
                col = COLORS[j % len(COLORS)]
                fp = ((G[:3, :3] @ cor[nm].T).T + G[:3, 3] - c0) @ B.T
                hull = cv2.convexHull(fp.astype(np.float32)).reshape(-1, 2)
                ax.add_patch(Polygon(hull, closed=True, fill=False, ec="white", lw=4, ls="-"))
                ax.add_patch(Polygon(hull, closed=True, fill=False, ec=col, lw=2, ls="--"))
                mine = [x for x in rows if x["name"] == nm]
                for x in mine:
                    fe = ((x["T"][:3, :3] @ cor[nm].T).T + x["T"][:3, 3] - c0) @ B.T
                    he = cv2.convexHull(fe.astype(np.float32)).reshape(-1, 2)
                    ax.add_patch(Polygon(he, closed=True, fc=col, alpha=0.35, ec=col, lw=2))
                    ax.plot(*((x["T"][:3, 3] - c0) @ B.T), "o", color=col, ms=7, mec="white", mew=1.2)
                if not zoom:
                    continue
                gc = (G[:3, 3] - c0) @ B.T
                tag = nm.replace("ycbv_", "")
                if mine:
                    tag += f"\n오차 {mine[0]['err_mm']:.0f} mm · 관측 {mine[0]['n_obs']}회"
                    if len(mine) > 1:
                        tag += f" (+{len(mine) - 1} 중복)"
                else:
                    tag += "\n미등록"
                dvec = gc - zc
                dvec = dvec / np.linalg.norm(dvec) if np.linalg.norm(dvec) > 1e-3 else np.array([0.0, -1.0])
                ax.annotate(tag, gc, xytext=tuple(dvec * 120), textcoords="offset points", ha="center", va="center",
                            fontsize=11, color="#111", bbox=dict(boxstyle="round,pad=0.3", fc="white", ec=col, lw=1.4, alpha=0.93),
                            arrowprops=dict(arrowstyle="-", color=col, lw=1.2))
            for x in rows:
                if x["err_mm"] is None:                                       # GT 에 없는 이름으로 등록된 객체
                    ax.plot(*((x["T"][:3, 3] - c0) @ B.T), "X", color="#b8342a", ms=11, mec="white")
            ax.set_aspect("equal")
            if zoom:
                ax.set_xlim(zc[0] - zh, zc[0] + zh); ax.set_ylim(zc[1] - zh, zc[1] + zh)
                ax.set_title("객체 영역 확대", fontsize=12)
            else:
                allp = np.vstack([pg, ps, uv])
                lo, hi = allp.min(0) - 0.05, allp.max(0) + 0.05
                cx, cy = (lo + hi) / 2; half = max(hi - lo) / 2
                ax.set_xlim(cx - half, cx + half); ax.set_ylim(cy - half, cy + half)
                ax.add_patch(Rectangle((zc[0] - zh, zc[1] - zh), 2 * zh, 2 * zh, fill=False, ec="#333", lw=1, ls=":"))
                ax.set_title("전체 맵 — 위에서 본 모습 (테이블 면 기준)", fontsize=12)
                hnd, lab = ax.get_legend_handles_labels()
                hnd += [Patch(fc="#999999", alpha=0.35, ec="#999999"), Patch(fill=False, ec="#999999", ls="--"),
                        Line2D([], [], marker="|", ls="", color="#1f6fb2", ms=8)]
                lab += ["객체 메모리가 등록한 객체 (최종)", "GT 객체 위치", "SAM-6D 결과가 나온 카메라 위치"]
                ax.legend(hnd, lab, loc="upper left", fontsize=10, framealpha=0.92)
            ax.set_xlabel("[m]"); ax.set_ylabel("[m]")
            ax.grid(alpha=0.2)
        sm = summary[-1]
        med = "—" if sm["err_median_mm"] is None else f"{sm['err_median_mm']:.0f} mm"
        src = "실시간" if a.sam == "rt" else "모든 프레임"
        fig.suptitle(f"YCB-Video {s} — 객체 메모리 최종 맵 (SAM-6D {src} 입력 + ORB-SLAM3 현재 위치)   "
                     f"등록 {sm['matched']}/{sm['gt_objects']}개 · 등록 위치 오차 중앙 {med} · SLAM 경로 오차 RMSE {sm['slam_ate_rmse_mm']:.0f} mm",
                     fontsize=13)
        fig.tight_layout(rect=(0, 0, 1, 0.97))
        fig.savefig(a.out / f"memory_map_{a.sam}_{s}.png", dpi=120)
        plt.close(fig)
        print(s, json.dumps({k: v for k, v in sm.items() if k != "objects"}, ensure_ascii=False), flush=True)
    (a.out / f"memory_summary_{a.sam}.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
