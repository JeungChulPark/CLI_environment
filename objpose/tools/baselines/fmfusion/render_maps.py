#!/usr/bin/env python3
"""render_maps.py — YCB-V 장면마다 최종 물체 지도를 위에서 본 그림으로 나란히 그린다 (eval_maps.py 와 같은 정렬·짝짓기).

  칸: FM-Fusion | Object Pose Map (제안) | Object Pose Map (원본 SAM-6D)
  배경 = SLAM 위치(정렬 후)로 쌓은 깊이 점군(build_memory_maps.point_cloud), 윗면 좌표 = build_memory_maps.top_frame
  점선 = 정답 물체 윤곽, 채움 = 지도 물체 (FM-Fusion: 복원 점, Object Pose Map: 추정 자세의 CAD 윤곽)
  표시: 이름 옆 '중복', '다른 이름', '먼 위치', 정답 쪽 '놓침'

    /home/jucpark/anaconda3/envs/sam6d/bin/python objpose/tools/baselines/fmfusion/render_maps.py \
        --fmf objpose/output/fmfusion_ycbv_gsam --ours objpose/output/ycbv_sam6d_realtime_260921 \
        --orig objpose/output/cmp_260924/orig_ycbv_realtime_top1 --title "자체 검출" --out objpose/output/fmfusion_ycbv_gsam/maps
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Patch
from matplotlib.lines import Line2D

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_memory_maps as BM  # noqa: E402
import eval_maps as EM  # noqa: E402

COLORS = ["#1f6fb2", "#e08a1e", "#3a9d5d", "#b8342a", "#7b5ea7", "#8c564b", "#d4a017"]
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "font.size": 11})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fmf", type=Path, required=True)
    ap.add_argument("--ours", type=Path, required=True)
    ap.add_argument("--orig", type=Path, default=None)
    ap.add_argument("--title", default="")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--scenes", nargs="*", default=None)
    ap.add_argument("--compact", action="store_true", help="발표용: 큰 글자, 칸 제목·개수만, 설명 글·범례 없음")
    a = ap.parse_args()
    DS = {"ycbv": "YCB-Video", "ycbm": "YCB-M"}.get(json.loads(BM.TABLE.read_text()).get("dataset"), "")
    a.out.mkdir(parents=True, exist_ok=True)
    table = json.loads(BM.TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    by_label = {o["ycb"].split("_", 1)[1].replace("_", " "): o["name"] for o in table}
    model = {o["name"]: np.load(BM.PTS / f"{o['name']}.npy") / 1000.0 for o in table}
    scenes = a.scenes or sorted(p.name for p in a.fmf.iterdir() if p.is_dir() and (p / "instance_info.txt").exists())
    for s in scenes:
        BM.RUNS["rt"] = a.ours
        r_ours = BM.run_memory(s, "rt", by_id)
        r_orig = None
        if a.orig:
            BM.RUNS["rt"] = a.orig
            r_orig = BM.run_memory(s, "rt", by_id)
        A = r_ours["A"]
        gobj = r_ours["gobj"]
        gt_pts = {nm: model[nm] @ T[:3, :3].T + T[:3, 3] for nm, T in gobj.items()}
        panels = []
        items = []
        for line in (a.fmf / s / "instance_info.txt").read_text().splitlines():
            if line.startswith("#") or not line.strip():
                continue
            f = line.split(";")
            P = EM.read_ply_xyz(a.fmf / s / f"{int(f[0]):04d}.ply")
            items.append((int(f[2]), by_label.get(f[1].split("(")[0], f[1].split("(")[0]), P @ A[:3, :3].T + A[:3, 3], None))
        items = [x[1:] for x in sorted(items, key=lambda x: -x[0])]
        panels.append(("FM-Fusion", "points", items, EM.score(items, gt_pts, gobj)))
        for title, r in (("Object Pose Map (제안)", r_ours), ("Object Pose Map (원본 SAM-6D)", r_orig)):
            if r is None:
                continue
            its = []
            for lm in sorted(r["lms"], key=lambda m_: -m_["n_obs"]):
                T = A @ np.asarray(lm["T_w_obj"], float)
                its.append((lm["name"], model[lm["name"]] @ T[:3, :3].T + T[:3, 3], T[:3, :3]))
            panels.append((title, "cad", its, EM.score(its, gt_pts, gobj)))

        P, C = BM.point_cloud(s, r_ours)
        c0, B, up, th = BM.top_frame(r_ours, P)
        h = (P - c0) @ up - th
        uvall = (P - c0) @ B.T
        gcs = np.array([(G[:3, 3] - c0) @ B.T for G in gobj.values()])
        zc = gcs.mean(0); zh = max(0.16, np.abs(gcs - zc).max() + 0.12)
        keep = (h > -0.01) & (h < 0.40) & (np.abs(uvall - zc).max(1) < zh)
        uv = uvall[keep]; order = np.argsort(h[keep])
        gray = C[keep][order] * 0.35 + 0.6                                     # 배경은 옅게
        col = {nm: COLORS[j % len(COLORS)] for j, nm in enumerate(sorted(gobj))}

        if a.compact:
            plt.rcParams.update({"font.size": 15})
            fig, axs = plt.subplots(1, len(panels), figsize=(4.6 * len(panels), 4.9))
        else:
            fig, axs = plt.subplots(1, len(panels), figsize=(8 * len(panels), 8.6))
        for ax, (title, kind, its, (rows, missing)) in zip(axs, panels):
            ax.scatter(uv[order, 0], uv[order, 1], c=gray, s=3, lw=0, rasterized=True)
            for nm, Pg in gt_pts.items():
                hull = cv2.convexHull(((Pg - c0) @ B.T).astype(np.float32)).reshape(-1, 2)
                ax.add_patch(Polygon(hull, closed=True, fill=False, ec=col[nm], lw=1.8, ls="--"))
            cnt = {"match": 0, "duplicate": 0, "far": 0, "wrong_class": 0}
            outside, n_tag = [], 0
            for (nm, Pw, _), row in zip(its, rows):
                cnt[row["kind"]] += 1
                c = col.get(nm, "#777777")
                q = (Pw - c0) @ B.T
                if kind == "points":
                    ax.scatter(q[::2, 0], q[::2, 1], s=2.5, color=c, lw=0, alpha=0.8, rasterized=True)
                else:
                    hull = cv2.convexHull(q.astype(np.float32)).reshape(-1, 2)
                    ax.add_patch(Polygon(hull, closed=True, fc=c, alpha=0.35, ec=c, lw=1.8))
                ctr = q.mean(0)
                if np.abs(ctr - zc).max() > zh:                                 # 확대 범위 밖 — 제목 아래에 따로 적는다
                    outside.append(f"{row['kind']}:{nm.replace('ycbv_', '')} {np.linalg.norm(ctr - zc):.2f} m")
                    continue
                if row["kind"] == "match":
                    ax.plot(*ctr, "o", color=c, ms=6, mec="white", mew=1)
                else:
                    tag = {"duplicate": "중복", "far": "먼 위치", "wrong_class": "이름 없음" if nm == "unknown" else "다른 이름"}[row["kind"]]
                    ax.plot(*ctr, "X", color="#b8342a", ms=12, mec="white", mew=1.2)
                    if a.compact:
                        continue
                    n_tag += 1
                    ax.annotate(f"{tag}: {nm.replace('ycbv_', '')}", ctr, xytext=(10, -14 - 16 * (n_tag % 3)), textcoords="offset points",
                                arrowprops=dict(arrowstyle="-", color="#b8342a", lw=0.8),
                                fontsize=9.5, color="#b8342a", bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.85))
            for j, nm in enumerate([] if a.compact else missing):
                gc = gt_pts[nm].mean(0)
                gcx = (gc - c0) @ B.T
                ax.annotate(f"놓침: {nm.replace('ycbv_', '')}", gcx, xytext=(0, 18 + 14 * (j % 2)), textcoords="offset points", ha="center",
                            fontsize=9.5, color="#555", bbox=dict(boxstyle="round,pad=0.2", fc="#eeeeee", ec="#999", alpha=0.9))
            ax.set_xlim(zc[0] - zh, zc[0] + zh); ax.set_ylim(zc[1] - zh, zc[1] + zh)
            ax.set_aspect("equal"); ax.grid(alpha=0.2); ax.set_xlabel("[m]")
            if a.compact:
                ax.set_xticks([]); ax.set_yticks([]); ax.set_xlabel("")     # 칸 이름·개수는 슬라이드 글로 뺀다
                if outside:
                    kinds = {"duplicate": "중복", "far": "먼 위치", "wrong_class": "이름 오류", "match": "맞음"}
                    import collections
                    kc = collections.Counter(kinds[o.split(":")[0]] for o in outside)
                    ax.text(0.98, 0.02, "그림 밖 " + ", ".join(f"{k} {v}개" for k, v in kc.items()), transform=ax.transAxes, ha="right", va="bottom", fontsize=13,
                            color="#b8342a", bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#b8342a", alpha=0.9))
                continue
            ax.set_title(f"{title}\n맞음 {cnt['match']}/{len(gt_pts)} · 중복 {cnt['duplicate']} · "
                         f"이름 오류 {cnt['wrong_class']} · 먼 위치 {cnt['far']} · 놓침 {len(missing)}", fontsize=12.5)
            if outside:
                txt = {"duplicate": "중복", "far": "먼 위치", "wrong_class": "이름 오류", "match": "맞음"}
                lines = [txt[o.split(":")[0]] + " " + o.split(":", 1)[1] for o in outside]
                ax.text(0.99, 0.01, "그림 밖 (중심에서 거리)\n" + "\n".join(lines[:8]) + ("\n…" if len(lines) > 8 else ""),
                        transform=ax.transAxes, ha="right", va="bottom", fontsize=9.5, color="#b8342a",
                        bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#b8342a", alpha=0.9))
        if a.compact:
            fig.tight_layout(pad=0.3)
            fig.savefig(a.out / f"map_{s}.png", dpi=200, bbox_inches="tight", pad_inches=0.03)
            plt.close(fig); print(s, flush=True)
            continue
        hnd = [Patch(fill=False, ec="#444", ls="--"), Patch(fc="#999", alpha=0.35, ec="#999"),
               Line2D([], [], marker="o", ls="", ms=4, color="#999"), Line2D([], [], marker="X", ls="", ms=10, color="#b8342a")]
        axs[0].legend(hnd, ["정답 물체 윤곽", "지도 물체 (CAD 자세)", "지도 물체 (복원 점)", "중복 · 이름 오류 · 먼 위치"],
                      loc="upper left", fontsize=10, framealpha=0.92)
        fig.suptitle(f"{DS} {s} — 최종 물체 지도, 위에서 본 모습 ({a.title}, 카메라 자세는 모든 방법이 같은 ORB-SLAM3)", fontsize=14)
        fig.tight_layout(rect=(0, 0, 1, 0.95))
        fig.savefig(a.out / f"map_{s}.png", dpi=90)
        plt.close(fig)
        print(s, flush=True)


if __name__ == "__main__":
    main()
