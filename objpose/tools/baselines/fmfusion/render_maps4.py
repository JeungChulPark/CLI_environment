#!/usr/bin/env python3
"""render_maps4.py — 장면마다 정답 | 제안 방법 | FM-Fusion | DualMap 객체 지도를 위에서 본 그림 한 장으로 그린다.

객체와 판정은 eval_maps_f1.py --dump 가 남긴 json 을 그대로 쓴다(그림과 채점이 같은 자료). 배경 점군과 윗면 좌표는 render_maps.py 와 같다.
  점선 = 정답 물체 윤곽,  채움 = 지도 객체(제안: 추정 자세의 CAD 윤곽, 나머지: 복원 점),  화살표 = 물체 x축(점선 정답, 실선 추정)
  ● 맞음   ✕ 중복 · 이름 틀림 · 먼 위치   회색 점 = 대상 물체 이름이 붙지 않은 객체
BENCH_ROOT / BENCH_TABLE / BENCH_SLAM / BENCH_FPS 로 데이터셋을 바꾼다.
    <sam6d python> objpose/tools/baselines/fmfusion/render_maps4.py --ours <proposed dir> --dump <dump dir> --out <dir> [--scenes …]
"""
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM  # noqa: E402

COLORS = ["#1f6fb2", "#e08a1e", "#3a9d5d", "#b8342a", "#7b5ea7", "#8c564b", "#d4a017"]
TAG = {"duplicate": "중복", "far": "먼 위치", "wrong_name": "이름 틀림"}
METHODS = ["제안 방법", "FM-Fusion", "DualMap"]
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "font.size": 11})
short = lambda n: (n or "이름 없음").replace("ycbv_", "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ours", type=Path, required=True); ap.add_argument("--dump", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True); ap.add_argument("--scenes", nargs="*", default=None)
    ap.add_argument("--compact", action="store_true", help="발표용: 큰 글자, 눈금 · 긴 제목 없음")
    ap.add_argument("--view-frame", type=float, default=None, help="이 지점(0~1)의 카메라가 보는 방향이 그림의 위쪽이 되도록 돌린다(왼쪽 · 오른쪽이 그 영상과 같아진다)")
    ap.add_argument("--suffix", default="")
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
    FS = 12 if a.compact else 9.5; TS = 17 if a.compact else 13
    meta = json.loads(BM.TABLE.read_text()); DS = {"ycbv": "YCB-Video", "ycbm": "YCB-M"}.get(meta.get("dataset"), "")
    table = meta["objects"]; by_id = {o["obj_id"]: o["name"] for o in table}
    model = {o["name"]: np.load(BM.PTS / f"{o['name']}.npy") / 1000.0 for o in table}
    for f in sorted(a.dump.glob("*.json")):
        s = f.stem
        if a.scenes and s not in a.scenes: continue
        D = json.loads(f.read_text())
        BM.RUNS["rt"] = a.ours; r = BM.run_memory(s, "rt", by_id)
        P, C = BM.point_cloud(s, r); c0, B, up, th = BM.top_frame(r, P)
        if a.view_frame is not None:                 # 위에서 본 시점은 그대로 두고, 그 카메라의 앞쪽이 위 · 오른쪽이 오른쪽이 되게 평면 안에서만 돌린다
            Tc = r["Twc_gt"][int(a.view_frame * (len(r["ids"]) - 1))]; f2 = B @ Tc[:3, 2]; f2 /= np.linalg.norm(f2)
            x2 = B @ Tc[:3, 0]; x2 = x2 - (x2 @ f2) * f2; x2 /= np.linalg.norm(x2); B = np.stack([x2, f2]) @ B
        to2 = lambda X: (np.asarray(X, float) - c0) @ B.T
        gT = {g["name"]: np.asarray(g["T"], float) for g in D["gt"]}
        gt_pts = {nm: model[nm] @ T[:3, :3].T + T[:3, 3] for nm, T in gT.items()}
        gcs = np.array([to2(g["center"]) for g in D["gt"]]); zc = gcs.mean(0); zh = max(0.16, np.abs(gcs - zc).max() + 0.12)
        h = (P - c0) @ up - th; uvall = to2(P)
        keep = (h > -0.01) & (h < 0.40) & (np.abs(uvall - zc).max(1) < zh)
        uv = uvall[keep]; order = np.argsort(h[keep]); gray = C[keep][order] * 0.35 + 0.6
        col = {nm: COLORS[j % len(COLORS)] for j, nm in enumerate(sorted(gT))}

        def base(ax, label_gt=False):
            ax.scatter(uv[order, 0], uv[order, 1], c=gray, s=3, lw=0, rasterized=True)
            for nm, Pg in gt_pts.items():
                hull = cv2.convexHull(to2(Pg).astype(np.float32)).reshape(-1, 2)
                ax.add_patch(Polygon(hull, closed=True, fill=label_gt, fc=col[nm], alpha=0.25 if label_gt else 1, ec=col[nm], lw=1.8, ls="--"))
                if label_gt:
                    ax.add_patch(Polygon(hull, closed=True, fill=False, ec=col[nm], lw=1.8, ls="--"))
                    ctr = to2(Pg.mean(0)); ax.plot(*ctr, "o", color=col[nm], ms=6, mec="white", mew=1)
                    ax.annotate(short(nm), ctr, xytext=(8, 8), textcoords="offset points", fontsize=FS, color=col[nm], bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.85))
                    arrow(ax, Pg.mean(0), gT[nm][:3, 0], col[nm], "--")
            ax.set_xlim(zc[0] - zh, zc[0] + zh); ax.set_ylim(zc[1] - zh, zc[1] + zh); ax.set_aspect("equal"); ax.grid(alpha=0.2); ax.set_xlabel("[m]")
            if a.compact: ax.set_xticks([]); ax.set_yticks([]); ax.set_xlabel("")

        def arrow(ax, c3, x3, c, ls):
            p0 = to2(c3); d = np.asarray(x3, float) @ B.T
            if np.linalg.norm(d) < 0.3: return                                  # x축이 거의 수직이면 위에서 볼 때 방향이 없다
            p1 = p0 + d / np.linalg.norm(d) * 0.05
            ax.annotate("", p1, p0, arrowprops=dict(arrowstyle="->", color=c, lw=1.6, ls=ls))

        fig, axs = plt.subplots(1, 4, figsize=(22, 6.1) if a.compact else (30, 8.4))
        base(axs[0], True); axs[0].set_title(f"정답\n물체 {len(gT)}개", fontsize=TS)
        for ax, m in zip(axs[1:], METHODS):
            base(ax); objs = D["methods"].get(m, []); cnt = {k: sum(o["kind"] == k for o in objs) for k in ("match", "duplicate", "wrong_name", "far", "unnamed")}
            outside, n_tag, hit = [], 0, set()
            for o in objs:
                q = to2(o["pts"]); ctr = to2(o["center"])
                if o["kind"] == "unnamed":
                    ax.scatter(q[:, 0], q[:, 1], s=2, color="#555555", lw=0, alpha=0.45, rasterized=True); continue
                c = col.get(o["name"], "#777777")
                if o["R"] is not None:
                    hull = cv2.convexHull(q.astype(np.float32)).reshape(-1, 2); ax.add_patch(Polygon(hull, closed=True, fc=c, alpha=0.35, ec=c, lw=1.8))
                    arrow(ax, o["center"], np.asarray(o["R"])[:, 0], c, "-")
                    if o["name"] in gT: arrow(ax, gt_pts[o["name"]].mean(0), gT[o["name"]][:3, 0], c, "--")
                else:
                    ax.scatter(q[:, 0], q[:, 1], s=4, color=c, lw=0, alpha=0.8, rasterized=True)
                if np.abs(ctr - zc).max() > zh:
                    outside.append(f"{TAG.get(o['kind'], '맞음')} {short(o['name'])} {np.linalg.norm(ctr - zc):.2f} m"); continue
                if o["kind"] == "match":
                    hit.add(o["name"]); ax.plot(*ctr, "o", color=c, ms=6, mec="white", mew=1)
                    txt = f"{short(o['name'])} {o['center_mm']:.0f} mm" + (f", {o['rot_deg']:.0f}°" if "rot_deg" in o else "")
                    ax.annotate(txt, ctr, xytext=(8, 8), textcoords="offset points", fontsize=FS, color=c, bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.85))
                else:
                    n_tag += 1; ax.plot(*ctr, "X", color="#b8342a", ms=12, mec="white", mew=1.2)
                    ax.annotate(f"{TAG[o['kind']]}: {short(o['name'])}", ctr, xytext=(10, -14 - 16 * (n_tag % 3)), textcoords="offset points", arrowprops=dict(arrowstyle="-", color="#b8342a", lw=0.8),
                                fontsize=FS, color="#b8342a", bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.85))
            miss = [nm for nm in gT if nm not in hit]
            for j, nm in enumerate(miss):
                ax.annotate(f"놓침: {short(nm)}", to2(gt_pts[nm].mean(0)), xytext=(0, 18 + 14 * (j % 2)), textcoords="offset points", ha="center", fontsize=FS, color="#555",
                            bbox=dict(boxstyle="round,pad=0.2", fc="#eeeeee", ec="#999", alpha=0.9))
            if a.compact:
                ax.set_title(f"{m}\n맞음 {cnt['match']}/{len(gT)} · 중복 {cnt['duplicate']} · 이름 틀림 {cnt['wrong_name']} · 먼 위치 {cnt['far']}", fontsize=TS)
            else:
              ax.set_title(f"{m}\n맞음 {cnt['match']}/{len(gT)} · 중복 {cnt['duplicate']} · 이름 틀림 {cnt['wrong_name']} · 먼 위치 {cnt['far']} · 놓침 {len(miss)} · 이름 없는 객체 {cnt['unnamed']}", fontsize=TS)
            if outside:
                ax.text(0.99, 0.01, "그림 밖 (중심에서 거리)\n" + "\n".join(outside[:8]) + ("\n…" if len(outside) > 8 else ""), transform=ax.transAxes, ha="right", va="bottom", fontsize=FS,
                        color="#b8342a", bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#b8342a", alpha=0.9))
        if a.compact:
            fig.tight_layout(pad=0.4); fig.subplots_adjust(left=0.003, right=0.997, wspace=0.015, top=0.88, bottom=0.01); fig.savefig(a.out / f"map4c_{s}{a.suffix}.png", dpi=110); plt.close(fig); print(s, flush=True); continue
        fig.suptitle(f"{DS} {s} — 위에서 본 객체 지도. 점선은 정답 윤곽, 화살표는 물체 x축(점선 정답, 실선 추정), 회색 점은 대상 물체 이름이 붙지 않은 객체", fontsize=14)
        fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(a.out / f"map4_{s}.png", dpi=80); plt.close(fig); print(s, flush=True)


if __name__ == "__main__":
    main()
