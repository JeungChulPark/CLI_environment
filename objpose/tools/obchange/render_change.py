#!/usr/bin/env python3
"""render_change.py — ObChange 두 방문(A → B)의 객체 지도를 방법별로 위에서 본 그림으로 그린다.

  행 = 방문 A(이전), 방문 B(이후) · 열 = 제안 방법, FM-Fusion, DualMap
  배경   그 방문의 깊이 영상을 데이터셋 자세로 쌓은 점군(위에서 본 색)
  정답   검은 점선 원 + 이름. 채점 대상(템플릿 11종)이 아니면 회색. 옆의 ○/× = 그 방법이 이 물체의 변화 범주를 맞혔는가
  지도   제안: CAD 모델을 추정 자세로 놓은 윤곽 + 방향 화살표(물체 x축) / FM-Fusion·DualMap: 복원한 물체 점 + 붙인 이름
  색     방법이 판정한 범주 — 그대로(회색) · 옮겨짐(파랑) · 사라짐(빨강) · 새로 생김(초록)
물체 없는 방문(<room>_s1)에도 있는 객체는 배경으로 빼고 그린다(채점의 --bg 와 같음).
    <sam6d python> objpose/tools/obchange/render_change.py --maps <out>/maps --pair kitchen_s2 kitchen_s4 --out <out>/figs
"""
import argparse, json, sys
from pathlib import Path
import cv2, numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from scipy.spatial import ConvexHull

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from change_eval import OB, CATALOG, MATCH, dist, diff_by_name, drop_bg   # noqa: E402

PTS = REPO / "sam6d_realtime/assets/model_points"
KO = {"002_master_chef_can": "커피 캔", "003_cracker_box": "크래커 상자", "004_sugar_box": "설탕 상자", "005_tomato_soup_can": "수프 캔",
      "008_pudding_box": "푸딩 상자", "009_gelatin_box": "젤라틴 상자", "019_pitcher_base": "피처", "024_bowl": "그릇", "025_mug": "머그",
      "040_large_marker": "마커", "061_foam_brick": "폼 브릭", "027_skillet": "프라이팬", "029_plate": "접시"}
COL = {"static": "#7f7f7f", "moved": "#1f6fb2", "removed": "#b8342a", "novel": "#3a9d5d"}
CAT_KO = {"static": "그대로", "moved": "옮겨짐", "removed": "사라짐", "novel": "새로 생김"}
METHODS = [("제안 방법 (Object Pose Map)", "proposed"), ("FM-Fusion", "fmfusion"), ("DualMap", "dualmap")]
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "font.size": 10})


def cloud(scene, stride=20, pix=5):
    sc = OB / "test" / scene
    cam = json.loads((sc / "scene_camera.json").read_text()); ids = sorted(cam, key=int)
    K = np.array(cam[ids[0]]["cam_K"]).reshape(3, 3)
    v, u = np.mgrid[0:480:pix, 0:640:pix]; P, C = [], []
    for k in ids[::stride]:
        d = cv2.imread(str(sc / "depth" / f"{int(k):06d}.png"), cv2.IMREAD_UNCHANGED)[::pix, ::pix].astype(np.float32) * 1e-4
        rgb = cv2.imread(str(sc / "rgb" / f"{int(k):06d}.png"))[::pix, ::pix, ::-1]
        m = (d > 0.3) & (d < 2.0)
        z = d[m]; x = (u[m] - K[0, 2]) * z / K[0, 0]; y = (v[m] - K[1, 2]) * z / K[1, 1]
        R = np.array(cam[k]["cam_R_w2c"]).reshape(3, 3); t = np.array(cam[k]["cam_t_w2c"]) / 1000.0
        P.append((np.c_[x, y, z] - t) @ R); C.append(rgb[m] / 255.0)
    P, C = np.concatenate(P), np.concatenate(C)
    _, idx = np.unique(np.floor(P / 0.01).astype(np.int64), axis=0, return_index=True)
    return P[idx], C[idx]


def gt_all(scene):
    return [{"name": o["ycb"], "center": o["center"]} for o in json.loads((OB / "gt" / f"{scene}.json").read_text())["objects"] if o.get("center")]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps", type=Path, required=True); ap.add_argument("--pair", nargs=2, required=True)
    ap.add_argument("--out", type=Path, required=True); ap.add_argument("--no-bg", action="store_true")
    a = ap.parse_args()
    A, B = a.pair; room = A.split("_s")[0]
    table = {o["ycb"]: o["name"] for o in json.loads((REPO / "objpose/rt/bop/obchange_objects.json").read_text())["objects"]}
    model = {y: np.load(PTS / f"{n}.npy") / 1000.0 for y, n in table.items()}
    G = {A: gt_all(A), B: gt_all(B)}
    Gc = {s: [g for g in G[s] if g["name"] in CATALOG] for s in (A, B)}
    gca, gcb = diff_by_name(Gc[A], Gc[B]); gcat = {A: gca, B: gcb}
    allc = np.array([g["center"] for s in (A, B) for g in G[s]])
    # 조리대·책상이 가로로 눕도록 방 전체 정답 물체의 주축으로 그림을 돌린다
    roomc = np.array([o["center"] for f in sorted((OB / "gt").glob(f"{room}_s*.json")) for o in json.loads(f.read_text())["objects"] if o.get("center")])
    Q = np.linalg.svd(roomc[:, :2] - roomc[:, :2].mean(0), full_matrices=False)[2]
    if np.linalg.det(Q) < 0: Q[1] *= -1
    rot = lambda xy: np.asarray(xy)[..., :2] @ Q.T
    ac = rot(allc); lo, hi = ac.min(0) - 0.30, ac.max(0) + 0.30
    zt = float(np.median(allc[:, 2]))
    clouds = {s: cloud(s) for s in (A, B)}
    asp = (hi[1] - lo[1]) / (hi[0] - lo[0]); fig, axes = plt.subplots(2, 3, figsize=(17, 17 / 3 * asp * 2 + 1.9))
    for j, (title, m) in enumerate(METHODS):
        md = a.maps / m
        M = {s: json.loads((md / f"{s}.json").read_text()) for s in (A, B)}
        bgf = md / f"{room}_s1.json"
        bg = json.loads(bgf.read_text()) if bgf.exists() and not a.no_bg else []
        M = {s: drop_bg(M[s], bg) for s in (A, B)}
        ca, cb = diff_by_name(M[A], M[B]); mcat = {A: ca, B: cb}
        n_ok = n_gt = 0
        for i, s in enumerate((A, B)):
            ax = axes[i, j]
            P, C = clouds[s]; keep = (P[:, 2] > zt - 0.25) & (P[:, 2] < zt + 0.6)
            pk = rot(P[keep]); ax.scatter(pk[:, 0], pk[:, 1], c=C[keep], s=1.2, alpha=0.55, linewidths=0, zorder=1)
            kk = 0
            for o, c in zip(M[s], mcat[s]):
                c3 = np.array(o["center"]); ctr = rot(c3)
                if not (lo[0] < ctr[0] < hi[0] and lo[1] < ctr[1] < hi[1]):
                    continue
                if o["name"] is None:
                    if "pts" in o:
                        q = rot(np.array(o["pts"])); ax.scatter(q[:, 0], q[:, 1], s=1.5, c="#c9c9c9", alpha=0.5, linewidths=0, zorder=2)
                    continue
                col = COL[c]
                if "R" in o and o["name"] in model:
                    R = np.array(o["R"]); q = rot(model[o["name"]] @ R.T + c3)
                    hull = q[ConvexHull(q).vertices]
                    ax.add_patch(Polygon(hull, closed=True, fc=col, ec=col, alpha=0.45, lw=1.5, zorder=3))
                    ax.annotate("", xy=ctr + rot(R[:, 0]) * 0.09, xytext=ctr, arrowprops=dict(arrowstyle="-|>", color=col, lw=1.8), zorder=5)
                    lab = KO.get(o["name"], o["name"])
                else:
                    q = rot(np.array(o.get("pts", [c3]))); ax.scatter(q[:, 0], q[:, 1], s=2.5, c=col, alpha=0.7, linewidths=0, zorder=3)
                    lab = KO.get(o["name"], o.get("label", "?"))
                kk += 1
                ax.text(ctr[0], ctr[1] - 0.075 - 0.07 * (kk % 2), f"{lab}·{CAT_KO[c]}", color=col, fontsize=8, ha="center", va="top", zorder=6,
                        bbox=dict(fc="white", ec="none", alpha=0.75, pad=0.8))
            for gi, g in enumerate(sorted(G[s], key=lambda g_: rot(np.array(g_["center"]))[0])):
                c3 = np.array(g["center"]); ctr = rot(c3); inc = g["name"] in CATALOG
                ax.add_patch(Circle(ctr, 0.07, fill=False, ec="black" if inc else "#999999", ls="--", lw=1.4, zorder=4))
                mark = ""
                if inc:
                    k = gcat[s][Gc[s].index(g)]; n_gt += 1
                    c = [(dist(c3, o["center"]), ii) for ii, o in enumerate(M[s]) if o["name"] == g["name"] and dist(c3, o["center"]) < MATCH]
                    ok = bool(c) and mcat[s][min(c)[1]] == k; n_ok += ok
                    ax.text(ctr[0], ctr[1] + 0.085 + 0.075 * (gi % 2), f"{KO.get(g['name'], g['name'])} ({CAT_KO[k]})", color="black", fontsize=8.5, ha="center", va="bottom", zorder=7, fontweight="bold",
                            bbox=dict(fc="white", ec="none", alpha=0.7, pad=0.6))
                    ax.text(ctr[0] + 0.078, ctr[1], "○" if ok else "×", color="#2e7d32" if ok else "#c62828", fontsize=17, ha="left", va="center", zorder=8, fontweight="bold")
                else:
                    ax.text(ctr[0], ctr[1] + 0.085, f"{KO.get(g['name'], g['name'])} (채점 제외)", color="#888888", fontsize=8, ha="center", va="bottom", zorder=7)
            ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
            ax.set_ylabel(("이전 방문" if i == 0 else "이후 방문") + f"  ({s})", fontsize=11) if j == 0 else None
        axes[0, j].set_title(f"{title}\n변화 범주 맞음 {n_ok}/{n_gt}", fontsize=13, fontweight="bold")
    handles = [plt.Line2D([], [], marker="s", ls="", color=COL[k], label=f"지도 객체: {CAT_KO[k]}", markersize=9) for k in COL]
    handles += [plt.Line2D([], [], ls="--", color="black", label="정답 물체 (이름과 정답 범주)"), plt.Line2D([], [], ls="", marker="$○$", color="#2e7d32", label="범주를 맞힘", markersize=11),
                plt.Line2D([], [], ls="", marker="x", color="#c62828", label="못 찾았거나 범주가 틀림", markersize=9, markeredgewidth=2)]
    fig.legend(handles=handles, loc="lower center", ncol=7, frameon=False, fontsize=10.5)
    fig.suptitle(f"ObChange {room} — 두 방문 사이의 물체 변화를 객체 지도로 판정 (위에서 본 모습, 그대로/옮겨짐 경계 20 cm)", fontsize=14)
    fig.tight_layout(rect=(0, 0.05, 1, 0.95))
    a.out.mkdir(parents=True, exist_ok=True)
    out = a.out / f"change_{A}__{B}.png"; fig.savefig(out, dpi=130); print(out)


if __name__ == "__main__":
    main()
