#!/usr/bin/env python3
"""render_visits.py — ObChange 를 눈으로 확인하는 그림.

  (1) <out>/layout_<room>.png        방의 모든 방문을 위에서 본 정답 배치 (방문마다 한 칸) + 방문마다 정답을 표시한 카메라 영상 한 장
  (2) <out>/visit_<scene>.png        같은 방문에서 정답 | 제안 방법 | FM-Fusion | DualMap 의 객체 지도를 나란히
        초록 = 정답과 같은 이름으로 10 cm 안,  주황 = 정답 자리에 있으나 이름이 다름,  빨강 = 정답이 없는 자리(대상 물체 이름),  회색 = 이름 없는 조각
        정답 물체 옆 ○/× = 그 방법이 이 물체를 맞게 등록했는가.  회색 점선 정답 = 채점 제외(템플릿 없는 물체)
    <sam6d python> objpose/tools/obchange/render_visits.py --maps <out>/maps --room kitchen --out <out>/figs_visits
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
from change_eval import OB, CATALOG, MATCH, dist   # noqa: E402
from render_change import cloud, KO as KO0, PTS   # noqa: E402

KO = dict(KO0, **{"001_chips_can": "감자칩 통", "014_lemon": "레몬", "015_peach": "복숭아", "016_pear": "배", "043_phillips_screwdriver": "십자 드라이버",
                  "044_flat_screwdriver": "일자 드라이버", "053_mini_soccer_ball": "축구공", "055_baseball": "야구공", "056_tennis_ball": "테니스공",
                  "076_timer": "타이머", "077_rubiks_cube": "큐브", "072-a_toy_airplane": "장난감 부품", "072-k_toy_airplane": "장난감 부품"})
ko = lambda n: KO.get(n, "컵" if n and n.startswith("065") else (n or "?"))
METHODS = [("제안 방법", "proposed"), ("FM-Fusion", "fmfusion"), ("DualMap", "dualmap")]
G, O_, R_, U = "#2e7d32", "#e08a1e", "#c62828", "#b9b9b9"
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.unicode_minus": False, "font.size": 10})


def gt_all(scene):
    return [{"name": o["ycb"], "center": o["center"]} for o in json.loads((OB / "gt" / f"{scene}.json").read_text())["objects"] if o.get("center")]


def best_frame(scene, gts):
    """정답 물체가 가장 많이 화면에 들어오는 프레임에 정답 위치와 이름을 그린다."""
    from PIL import Image, ImageDraw, ImageFont
    F = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 16)
    cam = json.loads((OB / "test" / scene / "scene_camera.json").read_text()); best = (-1, None, None)
    for k in sorted(cam, key=int)[::5]:
        K = np.array(cam[k]["cam_K"]).reshape(3, 3); R = np.array(cam[k]["cam_R_w2c"]).reshape(3, 3); t = np.array(cam[k]["cam_t_w2c"]) / 1000
        pts = []
        for g in gts:
            p = R @ np.array(g["center"]) + t
            if p[2] > 0.25:
                u, v = K[0, 0] * p[0] / p[2] + K[0, 2], K[1, 1] * p[1] / p[2] + K[1, 2]
                if 20 < u < 620 and 20 < v < 460: pts.append((u, v, g["name"]))
        if len(pts) > best[0]: best = (len(pts), k, pts)
    k = best[1] or sorted(cam, key=int)[len(cam) // 2]
    im = Image.fromarray(cv2.imread(str(OB / "test" / scene / "rgb" / f"{int(k):06d}.png"))[:, :, ::-1]); d = ImageDraw.Draw(im)
    for u, v, n in best[2] or []:
        c = (30, 110, 30) if n in CATALOG else (110, 110, 110)
        d.ellipse([u - 9, v - 9, u + 9, v + 9], outline=c, width=3); w = d.textlength(ko(n), font=F)
        d.rectangle([u + 11, v - 11, u + 16 + w, v + 10], fill=(255, 255, 255)); d.text((u + 13, v - 12), ko(n), fill=c, font=F)
    return np.asarray(im)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps", type=Path, required=True); ap.add_argument("--room", required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
    room = a.room
    scenes = sorted((p.stem for p in (OB / "gt").glob(f"{room}_s*.json")), key=lambda s: int(s.split("_s")[1]))
    scenes = [s for s in scenes if gt_all(s)]
    table = {o["ycb"]: o["name"] for o in json.loads((REPO / "objpose/rt/bop/obchange_objects.json").read_text())["objects"]}
    model = {y: np.load(PTS / f"{n}.npy") / 1000.0 for y, n in table.items()}
    GT = {s: gt_all(s) for s in scenes}
    roomc = np.array([g["center"] for s in scenes for g in GT[s]])
    Q = np.linalg.svd(roomc[:, :2] - roomc[:, :2].mean(0), full_matrices=False)[2]
    if np.linalg.det(Q) < 0: Q[1] *= -1
    rot = lambda xy: np.asarray(xy)[..., :2] @ Q.T
    rc = rot(roomc); lo, hi = rc.min(0) - 0.35, rc.max(0) + 0.35; asp = (hi[1] - lo[1]) / (hi[0] - lo[0]); zt = float(np.median(roomc[:, 2]))
    clouds = {s: cloud(s) for s in scenes}

    def base(ax, s):
        P, C = clouds[s]; keep = (P[:, 2] > zt - 0.3) & (P[:, 2] < zt + 0.7); pk = rot(P[keep])
        ax.scatter(pk[:, 0], pk[:, 1], c=C[keep], s=1.2, alpha=0.5, linewidths=0, zorder=1)
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])

    def draw_gt(ax, s, marks=None, big=False):
        for gi, g in enumerate(sorted(GT[s], key=lambda g_: rot(np.array(g_["center"]))[0])):
            c = rot(np.array(g["center"])); inc = g["name"] in CATALOG
            ax.add_patch(Circle(c, 0.07, fill=False, ec="black" if inc else "#8a8a8a", ls="--", lw=1.5, zorder=6))
            ax.text(c[0], c[1] + 0.085 + 0.08 * (gi % 2), ko(g["name"]), color="black" if inc else "#777777", fontsize=10 if big else 8.5, ha="center", va="bottom", zorder=8,
                    fontweight="bold" if inc else "normal", bbox=dict(fc="white", ec="none", alpha=0.75, pad=0.5))
            if marks is not None and inc:
                ok = marks[id(g)]
                ax.text(c[0] + 0.08, c[1], "○" if ok else "×", color=G if ok else R_, fontsize=17, ha="left", va="center", zorder=9, fontweight="bold")

    # (1) 방 전체 배치
    n = len(scenes); fw = 4.6
    fig, axes = plt.subplots(2, n, figsize=(fw * n, fw * asp + fw * 0.75 + 0.9), gridspec_kw={"height_ratios": [fw * asp, fw * 0.75]})
    for j, s in enumerate(scenes):
        base(axes[0, j], s); draw_gt(axes[0, j], s, big=True)
        axes[0, j].set_title(f"{int(s.split('_s')[1])}번째 방문 — 물체 {len(GT[s])}개 (채점 대상 {sum(g['name'] in CATALOG for g in GT[s])}개)", fontsize=12, fontweight="bold")
        axes[1, j].imshow(best_frame(s, GT[s])); axes[1, j].axis("off")
    fig.suptitle(f"ObChange {room} — 방문마다의 정답 물체 배치 (위: 위에서 본 모습, 아래: 카메라 영상에 정답 위치 표시. 검정 = 채점 대상 11종, 회색 = 제외)", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(a.out / f"layout_{room}.png", dpi=110); plt.close(fig); print(a.out / f"layout_{room}.png")

    # (2) 방문별 비교
    for s in scenes:
        pw = 9.0 if asp < 0.6 else 6.0
        fig, axes = plt.subplots(2, 2, figsize=(2 * pw, 2 * (pw * asp + 0.55) + 1.1)); axes = axes.ravel()
        base(axes[0], s); draw_gt(axes[0], s, big=True); axes[0].set_title("정답", fontsize=14, fontweight="bold")
        for j, (title, m) in enumerate(METHODS, 1):
            ax = axes[j]; base(ax, s); f = a.maps / m / f"{s}.json"
            if not f.exists():
                ax.set_title(f"{title}\n(결과 없음)", fontsize=13); continue
            M = json.loads(f.read_text()); gc = [g for g in GT[s] if g["name"] in CATALOG]; marks = {}; used = set(); cnt = {"ok": 0, "wrong": 0, "false": 0}
            for g in gc:
                c = sorted((dist(g["center"], o["center"]), i) for i, o in enumerate(M) if o["name"] == g["name"] and dist(g["center"], o["center"]) < MATCH)
                marks[id(g)] = bool(c)
                if c: used.add(c[0][1]); cnt["ok"] += 1
            kk = 0
            for i, o in enumerate(M):
                c3 = np.array(o["center"]); ctr = rot(c3)
                if not (lo[0] < ctr[0] < hi[0] and lo[1] < ctr[1] < hi[1]): continue
                if o["name"] not in CATALOG:
                    if "pts" in o: q = rot(np.array(o["pts"])); ax.scatter(q[:, 0], q[:, 1], s=1.5, c=U, alpha=0.5, linewidths=0, zorder=2)
                    continue
                near = [g for g in GT[s] if dist(g["center"], o["center"]) < MATCH]
                col = G if i in used else (O_ if near else R_)
                if i not in used: cnt["wrong" if near else "false"] += 1
                if "R" in o and o["name"] in model:
                    Rm = np.array(o["R"]); q = rot(model[o["name"]] @ Rm.T + c3); hull = q[ConvexHull(q).vertices]
                    ax.add_patch(Polygon(hull, closed=True, fc=col, ec=col, alpha=0.5, lw=1.5, zorder=3))
                    ax.annotate("", xy=ctr + rot(Rm[:, 0]) * 0.09, xytext=ctr, arrowprops=dict(arrowstyle="-|>", color=col, lw=1.8), zorder=5)
                else:
                    q = rot(np.array(o.get("pts", [c3]))); ax.scatter(q[:, 0], q[:, 1], s=3, c=col, alpha=0.75, linewidths=0, zorder=3)
                kk += 1
                ax.text(ctr[0], ctr[1] - 0.08 - 0.075 * (kk % 2), ko(o["name"]), color=col, fontsize=8.5, ha="center", va="top", zorder=7, bbox=dict(fc="white", ec="none", alpha=0.75, pad=0.5))
            draw_gt(ax, s, marks)
            ax.set_title(f"{title}\n맞음 {cnt['ok']}/{len(gc)} · 이름 틀림 {cnt['wrong']} · 없는 물체 {cnt['false']}", fontsize=13, fontweight="bold")
        h = [plt.Line2D([], [], marker="s", ls="", color=c, label=l, markersize=9) for c, l in ((G, "정답과 같은 이름, 10 cm 안"), (O_, "정답 자리, 다른 이름"), (R_, "정답 없는 자리"), (U, "이름 없는 조각"))]
        h += [plt.Line2D([], [], ls="--", color="black", label="정답 (검정 = 채점 대상, 회색 = 제외)")]
        fig.legend(handles=h, loc="lower center", ncol=5, frameon=False, fontsize=10.5)
        fig.suptitle(f"ObChange {room} · {int(s.split('_s')[1])}번째 방문 — 같은 장면에서 세 방법의 객체 지도", fontsize=14)
        fig.tight_layout(rect=(0, 0.045, 1, 0.95), h_pad=3.0); fig.savefig(a.out / f"visit_{s}.png", dpi=110); plt.close(fig)
    print(room, "visits", len(scenes))


if __name__ == "__main__":
    main()
