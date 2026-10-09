"""Figures of docs/데이터_방법_소개.md: YCB-V objects, one sequence, its GT map, method pipelines."""
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch
from PIL import Image

from common import GT_DIR, HERE, PROMPTS, YCB_NAMES, YCBV, load_map

font_manager.fontManager.addfont(str(__import__("pathlib").Path.home() / ".local/share/fonts/malgun.ttf"))
plt.rcParams["font.family"] = "Malgun Gothic"
plt.rcParams["axes.unicode_minus"] = False
DOCS = HERE / "docs"
SEQ = YCBV / "full" / "test"
CMAP = plt.get_cmap("tab20")


def load_scene(s):
    d = SEQ / f"{s:06d}"
    return d, json.load(open(d / "scene_gt.json")), json.load(open(d / "scene_gt_info.json"))


def fig_objects():
    """Best-visible crop of every object across the 12 sequences."""
    best = {}
    for s in range(48, 60):
        d, gt, info = load_scene(s)
        for f in list(gt)[::25]:
            for k, (o, i) in enumerate(zip(gt[f], info[f])):
                sc = i["visib_fract"] * min(i["px_count_visib"], 20000)
                if sc > best.get(o["obj_id"], (0,))[0]:
                    best[o["obj_id"]] = (sc, d, int(f), i["bbox_visib"])
    fig, axs = plt.subplots(3, 7, figsize=(15, 7.2))
    for oid, ax in zip(range(1, 22), axs.flat):
        ax.axis("off")
        if oid not in best:
            continue
        _, d, f, (x, y, w, h) = best[oid]
        m = int(0.15 * max(w, h))
        im = Image.open(d / "rgb" / f"{f:06d}.png").crop((x - m, y - m, x + w + m, y + h + m))
        ax.imshow(im)
        ax.set_title(f"{oid}. {YCB_NAMES[oid - 1][4:]}\n“{PROMPTS[oid - 1]}”", fontsize=8.5)
    fig.suptitle("YCB-V 대상 물체 21종 (번호. 이름 / 인식 프롬프트)", fontsize=13)
    fig.tight_layout()
    fig.savefig(DOCS / "ycbv_objects.png", dpi=110)


def fig_sequence(s=48):
    d, gt, info = load_scene(s)
    frames = sorted(gt, key=int)
    pick = [frames[int(r * (len(frames) - 1))] for r in (0, 0.35, 0.7, 1.0)]
    fig, axs = plt.subplots(3, 4, figsize=(15, 8.4))
    for c, f in enumerate(pick):
        f = int(f)
        rgb = np.array(Image.open(d / "rgb" / f"{f:06d}.png"))
        dep = np.array(Image.open(d / "depth" / f"{f:06d}.png")) * 0.1   # mm
        over = rgb.astype(float) / 255
        for k, o in enumerate(gt[str(f)]):
            m = np.array(Image.open(d / "mask_visib" / f"{f:06d}_{k:06d}.png")) > 0
            over[m] = 0.45 * over[m] + 0.55 * np.array(CMAP(o["obj_id"] % 20)[:3])
        axs[0, c].imshow(rgb)
        axs[0, c].set_title(f"프레임 {f} / {len(frames)}", fontsize=10)
        axs[1, c].imshow(np.where(dep > 0, dep, np.nan), cmap="turbo", vmin=500, vmax=1300)
        axs[2, c].imshow(over)
        for a in axs[:, c]:
            a.axis("off")
    for r, t in enumerate(["RGB", "깊이 (0.5–1.3 m)", "물체별 정답 마스크"]):
        axs[r, 0].text(-0.06, 0.5, t, transform=axs[r, 0].transAxes, rotation=90, va="center",
                       ha="right", fontsize=11)
    names = ", ".join(f"{o['obj_id']}.{YCB_NAMES[o['obj_id'] - 1][4:]}" for o in gt[frames[0]])
    fig.suptitle(f"YCB-V 장면 {s:06d}: 손에 든 카메라로 탁상 위 물체를 돌며 촬영 ({names})", fontsize=12)
    fig.tight_layout()
    fig.savefig(DOCS / "ycbv_sequence_000048.png", dpi=100)


def fig_gt_map(s=48):
    _, gt = load_map(GT_DIR / "ycbv_vis" / f"{s:06d}")
    _, gtf = load_map(GT_DIR / "ycbv" / f"{s:06d}")
    cams = json.load(open(GT_DIR / "ycbv" / f"{s:06d}" / "cameras.json"))
    Ts = np.array([fr["T_wc"] for fr in cams["frames"]])
    # display frame: z up = mean image-up direction of the cameras, origin at the objects
    up = -Ts[:, :3, 1].mean(0)
    up /= np.linalg.norm(up)
    x = np.cross(up, [0, 0, 1.0]) if abs(up[2]) < 0.9 else np.cross(up, [1.0, 0, 0])
    x /= np.linalg.norm(x)
    R = np.stack([x, np.cross(up, x), up])
    o0 = np.concatenate([o["points"] for o in gtf]).mean(0)
    tf = lambda P: (P - o0) @ R.T
    C = tf(Ts[:, :3, 3])
    cols = plt.get_cmap("tab10")
    fig = plt.figure(figsize=(15, 6.5))
    for i, (title, objs) in enumerate([("① 카메라 궤적(회색)과 정답 지도 (전체 CAD 표면)", gtf),
                                       ("② 실제로 보인 표면만 남긴 정답 (평가 기본값, 확대)", gt)]):
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        hs = []
        for k, o in enumerate(objs):
            P = tf(o["points"][::3])
            ax.scatter(*P.T, s=0.8, color=cols(k))
            hs.append(plt.Line2D([], [], ls="", marker="o", color=cols(k),
                                 label=f"{o['label']}.{YCB_NAMES[o['label'] - 1][4:]}"))
        if i == 0:
            ax.plot(*C.T, color="gray", lw=0.7)
            ax.scatter(*C[0], color="green", s=25)
            ax.scatter(*C[-1], color="red", s=25)
            hs += [plt.Line2D([], [], ls="", marker="o", color="green", label="카메라 시작"),
                   plt.Line2D([], [], ls="", marker="o", color="red", label="카메라 끝")]
            pts = np.concatenate([tf(o["points"]) for o in gtf] + [C])
        else:
            pts = np.concatenate([tf(o["points"]) for o in gtf])
        ctr, r = (pts.max(0) + pts.min(0)) / 2, (pts.max(0) - pts.min(0)).max() / 2
        ax.set_xlim(ctr[0] - r, ctr[0] + r); ax.set_ylim(ctr[1] - r, ctr[1] + r); ax.set_zlim(ctr[2] - r, ctr[2] + r)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=35 if i == 0 else 25, azim=-60)
        ax.set_title(title, fontsize=11)
        ax.set_xticklabels([]); ax.set_yticklabels([]); ax.set_zticklabels([])
        ax.legend(handles=hs, fontsize=8, loc="upper left")
    fig.suptitle(f"YCB-V 장면 {s:06d} 정답 객체 지도 ({len(Ts)}프레임, 물체 {len(gtf)}개)", fontsize=12)
    fig.tight_layout()
    fig.savefig(DOCS / "ycbv_gt_map_000048.png", dpi=100)


def fig_methods():
    rows = [
        ("제안 방법", "#d62728",
         ["RGB-D\n+ 카메라 포즈", "YOLO-World\n(고정 프롬프트)", "SAM-6D\nISM·PEM\n(6D 자세)", "자세 검증\n(관문)", "객체 지도\n(CAD를 자세에\n등록·갱신)"]),
        ("FM-Fusion\n(RA-L 2024)", "#1f77b4",
         ["RGB-D\n+ 카메라 포즈", "RAM 태깅\n→ GroundingDINO\n→ SAM", "확률적\n라벨 융합\n(닫힌 클래스)", "인스턴스 정리\n(과분할 병합)", "인스턴스별\n복셀 지도\n(1.5 cm) + TSDF"]),
        ("DualMap\n(RA-L 2025)", "#2ca02c",
         ["RGB-D\n+ 카메라 포즈", "YOLO-World +\nMobileSAM\n(+FastSAM)", "MobileCLIP\n특징", "객체 상태 확인\n(안정성·분리)", "구체 지도(점군)\n+ 추상 지도\n(기준 물체)"]),
    ]
    fig, ax = plt.subplots(figsize=(15, 6))
    ax.set_xlim(0, 15); ax.set_ylim(0, 6.2); ax.axis("off")
    for r, (name, col, steps) in enumerate(rows):
        y = 5.0 - r * 2.0
        ax.text(0.9, y, name, ha="center", va="center", fontsize=12, color=col, weight="bold")
        for i, st in enumerate(steps):
            x = 2.2 + i * 2.6
            ax.add_patch(FancyBboxPatch((x, y - 0.7), 2.1, 1.4, boxstyle="round,pad=0.05",
                                        fc=col if i == len(steps) - 1 else "white", ec=col, lw=1.5,
                                        alpha=0.9 if i == len(steps) - 1 else 1))
            ax.text(x + 1.05, y, st, ha="center", va="center", fontsize=9.5,
                    color="white" if i == len(steps) - 1 else "black")
            if i:
                ax.annotate("", (x, y), (x - 0.5, y), arrowprops=dict(arrowstyle="->", color="gray"))
    ax.text(7.5, 6.05, "세 방법 모두 같은 RGB-D와 같은 카메라 포즈를 받고, 객체 지도를 공통 형식으로 내보낸다",
            ha="center", fontsize=11)
    fig.tight_layout()
    fig.savefig(DOCS / "methods_pipeline.png", dpi=100)


if __name__ == "__main__":
    import sys
    for name in sys.argv[1:] or ["objects", "sequence", "gt_map", "methods"]:
        globals()[f"fig_{name}"]()
    print("ok")
