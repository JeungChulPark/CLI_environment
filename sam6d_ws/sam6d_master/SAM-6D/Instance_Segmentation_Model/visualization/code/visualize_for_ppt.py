"""
visualize_for_ppt.py
====================
SAM-6D PPT용 4-panel 시각화 스크립트

Panel 1 : ISM DINOv2 (ViT-L/14) 입력→출력 변환 과정
Panel 2 : PEM ViT-Base  레이어 3/6/9/12 Concat → Linear → Reshape → Bilinear 업스케일
Panel 3 : Geometric Transformer Self-Attention에 의한 포인트 위치 업데이트
Panel 4 : Sparse(듬성) → Dense(촘촘) 포인트 클라우드 정교화 과정

실행:  conda activate sam_6d && python visualize_for_ppt.py
출력:  ../image/vis_ppt_panel{1..4}.png  +  ../image/vis_ppt_all.png
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle
from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import font_manager as fm
from scipy.ndimage import gaussian_filter
import os

# ── Korean-capable font fallback ──────────────────────────────────────────
def _find_korean_font():
    candidates = ["Malgun Gothic", "NanumGothic", "HCR Dotum", "gulim"]
    for name in candidates:
        if any(f.name == name for f in fm.fontManager.ttflist):
            return name
    return "DejaVu Sans"   # fallback (Korean glyphs will be missing but no crash)

KO_FONT = _find_korean_font()

def _has_korean(s):
    return any("\uAC00" <= c <= "\uD7A3" or "\u3130" <= c <= "\u318F" for c in s)

# ─────────────────────────── colour palette ────────────────────────────────
BG      = "#0D1117"
PANEL   = "#161B22"
BORDER  = "#30363D"
BLUE    = "#58A6FF"
GREEN   = "#3FB950"
ORANGE  = "#F0883E"
PURPLE  = "#BC8CFF"
YELLOW  = "#E3B341"
RED     = "#FF7B72"
TEAL    = "#39D353"
PINK    = "#F778BA"
WHITE   = "#E6EDF3"
GRAY    = "#8B949E"
DGRAY   = "#21262D"

OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "image")
os.makedirs(OUT_DIR, exist_ok=True)

RNG = np.random.default_rng(0)

# ─────────────────────────── low-level helpers ─────────────────────────────

def fbox(ax, x, y, w, h, fc, ec=None, lw=1.2, alpha=0.90, radius="round,pad=0", zorder=3):
    patch = FancyBboxPatch((x, y), w, h, boxstyle=radius,
                           facecolor=fc, edgecolor=ec or fc,
                           linewidth=lw, alpha=alpha,
                           transform=ax.transAxes, zorder=zorder)
    ax.add_patch(patch)
    return patch

def txt(ax, x, y, s, size=8, color=WHITE, weight="normal", ha="center", va="center",
        zorder=6, family="monospace"):
    font = KO_FONT if _has_korean(s) else family
    ax.text(x, y, s, transform=ax.transAxes,
            fontsize=size, color=color, fontweight=weight,
            ha=ha, va=va, fontfamily=font, zorder=zorder)

def arr(ax, x0, y0, x1, y1, color=GRAY, lw=1.5, rad=0.0, style="-|>", ms=10, zorder=5):
    ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                xycoords="axes fraction", textcoords="axes fraction",
                arrowprops=dict(arrowstyle=style, color=color, lw=lw,
                                mutation_scale=ms,
                                connectionstyle=f"arc3,rad={rad}"),
                zorder=zorder)

def panel_bg(ax, title=""):
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_facecolor(BG)
    fbox(ax, 0.01, 0.01, 0.98, 0.98, PANEL, ec=BORDER, lw=1.5, alpha=1.0, zorder=0)
    if title:
        txt(ax, 0.5, 0.955, title, size=10, weight="bold", color=WHITE, zorder=7)


# ═══════════════════════════════════════════════════════════════════════════
# PANEL 1 – DINOv2 ViT-L/14  입력 → 출력 변환 과정
# ═══════════════════════════════════════════════════════════════════════════

def draw_panel1(ax):
    from PIL import Image as PILImage

    panel_bg(ax, "[1] ISM  DINOv2 ViT-L/14  –  Input → Feature Extraction → Output")

    # ── load real images ─────────────────────────────────────────────────
    _DATA = "d:/LDH_ws/sam_6d/SAM-6D/Data/Example/outputs/templates"
    img_rgb  = np.array(PILImage.open(f"{_DATA}/rgb_0.png").convert("RGB").resize((224, 224)))
    img_mask = np.array(PILImage.open(f"{_DATA}/mask_0.png").convert("L").resize((224, 224)))
    img_masked = img_rgb.copy()
    img_masked[img_mask < 128] = 0        # background → black

    # patch-grid overlay on masked image
    img_grid = img_masked.copy().astype(float)
    for gi in range(0, 224, 14):          # every 14px draw a teal line
        img_grid[gi, :, 0] = 57
        img_grid[gi, :, 1] = 211
        img_grid[gi, :, 2] = 83
        img_grid[:, gi, 0] = 57
        img_grid[:, gi, 1] = 211
        img_grid[:, gi, 2] = 83
    img_grid = np.clip(img_grid, 0, 255).astype(np.uint8)

    # attention heatmap (simulated 14×14)
    heat = RNG.random((14, 14))
    heat = gaussian_filter(heat, sigma=1.5)
    # shape the heat towards object region using mask downsampled
    mask_small = np.array(
        PILImage.fromarray(img_mask).resize((14, 14), PILImage.NEAREST)) / 255.0
    heat = heat * 0.4 + mask_small * 0.6
    heat = (heat - heat.min()) / (heat.ptp() + 1e-9)

    # ── layout constants ─────────────────────────────────────────────────
    # by=0.38 → boxes span 0.38–0.48, thumbnails 0.52–0.73, labels ~0.76
    # notes at 0.26 → total vertical range ≈ 0.26–0.78, centred in panel
    bw, bh, by   = 0.155, 0.10, 0.36
    thumb_h      = 0.21
    thumb_gap    = 0.04   # gap between box top and thumbnail bottom

    stages = [
        ("RGB\nImage",              0.030, "#1F3A5F", BLUE),
        ("Masked\nImage",           0.215, "#3B2000", ORANGE),
        ("Patch Embed\n(14×14)",    0.400, "#27004A", PURPLE),
        ("×24 Transformer\nBlocks", 0.585, "#003020", GREEN),
        ("Output\nTokens",          0.770, "#1A1A00", YELLOW),
    ]

    # ── stage boxes ───────────────────────────────────────────────────────
    for label, bx, fc, ec in stages:
        fbox(ax, bx, by, bw, bh, fc, ec=ec, lw=1.8, zorder=3)
        txt(ax, bx + bw / 2, by + bh / 2, label, size=8, weight="bold",
            color=WHITE, zorder=5)

    # arrows between boxes
    for i in range(len(stages) - 1):
        arr(ax, stages[i][1] + bw, by + bh / 2,
            stages[i+1][1],        by + bh / 2, color=GRAY, lw=1.5)

    # ── thumbnails (real images via inset_axes) ───────────────────────────
    ty = by + bh + thumb_gap    # thumbnail bottom y
    pad = 0.005                  # small horizontal padding inside column

    def _thumb(data, lx, cmap=None, border=WHITE):
        ia = ax.inset_axes([lx + pad, ty, bw - pad * 2, thumb_h],
                           transform=ax.transAxes)
        if cmap:
            ia.imshow(data, cmap=cmap, aspect="auto", origin="upper",
                      vmin=0, vmax=1)
        else:
            ia.imshow(data, aspect="auto", origin="upper")
        ia.axis("off")
        for sp in ia.spines.values():
            sp.set_edgecolor(border); sp.set_linewidth(1.2)
        ia.set_visible(True)
        return ia

    # Stage 0 – raw RGB
    _thumb(img_rgb,    stages[0][1], border=BLUE)
    txt(ax, stages[0][1] + bw/2, ty + thumb_h + 0.025,
        "Template RGB\n224×224", size=7, color=BLUE)

    # Stage 1 – masked
    _thumb(img_masked, stages[1][1], border=ORANGE)
    txt(ax, stages[1][1] + bw/2, ty + thumb_h + 0.025,
        "Masked\n(BG → 0)", size=7, color=ORANGE)

    # Stage 2 – patch grid overlay
    _thumb(img_grid,   stages[2][1], border=PURPLE)
    txt(ax, stages[2][1] + bw/2, ty + thumb_h + 0.025,
        "14×14 patches\npatch_size=14", size=7, color=PURPLE)

    # Stage 3 – attention heatmap
    cmap_h = LinearSegmentedColormap.from_list("gh", [BG, "#008080", "#39D353"])
    _thumb(heat, stages[3][1], cmap=cmap_h, border=GREEN)
    txt(ax, stages[3][1] + bw/2, ty + thumb_h + 0.025,
        "Self-Attention\nheatmap (14×14)", size=7, color=GREEN)

    # Stage 4 – token vector bars
    s4x = stages[4][1]
    fbox(ax, s4x + pad, ty, bw - pad*2, thumb_h, DGRAY,
         ec=YELLOW, lw=1.2, alpha=0.9, zorder=3)
    # CLS block
    fbox(ax, s4x + pad + 0.008, ty + thumb_h * 0.55,
         0.030, thumb_h * 0.34, YELLOW, ec=YELLOW, lw=0.5, alpha=0.9, zorder=4)
    txt(ax, s4x + pad + 0.023, ty + thumb_h * 0.72,
        "CLS\n1×1024", size=6, color="#111111", weight="bold")
    # patch token bars (mini strip)
    for bi in range(40):
        col_v = float(RNG.uniform(0.25, 1.0))
        bxi   = s4x + pad + 0.048
        byi   = ty + thumb_h * 0.08 + bi * (thumb_h * 0.84 / 40)
        fbox(ax, bxi, byi, 0.060, thumb_h * 0.018, GREEN,
             ec="none", alpha=col_v, zorder=4)
    txt(ax, s4x + pad + 0.078, ty + thumb_h * 0.50,
        "P₁…P₁₉₆\n196×1024", size=6, color=GREEN, ha="center")
    txt(ax, s4x + bw/2, ty + thumb_h + 0.025,
        "CLS + 196 Patch\ntokens  (dim=1024)", size=7, color=YELLOW)

    # ── bottom annotations ────────────────────────────────────────────────
    notes = [
        (stages[0][1], "실제 렌더링 템플릿"),
        (stages[1][1], "Binary Mask 적용"),
        (stages[2][1], "Conv2D stride 14\n+ Pos Encoding"),
        (stages[3][1], "Depth=24  Heads=16\nDim=1024"),
        (stages[4][1], "cosine sim →\nSSEM / SAPPE"),
    ]
    for bx_n, note in notes:
        txt(ax, bx_n + bw / 2, by - 0.085, note, size=6.5, color=GRAY)


# ═══════════════════════════════════════════════════════════════════════════
# PANEL 2 – PEM  ViT-Base  L3/L6/L9/L12 → Concat → Linear → Bilinear
# ═══════════════════════════════════════════════════════════════════════════

def _feature_map_axes(ax, lx, ly, w, h, data, title, border_color):
    """inset axes로 히트맵 그리기."""
    ia = ax.inset_axes([lx, ly, w, h], transform=ax.transAxes)
    cmap = LinearSegmentedColormap.from_list(
        "fm", ["#0D1117", border_color, "#FFFFFF"])
    ia.imshow(data, cmap=cmap, aspect="auto", origin="lower",
              vmin=0, vmax=1)
    ia.axis("off")
    # thin border
    for sp in ia.spines.values():
        sp.set_edgecolor(border_color); sp.set_linewidth(1.2)
    ia.set_visible(True)
    txt(ax, lx + w / 2, ly + h + 0.025, title, size=7.5,
        color=border_color, zorder=6)
    return ia


def _fake_feat(scale=14, seed=0):
    rng2 = np.random.default_rng(seed)
    d = rng2.random((scale, scale))
    return gaussian_filter(d, sigma=1.2)


def draw_panel2(ax):
    panel_bg(ax, "[2] PEM  ViT-Base  –  Multi-Scale Feature → Concat → Linear → Bilinear Upsample")

    fm_w, fm_h = 0.10, 0.22
    y_fm = 0.60
    layer_xs  = [0.05, 0.18, 0.31, 0.44]
    colors    = [BLUE, ORANGE, PURPLE, GREEN]
    layer_lbl = ["Layer 3\n(H/4,W/4)", "Layer 6\n(H/4,W/4)",
                 "Layer 9\n(H/4,W/4)", "Layer 12\n(H/4,W/4)"]

    for i, (lx, col, lbl) in enumerate(zip(layer_xs, colors, layer_lbl)):
        d = _fake_feat(14, seed=i)
        _feature_map_axes(ax, lx, y_fm, fm_w, fm_h, d, lbl, col)

    # ── Concat bracket ──
    bx_c = 0.57
    fbox(ax, bx_c, y_fm - 0.02, 0.095, fm_h + 0.04, "#1A1A2E",
         ec=YELLOW, lw=1.5, zorder=3)
    txt(ax, bx_c + 0.048, y_fm + fm_h / 2, "Concat\n(channel)", size=8,
        weight="bold", color=YELLOW, zorder=5)
    txt(ax, bx_c + 0.048, y_fm - 0.055, "C = 4 × 768\n  = 3072 ch", size=7,
        color=GRAY)

    # arrows  feature maps → concat
    for lx in layer_xs:
        arr(ax, lx + fm_w, y_fm + fm_h / 2,
            bx_c - 0.003, y_fm + fm_h / 2, color=GRAY, lw=1.2)

    # ── Linear layer box ──
    bx_l = 0.695
    fbox(ax, bx_l, y_fm - 0.02, 0.095, fm_h + 0.04, "#1A002A",
         ec=PURPLE, lw=1.5, zorder=3)
    txt(ax, bx_l + 0.048, y_fm + fm_h / 2,
        "Linear\nProjection", size=8, weight="bold", color=PURPLE, zorder=5)
    txt(ax, bx_l + 0.048, y_fm - 0.055, "3072 → 256 ch", size=7, color=GRAY)
    arr(ax, bx_c + 0.095, y_fm + fm_h / 2,
        bx_l, y_fm + fm_h / 2, color=GRAY, lw=1.5)

    # ── Reshape ──
    bx_r = 0.820
    fbox(ax, bx_r, y_fm - 0.02, 0.080, fm_h + 0.04, "#001A10",
         ec=TEAL, lw=1.5, zorder=3)
    txt(ax, bx_r + 0.040, y_fm + fm_h / 2,
        "Reshape", size=8, weight="bold", color=TEAL, zorder=5)
    txt(ax, bx_r + 0.040, y_fm - 0.055, "(N, 196, 256)\n→(N,14,14,256)", size=6.5,
        color=GRAY)
    arr(ax, bx_l + 0.095, y_fm + fm_h / 2,
        bx_r, y_fm + fm_h / 2, color=GRAY, lw=1.5)

    # ── bottom row: before / after bilinear ──
    y_low = 0.14

    # before: 14×14 feature map
    d_low = _fake_feat(14, seed=10)
    _feature_map_axes(ax, 0.17, y_low, 0.18, 0.30,
                      d_low, "After Reshape\n14×14×256", TEAL)

    # arrow
    arr(ax, 0.370, y_low + 0.15, 0.630, y_low + 0.15, color=ORANGE, lw=2.0)
    txt(ax, 0.500, y_low + 0.22,
        "Bilinear Interpolation\n(upsample × 4)", size=8, color=ORANGE)
    txt(ax, 0.500, y_low + 0.07, "align_corners=False", size=6.5, color=GRAY)

    # after: 56×56 feature map (simulated via 28×28 for display)
    d_hi = np.repeat(np.repeat(d_low, 4, axis=0), 4, axis=1)
    from scipy.ndimage import zoom as sz
    d_hi = sz(d_low, 4, order=1)
    d_hi = (d_hi - d_hi.min()) / (d_hi.ptp() + 1e-9)
    _feature_map_axes(ax, 0.635, y_low, 0.18, 0.30,
                      d_hi, "After Bilinear\n56×56×256", GREEN)

    # connector lines from reshape box down
    arr(ax, bx_r + 0.040, y_fm - 0.02, 0.260, y_low + 0.30,
        color=TEAL, lw=1.2, rad=-0.2)


# ═══════════════════════════════════════════════════════════════════════════
# PANEL 3 – Geometric Transformer  Self-Attention 포인트 위치 업데이트
# ═══════════════════════════════════════════════════════════════════════════

def _draw_points(ax, xs, ys, colors, sizes, zorder=5):
    for x, y, c, s in zip(xs, ys, colors, sizes):
        dot = mpatches.Circle((x, y), s, facecolor=c, edgecolor=WHITE,
                               linewidth=0.5, transform=ax.transAxes, zorder=zorder)
        ax.add_patch(dot)


def draw_panel3(ax):
    panel_bg(ax, "[3] Geometric Transformer  –  Self-Attention 포인트 위치 업데이트")

    n_pts = 8
    # initial positions (axes fraction, kept in centre zone)
    init_xs = np.array([0.12, 0.18, 0.14, 0.22, 0.16, 0.20, 0.11, 0.24])
    init_ys = np.array([0.65, 0.72, 0.55, 0.60, 0.45, 0.50, 0.38, 0.42])
    # attention-weighted shift (simulated)
    delta_x = np.array([ 0.022, -0.015,  0.018, -0.020,  0.025, -0.012,  0.030, -0.018])
    delta_y = np.array([ 0.015,  0.020, -0.018,  0.025, -0.010,  0.022, -0.015,  0.018])
    final_xs = init_xs + delta_x
    final_ys = init_ys + delta_y

    pt_colors = [BLUE, ORANGE, GREEN, PURPLE, YELLOW, TEAL, PINK, RED]

    # ── Left column: input positions ──────────────────────────────────────
    fbox(ax, 0.04, 0.30, 0.28, 0.55, DGRAY, ec=BLUE, lw=1.5, alpha=0.6, zorder=2)
    txt(ax, 0.18, 0.89, "Input Point Set", size=9, weight="bold", color=BLUE)
    txt(ax, 0.18, 0.83, "P ∈ ℝᴺˣ³  (N=8, xyz)", size=7, color=GRAY)
    _draw_points(ax, init_xs, init_ys, pt_colors, [0.014] * n_pts)
    # edge connections (sparse)
    for i in range(n_pts):
        for j in range(i + 1, n_pts):
            if abs(init_xs[i] - init_xs[j]) + abs(init_ys[i] - init_ys[j]) < 0.15:
                ax.plot([init_xs[i], init_xs[j]], [init_ys[i], init_ys[j]],
                        color=BORDER, lw=0.8, alpha=0.7,
                        transform=ax.transAxes, zorder=3)
    # point labels
    for i, (x, y) in enumerate(zip(init_xs, init_ys)):
        txt(ax, x + 0.018, y, f"p{i}", size=6, color=WHITE)

    # ── Center: attention block ────────────────────────────────────────────
    fbox(ax, 0.36, 0.25, 0.28, 0.60, "#0A0020", ec=PURPLE, lw=2.0, alpha=0.9, zorder=2)
    txt(ax, 0.50, 0.90, "Self-Attention Block", size=9, weight="bold", color=PURPLE)
    # simulated attention matrix (8×8)
    attn = RNG.random((n_pts, n_pts))
    attn = np.exp(attn) / np.exp(attn).sum(axis=1, keepdims=True)
    cmap_a = LinearSegmentedColormap.from_list("attn", [BG, "#6B3FA0", PURPLE, WHITE])
    ia = ax.inset_axes([0.375, 0.52, 0.25, 0.25], transform=ax.transAxes)
    ia.imshow(attn, cmap=cmap_a, aspect="auto", vmin=0, vmax=1)
    ia.set_xticks(range(n_pts)); ia.set_yticks(range(n_pts))
    ia.tick_params(labelsize=5.5, colors=GRAY, length=2)
    ia.set_xlabel("Key (pts)", fontsize=6, color=GRAY)
    ia.set_ylabel("Query (pts)", fontsize=6, color=GRAY)
    for sp in ia.spines.values():
        sp.set_edgecolor(PURPLE); sp.set_linewidth(1.0)
    txt(ax, 0.50, 0.50, "Attention Matrix  A = softmax(QKᵀ/√d)", size=6.5, color=GRAY)

    # formula
    fbox(ax, 0.375, 0.30, 0.25, 0.17, "#10001A", ec=PURPLE, lw=1, alpha=0.8, zorder=4)
    txt(ax, 0.50, 0.42, "P' = P + A · V", size=9, weight="bold", color=YELLOW)
    txt(ax, 0.50, 0.355, "Q = K = V = Linear(P)", size=7, color=GRAY)
    txt(ax, 0.50, 0.31, "Positional Encoding included", size=6.5, color=GRAY)

    # ── Right column: updated positions ──────────────────────────────────
    fbox(ax, 0.68, 0.30, 0.28, 0.55, DGRAY, ec=GREEN, lw=1.5, alpha=0.6, zorder=2)
    txt(ax, 0.82, 0.89, "Updated Point Set", size=9, weight="bold", color=GREEN)
    txt(ax, 0.82, 0.83, "P' ∈ ℝᴺˣ³  (refined)", size=7, color=GRAY)
    rx = final_xs + 0.56   # shift to right panel
    ry = final_ys
    _draw_points(ax, rx, ry, pt_colors, [0.014] * n_pts, zorder=5)
    for i in range(n_pts):
        for j in range(i + 1, n_pts):
            if abs(rx[i] - rx[j]) + abs(ry[i] - ry[j]) < 0.15:
                ax.plot([rx[i], rx[j]], [ry[i], ry[j]],
                        color=BORDER, lw=0.8, alpha=0.7,
                        transform=ax.transAxes, zorder=3)
    for i, (x, y) in enumerate(zip(rx, ry)):
        txt(ax, x + 0.018, y, f"p{i}'", size=6, color=WHITE)

    # ── Arrows: input → block → output ────────────────────────────────────
    arr(ax, 0.32, 0.575, 0.36, 0.575, color=BLUE, lw=2.0)
    arr(ax, 0.64, 0.575, 0.68, 0.575, color=GREEN, lw=2.0)

    # ── displacement arrows (per point, across panels) ────────────────────
    for i in range(n_pts):
        x0 = init_xs[i] + 0.014
        y0 = init_ys[i]
        x1 = rx[i] - 0.014
        y1 = ry[i]
        ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                    xycoords="axes fraction", textcoords="axes fraction",
                    arrowprops=dict(arrowstyle="-|>", color=pt_colors[i],
                                   lw=0.8, mutation_scale=7,
                                   connectionstyle="arc3,rad=0.2",
                                   alpha=0.55),
                    zorder=4)

    # ── bottom note ──────────────────────────────────────────────────────
    txt(ax, 0.50, 0.20, "각 포인트는 모든 이웃 포인트의 특징을 Attention으로 집계하여 위치 업데이트",
        size=7.5, color=GRAY)
    txt(ax, 0.50, 0.13, "→  기하학적 관계(거리, 방향)를 학습하여 더 정확한 3D 위치 추정",
        size=7.5, color=GREEN)

    # stacked blocks legend
    for bi, blabel in enumerate(["Block 1", "Block 2", "Block 3", "Block 4"]):
        bx_ = 0.375 + bi * 0.062
        fbox(ax, bx_, 0.06, 0.055, 0.04, DGRAY, ec=PURPLE, lw=0.8, alpha=0.9, zorder=3)
        txt(ax, bx_ + 0.028, 0.08, blabel, size=6, color=PURPLE)
    txt(ax, 0.50, 0.04, "Geometric Transformer: N개 Self-Attention Block 반복", size=6.5,
        color=GRAY)


# ═══════════════════════════════════════════════════════════════════════════
# PANEL 4 – Sparse → Dense  포인트 클라우드 정교화 과정
# ═══════════════════════════════════════════════════════════════════════════

def _gen_cloud(n, center=(0.5, 0.5), spread=0.18, seed=0):
    rng2 = np.random.default_rng(seed)
    angles = rng2.uniform(0, 2 * np.pi, n)
    radii  = rng2.beta(2, 3, n) * spread
    xs = center[0] + radii * np.cos(angles)
    ys = center[1] + radii * np.sin(angles)
    return np.clip(xs, 0.05, 0.95), np.clip(ys, 0.05, 0.95)


def _draw_cloud(ax, xs, ys, color, size, alpha=0.85, zorder=4, label=None):
    for x, y in zip(xs, ys):
        d = mpatches.Circle((x, y), size, facecolor=color, edgecolor="none",
                             alpha=alpha, transform=ax.transAxes, zorder=zorder)
        ax.add_patch(d)
    if label:
        txt(ax, xs.mean(), ys.max() + 0.06, label, size=7.5,
            color=color, zorder=6)


def draw_panel4(ax):
    panel_bg(ax, "[4] Sparse → Dense  포인트 클라우드 정교화  (Coarse-to-Fine)")

    # ── 4 stage column panels ─────────────────────────────────────────────
    col_w   = 0.195
    col_gap = 0.020
    col_xs  = [0.02 + i * (col_w + col_gap) for i in range(4)]
    col_h   = 0.72
    col_y   = 0.18
    colors_stage = [BLUE, ORANGE, GREEN, YELLOW]
    col_titles   = [
        "Stage 0\nSparse Init",
        "Stage 1\nCoarse Refine",
        "Stage 2\nMedium Refine",
        "Stage 3\nDense Final",
    ]
    n_pts_stages = [32, 128, 512, 2048]
    pt_sizes     = [0.018, 0.012, 0.007, 0.004]

    # object silhouette reference (axes fraction coords of each sub-panel)
    obj_polys = np.array([
        [0.25, 0.30], [0.35, 0.20], [0.65, 0.20], [0.75, 0.30],
        [0.75, 0.70], [0.65, 0.80], [0.35, 0.80], [0.25, 0.70],
    ])  # normalized 0-1 within sub-panel

    for si, (cx, col, title, n_pts, ptsz) in enumerate(
            zip(col_xs, colors_stage, col_titles, n_pts_stages, pt_sizes)):

        # panel box
        fbox(ax, cx, col_y, col_w, col_h, DGRAY, ec=col, lw=1.6, alpha=0.9, zorder=2)

        # title
        txt(ax, cx + col_w / 2, col_y + col_h + 0.02, title, size=8,
            weight="bold", color=col)

        # point cloud inside panel (map 0-1 → axes fraction)
        xs_pts, ys_pts = _gen_cloud(n_pts,
                                    center=(cx + col_w / 2,
                                            col_y + col_h / 2),
                                    spread=col_w * 0.35,
                                    seed=si)
        # clip to panel
        mask = ((xs_pts > cx + 0.008) & (xs_pts < cx + col_w - 0.008) &
                (ys_pts > col_y + 0.01) & (ys_pts < col_y + col_h - 0.01))
        _draw_cloud(ax, xs_pts[mask], ys_pts[mask], col, ptsz, alpha=0.8)

        # object outline overlay
        poly_xs = cx + obj_polys[:, 0] * col_w
        poly_ys = col_y + obj_polys[:, 1] * col_h
        obj_patch = plt.Polygon(np.stack([poly_xs, poly_ys], axis=1),
                                fill=False, edgecolor=col, linewidth=1.0,
                                alpha=0.35, transform=ax.transAxes, zorder=5)
        ax.add_patch(obj_patch)

        # point count badge
        fbox(ax, cx + 0.005, col_y + 0.01, col_w - 0.01, 0.055,
             "#000000", ec=col, lw=0.8, alpha=0.6, zorder=5)
        txt(ax, cx + col_w / 2, col_y + 0.038, f"N = {n_pts:,} pts",
            size=7.5, color=col, zorder=6)

    # ── arrows between stages ─────────────────────────────────────────────
    stage_labels = [
        "Geometric\nTransformer ×1",
        "Geometric\nTransformer ×2",
        "Geometric\nTransformer ×2",
    ]
    for si in range(3):
        x0 = col_xs[si] + col_w
        x1 = col_xs[si + 1]
        ya = col_y + col_h / 2
        arr(ax, x0 + 0.002, ya, x1 - 0.002, ya, color=GRAY, lw=2.0)
        txt(ax, (x0 + x1) / 2, ya + 0.07, stage_labels[si],
            size=6.5, color=GRAY)

    # ── density comparison bar ─────────────────────────────────────────────
    bar_y  = 0.07
    bar_h2 = 0.05
    bar_xs = [cx + col_w / 2 for cx in col_xs]
    bar_ws = [0.01, 0.03, 0.08, 0.16]
    for bx_, bw_, col in zip(bar_xs, bar_ws, colors_stage):
        fbox(ax, bx_ - bw_ / 2, bar_y, bw_, bar_h2, col,
             ec="none", alpha=0.85, zorder=4)

    # ── legends ──────────────────────────────────────────────────────────
    txt(ax, 0.50, 0.14,
        "Coarse stage: 큰 구조 파악 (글로벌)  →  Fine stage: 세부 형태 정교화 (로컬)",
        size=8, color=GRAY)
    txt(ax, 0.50, 0.09, "← 밀도 점진 증가 →", size=8, color=WHITE, weight="bold")

    # depth labels
    for bx_, col, n in zip(bar_xs, colors_stage, n_pts_stages):
        txt(ax, bx_, bar_y - 0.035, f"{n:,}", size=7, color=col)


# ═══════════════════════════════════════════════════════════════════════════
# SAVE helpers
# ═══════════════════════════════════════════════════════════════════════════

def save_single(draw_fn, panel_idx, dpi=180):
    fig, ax = plt.subplots(1, 1, figsize=(14, 7), facecolor=BG)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    draw_fn(ax)
    out = os.path.join(OUT_DIR, f"vis_ppt_panel{panel_idx}.png")
    fig.savefig(out, dpi=dpi, bbox_inches="tight",
                facecolor=BG, edgecolor="none")
    plt.close(fig)
    print(f"  saved → {out}")


def save_all(dpi=160):
    fig = plt.figure(figsize=(26, 22), facecolor=BG)
    fig.patch.set_facecolor(BG)
    gs = GridSpec(2, 2, figure=fig,
                  left=0.01, right=0.99, top=0.97, bottom=0.01,
                  wspace=0.025, hspace=0.06)
    axes = [fig.add_subplot(gs[r, c]) for r, c in [(0,0),(0,1),(1,0),(1,1)]]
    for a in axes:
        a.set_facecolor(BG)

    draw_panel1(axes[0])
    draw_panel2(axes[1])
    draw_panel3(axes[2])
    draw_panel4(axes[3])

    fig.text(0.5, 0.988,
             "SAM-6D  –  ISM / PEM Pipeline  Architecture Visualization",
             ha="center", fontsize=14, fontweight="bold",
             color=WHITE, fontfamily="monospace")

    out = os.path.join(OUT_DIR, "vis_ppt_all.png")
    fig.savefig(out, dpi=dpi, bbox_inches="tight",
                facecolor=BG, edgecolor="none")
    plt.close(fig)
    print(f"  saved → {out}")


# ═══════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("Generating PPT visualizations …")
    panels = [
        (draw_panel1, 1),
        (draw_panel2, 2),
        (draw_panel3, 3),
        (draw_panel4, 4),
    ]
    for fn, idx in panels:
        save_single(fn, idx)

    save_all()
    print("Done.")
