"""
DINOv2 (ViT-L/14) Pipeline Visualization for SAM-6D ISM
Generates a 3-panel figure:
  Left  : Masked input image with 14×14 patch grid overlay
  Center: ViT-L/14 block diagram
  Right : CLS token vs Patch token similarity flowchart
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle
from matplotlib.gridspec import GridSpec
from matplotlib.colors import LinearSegmentedColormap

# ── colour palette ──────────────────────────────────────────────────────────
C_BG      = "#0F1117"
C_PANEL   = "#1A1D27"
C_BORDER  = "#2E3250"
C_BLUE    = "#4A90E2"
C_GREEN   = "#50C878"
C_ORANGE  = "#FF8C42"
C_PURPLE  = "#9B59B6"
C_YELLOW  = "#F1C40F"
C_RED     = "#E74C3C"
C_WHITE   = "#ECEFF4"
C_GRAY    = "#8892A4"
C_TEAL    = "#1ABC9C"
C_PINK    = "#FF6B9D"

FONT_TITLE  = dict(fontsize=11, fontweight="bold", color=C_WHITE, fontfamily="monospace")
FONT_LABEL  = dict(fontsize=8.5, color=C_WHITE, fontfamily="monospace")
FONT_SMALL  = dict(fontsize=7.5, color=C_GRAY,  fontfamily="monospace")
FONT_CODE   = dict(fontsize=7,   color=C_GREEN,  fontfamily="monospace")

# ── helpers ──────────────────────────────────────────────────────────────────

def fancy_box(ax, x, y, w, h, color, alpha=0.85, radius=0.03, lw=1.2, ec=None):
    """Draw a rounded rectangle."""
    ec = ec or color
    box = FancyBboxPatch((x, y), w, h,
                         boxstyle=f"round,pad=0",
                         linewidth=lw, edgecolor=ec,
                         facecolor=color, alpha=alpha,
                         transform=ax.transAxes, zorder=3)
    ax.add_patch(box)
    return box

def arrow(ax, x0, y0, x1, y1, color=C_GRAY, lw=1.5, style="->", zorder=4):
    ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                xycoords="axes fraction", textcoords="axes fraction",
                arrowprops=dict(arrowstyle=style, color=color,
                                lw=lw, connectionstyle="arc3,rad=0.0"),
                zorder=zorder)

def text(ax, x, y, s, **kwargs):
    ha = kwargs.pop("ha", "center")
    va = kwargs.pop("va", "center")
    kwargs.pop("transform", None)  # remove if caller passed it accidentally
    ax.text(x, y, s, transform=ax.transAxes, ha=ha, va=va, **kwargs)

# ════════════════════════════════════════════════════════════════════════════
# Panel A – Masked input + patch grid
# ════════════════════════════════════════════════════════════════════════════

def draw_panel_a(ax):
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_facecolor(C_BG)

    # ── background ──
    fancy_box(ax, 0.02, 0.02, 0.96, 0.96, C_PANEL, alpha=1.0, lw=1.5, ec=C_BORDER)

    # ── title ──
    text(ax, 0.5, 0.93, "① Masked Input  +  Patch Grid", **FONT_TITLE)

    # ── synthetic "masked image" ──
    # Draw a 224×224 object silhouette (stylised mug-like shape)
    img_l, img_b, img_w, img_h = 0.08, 0.22, 0.84, 0.62

    # Background of image area
    img_bg = FancyBboxPatch((img_l, img_b), img_w, img_h,
                             boxstyle="round,pad=0", linewidth=1,
                             edgecolor=C_BORDER, facecolor="#12151E",
                             transform=ax.transAxes, zorder=2)
    ax.add_patch(img_bg)

    # ── draw stylised object (mug silhouette) using filled rectangles ──
    obj_l, obj_b = img_l + 0.18, img_b + 0.06
    obj_w, obj_h = 0.48, 0.50
    obj = FancyBboxPatch((obj_l, obj_b), obj_w, obj_h,
                          boxstyle="round,pad=0.01", linewidth=0,
                          facecolor="#2A5298", alpha=0.80,
                          transform=ax.transAxes, zorder=3)
    ax.add_patch(obj)
    # handle
    handle = mpatches.Ellipse((obj_l + obj_w + 0.06, obj_b + obj_h * 0.45),
                               0.10, 0.20, color="#2A5298", alpha=0.80,
                               transform=ax.transAxes, zorder=3)
    ax.add_patch(handle)

    # ── black mask overlay (simulate binary mask) ──
    # Random mask effect: draw semi-transparent dots outside
    rng = np.random.default_rng(42)
    for _ in range(60):
        cx = rng.uniform(img_l + 0.02, img_l + img_w - 0.02)
        cy = rng.uniform(img_b + 0.02, img_b + img_h - 0.02)
        if cx < obj_l or cx > obj_l + obj_w + 0.12:  # outside object
            dot = mpatches.Circle((cx, cy), 0.018,
                                  color="#050709", alpha=0.9,
                                  transform=ax.transAxes, zorder=4)
            ax.add_patch(dot)

    # ── 14×14 patch grid ──
    n_patches = 14
    cell_w = img_w / n_patches
    cell_h = img_h / n_patches
    for i in range(n_patches + 1):
        xg = img_l + i * cell_w
        yg_b = img_b
        yg_t = img_b + img_h
        ax.plot([xg, xg], [yg_b, yg_t], color=C_TEAL, lw=0.45,
                alpha=0.55, transform=ax.transAxes, zorder=5)
    for j in range(n_patches + 1):
        yg = img_b + j * cell_h
        ax.plot([img_l, img_l + img_w], [yg, yg], color=C_TEAL, lw=0.45,
                alpha=0.55, transform=ax.transAxes, zorder=5)

    # ── highlight one patch ──
    hi_i, hi_j = 5, 8
    hi_x = img_l + hi_i * cell_w
    hi_y = img_b + hi_j * cell_h
    hi_patch = Rectangle((hi_x, hi_y), cell_w, cell_h,
                          linewidth=1.2, edgecolor=C_YELLOW,
                          facecolor=C_YELLOW, alpha=0.35,
                          transform=ax.transAxes, zorder=6)
    ax.add_patch(hi_patch)
    text(ax, hi_x + cell_w / 2, hi_y + cell_h / 2,
         "p", fontsize=6, color=C_YELLOW, va="center", ha="center",
         transform=ax.transAxes, zorder=7)

    # ── annotations ──
    text(ax, 0.5, 0.18, "224 × 224 input  →  14 × 14 = 196 patches",
         fontsize=7.5, color=C_TEAL, ha="center", transform=ax.transAxes)
    text(ax, 0.5, 0.11, "patch size = 14 × 14 px",
         fontsize=7.5, color=C_GRAY, ha="center", transform=ax.transAxes)
    text(ax, 0.5, 0.05, "+ Binary Mask  (background → zero)",
         fontsize=7.5, color=C_ORANGE, ha="center", transform=ax.transAxes)


# ════════════════════════════════════════════════════════════════════════════
# Panel B – ViT-L/14 block diagram
# ════════════════════════════════════════════════════════════════════════════

def draw_panel_b(ax):
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_facecolor(C_BG)

    fancy_box(ax, 0.02, 0.02, 0.96, 0.96, C_PANEL, alpha=1.0, lw=1.5, ec=C_BORDER)
    text(ax, 0.5, 0.93, "② ViT-L/14 Architecture", **FONT_TITLE)

    # ── layout constants ──
    bx, bw, bh = 0.18, 0.64, 0.065   # box x-start, width, height
    gap = 0.025

    def block(ax, y, label, color, sublabel=""):
        fancy_box(ax, bx, y, bw, bh, color, alpha=0.85, lw=1.2, ec=color)
        text(ax, bx + bw / 2, y + bh / 2 + (0.008 if sublabel else 0),
             label, fontsize=8, fontweight="bold", color=C_WHITE,
             ha="center", transform=ax.transAxes, zorder=5)
        if sublabel:
            text(ax, bx + bw / 2, y + bh / 2 - 0.014,
                 sublabel, fontsize=6.5, color=C_GRAY,
                 ha="center", transform=ax.transAxes, zorder=5)

    # bottom → top layout
    y0 = 0.06
    layers = [
        ("Input:  196 patch tokens  +  1 CLS", C_BLUE,    "dim = 1024"),
        ("Patch Embed  (14×14 conv)",           "#34495E",  ""),
        ("Positional Encoding",                 "#34495E",  ""),
        ("× 24 Transformer Blocks",             C_PURPLE,   "MHA  +  FFN  +  LayerNorm"),
        ("Layer Norm",                          "#34495E",  ""),
        ("Output Tokens",                       C_GREEN,    "CLS ∈ ℝ¹ˣ¹⁰²⁴   |  Patch ∈ ℝ¹⁹⁶ˣ¹⁰²⁴"),
    ]

    ys = []
    y = y0
    for label, color, sub in layers:
        block(ax, y, label, color, sub)
        ys.append(y)
        y += bh + gap

    # arrows between blocks
    cx = bx + bw / 2
    for i in range(len(ys) - 1):
        ya = ys[i] + bh
        yb = ys[i + 1]
        arrow(ax, cx, ya, cx, yb, color=C_GRAY, lw=1.2)

    # ── Transformer block zoom-in ──
    zx, zy, zw, zh_tot = 0.06, 0.455, 0.88, 0.195
    # subtle highlight
    hi = FancyBboxPatch((zx - 0.01, zy - 0.005), zw + 0.02, zh_tot + 0.01,
                         boxstyle="round,pad=0", linewidth=1,
                         edgecolor=C_PURPLE, facecolor=C_PURPLE,
                         alpha=0.06, transform=ax.transAxes, zorder=1)
    ax.add_patch(hi)

    # ── spec labels ──
    specs = [
        ("Depth",   "24 layers"),
        ("Heads",   "16"),
        ("Dim",     "1024"),
        ("MLP×",    "4096"),
        ("Params",  "~307 M"),
    ]
    sx = 0.08
    sy = 0.89
    for k, v in specs:
        text(ax, sx, sy, f"{k}:", fontsize=7, color=C_GRAY, ha="left",
             transform=ax.transAxes)
        text(ax, sx + 0.17, sy, v, fontsize=7, color=C_YELLOW, ha="left",
             transform=ax.transAxes)
        sy -= 0.042

    # Patch size badge
    badge = FancyBboxPatch((0.64, 0.865), 0.29, 0.058,
                            boxstyle="round,pad=0", linewidth=1.2,
                            edgecolor=C_TEAL, facecolor=C_TEAL,
                            alpha=0.15, transform=ax.transAxes, zorder=3)
    ax.add_patch(badge)
    text(ax, 0.785, 0.894, "patch_size = 14",
         fontsize=8.5, fontweight="bold", color=C_TEAL,
         ha="center", transform=ax.transAxes, zorder=5)


# ════════════════════════════════════════════════════════════════════════════
# Panel C – CLS / Patch token similarity flowchart
# ════════════════════════════════════════════════════════════════════════════

def draw_panel_c(ax):
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_facecolor(C_BG)

    fancy_box(ax, 0.02, 0.02, 0.96, 0.96, C_PANEL, alpha=1.0, lw=1.5, ec=C_BORDER)
    text(ax, 0.5, 0.93, "③ Similarity Branching", **FONT_TITLE)

    # ── helper ──
    def box(ax, cx, cy, w, h, label, sublabel, fc, ec, lw=1.3):
        lx, ly = cx - w / 2, cy - h / 2
        rect = FancyBboxPatch((lx, ly), w, h,
                               boxstyle="round,pad=0", linewidth=lw,
                               edgecolor=ec, facecolor=fc, alpha=0.88,
                               transform=ax.transAxes, zorder=4)
        ax.add_patch(rect)
        off = 0.012 if sublabel else 0
        text(ax, cx, cy + off, label,
             fontsize=8, fontweight="bold", color=C_WHITE,
             ha="center", transform=ax.transAxes, zorder=5)
        if sublabel:
            text(ax, cx, cy - 0.015, sublabel,
                 fontsize=6.5, color=C_GRAY,
                 ha="center", transform=ax.transAxes, zorder=5)

    def arr(ax, x0, y0, x1, y1, color, label="", curved=False):
        rad = "arc3,rad=0.18" if curved else "arc3,rad=0.0"
        ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                    xycoords="axes fraction", textcoords="axes fraction",
                    arrowprops=dict(arrowstyle="-|>", color=color,
                                   lw=1.5, mutation_scale=12,
                                   connectionstyle=rad),
                    zorder=6)
        if label:
            mx, my = (x0 + x1) / 2, (y0 + y1) / 2
            text(ax, mx + 0.04, my, label,
                 fontsize=6.5, color=color,
                 ha="left", transform=ax.transAxes, zorder=6)

    # ── DINOv2 source node ──
    box(ax, 0.50, 0.825, 0.54, 0.07,
        "DINOv2  ViT-L/14",
        "196 patch tokens  +  1 CLS token",
        "#1B2A4A", C_BLUE, lw=2.0)

    # ── split arrows ──
    arr(ax, 0.30, 0.79, 0.23, 0.70, C_ORANGE)   # CLS branch
    arr(ax, 0.70, 0.79, 0.77, 0.70, C_GREEN)    # Patch branch

    # ── CLS branch ──
    box(ax, 0.22, 0.655, 0.36, 0.065,
        "CLS Token",
        "shape: (1, 1024)",
        "#3D1C00", C_ORANGE, lw=1.5)

    arr(ax, 0.22, 0.62, 0.22, 0.545, C_ORANGE)

    box(ax, 0.22, 0.51, 0.36, 0.065,
        "Global Cosine Similarity",
        "sim_cls = CLS_q · CLS_t",
        "#3D1C00", C_ORANGE)

    arr(ax, 0.22, 0.478, 0.22, 0.405, C_ORANGE)

    box(ax, 0.22, 0.37, 0.36, 0.065,
        "SSEM Score",
        "top-k template selection",
        "#3D2800", C_YELLOW)

    # ── Patch branch ──
    box(ax, 0.78, 0.655, 0.36, 0.065,
        "Patch Tokens",
        "shape: (196, 1024)",
        "#0D2E1A", C_GREEN, lw=1.5)

    arr(ax, 0.78, 0.62, 0.78, 0.545, C_GREEN)

    box(ax, 0.78, 0.51, 0.36, 0.065,
        "Local Patch Similarity",
        "sim_patch = Σ cosine(p_q, p_t)",
        "#0D2E1A", C_GREEN)

    arr(ax, 0.78, 0.478, 0.78, 0.405, C_GREEN)

    box(ax, 0.78, 0.37, 0.36, 0.065,
        "SAPPE Score",
        "geo-aware aggregation",
        "#0A2A1A", C_TEAL)

    # ── merge arrows ──
    arr(ax, 0.40, 0.337, 0.50, 0.265, C_YELLOW)
    arr(ax, 0.60, 0.337, 0.50, 0.265, C_TEAL)

    # ── final fusion node ──
    box(ax, 0.50, 0.235, 0.44, 0.065,
        "Score Fusion",
        "S = α·SSEM + β·SAPPE",
        "#1A1240", C_PURPLE, lw=1.8)

    arr(ax, 0.50, 0.20, 0.50, 0.135, C_PURPLE)

    # ── output ──
    box(ax, 0.50, 0.10, 0.50, 0.065,
        "Top-1 Pose Hypothesis",
        "R, t  →  6-DoF pose",
        "#1A2A10", C_GREEN, lw=1.8)

    # ── legend ──
    items = [
        (C_ORANGE, "CLS  →  global shape similarity"),
        (C_GREEN,  "Patch  →  local texture similarity"),
        (C_PURPLE, "Fused final score"),
    ]
    lx = 0.05
    ly = 0.045
    for col, lab in items:
        dot = mpatches.Circle((lx + 0.012, ly), 0.008,
                               color=col, transform=ax.transAxes, zorder=7)
        ax.add_patch(dot)
        text(ax, lx + 0.03, ly, lab,
             fontsize=6.5, color=C_GRAY, ha="left",
             transform=ax.transAxes, zorder=7)
        lx += 0.34


# ════════════════════════════════════════════════════════════════════════════
# Compose figure
# ════════════════════════════════════════════════════════════════════════════

def main():
    fig = plt.figure(figsize=(20, 9), facecolor=C_BG)
    fig.patch.set_facecolor(C_BG)

    gs = GridSpec(1, 3, figure=fig,
                  left=0.01, right=0.99,
                  top=0.94, bottom=0.02,
                  wspace=0.03)

    ax_a = fig.add_subplot(gs[0])
    ax_b = fig.add_subplot(gs[1])
    ax_c = fig.add_subplot(gs[2])

    for ax in (ax_a, ax_b, ax_c):
        ax.set_facecolor(C_BG)

    draw_panel_a(ax_a)
    draw_panel_b(ax_b)
    draw_panel_c(ax_c)

    # ── super title ──
    fig.text(0.5, 0.975,
             "SAM-6D ISM — DINOv2 ViT-L/14 Feature Extraction Pipeline",
             ha="center", va="center",
             fontsize=13, fontweight="bold", color=C_WHITE,
             fontfamily="monospace")

    out = "d:/LDH_ws/sam_6d/SAM-6D/Instance_Segmentation_Model/visualization/image/vis_dinov2_pipeline.png"
    fig.savefig(out, dpi=150, bbox_inches="tight",
                facecolor=C_BG, edgecolor="none")
    print(f"Saved: {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
