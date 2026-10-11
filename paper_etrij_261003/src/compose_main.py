"""Replace the images of the drafted overview figure with real data, keeping its layout and type.

Input : images/SLAM과 6D 인식 통합 파이프라인.png (layout draft, 1672x940)
Output: images/fig_main_real.png (2x, 3344x1880)

Every picture placed here comes from the recordings or logs (see real_tiles.py, render_map3d.py and
the anchoring replay corr04.json); the keyframe-graph icon is kept as a schematic. Wrong text in the
draft is painted over and re-typeset: the LiDAR rate, the keyframe-list timing, the depth label,
equation (1) (the extrinsic X was missing) and the timeline (one capture, one arrival).
"""
import json, sys
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFilter
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'images' / 'SLAM과 6D 인식 통합 파이프라인.png'
REAL = ROOT / 'src' / 'real'
PHOTO = Path('/home/jucpark/DeepLearning/CLI_environment/객체 위치 추정/IMG_3942.jpeg')
CORR = Path(sys.argv[1])
S = 2                                                    # output scale

base = Image.open(SRC).convert('RGB')
img = base.resize((base.width * S, base.height * S), Image.LANCZOS)
draw = ImageDraw.Draw(img)


def B(x0, y0, x1, y1):
    return tuple(int(v * S) for v in (x0, y0, x1, y1))


def place(tile, box, mode='cover'):
    """Fit tile into box (draft coordinates); cover = fill and crop centre, contain = letterbox."""
    x0, y0, x1, y1 = B(*box); w, h = x1 - x0, y1 - y0
    t = tile.convert('RGB')
    r = max(w / t.width, h / t.height) if mode == 'cover' else min(w / t.width, h / t.height)
    t = t.resize((max(1, round(t.width * r)), max(1, round(t.height * r))), Image.LANCZOS)
    if mode == 'cover':
        l, u = (t.width - w) // 2, (t.height - h) // 2
        img.paste(t.crop((l, u, l + w, u + h)), (x0, y0))
    else:
        bg = bgcol(box)
        draw.rectangle((x0, y0, x1, y1), fill=bg)
        img.paste(t, (x0 + (w - t.width) // 2, y0 + (h - t.height) // 2))


def bgcol(box, dx=-3):
    x0, y0, x1, y1 = box
    px = [base.getpixel((x0 + dx, y)) for y in range(int(y0), int(y1), 3)]
    return tuple(int(np.median([p[i] for p in px])) for i in range(3))


def typeset(text, box, bold=False, color='#111111', align='center', size=None, bg=None):
    """Paint the box with its background and render text (mathtext allowed) at the box height."""
    x0, y0, x1, y1 = B(*box)
    bg = bg or bgcol(box)
    draw.rectangle((x0, y0, x1, y1), fill=bg)
    h = y1 - y0
    fig = plt.figure(figsize=(12, 1)); fig.patch.set_alpha(0)
    fs = size or 40
    fig.text(0, 0.5, text, fontsize=fs, fontweight='bold' if bold else 'normal', color=color, va='center',
             family='Liberation Serif', math_fontfamily='stix')
    p = ROOT / 'src' / '_t.png'
    fig.savefig(p, dpi=100, transparent=True, bbox_inches='tight', pad_inches=0.02); plt.close(fig)
    t = Image.open(p).convert('RGBA')
    r = h / t.height if size is None else 1.0
    t = t.resize((max(1, round(t.width * r)), max(1, round(t.height * r))), Image.LANCZOS)
    if t.width > x1 - x0:
        k = (x1 - x0) / t.width; t = t.resize((x1 - x0, max(1, round(t.height * k))), Image.LANCZOS)
    xs = x0 + ((x1 - x0) - t.width) // 2 if align == 'center' else x0
    img.paste(t, (xs, y0 + (h - t.height) // 2), t)


# ---- photos -------------------------------------------------------------------------------
photo = Image.open(PHOTO)
from PIL import ImageOps
photo = ImageOps.exif_transpose(photo)                    # 4284 x 5712
sc = photo.width / 1200
cart = photo.crop(tuple(int(v * sc) for v in (150, 40, 1050, 1600)))
place(cart, (71, 85, 192, 212))
cam_l = photo.crop(tuple(int(v * sc) for v in (140, 540, 320, 640)))
cam_r = photo.crop(tuple(int(v * sc) for v in (860, 620, 1010, 720)))
place(cam_l, (32, 760, 96, 818)); place(cam_r, (328, 760, 389, 818))

# ---- SLAM camera frames ----------------------------------------------------------------------
for k, box in enumerate([(361, 98, 427, 187), (433, 98, 499, 187), (529, 98, 595, 187)]):
    place(Image.open(REAL / f'slam_frame_{k}.png'), box)

# ---- recognition tiles -----------------------------------------------------------------------
place(Image.open(REAL / 'rgb.png'), (32, 422, 234, 512))
place(Image.open(REAL / 'depth.png'), (32, 543, 234, 632))
place(Image.open(REAL / 'est.png'), (287, 423, 527, 525))
x0, y0, x1, y1 = B(300, 577, 512, 630); draw.rectangle((x0, y0, x1, y1), fill=bgcol((300, 577, 512, 630)))
for j, xb in enumerate([(306, 362), (380, 436), (454, 510)]):
    place(Image.open(REAL / f'obj_{j}.png'), (xb[0], 578, xb[1], 629), mode='contain')
place(Image.open(REAL / 'map3d.png'), (1080, 425, 1352, 585), mode='contain')
place(Image.open(REAL / 'display.png'), (1394, 426, 1645, 607))

# ---- timeline (one capture, loop closure in flight, one arrival) --------------------------------
x0, y0, x1, y1 = B(692, 424, 1023, 553); draw.rectangle((x0, y0, x1, y1), fill=(255, 255, 255))
BLUE, ORANGE, AQUA = (42, 120, 214), (226, 74, 40), (27, 175, 122)
X = lambda x: int(x * S); Y = lambda y: int(y * S)
draw.line((X(696), Y(445), X(1018), Y(445)), fill=(205, 205, 200), width=S)
for x in np.arange(700, 1016, 7.5):
    draw.line((X(x), Y(439), X(x), Y(451)), fill=BLUE, width=S)
draw.line((X(696), Y(487), X(1018), Y(487)), fill=(205, 205, 200), width=S)
for x, k in [(733, 'K6'), (790, 'K7'), (891, 'K8'), (965, 'K9')]:
    draw.rectangle((X(x - 15), Y(478), X(x + 15), Y(496)), fill=BLUE)
    typeset(r'$\mathbf{%s}$' % k, (x - 13, 480, x + 13, 494), bold=True, color='white', bg=BLUE)
draw.line((X(696), Y(531), X(1018), Y(531)), fill=(205, 205, 200), width=S)
tc, tl, ta = 785, 839, 918
draw.line((X(tc), Y(452), X(tc), Y(524)), fill=(150, 150, 150), width=S)
draw.rectangle((X(tc), Y(527), X(ta), Y(535)), fill=(190, 232, 214))
draw.ellipse((X(tc - 7), Y(524), X(tc + 7), Y(538)), fill=AQUA)
draw.rectangle((X(ta - 7), Y(524), X(ta + 7), Y(538)), fill=AQUA)
draw.line((X(tl), Y(430), X(tl), Y(503)), fill=ORANGE, width=3 * S)

# ---- corrected text ----------------------------------------------------------------------------
typeset('RGB-D 30 Hz /', (236, 121, 350, 139))
typeset('LiDAR 10 Hz', (236, 140, 350, 158))
typeset('Keyframe list (periodic + map change)', (985, 180, 1325, 204), bold=True)
typeset(r'all keyframes (K, T$_{\mathrm{map}\leftarrow\mathrm{K}}$), re-sent after each map change', (950, 214, 1356, 240))
typeset('Depth (aligned)', (60, 637, 206, 656))
typeset(r'store T$_{\mathrm{K}\leftarrow\mathrm{O}}$ = T$_{\mathrm{map}\leftarrow\mathrm{K}}(t)^{-1}$ T$_{\mathrm{map}\leftarrow\mathrm{S}}(t)$ $\mathbf{X}$ T$_{\mathrm{R}\leftarrow\mathrm{O}}$',
        (620, 592, 1000, 619), align='left')

# ---- (d) real maps -----------------------------------------------------------------------------
D = json.load(open(CORR))
REF = {}
for o, p, i in D['positions']['D']:
    REF.setdefault(o, []).append(p)
REF = {o: np.median(np.array(v), axis=0) for o, v in REF.items()}
Rm = np.array(list(REF.values()))
K = np.array(list(D['kf'].values()))[np.argsort(np.array(list(map(int, D['kf'].keys()))))]
lim = None
for v, box in (('B', (463, 769, 836, 900)), ('D', (906, 769, 1305, 900))):
    x0, y0, x1, y1 = B(*box)
    fig, ax = plt.subplots(figsize=((x1 - x0) / 200, (y1 - y0) / 200), dpi=200)
    fig.subplots_adjust(0, 0, 1, 1)
    ax.plot(K[:, 2], K[:, 0], color='#b8b7b1', lw=1.4)
    P = D['positions'][v]; xy = np.array([[p[2], p[0]] for _, p, _ in P])
    far = np.array([np.hypot(p[0] - REF[o][0], p[2] - REF[o][2]) > 0.10 for o, p, _ in P])
    ax.scatter(xy[~far, 0], xy[~far, 1], s=34, color='#2a78d6', lw=0, zorder=3)
    ax.scatter(xy[far, 0], xy[far, 1], s=38, color='#eb6834', lw=0, zorder=4)
    ax.scatter(Rm[:, 2], Rm[:, 0], s=90, marker='+', color='#111', lw=1.6, zorder=5)
    ax.set_aspect('equal', adjustable='box'); ax.axis('off')
    if lim is None:
        allx = np.concatenate([np.percentile(K[:, 2], [1, 99]), xy[:, 0]]); ally = np.concatenate([np.percentile(K[:, 0], [1, 99]), xy[:, 1]])
        lim = ((allx.min() - 0.25, allx.max() + 0.25), (ally.min() - 0.2, ally.max() + 0.2))
    ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1])
    if v == 'D':
        bx = lim[0][0] + 0.3; by = lim[1][0] + 0.4
        ax.plot([bx, bx + 1], [by, by], color='#222', lw=1.6)
    fig.savefig(ROOT / 'src' / '_m.png', dpi=200, facecolor='white'); plt.close(fig)
    draw.rectangle((x0, y0, x1, y1), fill=(255, 255, 255))
    t = Image.open(ROOT / 'src' / '_m.png').convert('RGB').resize((x1 - x0, y1 - y0), Image.LANCZOS)
    img.paste(t, (x0, y0))
    print(v, int(far.sum()), len(P))

x0, y0, x1, y1 = B(1348, 862, 1662, 916); draw.rectangle((x0, y0, x1, y1), fill=bgcol((1348, 862, 1662, 916)))
for k, line in enumerate(['Copies on the right of the left map', 'were left behind by the', 'loop-closure correction.']):
    typeset(line, (1352, 862 + 18 * k, 1660, 879 + 18 * k), color='#c0392b', align='left')
out = ROOT / 'images' / 'fig_main_real.png'
img.save(out, dpi=(500, 500)); print(out, img.size)
for f in ('_t.png', '_m.png'):
    (ROOT / 'src' / f).unlink(missing_ok=True)
