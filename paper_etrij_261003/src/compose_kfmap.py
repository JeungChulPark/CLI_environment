"""Replace the pictures of the Korean overview draft with real data, keeping its layout and text.

Input : images/키프레임 앵커링 기반 객체 맵 업데이트 도식.png (draft, 1491x1055)
Output: images/fig_kfmap_real.png (2x)

Run used everywhere: large figure-eight, ORB-SLAM3 4000 features, repeat corr04 (loop closure moved
keyframes by up to 3.05 m). Sources: camera photo IMG_3942, recognition-camera frames, the run's logs
(kf_updates, sam6d_estimates, display_objects) and the anchoring replay (corr04.json, variants B and D).
The keyframe-graph icon is kept as a schematic.
"""
import json, sys
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np, cv2
from PIL import Image, ImageDraw, ImageFont, ImageOps
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib import font_manager

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
import os
RUN = REPO / 'objpose/output' / os.environ.get('OBJ_RUN', 'live_260901_cbnu_bigeightcircle_orbslam3_f4000_corr04')
REAL = ROOT / 'src' / os.environ.get('OBJ_REAL', 'real')
SRC = ROOT / 'images' / '키프레임 앵커링 기반 객체 맵 업데이트 도식.png'
CORR = json.load(open(sys.argv[1]))
S = 2
KFONT = '/home/jucpark/.local/share/fonts/malgun.ttf'
for p in ('/mnt/c/Windows/Fonts/malgun.ttf', '/mnt/c/Windows/Fonts/malgunbd.ttf'):
    if Path(p).exists():
        KFONT = p; break
font_manager.fontManager.addfont(KFONT)
KNAME = font_manager.FontProperties(fname=KFONT).get_name()
plt.rcParams.update({'font.family': [KNAME, 'Liberation Serif'], 'mathtext.fontset': 'stix', 'axes.unicode_minus': False})

base = Image.open(SRC).convert('RGB')
img = base.resize((base.width * S, base.height * S), Image.LANCZOS)
draw = ImageDraw.Draw(img)
B = lambda *b: tuple(int(v * S) for v in b)


def bg(box):
    x0, y0, x1, y1 = box
    px = [base.getpixel((x0 + 1, y)) for y in range(y0, y1, 3)] + [base.getpixel((x1 - 1, y)) for y in range(y0, y1, 3)]
    return tuple(int(np.median([p[i] for p in px])) for i in range(3))


def place(tile, box, mode='cover', fill=None):
    x0, y0, x1, y1 = B(*box); w, h = x1 - x0, y1 - y0
    t = tile.convert('RGB')
    r = max(w / t.width, h / t.height) if mode == 'cover' else min(w / t.width, h / t.height)
    t = t.resize((max(1, round(t.width * r)), max(1, round(t.height * r))), Image.LANCZOS)
    if mode == 'cover':
        l, u = (t.width - w) // 2, (t.height - h) // 2
        img.paste(t.crop((l, u, l + w, u + h)), (x0, y0))
    else:
        draw.rectangle((x0, y0, x1, y1), fill=fill or bg(box))
        img.paste(t, (x0 + (w - t.width) // 2, y0 + (h - t.height) // 2))


def text(s, box, size, color='#222', bold=False, fill=None, align='left'):
    x0, y0, x1, y1 = B(*box)
    draw.rectangle((x0, y0, x1, y1), fill=fill or bg(box))
    f = ImageFont.truetype(KFONT, size * S)
    tw = draw.textlength(s, font=f)
    x = x0 if align == 'left' else x0 + ((x1 - x0) - tw) / 2
    draw.text((x, y0 + ((y1 - y0) - size * S * 1.25) / 2), s, font=f, fill=color)


def fig_to_pil(fig):
    p = ROOT / 'src' / '_p.png'; fig.savefig(p, dpi=300, facecolor='white'); plt.close(fig)
    return Image.open(p).convert('RGB')


jl = lambda p: [json.loads(l) for l in open(p) if l.strip()]

# ---------------------------------------------------------------- data: keyframes before/after the 3 m correction
ups = jl(RUN / 'kf_updates.jsonl')
state, best = {}, (0, None, None)
for k, u in enumerate(ups):
    new = {int(r[0]): np.vstack([np.array(r[1:13]).reshape(3, 4), [0, 0, 0, 1]]) for r in u['kfs']}
    mv = [np.linalg.norm(new[i][:3, 3] - state[i][:3, 3]) for i in new if i in state]
    m = max(mv) if mv else 0
    if m > best[0]:
        best = (m, k, dict(state))
    state = new
KF_after, KF_before = state, best[2]
print('largest keyframe correction %.2f m' % best[0])
REF = defaultdict(list)
for o, p, _ in CORR['positions']['D']:
    REF[o].append(p)
REF = {o: np.median(np.array(v), 0) for o, v in REF.items()}
cnt = defaultdict(Counter)
for e in jl(RUN / 'sam6d_estimates.jsonl'):
    if e.get('anchor_kf') is not None:
        cnt[e['object']][int(e['anchor_kf'])] += 1
common = [k for k in KF_after if k in KF_before]
ANCH = {}
for o, p in REF.items():
    cand = [k for k, _ in cnt[o].most_common() if k in common]
    k = cand[0] if cand else min(common, key=lambda k: np.linalg.norm(KF_after[k][:3, 3] - p))
    ANCH[o] = (k, np.linalg.inv(KF_after[k]) @ np.append(p, 1))      # T_K<-O (position part)
OBJ_after = {o: (KF_after[k] @ r)[:3] for o, (k, r) in ANCH.items()}
OBJ_before = {o: (KF_before[k] @ r)[:3] for o, (k, r) in ANCH.items()}
EXT = json.loads((REPO / 'integration/cad_extents.json').read_text())
OBJT = {n: np.asarray(T) for n, T in json.load(open(REAL / 'map3d_objects.json')).items()}
COLB = {'milk': '#2a78d6', 'Febreze_high': '#1baf7a', 'saffron': '#eda100', 'choco_hazelnut_high': '#eb6834',
        'Bear': '#4a3aa7', 'Dinosaur': '#008300', 'Mugcup_high': '#e87ba4', 'Sikhye_high': '#e34948'}


def traj(K):
    ids = sorted(K); return np.array([K[i][:3, 3] for i in ids])


# ---------------------------------------------------------------- 3D renderer (point cloud + keyframe frusta + boxes)
d = np.load(REAL / 'map3d_data.npz'); PC, PCC = d['P'], d['C']
sys.path.insert(0, str(REPO / 'objpose' / 'pc'))
from hub import box_corners, BOX_EDGES  # noqa: E402
FLOOR = np.percentile(PC[:, 1], 97)


def render(center, az, el, dist, W, H, objs, kfs, anchors, labels, f=900, radius=2.6, pc_alpha=0.55, fr=0.18, lab_size=40, box_w=3):
    c = np.asarray(center, float)
    a, e = np.radians(az), np.radians(el)
    fwd = np.array([np.sin(a) * np.cos(e), np.sin(e), np.cos(a) * np.cos(e)])
    eye = c - fwd * dist
    right = np.cross(fwd, [0, -1.0, 0]); right /= np.linalg.norm(right); up = np.cross(right, fwd)
    R = np.stack([right, -up, fwd])
    def pr(X):
        Q = (R @ (np.atleast_2d(X) - eye).T).T
        return np.column_stack([f * Q[:, 0] / Q[:, 2] + W / 2, f * Q[:, 1] / Q[:, 2] + H / 2]), Q[:, 2]
    canvas = np.full((H, W, 3), 255, np.uint8)
    h = FLOOR - PC[:, 1]
    m = (h > 0.03) & (h < 1.8) & (np.linalg.norm(PC[:, [0, 2]] - c[[0, 2]], axis=1) < radius)
    uv, z = pr(PC[m]); col = PCC[m]
    for k in np.argsort(-z):
        u, v = int(uv[k, 0]), int(uv[k, 1])
        if 0 <= u < W - 1 and 0 <= v < H - 1 and z[k] > 0.1:
            canvas[v:v + 2, u:u + 2] = (col[k] * pc_alpha + 255 * (1 - pc_alpha)).astype(np.uint8)
    im = Image.fromarray(canvas[:, :, ::-1]); dr = ImageDraw.Draw(im)
    for kid, T, colr, w in kfs:                       # keyframe frusta
        pts = np.array([[0, 0, 0], [-1, -.75, 1.6], [1, -.75, 1.6], [1, .75, 1.6], [-1, .75, 1.6]]) * fr
        P = (T[:3, :3] @ pts.T).T + T[:3, 3]; q, zz = pr(P)
        if np.any(zz < 0.2):
            continue
        q = [tuple(p) for p in q]
        dr.polygon(q[1:], outline=colr, fill=None, width=w)
        for j in range(1, 5):
            dr.line([q[0], q[j]], fill=colr, width=w)
    for n, T in objs:                                 # object boxes
        P = (T[:3, :3] @ box_corners(EXT[n]).T).T + T[:3, 3]; q, zz = pr(P)
        if np.any(zz < 0.2):
            continue
        q = [tuple(p) for p in q]
        for a_, b_ in BOX_EDGES:
            dr.line([q[a_], q[b_]], fill=COLB.get(n, '#333'), width=box_w)
    for p0, p1, colr in anchors:                      # dashed relative-pose lines
        q, zz = pr(np.array([p0, p1]))
        for t in np.linspace(0, 1, 18)[::2]:
            a_ = q[0] + (q[1] - q[0]) * t; b_ = q[0] + (q[1] - q[0]) * min(t + 1 / 17, 1)
            dr.line([tuple(a_), tuple(b_)], fill=colr, width=2)
    fnt = ImageFont.truetype('/usr/share/fonts/truetype/liberation/LiberationSerif-Italic.ttf', lab_size)
    for X, s, colr, off in labels:
        q, zz = pr(np.array([X]))
        dr.text((q[0, 0] + off[0], q[0, 1] + off[1]), s, font=fnt, fill=colr)
    return im, pr


# ---------------------------------------------------------------- (a) existing modules
photo = ImageOps.exif_transpose(Image.open(Path('/home/jucpark/DeepLearning/CLI_environment/객체 위치 추정/IMG_3942.jpeg')))
sc = photo.width / 1200
cam = photo.crop(tuple(int(v * sc) for v in (140, 540, 320, 640)))
place(cam, (30, 136, 130, 222), mode='contain', fill=(255, 255, 255))
place(Image.open(REAL / 'kf_traj.png'), (156, 136, 285, 222), mode='contain', fill=(255, 255, 255))
place(Image.open(REAL / 'rgb.png'), (36, 358, 128, 442))
place(Image.open(REAL / 'depth.png'), (131, 358, 202, 442))
est = Image.open(REAL / 'est.png'); place(est.crop(tuple(map(int, os.environ.get('OBJ_EST_CROP', '90,60,560,380').split(',')))), (258, 352, 414, 434))

# ---------------------------------------------------------------- (b)-1 object in the keyframe frame
o1 = 'Febreze_high'; k1, _ = ANCH[o1]
T1 = OBJT[o1].copy(); T1[:3, 3] = OBJ_after[o1]
mid = OBJ_after[o1] * 0.5 + KF_after[k1][:3, 3] * 0.5
sep = np.linalg.norm(OBJ_after[o1] - KF_after[k1][:3, 3])
im1, _ = render(mid, 100, 18, 0.9 + 0.9 * max(sep, 1.0), 640, 300,
                [(o1, T1)], [(k1, KF_after[k1], '#1F5FA8', 5)],
                [(KF_after[k1][:3, 3], OBJ_after[o1], '#1F5FA8')],
                [(KF_after[k1][:3, 3], 'K_ref(t)', '#1F5FA8', (-60, 20)), (OBJ_after[o1], 'O', '#0d5c40', (30, -50))],
                f=520, radius=1.2 + sep, fr=0.3 * min(1.0, sep / 1.4), lab_size=34, box_w=5, pc_alpha=0.35)
print('obj-kf distance %.2f m' % sep)
place(im1, (488, 622, 722, 728), mode='contain', fill=(255, 255, 255))

# ---------------------------------------------------------------- (b)-2 keyframe-anchored object map (real run)
# the largest group of objects within 1 m of each other (one shelf or table)
_nb = {o: [q for q in REF if np.linalg.norm(REF[o][[0, 2]] - REF[q][[0, 2]]) < 1.0] for o in REF}
grp = max(_nb.values(), key=len)
print('group', grp)
cen = np.mean([OBJ_after[o] for o in grp], 0)
kf_ids = sorted(k for k in KF_after if np.linalg.norm(KF_after[k][:3, 3][[0, 2]] - cen[[0, 2]]) < 2.4)
anch = {ANCH[o][0] for o in grp}
kfs = [(k, KF_after[k], '#9fb8d8', 2) for k in kf_ids[::int(os.environ.get('OBJ_KF_STEP', '6'))] if k not in anch] + [(k, KF_after[k], '#1F5FA8', 5) for k in sorted(anch)]
objs = []
for o in grp:
    T = OBJT[o].copy(); T[:3, 3] = OBJ_after[o]; objs.append((o, T))
anc = [(KF_after[ANCH[o][0]][:3, 3], OBJ_after[o], COLB[o]) for o in grp]
labs = [(KF_after[k][:3, 3], f'K{k}', '#1F5FA8', (-30, 18)) for k in sorted(anch)] + [(OBJ_after[o], 'O', COLB[o], (14, -46)) for o in grp[:1]]
kc = np.mean([KF_after[k][:3, 3] for k in anch], 0)
im2, _ = render(cen * 0.55 + kc * 0.45, 200, 28, 3.2, 1200, 720, objs, kfs, anc, labs, f=900, radius=2.6, fr=float(os.environ.get('OBJ_FR', '0.22')), lab_size=44, box_w=5, pc_alpha=0.4)
place(im2, (745, 459, 1122, 697), mode='contain', fill=(255, 255, 255))
text('키프레임에 앵커링된 객체 맵 (실제 주행)', (748, 436, 1060, 459), 12, color='#111', fill=bg((748, 436, 1060, 459)))

# ---------------------------------------------------------------- (b)-3 loop closure: keyframes and objects before / after
Tb, Ta = traj(KF_before), traj(KF_after)
x0_, y0_, x1_, y1_ = B(1146, 318, 1212, 337); draw.rectangle((x0_, y0_, x1_, y1_), fill=(255, 255, 255))
fig, ax = plt.subplots(figsize=(1.8, 0.75)); fig.subplots_adjust(0, 0, 1, 1)
ax.plot(Tb[:, 2], Tb[:, 0], color='#9a9a94', lw=0.9, ls='--')
ax.plot(Ta[:, 2], Ta[:, 0], color='#1F5FA8', lw=0.9)
ax.set_aspect('equal'); ax.axis('off')
place(fig_to_pil(fig), (1146, 334, 1330, 400), mode='contain', fill=(255, 255, 255))
text('보정 전 (점선)', (1150, 403, 1238, 425), 12, fill=bg((1150, 403, 1238, 425)))
text('보정 후 (실선)', (1245, 403, 1330, 425), 12, fill=bg((1245, 403, 1330, 425)))
for objsrc, box in ((OBJ_before, (1146, 638, 1242, 738)), (OBJ_after, (1250, 638, 1334, 738))):
    fig, ax = plt.subplots(figsize=(0.95, 1.0)); fig.subplots_adjust(0, 0, 1, 1)
    TT = Tb if objsrc is OBJ_before else Ta
    ax.plot(TT[:, 2], TT[:, 0], color='#c9c8c2', lw=0.6, ls='--' if objsrc is OBJ_before else '-')
    for o, p in objsrc.items():
        ax.scatter(p[2], p[0], s=22, color=COLB[o], lw=0)
    allp = np.vstack([Ta, Tb]); ax.set_xlim(allp[:, 2].min() - .2, allp[:, 2].max() + .2); ax.set_ylim(allp[:, 0].min() - .2, allp[:, 0].max() + .2)
    ax.set_aspect('equal'); ax.axis('off')
    place(fig_to_pil(fig), box, mode='contain', fill=(255, 255, 255))

# ---------------------------------------------------------------- (b)-4 visualization
place(Image.open(REAL / 'map3d.png'), (1353, 324, 1466, 470), mode='contain', fill=(255, 255, 255))
place(Image.open(REAL / 'display.png'), (1353, 538, 1466, 680))

# ---------------------------------------------------------------- (c) conventional vs proposed on the real run
REFa = np.array(list(REF.values()))
for v, box, title in (('B', (18, 866, 528, 1042), None), ('D', (772, 866, 1228, 1042), None)):
    fig, ax = plt.subplots(figsize=((box[2] - box[0]) / 100, (box[3] - box[1]) / 100)); fig.subplots_adjust(0, 0, 1, 1)
    ax.plot(Tb[:, 2], Tb[:, 0], color='#2a78d6', lw=0.8, ls='--', label='키프레임 (보정 전)')
    ax.plot(Ta[:, 2], Ta[:, 0], color='#e34948', lw=0.8, label='키프레임 (보정 후)')
    P = CORR['positions'][v]; xy = np.array([[p[2], p[0]] for _, p, _ in P])
    far = np.array([np.hypot(p[0] - REF[o][0], p[2] - REF[o][2]) > 0.10 for o, p, _ in P])
    ax.scatter(xy[~far, 0], xy[~far, 1], s=26, color='#1baf7a', lw=0, zorder=3, label='객체 (최종 위치 10 cm 이내)')
    ax.scatter(xy[far, 0], xy[far, 1], s=30, color='#eb6834', lw=0, zorder=4, label='객체 (10 cm 초과, 남은 사본)')
    ax.scatter(REFa[:, 2], REFa[:, 0], s=70, marker='+', color='#111', lw=1.2, zorder=5, label='최종 객체 위치')
    ax.set_aspect('equal', adjustable='datalim'); ax.axis('off')
    ax.text(0.01, 0.97, f'{int(far.sum())}/{len(P)} 객체 > 10 cm', transform=ax.transAxes, ha='left', va='top', fontsize=10, color='#a63c12' if v == 'B' else '#0d5c40', fontweight='bold')
    if v == 'B':
        h, l = ax.get_legend_handles_labels()
    place(fig_to_pil(fig), box, mode='contain', fill=bg((box[0], box[1], box[2], box[3])))
# redraw the left callout over the plot (same wording as the draft)
x0, y0, x1, y1 = B(360, 958, 526, 1030)
draw.rounded_rectangle((x0, y0, x1, y1), radius=8 * S, fill=(253, 233, 233), outline=(232, 120, 120), width=S)
for i, s in enumerate(['루프 클로저 후에도', '객체가 원래 위치에 남아', '일관성이 깨짐']):
    f = ImageFont.truetype(KFONT, 12 * S)
    draw.text((x0 + 8 * S, y0 + (5 + 21 * i) * S), s, font=f, fill=(192, 57, 43))
# new legend in place of the drawn-symbol legend
lx0, ly0, lx1, ly1 = B(532, 872, 712, 1040)
draw.rectangle((lx0, ly0, lx1, ly1), fill=bg((532, 872, 712, 1040)))
fig = plt.figure(figsize=(1.8, 1.68)); fig.legend(h, l, loc='center', frameon=False, fontsize=8, handlelength=1.6, markerscale=1.4)
place(fig_to_pil(fig), (532, 872, 712, 1040), mode='contain', fill=bg((532, 872, 712, 1040)))

out = ROOT / 'images' / os.environ.get('OBJ_OUT', 'fig_kfmap_real.png')
img.save(out, dpi=(400, 400)); print(out, img.size)
(ROOT / 'src' / '_p.png').unlink(missing_ok=True)
