"""Keep the drafted figure (layout, text, style) and replace its pictures with real data.

Base : images/비동기 인식과 키프레임 기반 3D 객체 매핑.png (1536x1024)
Out  : images/fig3_real_v2.png (2x)

Real sources: 260915 run (sensor frames, keyframes, 6D stage dumps in src/real_stage, object state map,
point cloud), 260901 corr04 run for the loop-closure panel (3.05 m correction), cart photo.
3D scenes are rendered from real point clouds, real keyframe poses and real object poses with filled
boxes sized by the CAD model, so they read like the draft.
"""
import json, sys
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np, cv2
from PIL import Image, ImageDraw, ImageFont, ImageOps

ROOT = Path(__file__).resolve().parents[1]; REPO = ROOT.parent
SRC = ROOT / 'src'; ST = SRC / 'real_stage'; F3 = SRC / 'fig3'; R915 = SRC / 'real915'; R01 = SRC / 'real'
SCR = Path(sys.argv[1])
sys.path.insert(0, str(REPO / 'objpose' / 'pc'))
from hub import box_corners  # noqa: E402

S = 2
base = Image.open(ROOT / 'images' / '비동기 인식과 키프레임 기반 3D 객체 매핑.png').convert('RGB')
img = base.resize((base.width * S, base.height * S), Image.LANCZOS)
dr = ImageDraw.Draw(img)
SANS = '/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf'
SANSB = '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf'
SERI = '/usr/share/fonts/truetype/liberation/LiberationSerif-Italic.ttf'
B = lambda *b: tuple(int(round(v * S)) for v in b)
COL = {'milk': (42, 120, 214), 'Febreze_high': (27, 175, 122), 'saffron': (237, 161, 0), 'choco_hazelnut_high': (235, 104, 52),
       'Bear': (124, 82, 50), 'Dinosaur': (0, 150, 60), 'Mugcup_high': (232, 123, 164), 'Sikhye_high': (227, 73, 72)}
NAME = {'milk': 'milk carton', 'Febreze_high': 'spray', 'saffron': 'jug', 'choco_hazelnut_high': 'snack box',
        'Bear': 'teddy bear', 'Dinosaur': 'dino doll', 'Mugcup_high': 'mug', 'Sikhye_high': 'can'}
jl = lambda p: [json.loads(l) for l in open(p) if l.strip()]


def bg(box):
    x0, y0, x1, y1 = [int(v) for v in box]
    px = [base.getpixel((x0 - 1, y)) for y in range(y0, y1, 2)] + [base.getpixel((x1 + 1, y)) for y in range(y0, y1, 2)]
    return tuple(int(np.median([p[i] for p in px])) for i in range(3))


def place(tile, box, mode='cover', fill=None, border=None):
    x0, y0, x1, y1 = B(*box); w, h = x1 - x0, y1 - y0
    t = tile.convert('RGB') if not isinstance(tile, np.ndarray) else Image.fromarray(tile[:, :, ::-1])
    r = max(w / t.width, h / t.height) if mode == 'cover' else min(w / t.width, h / t.height)
    t = t.resize((max(1, round(t.width * r)), max(1, round(t.height * r))), Image.LANCZOS)
    if mode == 'cover':
        l, u = (t.width - w) // 2, (t.height - h) // 2; img.paste(t.crop((l, u, l + w, u + h)), (x0, y0))
    else:
        dr.rectangle((x0, y0, x1, y1), fill=fill or bg(box)); img.paste(t, (x0 + (w - t.width) // 2, y0 + (h - t.height) // 2))
    if border:
        dr.rectangle((x0, y0, x1, y1), outline=border, width=2 * S)


def text(s, box, size, color=(30, 30, 30), font=SANS, fill=None, align='center'):
    x0, y0, x1, y1 = B(*box)
    if fill is not False:
        dr.rectangle((x0, y0, x1, y1), fill=fill or bg(box))
    f = ImageFont.truetype(font, int(size * S)); tw = dr.textlength(s, font=f)
    x = x0 + ((x1 - x0) - tw) / 2 if align == 'center' else x0
    dr.text((x, y0 + ((y1 - y0) - size * S * 1.15) / 2), s, font=f, fill=color)


def crop_rgb(name):
    return Image.open(ST / f'crop_rgb_{name}.png')


# ---------------------------------------------------------------- filled-box 3D renderer
class Cam:
    def __init__(s, center, az, el, dist, W, H, f):
        c = np.asarray(center, float); a, e = np.radians(az), np.radians(el)
        fwd = np.array([np.sin(a) * np.cos(e), np.sin(e), np.cos(a) * np.cos(e)]); s.eye = c - fwd * dist
        right = np.cross(fwd, [0, -1.0, 0]); right /= np.linalg.norm(right); up = np.cross(right, fwd)
        s.R = np.stack([right, -up, fwd]); s.W, s.H, s.f = W, H, f

    def __call__(s, X):
        Q = (s.R @ (np.atleast_2d(X) - s.eye).T).T
        return np.column_stack([s.f * Q[:, 0] / Q[:, 2] + s.W / 2, s.f * Q[:, 1] / Q[:, 2] + s.H / 2]), Q[:, 2]


FACES = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def scene(cam, P=None, C=None, alpha=0.5, radius=None, center=None, objs=(), kfs=(), lines=(), scale=1.0, pts=2):
    im = Image.new('RGB', (cam.W, cam.H), 'white')
    if P is not None and len(P):
        floor = np.percentile(P[:, 1], 97); h = floor - P[:, 1]
        m = (h > 0.02) & (h < 1.9)
        if radius:
            m &= np.linalg.norm(P[:, [0, 2]] - np.asarray(center)[[0, 2]], axis=1) < radius
        uv, z = cam(P[m]); col = C[m]; a = np.full((cam.H, cam.W, 3), 255, np.uint8)
        for k in np.argsort(-z):
            u, v = int(uv[k, 0]), int(uv[k, 1])
            if 0 <= u < cam.W - pts and 0 <= v < cam.H - pts and z[k] > 0.1:
                a[v:v + pts, u:u + pts] = (col[k] * alpha + 255 * (1 - alpha)).astype(np.uint8)
        im = Image.fromarray(a[:, :, ::-1])
    ov = Image.new('RGBA', im.size, (0, 0, 0, 0)); d = ImageDraw.Draw(ov); pix = {}
    for p0, p1, colr in lines:
        q, _ = cam(np.array([p0, p1]))
        for t in np.linspace(0, 1, 26)[::2]:
            A = q[0] + (q[1] - q[0]) * t; Bq = q[0] + (q[1] - q[0]) * min(t + 1 / 25, 1)
            d.line([tuple(A), tuple(Bq)], fill=colr + (255,), width=3)
    for kid, T, colr, w, fr in kfs:                      # keyframe pyramids
        P5 = np.array([[0, 0, 0], [-1, -.75, 1.6], [1, -.75, 1.6], [1, .75, 1.6], [-1, .75, 1.6]]) * fr
        q, zz = cam((T[:3, :3] @ P5.T).T + T[:3, 3])
        if np.any(zz < 0.2):
            continue
        q = [tuple(p) for p in q]
        d.polygon(q[1:], fill=colr + (60,), outline=colr + (255,), width=w)
        for j in range(1, 5):
            d.line([q[0], q[j]], fill=colr + (255,), width=w)
        pix[kid] = q[0]
    order = []
    for name, T, style in objs:
        X = (T[:3, :3] @ (box_corners(EXT[name]) * scale).T).T + T[:3, 3]; q, zz = cam(X)
        if np.any(zz < 0.2):
            continue
        order.append((zz.mean(), name, q, style))
    for _, name, q, style in sorted(order, key=lambda o: -o[0]):
        c = COL[name]; q = [tuple(p) for p in q]
        if style == 'ghost':
            for a_, b_ in EDGES:
                A = np.array(q[a_]); Bq = np.array(q[b_])
                for t in np.linspace(0, 1, 9)[::2]:
                    d.line([tuple(A + (Bq - A) * t), tuple(A + (Bq - A) * min(t + .125, 1))], fill=c + (200,), width=3)
            continue
        for fa in FACES:
            shade = 0.75 + 0.25 * (FACES.index(fa) % 3) / 2
            d.polygon([q[i] for i in fa], fill=tuple(int(v * shade) for v in c) + (150,))
        for a_, b_ in EDGES:
            d.line([q[a_], q[b_]], fill=tuple(int(v * 0.6) for v in c) + (255,), width=3)
        pix[name] = (float(np.mean([p[0] for p in q])), float(min(p[1] for p in q)), float(max(p[1] for p in q)))
    im = Image.alpha_composite(im.convert('RGBA'), ov).convert('RGB')
    return im, pix


def kf_states(run):
    state, best = {}, (0, None, {})
    for u in jl(run / 'kf_updates.jsonl'):
        new = {int(r[0]): np.vstack([np.array(r[1:13]).reshape(3, 4), [0, 0, 0, 1]]) for r in u['kfs']}
        mv = [np.linalg.norm(new[i][:3, 3] - state[i][:3, 3]) for i in new if i in state]
        if mv and max(mv) > best[0]:
            best = (max(mv), u['t_ns'], dict(state))
        state = new
    return state, best


def anchors(run, corr, kA, kB):
    REF = defaultdict(list)
    for o, p, _ in corr['positions']['D']:
        REF[o].append(p)
    REF = {o: np.median(np.array(v), 0) for o, v in REF.items()}
    cnt = defaultdict(Counter)
    for e in jl(run / 'sam6d_estimates.jsonl'):
        if e.get('anchor_kf') is not None:
            cnt[e['object']][int(e['anchor_kf'])] += 1
    common = [k for k in kA if k in kB]; A = {}
    for o, p in REF.items():
        cand = [k for k, _ in cnt[o].most_common() if k in common]
        k = cand[0] if cand else min(common, key=lambda k: np.linalg.norm(kA[k][:3, 3] - p))
        A[o] = (k, np.linalg.inv(kA[k]) @ np.append(p, 1))
    return REF, A


def objT(real, name, pos):
    T = np.asarray(json.load(open(real / 'map3d_objects.json'))[name]).copy(); T[:3, 3] = pos; return T


# CAD extents: integration/cad_extents.json is stale for milk and Dinosaur (see real_stage report); scale them
EXT = json.loads((REPO / 'integration/cad_extents.json').read_text())
for n, k in (('milk', 230.65 / 195.0), ('Dinosaur', 1.25)):
    EXT[n] = {'min_mm': (np.array(EXT[n]['min_mm']) * k).tolist(), 'max_mm': (np.array(EXT[n]['max_mm']) * k).tolist(), 'size_mm': (np.array(EXT[n]['size_mm']) * k).tolist()}

# ================================================================ panel 1: sensor input
photo = ImageOps.exif_transpose(Image.open(Path('/home/jucpark/DeepLearning/CLI_environment/객체 위치 추정/IMG_3942.jpeg')))
sc = photo.width / 1200
place(photo.crop(tuple(int(v * sc) for v in (130, 60, 1080, 1560))), (28, 117, 167, 328))
place(Image.open(ST / 'rgb.png'), (180, 143, 297, 215))
place(Image.open(ST / 'depth.png'), (180, 246, 297, 329))

# ================================================================ SLAM strip (real keyframe images)
for j, x in enumerate((515, 584, 653, 722)):
    place(Image.open(F3 / f'strip_{j + 1}.png'), (x, 82, x + 66, 109))

# ================================================================ panel 2: 6D estimation
place(Image.open(ST / 'rgb.png'), (333, 273, 416, 335))
# object icons (2x2) -> real accepted crops
for (x, y), n in zip(((333, 346), (375, 346), (333, 409), (375, 409)), ('milk', 'saffron', 'Febreze_high', 'Dinosaur')):
    place(crop_rgb(n), (x + 2, y + 2, x + 40, y + 60), mode='contain', fill=(246, 249, 253))
text('"milk carton"', (431, 285, 489, 296), 6.2, color=(60, 60, 160), fill=(240, 240, 252))
text('"white jug"', (431, 297, 489, 308), 6.2, color=(60, 60, 160), fill=(240, 240, 252))
text('"dino doll" ...', (431, 309, 489, 320), 6.2, color=(60, 60, 160), fill=(240, 240, 252))
place(Image.open(ST / 'yolo_boxes.png').crop((60, 60, 600, 440)), (496, 279, 571, 334))
place(Image.open(ST / 'masks.png').crop((60, 60, 600, 440)), (577, 279, 649, 334))
g = json.load(open(ST / 'gates.json')); cands = [c for c in g['candidates']] if 'candidates' in g else None
# gate rows: accepted example (left icon) / rejected example (right icon)
masks = Image.open(ST / 'masks.png')
acc = [crop_rgb('milk'), crop_rgb('saffron'), crop_rgb('Febreze_high'), crop_rgb('Dinosaur')]
rej = Image.open(ST / 'crop_reject.png'); rej2 = Image.open(ST / 'crop_reject_alt.png')
for i, y in enumerate((339, 370, 401, 432)):
    place(acc[i], (577, y + 1, 598, y + 23), mode='contain', fill=(255, 255, 255))
    place([rej, rej2, rej, rej2][i], (612, y + 1, 633, y + 23), mode='contain', fill=(255, 255, 255))
# 2-2
place(crop_rgb('milk'), (674, 279, 700, 326), mode='contain', fill=(255, 255, 255))
place(Image.open(ST / 'crop_mask_milk.png'), (701, 279, 726, 326), mode='contain', fill=(0, 0, 0))
hyps = [Image.open(ST / f'hyp_{k}.png') for k in range(6) if (ST / f'hyp_{k}.png').exists()]
x0, y0 = 741, 274
for k, h in enumerate(hyps[:4]):
    place(h, (x0 + (k % 2) * 27, y0 + (k // 2) * 27, x0 + (k % 2) * 27 + 25, y0 + (k // 2) * 27 + 25), mode='contain', fill=(255, 255, 255))
text('· · ·', (796, 290, 822, 310), 9, fill=(255, 255, 255))
place(Image.open(ST / 'fine.png'), (835, 274, 894, 327), mode='contain', fill=(255, 255, 255))
for k, (xa, xb) in enumerate(((667, 744), (746, 824), (826, 903))):
    if len(hyps) > k:
        place(hyps[[0, 2, 5][k] if len(hyps) > 5 else k], (xa + 2, 382, xb - 2, 414), mode='contain', fill=(255, 255, 255))
pem = json.load(open(ST / 'pem.json'))
text(f'IoU {pem["final_mask_iou"]:.2f} > 0.42 ✓', (667, 415, 744, 425), 5.6, color=(20, 120, 60))
text(f'feature {pem["final_texture_score"]:.2f} > 0.45 ✓', (746, 415, 824, 425), 5.6, color=(20, 120, 60))
text(f'cluster {pem["cluster_size"]}/300 ✓', (826, 415, 903, 425), 5.6, color=(20, 120, 60))
# output row of the 6D estimation
text('1', (666, 469, 690, 494), 10); text('milk', (692, 469, 728, 494), 10)
place(crop_rgb('milk'), (736, 466, 760, 494), mode='contain', fill=(255, 255, 255))
text('59.75 s', (764, 469, 814, 494), 10); text('1.00', (816, 469, 864, 494), 10)
place(Image.open(ST / 'crop_mask_milk.png'), (866, 470, 884, 492), mode='contain', fill=(0, 0, 0))

# ================================================================ panel 3: real rates

# ================================================================ panel 4: real reference keyframes
for k, x in zip(('kf_prev', 'kf_ref', 'kf_next'), ((1197, 1275), (1300, 1372), (1396, 1472))):
    place(Image.open(F3 / f'{k}.png'), (x[0], 345, x[1], 381))
place(Image.open(ST / 'fine.png'), (1199, 462, 1235, 506), mode='contain', fill=(250, 250, 255))

# ================================================================ anchored observation row
place(crop_rgb('milk'), (436, 550, 498, 602), mode='contain', fill=(255, 245, 245))
text('milk', (556, 580, 616, 600), 11); text('59.75 s', (848, 580, 930, 600), 11); text('1.00', (962, 580, 1018, 600), 11)
place(Image.open(ST / 'crop_mask_milk.png'), (1050, 576, 1074, 600), mode='contain', fill=(0, 0, 0))
place(Image.open(ST / 'crop_mask_saffron.png'), (1076, 576, 1100, 600), mode='contain', fill=(0, 0, 0))

# ================================================================ panel 5: real keyframe-anchored object map (260915)
run915 = REPO / 'objpose/output/live_260915_eightcircle_orbslam3'
C915 = json.load(open(SCR / 'r915.json'))
kA, (mv, _, kB) = kf_states(run915)
REF, ANC = anchors(run915, C915, kA, kB)
OBJ = {o: (kA[k] @ r)[:3] for o, (k, r) in ANC.items()}
d = np.load(R915 / 'map3d_data.npz')
cen = np.mean(list(OBJ.values()), 0)
cam = Cam(cen + np.array([0, -0.25, 0]), 35, 42, 3.6, 1320, 720, 820)
an = sorted({k for k, _ in ANC.values()})
kfs = [(k, kA[k], (120, 150, 200), 2, 0.12) for k in sorted(kA)[::4] if k not in an] + [(k, kA[k], (31, 95, 168), 4, 0.17) for k in an]
lines = [(kA[k][:3, 3], OBJ[o], COL[o]) for o, (k, _) in ANC.items()]
traj = np.array([kA[k][:3, 3] for k in sorted(kA)])
im5, pix = scene(cam, d['P'], d['C'], alpha=0.42, radius=4.2, center=cen,
                 objs=[(o, objT(R915, o, OBJ[o]), 'solid') for o in OBJ], kfs=kfs, lines=lines, scale=1.7)
V3 = SRC / 'fig_v3'; V3.mkdir(exist_ok=True)
_q, _ = cam(np.array([kA[k][:3, 3] for k in sorted(kA)]))
_tmp = im5.copy(); _d = ImageDraw.Draw(_tmp)
for a_, b_ in zip(_q[:-1], _q[1:]):
    _d.line([tuple(a_), tuple(b_)], fill=(31, 95, 168), width=2)
_tmp.save(V3 / 'statemap.png')
json.dump({'W': cam.W, 'H': cam.H, 'pix': {k: list(map(float, v)) for k, v in pix.items() if isinstance(k, str)},
           'kf': {str(k): list(map(float, v)) for k, v in pix.items() if not isinstance(k, str)},
           'anchor': {o: int(k) for o, (k, _) in ANC.items()}, 'ids': ids if 'ids' in dir() else None}, open(V3 / 'statemap.json', 'w'))
dd = ImageDraw.Draw(im5)
q, _ = cam(traj)
for a_, b_ in zip(q[:-1], q[1:]):
    dd.line([tuple(a_), tuple(b_)], fill=(31, 95, 168), width=2)
# callouts in the draft's style: "ID: n (label) ref: K_k"
fB = ImageFont.truetype(SANSB, 26); fN = ImageFont.truetype(SANS, 24)
srt = sorted(C915 and json.load(open(run915 / 'summary.json'))['objects'].items(), key=lambda x: -x[1]['est_count'])
ids = {n: i for i, (n, _) in enumerate(srt, 1)}
vis = sorted([o for o in ANC if o in pix], key=lambda o: pix[o][0])
for j, o in enumerate(vis):          # callouts in fixed slots along the top, ordered by x like the draft
    k = ANC[o][0]; cx, ytop, _ = pix[o]; c = COL[o]
    lines_t = [f'ID: {ids[o]}', f'({NAME[o]})', f'ref: K{k}']
    w = max(dd.textlength(t, font=fN) for t in lines_t) + 16; h = 3 * 28 + 8
    bx = 430 + j * (1320 - 430 - 110) / max(1, len(vis) - 1) - w / 2; by = 18 + (j % 2) * 118
    dd.line([(bx + w / 2, by + h), (cx, ytop)], fill=c, width=3)
    dd.rounded_rectangle((bx, by, bx + w, by + h), radius=8, fill=(255, 255, 255), outline=c, width=3)
    for i_, t in enumerate(lines_t):
        dd.text((bx + 8, by + 4 + i_ * 28), t, font=fB if i_ == 0 else fN, fill=c)
place(im5, (15, 698, 456, 936))
f6 = ImageFont.truetype(SANS, int(8.4 * S))
dr.text(B(20, 702)[:2], '▲ : SLAM keyframe (K)', font=f6, fill=(31, 95, 168))
dr.text(B(20, 716)[:2], '— : keyframe trajectory', font=f6, fill=(31, 95, 168))
dr.text(B(20, 730)[:2], '■ : tracked object (anchored to K)', font=f6, fill=(180, 120, 20))
# object state table (top 4 by observations, real)
summ = json.load(open(run915 / 'summary.json'))['objects']
scores = defaultdict(list)
for l in open(run915 / 'sam6d/detections.jsonl'):
    x = json.loads(l); scores[x['object']].append(x['score'])
rows = [n for n, _ in srt][:4]
for i, n in enumerate(rows):
    y = 747 + i * 20.5
    dr.rectangle(B(462, y - 1, 800, y + 16), fill=(255, 255, 255))
    for txt, (xa, xb), font in ((str(ids[n]), (462, 482), SANS), (NAME[n], (488, 545), SANS), (f'K{ANC[n][0]}', (548, 590), SERI),
                                (f'{np.median(scores[n]):.2f}', (640, 690), SANS), (str(summ[n]['est_count']), (705, 750), SANS)):
        text(txt, (xa, y, xb, y + 15), 9.4, fill=(255, 255, 255), font=font, color=COL[n] if xa == 488 else (30, 30, 30))
    place(crop_rgb(n) if (ST / f'crop_rgb_{n}.png').exists() else Image.open(F3 / f'objcrop_{n}.png'), (606, y, 626, y + 16), mode='contain', fill=(255, 255, 255))
    x0_, y0_, x1_, y1_ = B(755, y, 798, y + 15); dr.rectangle((x0_, y0_, x1_, y1_), fill=(205, 238, 214)); text('active', (755, y, 798, y + 15), 8, fill=False)

# ================================================================ panel 6: real loop closure (260901, 3.05 m)
run01 = REPO / 'objpose/output/live_260901_cbnu_bigeightcircle_orbslam3_f4000_corr04'
C01 = json.load(open(SCR / 'corr04.json'))
kA1, (mv1, _, kB1) = kf_states(run01)
REF1, ANC1 = anchors(run01, C01, kA1, kB1)
grp = max(({o: [q for q in REF1 if np.linalg.norm(REF1[o][[0, 2]] - REF1[q][[0, 2]]) < 1.0] for o in REF1}).values(), key=len)
an1 = sorted({ANC1[o][0] for o in grp})
posA = {o: (kA1[ANC1[o][0]] @ ANC1[o][1])[:3] for o in grp}
posB = {o: (kB1[ANC1[o][0]] @ ANC1[o][1])[:3] for o in grp}
mid = np.mean(list(posA.values()) + list(posB.values()), 0)
for tag, KF, pos, box, kc in (('before', kB1, posB, (955, 720, 1170, 840), (200, 70, 50)),
                              ('after', kA1, posA, (1285, 720, 1500, 840), (31, 95, 168))):
    ctr = mid if tag == 'before' else np.mean(list(posA.values()), 0)
    cam = Cam(ctr, 215, 40, 4.6 if tag == 'before' else 3.4, 900, 520, 660)
    near = [k for k in sorted(KF) if np.linalg.norm(KF[k][:3, 3][[0, 2]] - mid[[0, 2]]) < 3.4][::3]
    kfs = [(k, KF[k], (170, 170, 170), 2, 0.2) for k in near if k not in an1] + [(k, KF[k], kc, 4, 0.28) for k in an1]
    objs = [(o, objT(R01, o, pos[o]), 'solid') for o in grp]
    if tag == 'before':
        objs += [(o, objT(R01, o, posA[o]), 'ghost') for o in grp]
    im6, _ = scene(cam, objs=objs, kfs=kfs, lines=[(KF[ANC1[o][0]][:3, 3], pos[o], COL[o]) for o in grp], scale=2.6)
    place(im6, box, mode='contain', fill=(255, 255, 255))
    im6.save(V3 / f'loop_{tag}.png')


# ================================================================ panel 7: result strip
_m = Image.open(R915 / 'map3d.png').convert('RGB'); _m = _m.crop(ImageOps.invert(_m).getbbox())
place(_m, (855, 950, 1180, 1012), mode='cover')
for x, n in zip((1205, 1272, 1339, 1406), ('milk', 'saffron', 'Dinosaur', 'Febreze_high')):
    place(Image.open(F3 / f'objcrop_{n}.png'), (x, 945, x + 60, 1005), mode='contain', fill=(255, 255, 255))

dr.rectangle(B(1500, 712, 1512, 845), fill=bg((1500, 712, 1512, 845)))
out = ROOT / 'images' / 'fig3_real_v2.png'
img.save(out, dpi=(400, 400)); print(out, img.size, 'loop', round(mv1, 2))
