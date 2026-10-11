"""Real tiles for the English overview figure (fig3.html).

  kf_<k>.png          SLAM-camera frame at the time keyframe k was the tracking reference (260915 run)
  statemap.png        260915: point cloud from final poses + keyframe frusta + object boxes + anchor lines
  statemap_lab.json   pixel positions of objects/anchor keyframes in statemap.png (for HTML callouts)
  loop_before.png / loop_after.png   260901 corr04 (loop closure moved keyframes by 3.05 m):
                      keyframes and anchored objects just before and after the largest correction
  objcrop_<name>.png  object crops with verified pose (from the 260915 run's estimate frame)
"""
import json, os, sys
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np, cv2
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]; REPO = ROOT.parent
sys.path.insert(0, str(REPO / 'objpose' / 'pc'))
from conv_session import ConvSession            # noqa: E402
from hub import box_corners, BOX_EDGES, project  # noqa: E402
OUT = ROOT / 'src' / 'fig3'; OUT.mkdir(exist_ok=True)
EXT = json.loads((REPO / 'integration/cad_extents.json').read_text())
COL = {'milk': '#2a78d6', 'Febreze_high': '#1baf7a', 'saffron': '#eda100', 'choco_hazelnut_high': '#eb6834',
       'Bear': '#4a3aa7', 'Dinosaur': '#008300', 'Mugcup_high': '#e87ba4', 'Sikhye_high': '#e34948'}
jl = lambda p: [json.loads(l) for l in open(p) if l.strip()]
SCR = Path(sys.argv[1])          # scratchpad with r915.json and corr04.json


def kf_states(run):
    ups = jl(run / 'kf_updates.jsonl'); state, best = {}, (0, None, {})
    for u in ups:
        new = {int(r[0]): np.vstack([np.array(r[1:13]).reshape(3, 4), [0, 0, 0, 1]]) for r in u['kfs']}
        mv = [np.linalg.norm(new[i][:3, 3] - state[i][:3, 3]) for i in new if i in state]
        if mv and max(mv) > best[0]:
            best = (max(mv), u['t_ns'], dict(state))
        state = new
    return state, best


def anchors(run, corr, kf_after, kf_before):
    REF = defaultdict(list)
    for o, p, _ in corr['positions']['D']:
        REF[o].append(p)
    REF = {o: np.median(np.array(v), 0) for o, v in REF.items()}
    cnt = defaultdict(Counter)
    for e in jl(run / 'sam6d_estimates.jsonl'):
        if e.get('anchor_kf') is not None:
            cnt[e['object']][int(e['anchor_kf'])] += 1
    common = [k for k in kf_after if k in kf_before]
    A = {}
    for o, p in REF.items():
        cand = [k for k, _ in cnt[o].most_common() if k in common]
        k = cand[0] if cand else min(common, key=lambda k: np.linalg.norm(kf_after[k][:3, 3] - p))
        A[o] = (k, np.linalg.inv(kf_after[k]) @ np.append(p, 1))
    return REF, A


def render(P, C, center, az, el, dist, W, H, objs, kfs, lines, f, radius, alpha=0.45, fr=0.18):
    c = np.asarray(center, float); a, e = np.radians(az), np.radians(el)
    fwd = np.array([np.sin(a) * np.cos(e), np.sin(e), np.cos(a) * np.cos(e)]); eye = c - fwd * dist
    right = np.cross(fwd, [0, -1.0, 0]); right /= np.linalg.norm(right); up = np.cross(right, fwd)
    R = np.stack([right, -up, fwd])
    def pr(X):
        Q = (R @ (np.atleast_2d(X) - eye).T).T
        return np.column_stack([f * Q[:, 0] / Q[:, 2] + W / 2, f * Q[:, 1] / Q[:, 2] + H / 2]), Q[:, 2]
    canvas = np.full((H, W, 3), 255, np.uint8)
    if len(P):
        floor = np.percentile(P[:, 1], 97); h = floor - P[:, 1]
        m = (h > 0.03) & (h < 1.8) & (np.linalg.norm(P[:, [0, 2]] - c[[0, 2]], axis=1) < radius)
        uv, z = pr(P[m]); col = C[m]
    else:
        uv, z, col = np.zeros((0, 2)), np.zeros(0), np.zeros((0, 3))
    for k in np.argsort(-z):
        u, v = int(uv[k, 0]), int(uv[k, 1])
        if 0 <= u < W - 1 and 0 <= v < H - 1 and z[k] > 0.1:
            canvas[v:v + 2, u:u + 2] = (col[k] * alpha + 255 * (1 - alpha)).astype(np.uint8)
    im = Image.fromarray(canvas[:, :, ::-1]); dr = ImageDraw.Draw(im); pix = {}
    for kid, T, colr, w in kfs:
        pts = np.array([[0, 0, 0], [-1, -.75, 1.6], [1, -.75, 1.6], [1, .75, 1.6], [-1, .75, 1.6]]) * fr
        q, zz = pr((T[:3, :3] @ pts.T).T + T[:3, 3])
        if np.any(zz < 0.2):
            continue
        q = [tuple(p) for p in q]; dr.polygon(q[1:], outline=colr, width=w)
        for j in range(1, 5):
            dr.line([q[0], q[j]], fill=colr, width=w)
        pix[f'K{kid}'] = [float(q[0][0]), float(q[0][1])]
    for n, T, dashed in objs:
        q, zz = pr((T[:3, :3] @ box_corners(EXT[n]).T).T + T[:3, 3])
        if np.any(zz < 0.2):
            continue
        q = [tuple(p) for p in q]
        for a_, b_ in BOX_EDGES:
            if dashed:
                for t in np.linspace(0, 1, 9)[::2]:
                    A_ = np.array(q[a_]) + (np.array(q[b_]) - np.array(q[a_])) * t
                    B_ = np.array(q[a_]) + (np.array(q[b_]) - np.array(q[a_])) * min(t + 1 / 8, 1)
                    dr.line([tuple(A_), tuple(B_)], fill=COL[n], width=3)
            else:
                dr.line([q[a_], q[b_]], fill=COL[n], width=5)
        pix[n] = [float(np.mean([p[0] for p in q])), float(min(p[1] for p in q))]
    for p0, p1, colr in lines:
        q, _ = pr(np.array([p0, p1]))
        for t in np.linspace(0, 1, 22)[::2]:
            A_ = q[0] + (q[1] - q[0]) * t; B_ = q[0] + (q[1] - q[0]) * min(t + 1 / 21, 1)
            dr.line([tuple(A_), tuple(B_)], fill=colr, width=2)
    return im, pix


def obj_T(run, real_dir, name, pos):
    T = np.asarray(json.load(open(real_dir / 'map3d_objects.json'))[name]); T = T.copy(); T[:3, 3] = pos; return T


# ---------------------------------------------------------------- 260915: keyframe images, state map, crops
R915 = REPO / 'objpose/output/live_260915_eightcircle_orbslam3'; REAL915 = ROOT / 'src' / 'real915'
C915 = json.load(open(SCR / 'r915.json'))
kfA, (mv, tcorr, kfB) = kf_states(R915)
REF, ANC = anchors(R915, C915, kfA, kfB)
poses = jl(R915 / 'slam_poses.jsonl')
first_ref = {}
for p in poses:
    if p.get('ref_kf') is not None and p['ref_kf'] not in first_ref:
        first_ref[int(p['ref_kf'])] = p['t_ns']
slam = ConvSession(Path.home() / 'DeepLearning/Dataset/260915_eightcircle/SLAM')
ks = sorted(first_ref)
mid = ks[len(ks) // 2]; trio = [ks[ks.index(mid) - 1], mid, ks[ks.index(mid) + 1]]
for name, k in zip(('kf_prev', 'kf_ref', 'kf_next'), trio):
    i = int(np.argmin(np.abs(slam.t_ns - first_ref[k])))
    cv2.imwrite(str(OUT / f'{name}.png'), slam.read(i, False).color_bgr)
strip = [ks[int(j)] for j in np.linspace(5, len(ks) - 5, 6)]
for j, k in enumerate(strip):
    i = int(np.argmin(np.abs(slam.t_ns - first_ref[k])))
    cv2.imwrite(str(OUT / f'strip_{j}.png'), slam.read(i, False).color_bgr)
json.dump({'trio': trio, 'strip': strip}, open(OUT / 'kf_ids.json', 'w'))

d = np.load(REAL915 / 'map3d_data.npz')
OBJA = {o: (kfA[k] @ r)[:3] for o, (k, r) in ANC.items()}
objs = [(o, obj_T(R915, REAL915, o, OBJA[o]), False) for o in OBJA]
anch = sorted({k for k, _ in ANC.values()})
kfs = [(k, kfA[k], '#9fb8d8', 2) for k in sorted(kfA)[::10] if k not in anch] + [(k, kfA[k], '#1F5FA8', 4) for k in anch]
lines = [(kfA[k][:3, 3], OBJA[o], COL[o]) for o, (k, _) in ANC.items()]
cen = np.mean(list(OBJA.values()), 0)
im, pix = render(d['P'], d['C'], cen, 35, 45, 3.4, 1400, 760, objs, kfs, lines, f=900, radius=3.6, fr=0.15)
im.save(OUT / 'statemap.png'); json.dump({'pix': pix, 'anchor': {o: k for o, (k, _) in ANC.items()}}, open(OUT / 'statemap_lab.json', 'w'))

est = cv2.imread(str(REAL915 / 'est.png'))
sam = ConvSession(Path.home() / 'DeepLearning/Dataset/260915_eightcircle/SAM', offset_ns=2_000_000)
K, dist = sam.K, np.asarray(sam.kc)
ests = [e for e in jl(R915 / 'sam6d_estimates.jsonl') if e.get('R')]
by = defaultdict(list)
for e in ests:
    by[e['object']].append(e)
for n, es in by.items():                 # a close, well-framed estimate per object
    e = min(es, key=lambda e: abs(np.linalg.norm(e['t_mm']) - 800))
    i = int(np.argmin(np.abs(sam.t_ns - e['t_ns']))); img = sam.read(i, False).color_bgr.copy()
    T = np.eye(4); T[:3, :3] = np.asarray(e['R']); T[:3, 3] = np.asarray(e['t_mm']) / 1000
    ext = EXT[n]; a = 0.6 * max(ext['size_mm']) / 1000
    pts = np.vstack([box_corners(ext), [[0, 0, 0], [a, 0, 0], [0, a, 0], [0, 0, a]]])
    uv = project((T[:3, :3] @ pts.T).T + T[:3, 3], K, dist); p = [tuple(int(round(v)) for v in q) for q in uv]
    cb = tuple(int(COL[n][i:i + 2], 16) for i in (5, 3, 1))
    for a_, b_ in BOX_EDGES:
        cv2.line(img, p[a_], p[b_], cb, 2, cv2.LINE_AA)
    for k_, c in ((9, (0, 0, 230)), (10, (0, 190, 0)), (11, (230, 80, 0))):
        cv2.line(img, p[8], p[k_], c, 2, cv2.LINE_AA)
    x0, y0 = np.maximum(uv[:8].min(0) - 14, 0).astype(int); x1, y1 = np.minimum(uv[:8].max(0) + 14, [639, 479]).astype(int)
    cv2.imwrite(str(OUT / f'objcrop_{n}.png'), img[y0:y1, x0:x1])

# ---------------------------------------------------------------- 260901 corr04: before / after the 3.05 m loop closure
R01 = REPO / 'objpose/output/live_260901_cbnu_bigeightcircle_orbslam3_f4000_corr04'; REAL01 = ROOT / 'src' / 'real'
C01 = json.load(open(SCR / 'corr04.json'))
kfA1, (mv1, t1, kfB1) = kf_states(R01)
REF1, ANC1 = anchors(R01, C01, kfA1, kfB1)
d1 = np.load(REAL01 / 'map3d_data.npz')
grp = max(({o: [q for q in REF1 if np.linalg.norm(REF1[o][[0, 2]] - REF1[q][[0, 2]]) < 1.0] for o in REF1}).values(), key=len)
cen1 = np.mean([REF1[o] for o in grp], 0)
allK = np.array([T[:3, 3] for T in kfA1.values()] + [T[:3, 3] for T in kfB1.values()])
cen1 = allK.mean(0); EMPTY = np.zeros((0, 3)), np.zeros((0, 3), np.uint8)
for tag, KF, kc, ac in (('before', kfB1, '#e9a99d', '#d0402b'), ('after', kfA1, '#9fb8d8', '#1F5FA8')):
    pos = {o: (KF[k] @ r)[:3] for o, (k, r) in ANC1.items() if k in KF}
    objs = [(o, obj_T(R01, REAL01, o, pos[o]), False) for o in pos]
    an = sorted({ANC1[o][0] for o in pos})
    kfs = [(k, KF[k], kc, 2) for k in sorted(KF)[::6] if k not in an] + [(k, KF[k], ac, 4) for k in an]
    lines = [(KF[ANC1[o][0]][:3, 3], pos[o], COL[o]) for o in pos]
    im, _ = render(*EMPTY, cen1, 200, 72, 11.5, 1100, 1100, objs, kfs, lines, f=820, radius=1, alpha=0.4, fr=0.3)
    im.save(OUT / f'loop_{tag}.png')
from PIL import ImageOps
for tag in ('before', 'after'):     # same crop for both so they stay comparable
    pass
boxes = [ImageOps.invert(Image.open(OUT / f'loop_{t}.png').convert('RGB')).getbbox() for t in ('before', 'after')]
bb = (min(b[0] for b in boxes) - 15, min(b[1] for b in boxes) - 15, max(b[2] for b in boxes) + 15, max(b[3] for b in boxes) + 15)
for t in ('before', 'after'):
    Image.open(OUT / f'loop_{t}.png').crop(bb).save(OUT / f'loop_{t}.png')
print('260915 corr %.2f m, 260901 corr %.2f m' % (mv, mv1), 'trio', trio, 'anchors', ANC.keys().__len__())
