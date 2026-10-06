"""Real image tiles for the main figure, all from one recorded live run.

Run: large figure-eight (bright indoor hall), ORB-SLAM3 4000 features, repeat corr04 — the same run as
the map comparison panel. Every tile comes from the recording or from that run's logs:
  slam_frames_*.png   SLAM camera frames (tracking input)
  rgb.png, depth.png  recognition camera colour / aligned depth at one capture time
  est.png             same frame with the verified SAM-6D poses of that capture time (3D boxes + axes)
  obj_*.png           crops of those objects
  display.png         a display frame: all map objects projected with the display-time pose
  map3d.png           point cloud from recognition-camera depth placed with the final (loop-corrected)
                      SLAM poses, with the objects as boxes
"""
import json, sys
from pathlib import Path
import numpy as np, cv2

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'objpose' / 'pc'))
from conv_session import ConvSession            # noqa: E402
from fusion import inv_se3, interp_se3, mat      # noqa: E402
from hub import box_corners, project, BOX_EDGES  # noqa: E402

import os
# run selection (default: the 260901 large figure-eight repeat corr04 used in the manuscript)
RUN = REPO / 'objpose/output' / os.environ.get('OBJ_RUN', 'live_260901_cbnu_bigeightcircle_orbslam3_f4000_corr04')
DATA = Path.home() / 'DeepLearning/Dataset' / os.environ.get('OBJ_DATA', '260901_cbnu_bigeightcircle')
XFILE = REPO / 'objpose/rt' / os.environ.get('OBJ_X', '260901/X_slam_sam_260901.json')
OUT = Path(sys.argv[1]); OUT.mkdir(parents=True, exist_ok=True)
EXT = json.loads((REPO / 'integration/cad_extents.json').read_text())
_xj = json.loads(XFILE.read_text())
X = np.asarray(_xj['T_slam_sam'], float).reshape(4, 4)
# one colour per object (BGR), fixed order
from real_tiles_colors import COL  # noqa: E402

sam = ConvSession(DATA / 'SAM', offset_ns=int(round(float(_xj.get('sam_tau_s', 0.0)) * 1e9)))
K = sam.K; dist = np.asarray(sam.kc)
jl = lambda p: [json.loads(l) for l in open(p) if l.strip()]


def draw_boxes(img, rows, thick=2, axes=False):
    for name, T in rows:
        ext = EXT.get(name)
        if ext is None:
            continue
        pts = box_corners(ext)
        if axes:
            a = 0.6 * max(ext['size_mm']) / 1000
            pts = np.vstack([pts, [[0, 0, 0], [a, 0, 0], [0, a, 0], [0, 0, a]]])
        cam = (T[:3, :3] @ pts.T).T + T[:3, 3]
        if np.any(cam[:, 2] < 0.2):
            continue
        uv = project(cam, K, dist); p = [tuple(int(round(v)) for v in q) for q in uv]
        for a_, b_ in BOX_EDGES:
            cv2.line(img, p[a_], p[b_], COL.get(name, (255, 255, 255)), thick, cv2.LINE_AA)
        if axes:
            for k, c in ((9, (0, 0, 230)), (10, (0, 190, 0)), (11, (230, 80, 0))):
                cv2.line(img, p[8], p[k], c, 2, cv2.LINE_AA)
    return img


def T_est(e):
    T = np.eye(4); T[:3, :3] = np.asarray(e['R']); T[:3, 3] = np.asarray(e['t_mm']) / 1000; return T


# ---- SLAM camera frames (tracking input). The PC copy of this SLAM bag has a damaged page, so the
# frames are read one message at a time instead of through ConvSession (which scans all stamps).
if 'OBJ_SLAM_IDS' in os.environ:
    # PC copy of the 260901 SLAM bag has a damaged page: read single messages by id
    import sqlite3
    sys.path.insert(0, str(REPO / 'objpose' / 'lidar'))
    from bag_io import parse_image  # noqa: E402
    con = sqlite3.connect(f"file:{DATA / 'SLAM' / 'SLAM_0.db3'}?mode=ro", uri=True)
    k = 0
    for base in map(int, os.environ['OBJ_SLAM_IDS'].split(',')):
        for i in range(base, base + 12):
            r = con.execute('select topic_id, data from messages where id=?', (i,)).fetchone()
            if r and r[0] == 1:
                cv2.imwrite(str(OUT / f'slam_frame_{k}.png'), parse_image(r[1])[1]); k += 1
                break
else:
    slam = ConvSession(DATA / 'SLAM')
    for k, f in enumerate(map(float, os.environ.get('OBJ_SLAM_FRAC', '0.2,0.5,0.8').split(','))):
        cv2.imwrite(str(OUT / f'slam_frame_{k}.png'), slam.read(int(f * len(slam)), False).color_bgr)

# ---- recognition: a capture time with several verified estimates
ests = [e for e in jl(RUN / 'sam6d_estimates.jsonl') if e.get('R')]
by_t = {}
for e in ests:
    by_t.setdefault(e['t_ns'], []).append(e)
t_best = max(by_t, key=lambda t: (len({e['object'] for e in by_t[t]}), -abs(t - np.median(list(by_t)))))
idx = int(np.argmin(np.abs(sam.t_ns - t_best)))
fr = sam.read(idx, True)
print('estimate frame', idx, 'dt ms', (sam.t_ns[idx] - t_best) / 1e6, [e['object'] for e in by_t[t_best]])
cv2.imwrite(str(OUT / 'rgb.png'), fr.color_bgr)
d = fr.depth_raw.astype(np.float32)
dn = np.clip(d / 4000.0, 0, 1); dc = cv2.applyColorMap((255 * (1 - dn)).astype(np.uint8), cv2.COLORMAP_TURBO); dc[d == 0] = 0
cv2.imwrite(str(OUT / 'depth.png'), dc)
rows = [(e['object'], T_est(e)) for e in by_t[t_best]]
est = draw_boxes(fr.color_bgr.copy(), rows, thick=2, axes=True)
cv2.imwrite(str(OUT / 'est.png'), est)
for j, (name, T) in enumerate(rows[:3]):
    cam = (T[:3, :3] @ box_corners(EXT[name]).T).T + T[:3, 3]
    uv = project(cam, K, dist); x0, y0 = np.maximum(uv.min(0) - 12, 0).astype(int); x1, y1 = np.minimum(uv.max(0) + 12, [sam.W, sam.H]).astype(int)
    cv2.imwrite(str(OUT / f'obj_{j}.png'), est[y0:y1, x0:x1])

# ---- display frame: all map objects at the display-time pose
disp = jl(RUN / 'display_objects.jsonl')
def near(d):  # objects close enough to be readable in the display tile
    return sum(1 for o in d['objects'] if o.get('T_cam_obj') and 0.4 < o['T_cam_obj'][11] < 1.6)
dbest = max(disp, key=lambda d: (near(d), len(d['objects'])))
if 'OBJ_DISPLAY_FRAME' in os.environ:   # a frame chosen by eye (no people in view)
    dbest = max((d for d in disp if d['frame_idx'] == int(os.environ['OBJ_DISPLAY_FRAME'])), key=lambda d: len(d['objects']))
frd = sam.read(int(dbest['frame_idx']), False)
rows = [(o['name'], np.asarray(o['T_cam_obj']).reshape(4, 4)) for o in dbest['objects'] if o.get('T_cam_obj')]
cv2.imwrite(str(OUT / 'display.png'), draw_boxes(frd.color_bgr.copy(), rows, thick=2))
print('display frame', dbest['frame_idx'], len(rows), 'objects')

# ---- final (loop-corrected) recognition-camera poses
kf_final = {}
for u in jl(RUN / 'kf_updates.jsonl'):
    for row in u['kfs']:
        T = np.eye(4); T[:3, :] = np.asarray(row[1:13]).reshape(3, 4); kf_final[int(row[0])] = T
poses = [p for p in jl(RUN / 'slam_poses.jsonl') if p.get('T_wc') and p.get('state') == 'OK' and p.get('ref_kf') is not None]
pt = np.array([p['t_ns'] for p in poses])


def T_ws_final(t):
    k = int(np.clip(np.searchsorted(pt, t), 1, len(pt) - 1))
    if abs(pt[k] - t) > abs(pt[k - 1] - t):
        k -= 1
    p = poses[k]
    if abs(pt[k] - t) > 0.1e9 or int(p['ref_kf']) not in kf_final:
        return None
    T = kf_final[int(p['ref_kf'])] @ inv_se3(mat(p['T_w_kf'])) @ mat(p['T_wc'])
    return T @ X


pts, cols = [], []
for i in range(0, len(sam), 45):
    T = T_ws_final(sam.t_ns[i])
    if T is None:
        continue
    f = sam.read(i, True); z = f.depth_raw.astype(np.float32) / 1000.0
    v, u = np.mgrid[0:sam.H:5, 0:sam.W:5]; zz = z[v, u]; m = (zz > 0.3) & (zz < 3.5)
    x = (u[m] - K[0, 2]) / K[0, 0] * zz[m]; y = (v[m] - K[1, 2]) / K[1, 1] * zz[m]
    P = np.column_stack([x, y, zz[m]]); P = (T[:3, :3] @ P.T).T + T[:3, 3]
    pts.append(P); cols.append(f.color_bgr[v[m], u[m]])
P = np.vstack(pts); C = np.vstack(cols)
objs = {}
for e in ests:
    T = T_ws_final(e['t_ns'])
    if T is not None:
        objs.setdefault(e['object'], []).append(T @ T_est(e))
# object pose: the estimate closest to the object's final position in the anchoring replay
# (anchor_ablation variant D, median over the run); misreads elsewhere in the hall are skipped
ref = {}
for o, p, _ in json.load(open(sys.argv[2]))['positions']['D']:
    ref.setdefault(o, []).append(p)
obj_T = {}
for n, Ts in objs.items():
    if n not in ref:
        continue
    med = np.median(np.array(ref[n]), 0)
    obj_T[n] = min(Ts, key=lambda T: np.linalg.norm(T[:3, 3] - med))
np.savez(OUT / 'map3d_data.npz', P=P, C=C)
json.dump({n: T.tolist() for n, T in obj_T.items()}, open(OUT / 'map3d_objects.json', 'w'))
print('points', len(P), 'objects', list(obj_T))
