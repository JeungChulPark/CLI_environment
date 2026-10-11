"""Render map3d.png: real point cloud (recognition-camera depth placed with final SLAM poses) + object boxes."""
import json, sys
from pathlib import Path
import numpy as np, cv2
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'objpose' / 'pc'))
from hub import box_corners, BOX_EDGES  # noqa: E402
from real_tiles_colors import COL       # noqa: E402
D = Path(sys.argv[1]); d = np.load(D / 'map3d_data.npz'); P, C = d['P'], d['C']
objs = {n: np.asarray(T) for n, T in json.load(open(D / 'map3d_objects.json')).items()}
EXT = json.loads((REPO / 'integration/cad_extents.json').read_text())
floor = np.percentile(P[:, 1], 97)                      # y points down
h = floor - P[:, 1]
c = np.median(np.array([T[:3, 3] for T in objs.values()]), 0)
keep = (h > 0.02) & (h < 1.7) & (np.linalg.norm(P[:, [0, 2]] - c[[0, 2]], axis=1) < float(sys.argv[3]))
P, C = P[keep], C[keep]
az = np.radians(float(sys.argv[2])); el = np.radians(38); dist = 4.2
fwd = np.array([np.sin(az) * np.cos(el), np.sin(el), np.cos(az) * np.cos(el)])   # +y is down: looking down
eye = c - fwd * dist
up = np.array([0, -1.0, 0]); right = np.cross(fwd, up); right /= np.linalg.norm(right); upv = np.cross(right, fwd)
R = np.stack([right, -upv, fwd])                         # camera x right, y down, z forward
W, H, f = 1200, 800, 1000.0
def proj(X):
    Q = (R @ (X - eye).T).T
    return np.column_stack([f * Q[:, 0] / Q[:, 2] + W / 2, f * Q[:, 1] / Q[:, 2] + H / 2]), Q[:, 2]
uv, z = proj(P)
img = np.full((H, W, 3), 255, np.uint8); zb = np.full((H, W), np.inf)
o = np.argsort(-z)
for k in o:                                              # far to near, 2x2 splats
    u, v = int(uv[k, 0]), int(uv[k, 1])
    if 0 <= u < W - 1 and 0 <= v < H - 1 and z[k] > 0.1:
        img[v:v + 2, u:u + 2] = C[k]
for n, T in objs.items():
    pts = (T[:3, :3] @ box_corners(EXT[n]).T).T + T[:3, 3]
    q, _ = proj(pts); q = [tuple(int(round(a)) for a in p) for p in q]
    for a, b in BOX_EDGES:
        cv2.line(img, q[a], q[b], COL.get(n, (0, 0, 0)), 3, cv2.LINE_AA)
ys, xs = np.where(img.min(2) < 250)
img = img[max(ys.min() - 10, 0):ys.max() + 10, max(xs.min() - 10, 0):xs.max() + 10]
cv2.imwrite(str(D / 'map3d.png'), img); print(img.shape)
