#!/usr/bin/env python3
"""render_example.py — one object, one frame: the box the ORIGINAL-SAM-6D map draws vs the box the PROPOSED map draws,
with the ground truth. Output: two tiles side by side, cropped around the object.
    python render_example.py <scene> <frame k> <object> <out.jpg> [crop_px]
"""
import json, sys
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0, str(Path(__file__).resolve().parent))
import display_eval as DE                                          # noqa: E402
from render_display import corners, EDGES                         # noqa: E402
from find_display_examples import RUNS                            # noqa: E402
BM = DE.BM


def box(img, T, nm, K, col, th):
    c = corners(nm) @ T[:3, :3].T + T[:3, 3]
    uv = (c[:, :2] / c[:, 2:3]) @ K[:2, :2].T + K[:2, 2]
    for a, b in EDGES:
        cv2.line(img, tuple(np.int32(uv[a])), tuple(np.int32(uv[b])), col, th, cv2.LINE_AA)
    return uv


scene, k, nm, out = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
crop = int(sys.argv[5]) if len(sys.argv) > 5 else 300
table = json.loads(BM.TABLE.read_text())["objects"]
by_id = {o["obj_id"]: o["name"] for o in table}
gt = json.loads((BM.YCBV / scene / "scene_gt.json").read_text())
tiles, cen = [], None
for key, col in (("orig", (40, 120, 230)), ("prop", (200, 110, 30))):
    ids, slam, K, snaps = DE.replay(scene, RUNS[key], by_id)
    fid = ids[k]
    img = cv2.imread(str(BM.YCBV / scene / "rgb" / f"{int(fid):06d}.png"))
    for r in gt[fid]:
        if by_id.get(r["obj_id"]) == nm:
            G = np.eye(4); G[:3, :3] = np.reshape(r["cam_R_m2c"], (3, 3)); G[:3, 3] = np.asarray(r["cam_t_m2c"]) / 1000.0
            uvg = box(img, G, nm, K, (60, 200, 60), 3)
            cen = uvg.mean(0) if cen is None else cen
    done = [s[0] for s in snaps]
    j = int(np.searchsorted(done, k / DE.FPS, side="right")) - 1
    Tcw = np.linalg.inv(slam[k])
    for n2, Tw in snaps[j][1]:
        if n2 == nm:
            box(img, Tcw @ Tw, nm, K, col, 3)
    tiles.append(img)
h, w = tiles[0].shape[:2]
x0 = int(np.clip(cen[0] - crop / 2, 0, w - crop)); y0 = int(np.clip(cen[1] - crop / 2, 0, h - crop))
tiles = [cv2.resize(t[y0:y0 + crop, x0:x0 + crop], (480, 480), interpolation=cv2.INTER_CUBIC) for t in tiles]
sep = np.full((480, 12, 3), 255, np.uint8)
cv2.imwrite(out, np.hstack([tiles[0], sep, tiles[1]]), [cv2.IMWRITE_JPEG_QUALITY, 92])
print(out)
