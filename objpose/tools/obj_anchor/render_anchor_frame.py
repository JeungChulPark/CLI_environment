#!/usr/bin/env python3
"""render_anchor_frame.py — what the Object Pose Map display shows at the same moment with the SLAM camera pose alone
vs with the object-anchored correction (same map, same image). Picks the snapshot with the most map objects in view.
    python render_anchor_frame.py <run> --x <X> --session <SAM session> --t0 40 --t1 53 --out a.jpg [eval_real flags]
"""
import argparse, json, sys
from pathlib import Path
import numpy as np, cv2
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "pc"))
import eval_real as ER                                              # noqa: E402
from obj_anchor import AnchorConfig, inv                           # noqa: E402
REPO = Path(__file__).resolve().parents[3]
EXT = json.loads((REPO / "integration/cad_extents.json").read_text())
EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def corners(nm):
    lo, hi = np.array(EXT[nm]["min_mm"]) / 1000, np.array(EXT[nm]["max_mm"]) / 1000
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def project(Tco, nm, K, W, H):
    c = corners(nm) @ Tco[:3, :3].T + Tco[:3, 3]
    if np.any(c[:, 2] < 0.15):
        return None
    uv = (c[:, :2] / c[:, 2:3]) @ K[:2, :2].T + K[:2, 2]
    ctr = uv.mean(0)
    return uv if (0 <= ctr[0] < W and 0 <= ctr[1] < H) else None


ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("--x", required=True); ap.add_argument("--session", required=True)
ap.add_argument("--tau-s", type=float, default=0.0); ap.add_argument("--t0", type=float); ap.add_argument("--t1", type=float)
ap.add_argument("--out", required=True); ap.add_argument("--set", action="append", default=[])
ap.add_argument("--anchor-map", default="slam"); ap.add_argument("--kf-anchors", action="store_true"); ap.add_argument("--reset-on-loop", action="store_true")
a = ap.parse_args()
cfg = AnchorConfig()
for kv in a.set:
    k, v = kv.split("="); cur = getattr(cfg, k)
    setattr(cfg, k, tuple(float(x) for x in v.split(",")) if isinstance(cur, tuple) else (v.lower() in ("1", "true")) if isinstance(cur, bool) else type(cur)(v))
first = min(json.loads(l)["t_ns"] for l in open(Path(a.run) / "slam_poses.jsonl"))
ns = argparse.Namespace(run=a.run, x=a.x, session=a.session, tau_s=a.tau_s, no_depth_check=False, k_npz=None, range_scale=1.0,
                        landmarks="memory", anchor_map=a.anchor_map, kf_anchors=a.kf_anchors, reset_on_loop=a.reset_on_loop,
                        snap_ns=[int(first + s * 1e9) for s in np.arange(a.t0, a.t1, 0.5)], snaps=[])
r = ER.run(ns, cfg)
X = np.asarray(json.loads(Path(a.x).read_text())["T_slam_sam"], float).reshape(4, 4)
dep = ER.Depths(a.session, int(round(a.tau_s * 1e9)))
K, W, H = dep.K, dep.W, dep.H
best = None
for s in ns.snaps:
    n = [sum(project(inv(Tc @ X) @ Tw, nm, K, W, H) is not None for nm, Tw in s["hub_lms"]) for Tc in (s["T_hub"], s["T_anc"])]
    e_hub = np.linalg.norm((s["T_hub"] @ X)[:3, 3] - (r["ref"][s["t"]] @ X)[:3, 3]) * 1000 if s["t"] in r["ref"] else 0
    e_anc = np.linalg.norm((s["T_anc"] @ X)[:3, 3] - (r["ref"][s["t"]] @ X)[:3, 3]) * 1000 if s["t"] in r["ref"] else 0
    score = min(n) * 1000 + (e_hub - e_anc)
    if best is None or score > best[0]:
        best = (score, s, e_hub, e_anc)
_, s, e_hub, e_anc = best
i = int(np.argmin(np.abs(dep.s.t_ns - s["t"])))
img = dep.s.read(i, False).color_bgr
tiles = []
for Tc, col in ((s["T_hub"], (60, 60, 220)), (s["T_anc"], (200, 110, 30))):
    im = img.copy()
    for nm, Tw in s["hub_lms"]:
        uv = project(inv(Tc @ X) @ Tw, nm, K, W, H)
        if uv is None:
            continue
        for p, q in EDGES:
            cv2.line(im, tuple(np.int32(uv[p])), tuple(np.int32(uv[q])), col, 3, cv2.LINE_AA)
    tiles.append(im)
sep = np.full((H, 14, 3), 255, np.uint8)
cv2.imwrite(a.out, np.hstack([tiles[0], sep, tiles[1]]), [cv2.IMWRITE_JPEG_QUALITY, 90])
print(json.dumps({"t_rel_s": (s["t"] - first) / 1e9, "err_slam_mm": e_hub, "err_anchored_mm": e_anc, "objects": len(s["hub_lms"])}))
