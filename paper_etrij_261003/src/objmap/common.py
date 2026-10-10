"""Shared helpers of the object-map comparison: CAD loading, surface sampling, map I/O.

Units: everything is metres in the map (world) frame. BOP files are in mm and are
converted on load.
"""
import json
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from ycbv.paths import YCBV, YCB_NAMES, PROMPTS  # noqa: E402

GT_DIR = Path(os.environ.get("OBJMAP_GT", HERE / "gt"))     # generated GT maps
OUT = Path(os.environ.get("OBJMAP_OUT", HERE / "out"))      # method maps + metrics
SURF_SPACING = 0.002   # m, mean spacing of sampled CAD surface points


def label_to_oid(label):
    """Map a predicted label (obj_id, '002_master_chef_can', 'master_chef_can', prompt text,
    'ycbv_01') to a YCB-V obj_id 1..21, or None if it is not a target object."""
    if isinstance(label, (int, np.integer)):
        return int(label) if 1 <= int(label) <= len(YCB_NAMES) else None
    s = str(label).strip().lower()
    if s.isdigit():
        return label_to_oid(int(s))
    if s.startswith("ycbv_") and s[5:].isdigit():
        return label_to_oid(int(s[5:]))
    for i, (n, p) in enumerate(zip(YCB_NAMES, PROMPTS), 1):
        if s in (n, n[4:], n[4:].replace("_", " "), p):
            return i
    return None


def load_ply(path):
    """Vertices (mm) and triangle faces of a BOP ascii/binary PLY."""
    with open(path, "rb") as f:
        fmt, props, n_v, n_f = None, [], 0, 0
        elem = None
        while True:
            line = f.readline().decode("ascii").strip()
            if line.startswith("format"):
                fmt = line.split()[1]
            elif line.startswith("element"):
                _, elem, n = line.split()
                if elem == "vertex":
                    n_v = int(n)
                elif elem == "face":
                    n_f = int(n)
            elif line.startswith("property") and elem == "vertex":
                props.append(line.split())
            elif line == "end_header":
                break
        if fmt == "ascii":
            rows = [f.readline().split() for _ in range(n_v)]
            V = np.array([[float(x) for x in r[:3]] for r in rows])
            F = np.array([[int(x) for x in f.readline().split()[1:4]] for _ in range(n_f)])
            return V, F
        tmap = {"float": "f4", "float32": "f4", "double": "f8", "uchar": "u1", "uint8": "u1",
                "int": "i4", "int32": "i4", "uint": "u4", "short": "i2", "ushort": "u2"}
        end = "<" if fmt == "binary_little_endian" else ">"
        dt = np.dtype([(p[2], end + tmap[p[1]]) for p in props])
        v = np.frombuffer(f.read(dt.itemsize * n_v), dtype=dt)
        V = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
        F = np.empty((n_f, 3), np.int64)
        for i in range(n_f):   # 'list uchar int vertex_indices', triangles only
            k = f.read(1)[0]
            F[i] = np.frombuffer(f.read(4 * k), dtype=end + "i4")[:3]
        return V, F


_CAD = {}


def cad_points(oid, spacing=SURF_SPACING, seed=0):
    """Area-uniform surface samples of the CAD model of obj_id `oid`, metres, object frame."""
    key = (oid, spacing, seed)
    if key not in _CAD:
        V, F = load_ply(YCBV / "models" / f"obj_{oid:06d}.ply")
        V = V / 1000.0
        a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        n = max(1000, int(area.sum() / spacing ** 2))
        rng = np.random.default_rng(seed)
        tri = rng.choice(len(F), n, p=area / area.sum())
        u, v = rng.random(n), rng.random(n)
        flip = u + v > 1
        u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
        _CAD[key] = a[tri] + u[:, None] * (b[tri] - a[tri]) + v[:, None] * (c[tri] - a[tri])
    return _CAD[key]


def transform(T, P):
    return P @ T[:3, :3].T + T[:3, 3]


def save_map(map_dir, objects, meta=None):
    """Write a map in the common format.

    objects: list of dicts with 'label' and 'points' (N x 3, metres, world) and optionally
    'T_w_o' (4x4) and 'score'. Layout: map_dir/map.json + map_dir/points/<k>.npy
    """
    map_dir = Path(map_dir)
    (map_dir / "points").mkdir(parents=True, exist_ok=True)
    rows = []
    for k, o in enumerate(objects):
        r = {"id": k, "label": o["label"] if not isinstance(o["label"], np.integer) else int(o["label"])}
        if o.get("points") is not None:
            np.save(map_dir / "points" / f"{k}.npy", np.asarray(o["points"], np.float32))
            r["points"] = f"points/{k}.npy"
        if o.get("T_w_o") is not None:
            r["T_w_o"] = np.asarray(o["T_w_o"]).tolist()
        for extra in ("score", "n_obs"):
            if extra in o:
                r[extra] = o[extra]
        rows.append(r)
    json.dump({"units": "m", "frame": "world", **(meta or {}), "objects": rows},
              open(map_dir / "map.json", "w"), indent=1)


def load_map(map_dir, cad_if_no_points=True):
    """Read a common-format map. Objects that only carry T_w_o and a YCB-V label get the
    CAD surface placed at that pose."""
    map_dir = Path(map_dir)
    m = json.load(open(map_dir / "map.json"))
    objs = []
    for r in m["objects"]:
        o = dict(r)
        if "points" in r:
            o["points"] = np.load(map_dir / r["points"]).astype(np.float64)
        elif "T_w_o" in r and cad_if_no_points and label_to_oid(r["label"]):
            o["points"] = transform(np.array(r["T_w_o"]), cad_points(label_to_oid(r["label"])))
        else:
            continue
        if "T_w_o" in r:
            o["T_w_o"] = np.array(r["T_w_o"])
        objs.append(o)
    return m, objs


# ── sequences other than the YCB-V test videos ──────────────────────────────
# dataset            scene k means                                     cameras.json
# ycbv               YCB-V test video k                                gt/ycbv/<k:06d>/
# obchange_kitchen   ObChange-Kitchen visit k alone                    gt/obchange_kitchen/seq/all/ (frames of visit k)
# obchange_kitchen_all  visits 1..k in a row (the map at the end of k)  gt/obchange_kitchen/seq/all/ (visits <= k)
# The ObChange sequences keep the global frame ids of the continuous sequence, so per-frame work
# (detections) is shared between them.
def seq_cameras(dataset, scene):
    """cameras.json of a sequence: {K, width, height, depth_scale_mm, seq_dir, frames}."""
    if dataset == "ycbv":
        return json.load(open(GT_DIR / "ycbv" / f"{scene:06d}" / "cameras.json"))
    cams = json.load(open(GT_DIR / "obchange_kitchen" / "seq" / "all" / "cameras.json"))
    if dataset == "obchange_kitchen":
        keep = lambda v: v == scene
    elif dataset == "obchange_kitchen_all":
        keep = lambda v: v <= scene
    else:
        raise ValueError(dataset)
    cams["frames"] = [f for f in cams["frames"] if keep(f["visit"])]
    return cams


def seq_name(dataset, scene):
    """Folder name of a sequence in the baselines' work dirs (YCB-V keeps its old names)."""
    return f"{scene:06d}" if dataset == "ycbv" else f"{dataset}_{scene:06d}"
