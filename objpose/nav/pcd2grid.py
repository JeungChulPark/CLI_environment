#!/usr/bin/env python3
"""pcd2grid.py — hdl_graph_slam dump (keyframe clouds + optimized poses) -> Nav2 map_server grid (PGM + YAML).

Map frame = hdl `map` (z=0 at the first LiDAR pose). The floor height is the densest z-bin below the sensor, or --floor
(real Husky: the map origin is base_link, 0.13 m above the ground, and the high VLP-16 hardly sees the floor).
Per keyframe, every point is a ray from the keyframe position. Hits and passes are counted per VOXEL (cell x height
bin of --zres inside [floor+obst_min, floor+obst_max]), with the ray's own height at each cell:
  voxel occupied: hits >= min_hits and hits/(hits+passes) >= occ_ratio
  cell occupied: any of its voxels is;  free: a ray crossed the column inside the band and no voxel is occupied
Cells hit from fewer than --min-kf keyframes are dropped: something that moved (the person who walked along with the robot
left blobs on the open floor, each seen from 1-2 of 18 keyframes; fixed things were seen from 5 or more).
A ray that flies OVER a low obstacle (sand box, desk, shelf foot) to a wall behind it therefore no longer erases it:
with the lidar 1.3 m up, most points are at 0.8-1.5 m and the old 2D version (--flat) counted all their rays as
"free" over everything lower (2026-10-14, place_261014: the middle of the room came out open).
  python3 pcd2grid.py <dump dir> out_prefix [--res 0.05] [--obst-min 0.15] [--obst-max 1.2]

--track <json [[x, y, ...], ...]>: the path the robot was driven by hand while mapping. Also writes <out>_pref.png/.yaml
(map_server "scale" mode): free cells within --track-width of that path keep cost 0, farther ones get a cost that rises
to --track-penalty (of 100) over --track-ramp. The global planner then PREFERS the driven path (it is cheaper) but may
still leave it; obstacles and unknown cells are the same as in the plain map. Needs static_layer trinary_costmap: false.
"""
import glob
import json
import os
import struct
import zlib
import argparse
import numpy as np


def write_png_rgba(path, a):
    """HxWx4 uint8 -> PNG (numpy + zlib only)"""
    h, w = a.shape[:2]
    raw = np.concatenate([np.zeros((h, 1), np.uint8), a.reshape(h, w * 4)], 1).tobytes()
    chunk = lambda t, d: struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def read_pcd(path):
    with open(path, "rb") as f:
        hdr = {}
        while True:
            line = f.readline().decode("ascii").strip()
            k, _, v = line.partition(" ")
            hdr[k] = v.split()
            if k == "DATA":
                break
        fields = hdr["FIELDS"]; sizes = list(map(int, hdr["SIZE"])); types = hdr["TYPE"]; n = int(hdr["POINTS"][0])
        assert hdr["DATA"][0] == "binary", "only binary PCD"
        dt = np.dtype([(fn, {"F": "f", "U": "u", "I": "i"}[t] + str(s)) for fn, s, t in zip(fields, sizes, types)])
        a = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
    return np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)


def read_dump(d):
    """[(T_map_kf 4x4, cloud Nx3 in keyframe frame)] from hdl_graph_slam dump_graph"""
    out = []
    for kd in sorted(glob.glob(os.path.join(d, "[0-9]" * 6))):
        L = open(os.path.join(kd, "data")).read().split("\n")
        i = L.index("estimate")
        T = np.array([list(map(float, L[i + 1 + r].split())) for r in range(4)])
        out.append((T, read_pcd(os.path.join(kd, "cloud.pcd"))))
    return out


def grow(m):
    """boolean mask grown by one cell in all eight directions"""
    p = np.pad(m, 1)
    return p[:-2, :-2] | p[:-2, 1:-1] | p[:-2, 2:] | p[1:-1, :-2] | p[1:-1, 1:-1] | p[1:-1, 2:] | p[2:, :-2] | p[2:, 1:-1] | p[2:, 2:]


def tidy(occ, speckle, close):
    """occupied mask -> without lone cells, and with small gaps inside an obstacle filled (numpy only: the laptop has no scipy)"""
    occ = occ.copy(); H, W = occ.shape
    if speckle > 1:                                           # 8-connected blobs smaller than `speckle` cells
        seen = np.zeros_like(occ)
        for y, x in zip(*np.nonzero(occ)):
            if seen[y, x]:
                continue
            blob, stack = [], [(y, x)]; seen[y, x] = True
            while stack:
                cy, cx = stack.pop(); blob.append((cy, cx))
                for ny in range(max(0, cy - 1), min(H, cy + 2)):
                    for nx in range(max(0, cx - 1), min(W, cx + 2)):
                        if occ[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True; stack.append((ny, nx))
            if len(blob) < speckle:
                for cy, cx in blob:
                    occ[cy, cx] = False

    for _ in range(1 if close > 0 else 0):                    # closing = dilate n times, then erode n times
        m = occ
        for _ in range(close): m = grow(m)
        for _ in range(close): m = ~grow(~m)
        occ = occ | m
    return occ


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump"); ap.add_argument("out")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--obst-min", type=float, default=0.15, help="m above floor")
    ap.add_argument("--obst-max", type=float, default=1.2, help="m above floor (robot + margin)")
    ap.add_argument("--floor-tol", type=float, default=0.08)
    ap.add_argument("--floor", type=float, default=None, help="floor z in the map frame (default: estimated)")
    ap.add_argument("--occ-ratio", type=float, default=0.1, help="hits/(hits+passes) to call a cell occupied")
    ap.add_argument("--min-hits", type=int, default=None, help="points per voxel to count as occupied (default 2; 3 per cell with --flat)")
    ap.add_argument("--zres", type=float, default=0.15, help="height of a voxel [m]")
    ap.add_argument("--flat", action="store_true", help="old behaviour: 2D rays, one hit/pass count per cell")
    ap.add_argument("--min-kf", type=int, default=3,
                    help="an obstacle cell must be hit (itself or a neighbour) from this many keyframes; fewer = something that "
                         "moved, e.g. the person walking along with the robot (0 = off; ignored under 6 keyframes)")
    ap.add_argument("--track", help="json list of [x, y, ...] map poses driven by hand: also write the preference map")
    ap.add_argument("--track-width", type=float, default=0.5, help="m on each side of the driven path that stay free of extra cost")
    ap.add_argument("--track-ramp", type=float, default=0.75, help="m over which the extra cost rises to its maximum")
    ap.add_argument("--track-penalty", type=float, default=30.0, help="largest extra cost, map units 0-100 (0 = free, 100 = wall)")
    ap.add_argument("--speckle", type=int, default=3, help="drop occupied blobs smaller than this many cells (0 = keep all)")
    ap.add_argument("--close", type=int, default=1, help="fill gaps up to this many cells between occupied cells (0 = off)")
    a = ap.parse_args()

    if a.min_hits is None:
        a.min_hits = 3 if a.flat else 2
    kfs = read_dump(a.dump)
    world = [(T, c @ T[:3, :3].T + T[:3, 3]) for T, c in kfs]
    p = np.concatenate([w for _, w in world])
    if a.floor is not None:
        floor = a.floor
    else:
        below = p[p[:, 2] < -0.1, 2]
        h, e = np.histogram(below, bins=np.arange(below.min(), -0.1, 0.02))
        floor = 0.5 * (e[h.argmax()] + e[h.argmax() + 1])
    print(f"keyframes={len(kfs)} points={len(p)} floor_z={floor:.3f} (sensor height ~{-floor:.2f} m)")

    pad = 1.0
    x0, y0 = p[:, 0].min() - pad, p[:, 1].min() - pad
    W = int(np.ceil((p[:, 0].max() + pad - x0) / a.res)); H = int(np.ceil((p[:, 1].max() + pad - y0) / a.res))
    NZ = 1 if a.flat else max(1, int(np.ceil((a.obst_max - a.obst_min) / a.zres)))
    hits = np.zeros((H, W, NZ), np.int32); passes = np.zeros((H, W, NZ), np.int32)
    kf_hit = np.zeros((H, W), np.int16)                          # keyframes that hit the cell or one next to it
    zbin = lambda h: np.clip(((h - a.obst_min) / a.zres).astype(int), 0, NZ - 1)
    step = a.res * 0.5
    for T, w in world:
        dz = w[:, 2] - floor
        keep = (dz > -a.floor_tol) & (dz < a.obst_max)          # floor + obstacle band; drop ceiling / below-floor noise
        w, dz = w[keep], dz[keep]
        o = T[:2, 3]
        v = w[:, :2] - o
        dist = np.linalg.norm(v, axis=1)
        n = np.maximum(1, (dist / step).astype(int))
        # samples along each ray, stopping one cell short of the end point
        k = np.repeat(np.arange(len(w)), n)
        t = (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)) / np.repeat(n, n)
        t *= np.repeat(np.clip(1 - a.res / np.maximum(dist, 1e-6), 0, 1), n)
        s_xy = o + v[k] * t[:, None]
        cx = ((s_xy[:, 0] - x0) / a.res).astype(int); cy = ((s_xy[:, 1] - y0) / a.res).astype(int)
        if a.flat:
            cz = np.zeros(len(k), int)
        else:                                                    # height of the ray above the floor at each sample
            hs = (T[2, 3] - floor) + (dz[k] - (T[2, 3] - floor)) * t
            inband = (hs >= a.obst_min) & (hs < a.obst_max)
            cx, cy, k, cz = cx[inband], cy[inband], k[inband], zbin(hs[inband])
        cell = np.unique(np.stack([cy, cx, cz, k], 1), axis=0)       # one pass per ray per voxel
        np.add.at(passes, (cell[:, 0], cell[:, 1], cell[:, 2]), 1)
        m = dz > a.obst_min
        hi = (((w[m, 1] - y0) / a.res).astype(int), ((w[m, 0] - x0) / a.res).astype(int),
              np.zeros(m.sum(), int) if a.flat else zbin(dz[m]))
        np.add.at(hits, hi, 1)
        h1 = np.zeros((H, W), bool); h1[hi[0], hi[1]] = True; kf_hit += grow(h1)
    occ = ((hits >= a.min_hits) & (hits >= a.occ_ratio * (hits + passes))).any(2)
    seen = passes.sum(2) > 0
    if not a.flat:
        if a.min_kf > 0 and len(kfs) >= 6:
            mov = occ & (kf_hit < a.min_kf)
            print(f"seen from fewer than {a.min_kf} keyframes (moving things): {int(mov.sum())} occupied cells dropped")
            occ &= ~mov
        occ = tidy(occ, a.speckle, a.close)
    free = seen & ~occ
    img = np.full((H, W), 205, np.uint8)       # unknown
    img[free] = 254
    img[occ] = 0
    img = img[::-1]                            # PGM row 0 = top = max y
    with open(a.out + ".pgm", "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (W, H)); f.write(img.tobytes())
    with open(a.out + ".yaml", "w") as f:
        f.write(f"image: {a.out.split('/')[-1]}.pgm\nmode: trinary\nresolution: {a.res}\n"
                f"origin: [{x0:.3f}, {y0:.3f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    if a.track and os.path.exists(a.track):
        tr = np.array([q[:2] for q in json.load(open(a.track))], float)
        keep = [0]
        for i in range(1, len(tr)):                                  # one point every 10 cm is plenty
            if np.hypot(*(tr[i] - tr[keep[-1]])) >= 0.10:
                keep.append(i)
        tr = tr[keep]
        if len(tr) >= 2:
            fy, fx = np.nonzero(free)
            cxy = np.column_stack([x0 + (fx + 0.5) * a.res, y0 + (fy + 0.5) * a.res])
            dmin = np.full(len(cxy), np.inf)
            for i in range(0, len(tr), 64):                          # distance to the nearest path point, in chunks
                dmin = np.minimum(dmin, np.sqrt(((cxy[:, None, :] - tr[None, i:i + 64, :]) ** 2).sum(2)).min(1))
            cost = a.track_penalty * np.clip((dmin - a.track_width) / max(a.track_ramp, 1e-6), 0, 1)       # 0..penalty
            FREE_T, OCC_T = 0.02, 0.65                               # scale mode: value = 100 * (occ - FREE_T) / (OCC_T - FREE_T)
            occv = np.where(cost < 0.5, 0.0, FREE_T + cost / 100.0 * (OCC_T - FREE_T))
            g = np.zeros((H, W, 4), np.uint8)                        # alpha 0 = unknown
            g[fy, fx, :3] = np.round(255 * (1 - occv))[:, None].astype(np.uint8); g[fy, fx, 3] = 255
            g[occ] = (0, 0, 0, 255)
            write_png_rgba(a.out + "_pref.png", g[::-1])
            with open(a.out + "_pref.yaml", "w") as f:
                f.write(f"image: {a.out.split('/')[-1]}_pref.png\nmode: scale\nresolution: {a.res}\n"
                        f"origin: [{x0:.3f}, {y0:.3f}, 0.0]\nnegate: 0\noccupied_thresh: {OCC_T}\nfree_thresh: {FREE_T}\n")
            on = int((cost < 0.5).sum())
            print(f"driven path: {len(tr)} points, {np.hypot(*np.diff(tr, axis=0).T).sum():.1f} m; free cells on it {on}, "
                  f"with extra cost {len(cost) - on} (max {a.track_penalty:.0f}) -> {a.out}_pref.png/.yaml")
        else:
            print("driven path too short: no preference map")
    print(f"grid {W}x{H} @ {a.res} m, origin ({x0:.2f},{y0:.2f}); occupied={occ.sum()} free={free.sum()} "
          f"unknown={(img == 205).sum()} -> {a.out}.pgm/.yaml")


if __name__ == "__main__":
    main()
