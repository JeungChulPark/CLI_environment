"""Ground-truth object maps of ObChange-Kitchen (Langer et al., TU Wien, CC BY 4.0) in the common map format.

ObChange gives, per visit (scene2..scene6; scene1 is the empty first visit) and per surface (planes/<k>):
  merged_plane_clouds_ds002.pcd       ElasticFusion surface reconstruction in the robot's map frame
  merged_plane_clouds_ds002_GT.anno   per object: name, point indices into that cloud, on-floor flag
  merged_plane_clouds_ds002.anno      per object: a rough 4x4 placement (origin at the object's base, yaw only)
The point labels are the reference. The rough placement does not match the BOP YCB-V model frame (centres off
by about half the object height), so each object's 6D pose is re-fitted: the YCB-V CAD is aligned to the labelled
points by trimmed point-to-point ICP, started from the placement's rotation times each of the 24 axis-aligned
rotations, and the start with the lowest residual is kept.

Output per visit k (2..6), gt/obchange_kitchen/<k:06d>/:
  map.json + points/<i>.npy   labelled points (the seen surface; default GT) with label and fitted T_w_o
  fit.json                    per object: ICP residual median/p90 (mm), inlier share, chosen start
Objects without a YCB-V CAD (027_skillet, 029_plate) are not scored; their labelled points go to
excluded/<i>.npy, listed under `excluded` in map.json, and evaluate.py leaves out every prediction lying on them
(don't-care regions, the same for every method). --excluded-only rewrites just these, keeping the fitted objects.
Other rooms (--room BigRoom|LivingArea|Office|SmallRoom; BigRoom has visits 2..7): the visits are the scene* folders
of --root (default ~/DeepLearning/Datasets/ObChange/anno/<Room>), output gt/obchange_<room>/.
Usage: python obchange_gt.py [--room Kitchen] [--root ~/DeepLearning/Dataset/obchange/Kitchen]
"""
import argparse, itertools, json, re
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from common import GT_DIR, cad_points, label_to_oid, save_map


def read_pcd(path):
    b = open(path, 'rb').read()
    i = b.index(b'DATA binary\n') + len('DATA binary\n')
    hdr = b[:i].decode()
    fields = re.search(r'FIELDS (.+)', hdr).group(1).split()
    n = int(re.search(r'POINTS (\d+)', hdr).group(1))
    a = np.frombuffer(b[i:i + n * 4 * len(fields)], dtype=np.float32).reshape(n, len(fields))
    return a[:, :3].astype(np.float64)


def read_anno(path):
    out = {}
    for line in open(path):
        m = re.match(r'(\S+) \((\d+)\):\s*(.*)', line.strip())
        if m:
            out.setdefault(m.group(1), []).append(np.array([float(x) for x in m.group(3).split()[:16]]).reshape(4, 4))
    return out


def read_gt(path):
    out = []
    for line in open(path):
        t = line.split()
        if t:
            # the flag is normally the last token; one LivingArea line (scene6 plane 2, 065-j_cups) has it in the
            # middle ("... 380 true 26931"), so take the flag wherever it is and every integer as a point index
            flags = [x.lower() for x in t[1:] if x.lower() in ('true', 'false')]
            out.append((t[0], np.array([int(x) for x in t[1:] if x.lower() not in ('true', 'false')]),
                        bool(flags) and flags[-1] == 'true'))
    return out


def axis_rotations():
    """The 24 proper rotations that permute and flip the coordinate axes."""
    R = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            M = np.zeros((3, 3))
            for r, (c, s) in enumerate(zip(perm, signs)):
                M[r, c] = s
            if np.linalg.det(M) > 0:
                R.append(M)
    return R


def kabsch(A, B):
    """R, t minimising |R A + t - B|."""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def icp(Q, cad_tree, cad, R, t, iters=60, trim=0.9):
    """Fit T_w_o (R, t) so that the CAD placed with it explains the observed points Q (world)."""
    for _ in range(iters):
        q_o = (Q - t) @ R                       # observed points in the object frame
        d, j = cad_tree.query(q_o)
        keep = d <= np.quantile(d, trim)
        dR, dt = kabsch(cad[j[keep]], q_o[keep])   # object-frame correction: cad ~ dR^-1 (q_o - dt)
        R, t = R @ dR, R @ dt + t
        if np.linalg.norm(dt) < 1e-5 and np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)) < 1e-5:
            break
    d = cad_tree.query((Q - t) @ R)[0]
    return R, t, d


def fit_pose(Q, oid, T0):
    cad = cad_points(oid)
    tree = cKDTree(cad)
    best = None
    for k, A in enumerate(axis_rotations()):
        R0 = T0[:3, :3] @ A
        R, t, d = icp(Q, tree, cad, R0, Q.mean(0) - R0 @ cad.mean(0))
        score = np.median(d)
        if best is None or score < best[3]:
            best = (R, t, d, score, k)
    R, t, d, _, k = best
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T, d, k


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--room', default='Kitchen', help='Kitchen | BigRoom | LivingArea | Office | SmallRoom')
    ap.add_argument('--root', default=None, help='annotation dir of the room (scene<k>/planes/...)')
    ap.add_argument('--excluded-only', action='store_true', help='rewrite only the excluded (don\'t-care) objects')
    a = ap.parse_args()
    root = Path(a.root) if a.root else (Path.home() / 'DeepLearning/Dataset/obchange/Kitchen' if a.room == 'Kitchen'
                                       else Path.home() / 'DeepLearning/Datasets/ObChange/anno' / a.room)
    for scene in sorted((p for p in root.glob('scene*') if p.name[5:].isdigit()), key=lambda p: int(p.name[5:])):
        k = int(scene.name[5:])
        objs, excluded, fits = [], [], []
        for plane in sorted((scene / 'planes').iterdir(), key=lambda p: int(p.name)):
            gtp = plane / 'merged_plane_clouds_ds002_GT.anno'
            if not gtp.exists():
                continue
            P = read_pcd(plane / 'merged_plane_clouds_ds002.pcd')
            A = read_anno(plane / 'merged_plane_clouds_ds002.anno')
            for name, idx, on_floor in read_gt(gtp):
                oid = label_to_oid(name)
                if oid is None:
                    excluded.append({'label': name, 'plane': int(plane.name), 'n_points': int(len(idx)), 'P': P[idx]})
                    continue
                if a.excluded_only:
                    continue
                Q = P[idx]
                T, d, start = fit_pose(Q, oid, A[name][0] if name in A else np.eye(4))
                objs.append({'label': name, 'points': Q, 'T_w_o': T, 'plane': int(plane.name), 'on_floor': on_floor})
                fits.append({'label': name, 'plane': int(plane.name), 'n_points': int(len(idx)),
                             'residual_med_mm': round(float(np.median(d)) * 1000, 2),
                             'residual_p90_mm': round(float(np.percentile(d, 90)) * 1000, 2),
                             'inlier_5mm': round(float((d < 0.005).mean()), 3), 'start': start})
        out = GT_DIR / f'obchange_{a.room.lower()}' / f'{k:06d}'
        (out / 'excluded').mkdir(parents=True, exist_ok=True)
        for i, e in enumerate(excluded):
            np.save(out / 'excluded' / f'{i}.npy', np.asarray(e.pop('P'), np.float32))
            e['points'] = f'excluded/{i}.npy'
        if a.excluded_only:
            m = json.load(open(out / 'map.json')); m['excluded'] = excluded
            json.dump(m, open(out / 'map.json', 'w'), indent=1)
            print(f'visit {k}: excluded {[e["label"] for e in excluded]} -> {out}')
            continue
        if not objs:
            continue
        save_map(out, objs, {'dataset': f'ObChange-{a.room}', 'visit': k, 'excluded': excluded,
                             'source': 'pointwise labels on the ElasticFusion surface reconstruction'})
        json.dump(fits, open(out / 'fit.json', 'w'), indent=1)
        for f in fits:
            print(f"visit {k} {f['label']:22s} pts {f['n_points']:5d}  ICP med {f['residual_med_mm']:5.1f} mm "
                  f"p90 {f['residual_p90_mm']:5.1f}  <5mm {f['inlier_5mm']:.2f}")
        print(f'visit {k}: {len(objs)} objects, excluded {[e["label"] for e in excluded]} -> {out}')


if __name__ == '__main__':
    main()
