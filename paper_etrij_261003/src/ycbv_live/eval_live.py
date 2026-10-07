"""Evaluate live YCB-V playback runs (hub outputs) against BOP ground truth.

Per scene run directory (objpose/output/<run>): reads display_objects.jsonl (what was drawn on each
display frame, T_cam_obj), sam6d_frames.jsonl (inference count), summary.json (final object map, T_w_obj).

Metrics (paper definitions)
  processed frames      : images the recognizer ran on / frames in the video
  on-screen match rate  : over display frames, visible GT objects whose drawn box is correct
                          (ADD-S < 10% of diameter) / visible GT objects
  wrong boxes per frame : drawn boxes with no correct GT match (wrong pose, wrong object, duplicate)
  final map             : objects registered vs GT objects; duplicates (same class > 1); position error
                          (median / p90 / max, mm) after expressing GT in the SLAM world (= camera of the
                          first replayed frame); ADD-S AUC (0-10 cm) of the final map objects.
  time to map           : per GT object of the scene, the video time (frame_idx / 30 s) at which it is
                          first drawn correctly from the map (ADD-S < 10 % of diameter); objects never
                          drawn correctly count as not found. Reported: median / p90 over found
                          objects, and the share of scene objects found by 2, 5, 10, 20, 30 s and by
                          the end of the video.
  wrong map objects     : final map objects (not dup-suppressed) whose class is not in the scene or
                          that are > 10 cm from the GT object of their class (incl. duplicates).

    python eval_live.py --runs '<glob>' --ycbv <bop>/ycbv/full/test [--name label]
"""
import argparse, glob, json, os
from pathlib import Path
import numpy as np, trimesh
from scipy.spatial import cKDTree

YCB = ['002_master_chef_can', '003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle',
       '007_tuna_fish_can', '008_pudding_box', '009_gelatin_box', '010_potted_meat_can', '011_banana',
       '019_pitcher_base', '021_bleach_cleanser', '024_bowl', '025_mug', '035_power_drill', '036_wood_block',
       '037_scissors', '040_large_marker', '051_large_clamp', '052_extra_large_clamp', '061_foam_brick']


def load_models(bop):
    info = json.load(open(Path(bop) / 'models_eval' / 'models_info.json'))
    pts, diam = {}, {}
    for k, v in info.items():
        m = trimesh.load(Path(bop) / 'models_eval' / f'obj_{int(k):06d}.ply')
        p = np.asarray(m.vertices, float) / 1000.0
        idx = np.random.default_rng(0).choice(len(p), min(1000, len(p)), replace=False)
        pts[int(k)] = p[idx]; diam[int(k)] = v['diameter'] / 1000.0
    return pts, diam


def adds(T1, T2, P):
    a = (T1[:3, :3] @ P.T).T + T1[:3, 3]; b = (T2[:3, :3] @ P.T).T + T2[:3, 3]
    return float(np.mean(cKDTree(b).query(a)[0]))


def auc(errs, n_gt, max_m=0.10):
    e = np.sort(np.minimum(np.array(errs, float), max_m))
    th = np.linspace(0, max_m, 1001)
    acc = np.array([(e <= t).sum() / max(n_gt, 1) for t in th])
    return float(np.trapezoid(acc, th) / max_m * 100)


def obj_id(name):
    """object name in the hub → BOP obj id (names are 'obj_000005' or YCB names)."""
    if name.startswith('obj_'):
        return int(name[4:])
    if name.startswith('ycbv_'):
        return int(name[5:])
    for i, n in enumerate(YCB, 1):
        if name == n or name == n[4:] or name.lower() == n.lower():
            return i
    raise KeyError(name)


def P_YCB(oid):
    return YCB[oid - 1]


def time_stats(tf, TT):
    found = [t for t in tf if t is not None]
    n = max(len(tf), 1)
    return {"scene_objects": len(tf), "found_by_end_pct": round(100 * len(found) / n, 1),
            "median_s": round(float(np.median(found)), 1) if found else None,
            "p90_s": round(float(np.percentile(found, 90)), 1) if found else None,
            **{f"found_by_{t}s_pct": round(100 * sum(1 for x in found if x <= t) / n, 1) for t in TT}}


def mat(v):
    return np.asarray(v, float).reshape(4, 4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--ycbv', default=os.environ.get('YCBV_FULL', str(Path.home() / 'DeepLearning/Dataset/bop/ycbv/full/test')))
    ap.add_argument('--bop', default=os.environ.get('YCBV_ROOT', str(Path.home() / 'DeepLearning/Dataset/bop/ycbv')))
    ap.add_argument('--out', default='')
    ap.add_argument('--first-frame', type=int, default=-1, help='dataset index of the SLAM world frame (default: from summary start_s)')
    a = ap.parse_args()
    pts, diam = load_models(a.bop)
    tot = dict(vis=0, match=0, wrong=0, frames=0, processed=0, total_frames=0)
    TT = [2, 5, 10, 20, 30]
    tfirst, n_scene_objs, wrong_map, map_objs = [], 0, 0, 0
    final = dict(reg=0, gt=0, dup=0, errs=[], adds=[])
    per = {}
    for run in sorted(glob.glob(a.runs)):
        run = Path(run); s = json.load(open(run / 'summary.json'))
        scene = int(''.join(c for c in Path(s.get('sam_session', str(run))).parent.name if c.isdigit())[-6:]) if 'sam_session' in s else int(run.name.split('_')[-1])
        src = Path(a.ycbv) / f'{scene:06d}'
        gt = json.load(open(src / 'scene_gt.json')); ginfo = json.load(open(src / 'scene_gt_info.json'))
        ids = sorted(int(k) for k in gt)
        def T_gt(fi, k):
            g = gt[str(ids[fi])][k]; T = np.eye(4); T[:3, :3] = np.reshape(g['cam_R_m2c'], (3, 3)); T[:3, 3] = np.array(g['cam_t_m2c']) / 1000; return T
        vis = lambda fi, k: ginfo[str(ids[fi])][k]['visib_fract'] > 0.1
        n_frames = len(ids); proc = sum(1 for _ in open(run / 'sam6d_frames.jsonl'))
        m = w = v = nf = 0
        first_ok = {}
        for line in open(run / 'display_objects.jsonl'):
            d = json.loads(line); fi = d['frame_idx']
            if fi >= n_frames:
                continue
            nf += 1
            g = {gt[str(ids[fi])][k]['obj_id']: k for k in range(len(gt[str(ids[fi])]))}
            matched = set()
            for o in d['objects']:
                if not o.get('T_cam_obj'):
                    continue
                oid = obj_id(o['name']); k = g.get(oid)
                if k is not None and vis(fi, k) and oid not in matched and adds(mat(o['T_cam_obj']), T_gt(fi, k), pts[oid]) < 0.1 * diam[oid]:
                    matched.add(oid)
                    first_ok.setdefault(oid, fi / 30.0)
                else:
                    w += 1
            vv = sum(1 for oid, k in g.items() if vis(fi, k)); v += vv; m += len(matched)
        # final map, GT in the SLAM world = camera of the first replayed frame
        # SLAM world = camera of the first frame SLAM tracked (ORB-SLAM3 initialises at its first frame)
        first = json.loads(open(run / 'slam_poses.jsonl').readline())
        f0 = a.first_frame if a.first_frame >= 0 else int(first['frame_idx'])
        Tw_c0 = np.eye(4)
        objs = s.get('objects', {})
        names = [n for n in objs]; cls = {}
        for n in names:
            cls.setdefault(obj_id(n.split('#')[0]), []).append(n)
        g0 = {gt[str(ids[f0])][k]['obj_id']: k for k in range(len(gt[str(ids[f0])]))}
        errs = []; addsl = []
        for oid, k in g0.items():
            final['gt'] += 1
            cand = cls.get(oid, [])
            if not cand:
                continue
            final['reg'] += 1; final['dup'] += len(cand) - 1
            Tg = Tw_c0 @ T_gt(f0, k)
            best = min(cand, key=lambda n: np.linalg.norm(mat(objs[n]['T_w_obj'])[:3, 3] - Tg[:3, 3]))
            T = mat(objs[best]['T_w_obj'])
            errs.append(np.linalg.norm(T[:3, 3] - Tg[:3, 3])); addsl.append(adds(T, Tg, pts[oid]))
        final['errs'] += errs; final['adds'] += addsl
        scene_objs = {gt[str(ids[fi])][k]['obj_id'] for fi in range(n_frames) for k in range(len(gt[str(ids[fi])]))
                      if vis(fi, k)}
        n_scene_objs += len(scene_objs)
        tfirst += [first_ok.get(o) for o in scene_objs]
        # wrong map objects: class not in the scene, or > 10 cm from that class's GT (first-frame world)
        wm = 0
        for n in names:
            if objs[n].get('dup_suppressed'):
                continue
            map_objs += 1
            oid = obj_id(n.split('#')[0])
            if oid not in g0:
                wm += oid not in scene_objs     # in the scene but not in the world frame: not judged
                continue
            if np.linalg.norm(mat(objs[n]['T_w_obj'])[:3, 3] - (Tw_c0 @ T_gt(f0, g0[oid]))[:3, 3]) > 0.10:
                wm += 1
        wrong_map += wm
        per[scene] = dict(frames=n_frames, processed=proc, match=round(m / max(v, 1) * 100, 1), wrong_per_frame=round(w / max(nf, 1), 3),
                          t_first_s={P_YCB(o): (round(first_ok[o], 1) if o in first_ok else None) for o in sorted(scene_objs)},
                          wrong_map_objects=wm,
                          reg=len(errs), gt=len(g0), pos_med_mm=round(float(np.median(errs) * 1000), 1) if errs else None)
        tot['vis'] += v; tot['match'] += m; tot['wrong'] += w; tot['frames'] += nf; tot['processed'] += proc; tot['total_frames'] += n_frames
        print(scene, per[scene])
    e = np.array(final['errs']) * 1000
    res = dict(processed=f"{tot['processed']} / {tot['total_frames']}", on_screen_match=round(tot['match'] / max(tot['vis'], 1) * 100, 1),
               wrong_boxes_per_frame=round(tot['wrong'] / max(tot['frames'], 1), 3),
               registered=f"{final['reg']} / {final['gt']}", duplicates=final['dup'],
               pos_err_mm=dict(median=round(float(np.median(e)), 1), p90=round(float(np.percentile(e, 90)), 1), max=round(float(e.max()), 1)) if len(e) else None,
               adds_auc_map=round(auc(final['adds'], final['gt']), 1),
               time_to_map=time_stats(tfirst, TT), wrong_map_objects=wrong_map, map_objects=map_objs,
               rate_hz=round(tot['processed'] / (tot['total_frames'] / 30.0), 2), per_scene=per)
    print(json.dumps({k: v for k, v in res.items() if k != 'per_scene'}, indent=1))
    if a.out:
        json.dump(res, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
