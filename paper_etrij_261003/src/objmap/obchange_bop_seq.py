"""ObChange rooms other than Kitchen: the BOP-format frames -> the sequence layout obchange_prep.py writes for Kitchen.

Input  <BOP>/<room>_s<v>/{rgb,depth}/NNNNNN.png + scene_camera.json (cam_K, cam_R_w2c, cam_t_w2c in mm, stamp s),
       every 3rd frame of the HSR head camera (10 fps), depth in 0.1 mm (depth_scale 0.1).
Output (symlinks only, nothing is re-encoded):
  <WORK>/<room>/all/rgb|depth/<gid:06d>.png   visit 1 frames, then visit 2, ... (global ids run on across visits)
  gt/obchange_<room>/seq/all/cameras.json      {K, width, height, depth_scale_mm: 0.1, fps: 10, seq_dir,
                                                frames: [{id, T_wc, stamp_ns, visit, src}], visits: [{visit, first, last}]}
  gt/obchange_<room>/seq/visit<v>/cameras.json the visit alone, ids = the BOP frame ids, seq_dir = the BOP scene dir
T_wc = inv([R | t/1000]) (camera -> map, metres): the frame of the ObChange reconstructions and GT. On Kitchen this is
the same pose obchange_prep.py gets from /tf (checked on frame 1 of visit 1).

    python obchange_bop_seq.py --room Office [--bop ~/DeepLearning/Datasets/ObChange/bop/test] [--work ...]
"""
import argparse, json, os
from pathlib import Path

import numpy as np
from PIL import Image

from common import GT_DIR

BOP = Path.home() / 'DeepLearning/Datasets/ObChange/bop/test'
WORK = Path(os.environ.get('OBJMAP_OBCHANGE_FRAMES', Path.home() / 'DeepLearning/Datasets/objmap_obchange/frames'))
FPS = 10


def read_visit(sdir):
    cams = json.load(open(sdir / 'scene_camera.json'))
    frames, K, scale = [], None, None
    for k in sorted(cams, key=int):
        c = cams[k]
        i = int(k)
        if not ((sdir / 'rgb' / f'{i:06d}.png').exists() and (sdir / 'depth' / f'{i:06d}.png').exists()):
            continue
        T = np.eye(4)
        T[:3, :3] = np.array(c['cam_R_w2c'], float).reshape(3, 3)
        T[:3, 3] = np.array(c['cam_t_w2c'], float) / 1000.0
        frames.append({'id': i, 'T_wc': np.linalg.inv(T).tolist(), 'stamp_ns': int(round(float(c['stamp']) * 1e9))})
        if K is None:
            K, scale = [float(x) for x in c['cam_K']], float(c.get('depth_scale', 0.1))
        elif not np.allclose(K, c['cam_K']):
            raise SystemExit(f'{sdir}: cam_K changes within the visit')
    return frames, K, scale


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--room', required=True, help='BigRoom | LivingArea | Office | SmallRoom (Kitchen: obchange_prep.py)')
    ap.add_argument('--bop', default=str(BOP))
    ap.add_argument('--work', default=str(WORK))
    a = ap.parse_args()
    r = a.room.lower()
    bop, work = Path(a.bop), Path(a.work) / r
    vdirs = sorted((p for p in bop.glob(f'{r}_s*') if p.name.rsplit('_s', 1)[1].isdigit()),
                   key=lambda p: int(p.name.rsplit('_s', 1)[1]))
    if not vdirs:
        raise SystemExit(f'no {r}_s<v> under {bop}')
    allf, visits, K0, scale0, wh = [], [], None, None, None
    for sdir in vdirs:
        v = int(sdir.name.rsplit('_s', 1)[1])
        frames, K, scale = read_visit(sdir)
        if K0 is None:
            K0, scale0 = K, scale
            wh = Image.open(sdir / 'rgb' / f"{frames[0]['id']:06d}.png").size
        elif not np.allclose(K, K0) or scale != scale0:
            raise SystemExit(f'{sdir}: cam_K / depth_scale differ from visit {vdirs[0].name}')
        cams = {'K': K, 'width': wh[0], 'height': wh[1], 'depth_scale_mm': scale, 'fps': FPS, 'seq_dir': str(sdir),
                'frames': [{**f, 'visit': v} for f in frames],
                'visits': [{'visit': v, 'first': frames[0]['id'], 'last': frames[-1]['id']}]}
        d = GT_DIR / f'obchange_{r}' / 'seq' / f'visit{v}'; d.mkdir(parents=True, exist_ok=True)
        json.dump(cams, open(d / 'cameras.json', 'w'))
        first = len(allf)
        for f in frames:
            allf.append({**f, 'id': len(allf), 'visit': v, 'src': f['id'], '_dir': sdir})
        visits.append({'visit': v, 'first': first, 'last': len(allf) - 1})
        print(f'visit {v}: {len(frames)} frames ({sdir})', flush=True)
    seq = work / 'all'
    for sub in ('rgb', 'depth'):
        (seq / sub).mkdir(parents=True, exist_ok=True)
    for f in allf:
        sdir = f.pop('_dir')
        for sub in ('rgb', 'depth'):
            l = seq / sub / f"{f['id']:06d}.png"
            tgt = sdir / sub / f"{f['src']:06d}.png"
            if l.is_symlink() and Path(os.readlink(l)) != tgt:
                l.unlink()
            if not l.is_symlink():
                l.symlink_to(tgt)
    d = GT_DIR / f'obchange_{r}' / 'seq' / 'all'; d.mkdir(parents=True, exist_ok=True)
    json.dump({'K': K0, 'width': wh[0], 'height': wh[1], 'depth_scale_mm': scale0, 'fps': FPS, 'seq_dir': str(seq),
               'frames': allf, 'visits': visits}, open(d / 'cameras.json', 'w'))
    print('all:', len(allf), 'frames', visits, '->', d)


if __name__ == '__main__':
    main()
