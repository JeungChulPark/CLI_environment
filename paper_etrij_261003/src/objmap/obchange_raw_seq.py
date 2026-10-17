"""ObChange rooms other than Kitchen at the camera's full rate (30 Hz) from the raw rosbags, in the layout
obchange_bop_seq.py writes from the 10 fps BOP frames. The same conversion as obchange_prep.py (Kitchen): colour +
depth_registered paired by stamp (<= 20 ms), T_wc = map <- head_rgbd_sensor_rgb_frame from /tf at the colour stamp.
The bags (3-13 GB per visit) are read twice without holding images in memory: pass 1 collects /tf and the image
stamps (read from the raw message header), pass 2 writes the kept frames. With --zip each bag is unpacked from the
room's zip to --tmp first and removed after.

Output (GT_DIR = $OBJMAP_GT, so the 10 fps tree is left alone when OBJMAP_GT points elsewhere):
  <WORK>/<room>/visit<v>/rgb|depth/NNNNNN.png   depth in mm (depth_scale_mm 1)
  <WORK>/<room>/all/rgb|depth/<gid>.png          symlinks, visit 1 frames, then visit 2, ...
  GT_DIR/obchange_<room>/seq/{all,visit<v>}/cameras.json   {K, width, height, depth_scale_mm: 1, fps: 30, ...}
  GT_DIR/obchange_<room>/<k:06d> -> <--gt-from>/obchange_<room>/<k:06d>  (the visit GT maps, shared)

    python obchange_raw_seq.py --room Office --zip ~/DeepLearning/Datasets/ObChange/Office.zip
"""
import argparse, bisect, json, os, shutil, struct, subprocess, zipfile
from pathlib import Path

import numpy as np
from PIL import Image
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from common import GT_DIR, HERE
from obchange_prep import CAM, DEPTH, INFO, RGB, WORLD, TfBuffer

WORK = Path(os.environ.get('OBJMAP_OBCHANGE_FRAMES', Path.home() / 'DeepLearning/Datasets/objmap_obchange/frames'))
FOLDER = {'office': 'GH30_office', 'livingarea': 'GH30_living', 'smallroom': 'Arena', 'bigroom': 'KennyLab'}


def stamp(raw):
    """Header stamp of a ROS1 message that starts with std_msgs/Header (uint32 seq, uint32 sec, uint32 nsec)."""
    s, ns = struct.unpack_from('<II', raw, 4)
    return s * 1_000_000_000 + ns


def read_visit(bag, out_dir, ts):
    tf = TfBuffer(); t_rgb, t_dep, K, wh = [], [], None, None
    with Reader(bag) as r:
        types = {}
        for c in r.connections:
            types.update(get_types_from_msg(c.msgdef.data, c.msgtype))
        ts.register(types)
        for c, _, raw in r.messages():
            if c.topic in ('/tf', '/tf_static'):
                for tr in ts.deserialize_ros1(raw, c.msgtype).transforms:
                    tf.add(tr, c.topic == '/tf_static')
            elif c.topic == RGB:
                t_rgb.append(stamp(raw))
            elif c.topic == DEPTH:
                t_dep.append(stamp(raw))
            elif c.topic == INFO and K is None:
                m = ts.deserialize_ros1(raw, c.msgtype)
                K = [float(v) for v in (m.K if hasattr(m, 'K') else m.k)]; wh = (m.width, m.height)
    tf.finish()
    t_rgb = sorted(set(t_rgb)); dts = sorted(set(t_dep))
    keep, n_nodepth, n_notf = {}, 0, 0           # rgb stamp -> (frame index, depth stamp, T_wc)
    for t in t_rgb:
        i = bisect.bisect_left(dts, t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(dts)]
        j = min(cand, key=lambda j: abs(dts[j] - t)) if cand else None
        if j is None or abs(dts[j] - t) > 20_000_000:
            n_nodepth += 1; continue
        T = tf.lookup(WORLD, CAM, t)
        if T is None:
            n_notf += 1; continue
        keep[t] = (len(keep), dts[j], T)
    want_dep = {}
    for t, (k, td, _) in keep.items():
        want_dep.setdefault(td, []).append(k)
    (out_dir / 'rgb').mkdir(parents=True, exist_ok=True); (out_dir / 'depth').mkdir(parents=True, exist_ok=True)
    with Reader(bag) as r:
        conns = [c for c in r.connections if c.topic in (RGB, DEPTH)]
        for c, _, raw in r.messages(connections=conns):
            t = stamp(raw)
            if c.topic == RGB and t in keep:
                m = ts.deserialize_ros1(raw, c.msgtype)
                a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                Image.fromarray(a if m.encoding == 'rgb8' else a[:, :, ::-1]).save(out_dir / 'rgb' / f'{keep[t][0]:06d}.png')
            elif c.topic == DEPTH and t in want_dep:
                m = ts.deserialize_ros1(raw, c.msgtype)
                d = np.frombuffer(m.data, np.uint16 if m.encoding == '16UC1' else np.float32).reshape(m.height, m.width)
                if m.encoding != '16UC1':
                    d = np.nan_to_num(d * 1000.0).astype(np.uint16)
                for k in want_dep.pop(t):
                    Image.fromarray(d).save(out_dir / 'depth' / f'{k:06d}.png')
    frames = [{'id': k, 'T_wc': T.tolist(), 'stamp_ns': int(t)} for t, (k, _, T) in sorted(keep.items(), key=lambda x: x[1][0])]
    return frames, K, wh, n_nodepth, n_notf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--room', required=True, help='Office | LivingArea | SmallRoom | BigRoom')
    ap.add_argument('--zip', help='the room zip of the ObChange record (bags are unpacked one at a time)')
    ap.add_argument('--bags', help='a folder with scene<v>.bag instead of --zip')
    ap.add_argument('--tmp', default=str(Path.home() / 'DeepLearning/Datasets/ObChange/raw/_tmp'))
    ap.add_argument('--gt-from', default=str(HERE / 'gt'), help='GT tree holding obchange_<room>/<k> (visit maps)')
    ap.add_argument('--visits', type=int, nargs='*')
    a = ap.parse_args()
    room = a.room.lower(); work = WORK / room; ts = get_typestore(Stores.ROS1_NOETIC)
    gsrc = Path(a.gt_from) / f'obchange_{room}'
    visits_avail = sorted(int(p.name) for p in gsrc.iterdir() if p.name.isdigit())
    visits = a.visits or list(range(1, max(visits_avail) + 1))
    gdst = GT_DIR / f'obchange_{room}'; gdst.mkdir(parents=True, exist_ok=True)
    if gdst.resolve() != gsrc.resolve():
        for k in visits_avail:
            l = gdst / f'{k:06d}'
            if not l.exists():
                l.symlink_to(gsrc / f'{k:06d}')
    allf, vlist, K, wh = [], [], None, None
    for v in visits:
        out = work / f'visit{v}'
        done = GT_DIR / f'obchange_{room}' / 'seq' / f'visit{v}' / 'cameras.json'
        if done.exists():
            cams = json.load(open(done)); frames = [{k: f[k] for k in ('id', 'T_wc', 'stamp_ns')} for f in cams['frames']]
            K, wh = cams['K'], (cams['width'], cams['height'])
            print(f'visit {v}: {len(frames)} frames (done before)', flush=True)
        else:
            if a.zip:
                Path(a.tmp).mkdir(parents=True, exist_ok=True)
                member = f'{FOLDER[room]}/scene{v}.bag'
                bag = Path(a.tmp) / f'{room}_scene{v}.bag'
                with zipfile.ZipFile(a.zip) as z, z.open(member) as src, open(bag, 'wb') as dst:
                    shutil.copyfileobj(src, dst, 64 << 20)
            else:
                bag = Path(a.bags) / f'scene{v}.bag'
            frames, K, wh, nd, nt = read_visit(bag, out, ts)
            if a.zip:
                bag.unlink()
            d = done.parent; d.mkdir(parents=True, exist_ok=True)
            json.dump({'K': K, 'width': wh[0], 'height': wh[1], 'depth_scale_mm': 1.0, 'fps': 30, 'seq_dir': str(out),
                       'frames': [{**f, 'visit': v} for f in frames],
                       'visits': [{'visit': v, 'first': 0, 'last': len(frames) - 1}]}, open(done, 'w'))
            dt = np.diff([f['stamp_ns'] for f in frames]) / 1e6
            print(f'visit {v}: {len(frames)} frames ({nd} without depth, {nt} without tf), '
                  f'dt median {np.median(dt):.1f} ms, max {dt.max():.0f} ms -> {out}', flush=True)
        first = len(allf)
        for f in frames:
            allf.append({**f, 'id': len(allf), 'visit': v, 'src': f['id']})
        vlist.append({'visit': v, 'first': first, 'last': len(allf) - 1})
    seq = work / 'all'
    for sub in ('rgb', 'depth'):
        (seq / sub).mkdir(parents=True, exist_ok=True)
    for f in allf:
        for sub in ('rgb', 'depth'):
            l = seq / sub / f"{f['id']:06d}.png"
            if not l.is_symlink():
                l.symlink_to(work / f"visit{f['visit']}" / sub / f"{f['src']:06d}.png")
    d = GT_DIR / f'obchange_{room}' / 'seq' / 'all'; d.mkdir(parents=True, exist_ok=True)
    json.dump({'K': K, 'width': wh[0], 'height': wh[1], 'depth_scale_mm': 1.0, 'fps': 30, 'seq_dir': str(seq),
               'frames': allf, 'visits': vlist}, open(d / 'cameras.json', 'w'))
    print('all:', len(allf), 'frames', vlist)


if __name__ == '__main__':
    main()
