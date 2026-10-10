"""ObChange-Kitchen rosbags (ROS1, HSR head RGB-D) -> the sequence layout every adapter of this folder reads.

Per visit k (scene1..scene6 of GH30_kitchen; visit 1 has no objects):
  <WORK>/visit<k>/rgb/NNNNNN.png      colour (rgb8 -> PNG)
  <WORK>/visit<k>/depth/NNNNNN.png    depth_registered (16UC1, mm, aligned to the colour camera)
and one continuous sequence of all visits in order (the revisit experiment), as symlinks:
  <WORK>/all/rgb|depth/NNNNNN.png     visit 1 frames, then visit 2, ...
Cameras, in the frame of the ObChange reconstructions and GT (`map`):
  gt/obchange_kitchen/seq/visit<k>/cameras.json and gt/obchange_kitchen/seq/all/cameras.json
  {K, width, height, depth_scale_mm: 1, seq_dir, frames: [{id, T_wc, visit, stamp_ns}], visits: [{visit, first, last}]}
T_wc = map -> head_rgbd_sensor_rgb_frame from /tf and /tf_static at the colour stamp; every dynamic link is
interpolated (linear translation, slerp rotation) between its two bracketing messages. A depth frame is paired
with the colour frame of the nearest stamp (<= 20 ms); unpaired frames and frames without a full tf chain are
skipped.
Usage: python obchange_prep.py [--bags ~/DeepLearning/Dataset/obchange/GH30_kitchen] [--work ...]
"""
import argparse, bisect, json, os
from pathlib import Path

import numpy as np
from PIL import Image
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from scipy.spatial.transform import Rotation, Slerp

from common import GT_DIR

RGB = '/hsrb/head_rgbd_sensor/rgb/image_raw'
DEPTH = '/hsrb/head_rgbd_sensor/depth_registered/image_raw'
INFO = '/hsrb/head_rgbd_sensor/rgb/camera_info'
WORLD, CAM = 'map', 'head_rgbd_sensor_rgb_frame'


def strip(f):
    return f.lstrip('/')


class TfBuffer:
    def __init__(self):
        self.static = {}        # child -> (parent, 4x4)
        self.dyn = {}           # child -> (parent, [t_ns], [xyz], [quat xyzw])

    def add(self, tr, static):
        p, c = strip(tr.header.frame_id), strip(tr.child_frame_id)
        q = tr.transform.rotation; v = tr.transform.translation
        if static:
            T = np.eye(4); T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(); T[:3, 3] = [v.x, v.y, v.z]
            self.static[c] = (p, T)
        else:
            t = tr.header.stamp.sec * 1_000_000_000 + tr.header.stamp.nanosec
            d = self.dyn.setdefault(c, (p, [], [], []))
            d[1].append(t); d[2].append([v.x, v.y, v.z]); d[3].append([q.x, q.y, q.z, q.w])

    def finish(self):
        for c, (p, t, x, q) in self.dyn.items():
            o = np.argsort(t, kind='stable')
            t = np.asarray(t)[o]; x = np.asarray(x)[o]; q = np.asarray(q)[o]
            keep = np.r_[True, np.diff(t) > 0]
            self.dyn[c] = (p, t[keep], x[keep], q[keep])

    def link(self, child, t_ns, max_gap_ns=200_000_000):
        if child in self.static:
            return self.static[child]
        p, t, x, q = self.dyn[child]
        i = bisect.bisect_left(t, t_ns)
        if i == 0 or i == len(t):
            j = 0 if i == 0 else len(t) - 1
            if abs(t[j] - t_ns) > max_gap_ns:
                return p, None
            T = np.eye(4); T[:3, :3] = Rotation.from_quat(q[j]).as_matrix(); T[:3, 3] = x[j]
            return p, T
        a, b = i - 1, i
        if t[b] - t[a] > max_gap_ns:
            return p, None
        w = (t_ns - t[a]) / (t[b] - t[a])
        T = np.eye(4)
        T[:3, :3] = Slerp([0, 1], Rotation.from_quat([q[a], q[b]]))([w]).as_matrix()[0]
        T[:3, 3] = (1 - w) * x[a] + w * x[b]
        return p, T

    def lookup(self, parent, child, t_ns):
        """T_parent<-child (maps child coordinates into parent), following child -> ... -> parent."""
        T = np.eye(4); f = child
        while f != parent:
            if f not in self.static and f not in self.dyn:
                return None
            p, L = self.link(f, t_ns)
            if L is None:
                return None
            T = L @ T; f = p
        return T


def read_visit(bag, out_dir, ts):
    tf = TfBuffer(); rgb, depth, K = [], [], None
    (out_dir / 'rgb').mkdir(parents=True, exist_ok=True); (out_dir / 'depth').mkdir(parents=True, exist_ok=True)
    with Reader(bag) as r:
        types = {}
        for c in r.connections:
            types.update(get_types_from_msg(c.msgdef.data, c.msgtype))
        ts.register(types)
        for c, _, raw in r.messages():
            if c.topic in ('/tf', '/tf_static'):
                for tr in ts.deserialize_ros1(raw, c.msgtype).transforms:
                    tf.add(tr, c.topic == '/tf_static')
            elif c.topic in (RGB, DEPTH):
                m = ts.deserialize_ros1(raw, c.msgtype)
                t = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
                (rgb if c.topic == RGB else depth).append((t, m))
            elif c.topic == INFO and K is None:
                m = ts.deserialize_ros1(raw, c.msgtype)
                K = [float(v) for v in (m.K if hasattr(m, 'K') else m.k)]
                wh = (m.width, m.height)
    tf.finish()
    rgb.sort(key=lambda x: x[0]); depth.sort(key=lambda x: x[0])
    dts = [t for t, _ in depth]
    frames, n_nodepth, n_notf = [], 0, 0
    for t, m in rgb:
        i = bisect.bisect_left(dts, t)
        cand = [j for j in (i - 1, i) if 0 <= j < len(dts)]
        j = min(cand, key=lambda j: abs(dts[j] - t)) if cand else None
        if j is None or abs(dts[j] - t) > 20_000_000:
            n_nodepth += 1; continue
        T = tf.lookup(WORLD, CAM, t)
        if T is None:
            n_notf += 1; continue
        k = len(frames)
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        Image.fromarray(a if m.encoding == 'rgb8' else a[:, :, ::-1]).save(out_dir / 'rgb' / f'{k:06d}.png')
        dm = depth[j][1]
        d = np.frombuffer(dm.data, np.uint16 if dm.encoding == '16UC1' else np.float32).reshape(dm.height, dm.width)
        if dm.encoding != '16UC1':
            d = np.nan_to_num(d * 1000.0).astype(np.uint16)
        Image.fromarray(d).save(out_dir / 'depth' / f'{k:06d}.png')
        frames.append({'id': k, 'T_wc': T.tolist(), 'stamp_ns': int(t)})
    return frames, K, wh, n_nodepth, n_notf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bags', default=str(Path.home() / 'DeepLearning/Dataset/obchange/GH30_kitchen'))
    ap.add_argument('--work', default=str(Path.home() / 'DeepLearning/Dataset/obchange/work/kitchen'))
    ap.add_argument('--visits', type=int, nargs='*', default=[1, 2, 3, 4, 5, 6])
    a = ap.parse_args()
    work = Path(a.work); ts = get_typestore(Stores.ROS1_NOETIC)
    allf, visits = [], []
    for v in a.visits:
        out = work / f'visit{v}'
        frames, K, wh, nd, nt = read_visit(Path(a.bags) / f'scene{v}.bag', out, ts)
        cams = {'K': K, 'width': wh[0], 'height': wh[1], 'depth_scale_mm': 1.0, 'seq_dir': str(out),
                'frames': [{**f, 'visit': v} for f in frames], 'visits': [{'visit': v, 'first': 0, 'last': len(frames) - 1}]}
        d = GT_DIR / 'obchange_kitchen' / 'seq' / f'visit{v}'; d.mkdir(parents=True, exist_ok=True)
        json.dump(cams, open(d / 'cameras.json', 'w'))
        print(f'visit {v}: {len(frames)} frames ({nd} without depth, {nt} without tf) -> {out}', flush=True)
        first = len(allf)
        for f in frames:
            k = len(allf)
            allf.append({**f, 'id': k, 'visit': v, 'src': f['id']})
        visits.append({'visit': v, 'first': first, 'last': len(allf) - 1})
    # continuous sequence: symlinks, ids run on across visits
    seq = work / 'all'
    for sub in ('rgb', 'depth'):
        (seq / sub).mkdir(parents=True, exist_ok=True)
    for f in allf:
        for sub in ('rgb', 'depth'):
            l = seq / sub / f"{f['id']:06d}.png"
            if not l.exists():
                l.symlink_to(work / f"visit{f['visit']}" / sub / f"{f['src']:06d}.png")
    d = GT_DIR / 'obchange_kitchen' / 'seq' / 'all'; d.mkdir(parents=True, exist_ok=True)
    json.dump({'K': K, 'width': wh[0], 'height': wh[1], 'depth_scale_mm': 1.0, 'seq_dir': str(seq),
               'frames': allf, 'visits': visits}, open(d / 'cameras.json', 'w'))
    print('all:', len(allf), 'frames', visits)


if __name__ == '__main__':
    main()
