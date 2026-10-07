"""Convert BOP YCB-V test videos (full sequences) into "converted" rosbag2 sessions that
objpose's slam_stream and ConvSession read: /camera/camera/color/image_raw (bgr8),
/camera/camera/aligned_depth_to_color/image_raw (16UC1, mm), /camera/camera/color/camera_info.
Stamps are synthetic at 30 fps (YCB-V frames are consecutive at 30 Hz).

    python make_bags.py --ycbv <bop>/ycbv/full/test --out <dir> [--scenes 48,49,...]
"""
import argparse, json, os
from pathlib import Path
import numpy as np, cv2
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

ts = get_typestore(Stores.ROS2_HUMBLE)
Image = ts.types['sensor_msgs/msg/Image']; CameraInfo = ts.types['sensor_msgs/msg/CameraInfo']
Header = ts.types['std_msgs/msg/Header']; Time = ts.types['builtin_interfaces/msg/Time']
ROI = ts.types['sensor_msgs/msg/RegionOfInterest']
T0 = 1_700_000_000_000_000_000
COLOR, DEPTH, CINFO = ('/camera/camera/color/image_raw', '/camera/camera/aligned_depth_to_color/image_raw',
                       '/camera/camera/color/camera_info')


def hdr(t, frame):
    return Header(stamp=Time(sec=t // 1_000_000_000, nanosec=t % 1_000_000_000), frame_id=frame)


def img_msg(t, a, enc):
    h, w = a.shape[:2]; step = a.strides[0]
    return Image(header=hdr(t, 'camera_color_optical_frame'), height=h, width=w, encoding=enc, is_bigendian=0,
                 step=step, data=np.ascontiguousarray(a).view(np.uint8).reshape(-1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ycbv', default=os.environ.get('YCBV_FULL', str(Path.home() / 'DeepLearning/Dataset/bop/ycbv/full/test')))
    ap.add_argument('--out', default=os.environ.get('YCBV_BAGS', str(Path.home() / 'DeepLearning/Dataset/bop/ycbv_bags')))
    ap.add_argument('--scenes', default='48,49,50,51,52,53,54,55,56,57,58,59')
    a = ap.parse_args()
    for s in [int(x) for x in a.scenes.split(',')]:
        src = Path(a.ycbv) / f'{s:06d}'; dst = Path(a.out) / f'ycbv_{s:06d}' / 'SAM'
        if (dst / 'SAM_0.db3').exists():
            print('exists', dst); continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        cams = json.load(open(src / 'scene_camera.json'))
        ids = sorted(int(k) for k in cams)
        with Writer(dst, version=8) as w:
            cc = w.add_connection(COLOR, Image.__msgtype__, typestore=ts)
            cd = w.add_connection(DEPTH, Image.__msgtype__, typestore=ts)
            ci = w.add_connection(CINFO, CameraInfo.__msgtype__, typestore=ts)
            for n, i in enumerate(ids):
                t = T0 + int(round(n * 1e9 / 30))
                cam = cams[str(i)]; K = np.array(cam['cam_K'], float)
                rgb = cv2.imread(str(src / 'rgb' / f'{i:06d}.png'))
                d = cv2.imread(str(src / 'depth' / f'{i:06d}.png'), cv2.IMREAD_UNCHANGED).astype(np.float32)
                dmm = np.clip(np.round(d * float(cam['depth_scale'])), 0, 65535).astype(np.uint16)
                info = CameraInfo(header=hdr(t, 'camera_color_optical_frame'), height=rgb.shape[0], width=rgb.shape[1],
                                  distortion_model='plumb_bob', d=np.zeros(5), k=K,
                                  r=np.eye(3).reshape(-1), p=np.array([K[0], 0, K[2], 0, 0, K[4], K[5], 0, 0, 0, 1, 0], float),
                                  binning_x=0, binning_y=0, roi=ROI(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
                w.write(ci, t, ts.serialize_cdr(info, CameraInfo.__msgtype__))
                w.write(cc, t, ts.serialize_cdr(img_msg(t, rgb, 'bgr8'), Image.__msgtype__))
                w.write(cd, t, ts.serialize_cdr(img_msg(t, dmm, '16UC1'), Image.__msgtype__))
        db = next(dst.glob('*.db3')); db.rename(dst / 'SAM_0.db3')
        meta = dst / 'metadata.yaml'
        if meta.exists():
            meta.write_text(meta.read_text().replace(db.name, 'SAM_0.db3'))
        # the SLAM side replays the same camera (single-camera dataset)
        (dst.parent / 'SLAM').mkdir(exist_ok=True)
        for f in dst.iterdir():
            tgt = dst.parent / 'SLAM' / f.name
            if not tgt.exists():
                tgt.symlink_to(f)
        print('wrote', dst, len(ids), 'frames')


if __name__ == '__main__':
    main()
