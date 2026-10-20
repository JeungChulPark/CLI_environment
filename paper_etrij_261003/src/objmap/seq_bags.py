"""Converted rosbag2 session (as ycbv_live/make_bags.py writes for YCB-V) for any sequence of common.seq_cameras,
so the live hub can replay it: /camera/camera/color/image_raw (bgr8), aligned depth (16UC1, mm), camera_info.
Frame n of the sequence (sorted by id) is stamped T0 + n/fps s (common.fps: 30, ObChange BOP rooms 10); gt_pose_stream.py uses the same clock.
Output: <BAGS>/<seq_name>/SAM (and SLAM -> the same files, single-camera data).

    python seq_bags.py --dataset obchange_kitchen_all --scenes 6
"""
import argparse, os, sys
from pathlib import Path

import cv2
import numpy as np
from rosbags.rosbag2 import Writer

from common import fps, seq_cameras, seq_name

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ycbv_live"))
from make_bags import COLOR, DEPTH, CINFO, ROI, T0, CameraInfo, Image, hdr, img_msg, ts  # noqa: E402

BAGS = Path(os.environ.get("SEQ_BAGS", Path.home() / "DeepLearning/Dataset/obchange/bags"))


def write(dataset, scene):
    cams = seq_cameras(dataset, scene)
    dst = BAGS / seq_name(dataset, scene) / "SAM"
    if (dst / "SAM_0.db3").exists():
        print("exists", dst); return dst.parent
    dst.parent.mkdir(parents=True, exist_ok=True)
    K = np.array(cams["K"], float); seq = Path(cams["seq_dir"]); scale = float(cams["depth_scale_mm"])
    frames = sorted(cams["frames"], key=lambda f: f["id"])
    rate = fps(cams)
    with Writer(dst, version=8) as w:
        cc = w.add_connection(COLOR, Image.__msgtype__, typestore=ts)
        cd = w.add_connection(DEPTH, Image.__msgtype__, typestore=ts)
        ci = w.add_connection(CINFO, CameraInfo.__msgtype__, typestore=ts)
        for n, f in enumerate(frames):
            t = T0 + int(round(n * 1e9 / rate))
            rgb = cv2.imread(str(seq / "rgb" / f"{f['id']:06d}.png"))
            d = cv2.imread(str(seq / "depth" / f"{f['id']:06d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32)
            dmm = np.clip(np.round(d * scale), 0, 65535).astype(np.uint16)
            info = CameraInfo(header=hdr(t, "camera_color_optical_frame"), height=rgb.shape[0], width=rgb.shape[1],
                              distortion_model="plumb_bob", d=np.zeros(5), k=K,
                              r=np.eye(3).reshape(-1), p=np.array([K[0], 0, K[2], 0, 0, K[4], K[5], 0, 0, 0, 1, 0], float),
                              binning_x=0, binning_y=0, roi=ROI(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
            w.write(ci, t, ts.serialize_cdr(info, CameraInfo.__msgtype__))
            w.write(cc, t, ts.serialize_cdr(img_msg(t, rgb, "bgr8"), Image.__msgtype__))
            w.write(cd, t, ts.serialize_cdr(img_msg(t, dmm, "16UC1"), Image.__msgtype__))
    db = next(dst.glob("*.db3")); db.rename(dst / "SAM_0.db3")
    meta = dst / "metadata.yaml"
    if meta.exists():
        meta.write_text(meta.read_text().replace(db.name, "SAM_0.db3"))
    (dst.parent / "SLAM").mkdir(exist_ok=True)
    for f in dst.iterdir():
        tgt = dst.parent / "SLAM" / f.name
        if not tgt.exists():
            tgt.symlink_to(f)
    print("wrote", dst, len(frames), "frames", flush=True)
    return dst.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--scenes", type=int, nargs="+", required=True)
    a = ap.parse_args()
    for s in a.scenes:
        write(a.dataset, s)


if __name__ == "__main__":
    main()
