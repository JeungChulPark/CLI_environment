#!/usr/bin/env python3
"""
Replay a librealsense-SDK-format rosbag2 (the /device_0/sensor_N/... layout produced
by sync_record_gui_v3d) as the standard RealSense ROS2 topics that this project's
SLAM pipelines expect.

Why this exists
---------------
The 260826 datasets are raw RealSense SDK recordings:
  /device_0/sensor_1/Color_0/image/data   sensor_msgs/Image  bgr8    640x480
  /device_0/sensor_0/Depth_0/image/data   sensor_msgs/Image  mono16  640x480
  /device_0/sensor_*/.../camera_info      std_msgs/String    (NOT CameraInfo!)
  /device_0/sensor_*/.../tf/ref_0         std_msgs/String    (extrinsics)

whereas scripts/run_orbslam_four_bags.py (and rtabmap_launch) expect
  /camera/camera/color/image_raw
  /camera/camera/aligned_depth_to_color/image_raw

Two things therefore have to happen here:
  1. topic/type translation, including synthesising real sensor_msgs/CameraInfo
  2. **depth->color alignment**, which the recording does not contain. Depth is in
     the depth (left-IR) frame; color sits ~59.2 mm away on X. Feeding unaligned
     depth to an RGB-D SLAM system silently corrupts every 3D point, so we
     reproject depth into the color frame here.

Alignment math
--------------
`tf/ref_0` turned out to be the **reference -> stream** transform, not the
stream's pose in the reference frame. Both readings were tried and scored by
correlating depth discontinuities against colour image edges over 6 frames:

    p_c = R_c p_d + t_c     (ref->stream)   edge corr  0.0352   <- chosen
    (no alignment at all)                   edge corr  0.0219
    p_c = R_c^T (p_d - t_c) (stream->ref)   edge corr -0.0008

The chosen form beats raw depth while the other reading is *worse* than raw
(it shifts the wrong way and doubles the error), which is the expected signature
of a sign flip. Depth is the reference stream here (its tf is identity), so the
expression reduces to p_c = R_c p_d + t_c.

We deproject each depth pixel with the depth intrinsics, transform, project with
the colour intrinsics, and z-buffer the result. Vectorised with numpy so it keeps
up with 30 Hz (~3 ms/frame).

Distortion is deliberately ignored on the projection step: the color stream's
"Inverse Brown Conrady" coefficients are small (|k1| ~ 0.055) and ORB-SLAM3 is
given the same coefficients in its own settings file, so it undistorts the pair
consistently. This is noted as a known approximation rather than hidden.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import rosbag2_py
import yaml
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import deserialize_message
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

COLOR_IMG = "/device_0/sensor_1/Color_0/image/data"
DEPTH_IMG = "/device_0/sensor_0/Depth_0/image/data"
COLOR_CI = "/device_0/sensor_1/Color_0/camera_info"
DEPTH_CI = "/device_0/sensor_0/Depth_0/camera_info"
COLOR_TF = "/device_0/sensor_1/Color_0/tf/ref_0"
DEPTH_TF = "/device_0/sensor_0/Depth_0/tf/ref_0"
DEPTH_UNITS = "/device_0/sensor_0/option/Depth_Units/value"


def parse_kv(s):
    """'width=640;height=480;fx=384.925;...' -> dict"""
    out = {}
    for part in s.strip().split(";"):
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def parse_tf(s):
    """'rotation=r00,..,r22;translation=x,y,z' -> (R 3x3, t 3)"""
    d = parse_kv(s)
    R = np.array([float(x) for x in d["rotation"].split(",")], dtype=np.float64).reshape(3, 3)
    t = np.array([float(x) for x in d["translation"].split(",")], dtype=np.float64)
    return R, t


class Intr:
    def __init__(self, d):
        self.w = int(float(d["width"]))
        self.h = int(float(d["height"]))
        self.fx = float(d["fx"])
        self.fy = float(d["fy"])
        self.cx = float(d["ppx"])
        self.cy = float(d["ppy"])
        self.model = d.get("model", "")
        self.coeffs = [float(x) for x in d.get("coeffs", "0,0,0,0,0").split(",")]

    def __repr__(self):
        return (f"{self.w}x{self.h} fx={self.fx:.3f} fy={self.fy:.3f} "
                f"cx={self.cx:.3f} cy={self.cy:.3f} [{self.model}]")


def read_header(bag_dir):
    """One pass over the head of the bag to collect the String-encoded config."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    wanted = {COLOR_CI, DEPTH_CI, COLOR_TF, DEPTH_TF, DEPTH_UNITS}
    got = {}
    n = 0
    while reader.has_next() and len(got) < len(wanted) and n < 400_000:
        topic, data, _ = reader.read_next()
        n += 1
        if topic in wanted and topic not in got:
            got[topic] = deserialize_message(data, String).data
    missing = wanted - set(got)
    if missing:
        raise RuntimeError(f"bag is missing required config topics: {sorted(missing)}")
    return got


class AlignBridge(Node):
    def __init__(self, args):
        super().__init__("rs_bag_bridge")
        self.args = args
        bag = Path(args.bag)

        cfg = read_header(bag)
        self.ci_color = Intr(parse_kv(cfg[COLOR_CI]))
        self.ci_depth = Intr(parse_kv(cfg[DEPTH_CI]))
        Rc, tc = parse_tf(cfg[COLOR_TF])
        Rd, td = parse_tf(cfg[DEPTH_TF])
        self.depth_units = float(cfg[DEPTH_UNITS])

        # tf/ref_0 is ref->stream (validated by edge correlation, see module docstring).
        # Depth is the reference stream, so Rd/td are identity and:
        #     p_color = Rc (Rd^T (p_depth - td)) + tc  ==  Rc p_depth + tc
        self.R = Rc @ Rd.T
        self.t = tc - self.R @ td
        self.Rf = self.R.astype(np.float32)
        self.tf = self.t.astype(np.float32)

        self.get_logger().info(f"color intr : {self.ci_color}")
        self.get_logger().info(f"depth intr : {self.ci_depth}")
        self.get_logger().info(f"depth units: {self.depth_units} m/count "
                               f"(DepthMapFactor = {1.0/self.depth_units:.0f})")
        self.get_logger().info(f"depth->color t = {np.round(self.t, 6).tolist()} m "
                               f"(baseline {np.linalg.norm(self.t)*1000:.1f} mm)")

        # Precompute the deprojection rays for the depth image (constant per run).
        u = np.arange(self.ci_depth.w, dtype=np.float32)
        v = np.arange(self.ci_depth.h, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)
        self.ray_x = (uu - self.ci_depth.cx) / self.ci_depth.fx
        self.ray_y = (vv - self.ci_depth.cy) / self.ci_depth.fy

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=args.queue,
            reliability=(QoSReliabilityPolicy.RELIABLE if args.reliable
                         else QoSReliabilityPolicy.BEST_EFFORT),
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_rgb = self.create_publisher(Image, args.rgb_topic, qos)
        self.pub_depth = self.create_publisher(Image, args.depth_topic, qos)
        self.pub_rgb_ci = self.create_publisher(CameraInfo, args.rgb_info_topic, latched)
        self.pub_depth_ci = self.create_publisher(CameraInfo, args.depth_info_topic, latched)

        self.cam_info = self._build_camera_info()
        self.frame_id = args.frame_id

        self.n_pub = 0
        self.n_drop = 0
        self.align_ms = []
        self.skew_ms = []

    def _build_camera_info(self):
        ci = CameraInfo()
        c = self.ci_color
        ci.width, ci.height = c.w, c.h
        ci.distortion_model = "plumb_bob"
        k1, k2, p1, p2, k3 = (list(c.coeffs) + [0] * 5)[:5]
        ci.d = [k1, k2, p1, p2, k3]
        ci.k = [c.fx, 0.0, c.cx, 0.0, c.fy, c.cy, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [c.fx, 0.0, c.cx, 0.0, 0.0, c.fy, c.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def align_depth_to_color(self, depth_u16):
        """Reproject a depth image (depth frame) into the color frame. z-buffered."""
        c = self.ci_color
        z = depth_u16.astype(np.float32) * np.float32(self.depth_units)
        valid = z > 0
        if not valid.any():
            return np.zeros((c.h, c.w), dtype=np.uint16)

        zc = z[valid]
        x = self.ray_x[valid] * zc
        y = self.ray_y[valid] * zc
        # Expanded 3x3 multiply: avoids the 3xN stack and its matmul temporary,
        # which together dominated the per-frame cost.
        R = self.Rf
        t = self.tf
        xc = R[0, 0] * x + R[0, 1] * y + R[0, 2] * zc + t[0]
        yc = R[1, 0] * x + R[1, 1] * y + R[1, 2] * zc + t[1]
        zz = R[2, 0] * x + R[2, 1] * y + R[2, 2] * zc + t[2]

        ok = zz > 0
        if not ok.any():
            return np.zeros((c.h, c.w), dtype=np.uint16)
        xc, yc, zz = xc[ok], yc[ok], zz[ok]

        inv = np.reciprocal(zz)
        uu = np.rint(xc * inv * c.fx + c.cx).astype(np.int32)
        vv = np.rint(yc * inv * c.fy + c.cy).astype(np.int32)
        inb = (uu >= 0) & (uu < c.w) & (vv >= 0) & (vv < c.h)
        uu, vv, zz = uu[inb], vv[inb], zz[inb]

        # z-buffer: keep the NEAREST sample per target pixel, so an occluding
        # surface wins over what is behind it. np.minimum.at does this in one
        # scatter (~2.4 ms) and is 10x faster than sorting far->near, with
        # bit-identical output (both were benchmarked).
        zi = np.rint(zz / self.depth_units).astype(np.uint16)
        buf = np.full(c.h * c.w, 65535, dtype=np.uint16)
        np.minimum.at(buf, vv.astype(np.int64) * c.w + uu, zi)
        buf[buf == 65535] = 0
        return buf.reshape(c.h, c.w)

    def run(self):
        args = self.args
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(Path(args.bag)), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""),
        )
        sf = rosbag2_py.StorageFilter(topics=[COLOR_IMG, DEPTH_IMG])
        reader.set_filter(sf)

        # Latch the intrinsics before anything subscribes to images.
        self.cam_info.header.frame_id = self.frame_id
        self.pub_rgb_ci.publish(self.cam_info)
        self.pub_depth_ci.publish(self.cam_info)

        if args.start_delay > 0:
            self.get_logger().info(f"waiting {args.start_delay:.1f}s for subscribers...")
            t_end = time.time() + args.start_delay
            while time.time() < t_end and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

        # Colour runs ~27.9 Hz and depth ~26.8 Hz, so the streams are not 1:1 and
        # they interleave irregularly in the bag. Keep a short ring of recent depth
        # frames and match each colour frame to its NEAREST depth within tolerance
        # (the recorder's own associations file reports 7088 of 7965 colour frames
        # do have a depth partner, i.e. ~89%).
        from collections import deque
        # Both streams run at 30 Hz but are offset: the nearest depth for a given
        # colour frame is a *past* one ~13 ms back roughly half the time and a
        # *future* one the rest. Buffering only past depths therefore always picks
        # the wrong neighbour (~20 ms away) and almost everything fails the skew
        # test -- measured 5.6% paired. So hold colour frames one step and only
        # emit once a depth with a LATER timestamp has arrived, then choose the
        # nearest of the two sides. Measured from the bag's own timestamps:
        # 89.7% of colour frames have a depth within 20 ms, matching the
        # recorder's own associations file (7088/7965 = 89.0%).
        depth_buf = deque(maxlen=8)   # [(bag_ts, ndarray)]
        color_buf = deque()           # [(bag_ts, Image)]
        t0_wall = None
        t0_bag = None
        pub_period = 1.0 / args.rate if args.rate > 0 else 0.0
        # single reading of the clock; every stamp is an exact 1/30 s step from it
        self.t0_ns = self.get_clock().now().nanoseconds
        max_skew_ns = int(args.max_skew_ms * 1e6)

        self.get_logger().info(
            f"replaying -> {args.rgb_topic} + {args.depth_topic} "
            f"(rate={'as-recorded' if args.rate<=0 else str(args.rate)+'Hz'}, "
            f"limit={args.limit or 'all'})")

        while reader.has_next() and rclpy.ok():
            topic, data, bag_ts = reader.read_next()
            msg = deserialize_message(data, Image)

            if topic == DEPTH_IMG:
                d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                depth_buf.append((bag_ts, d))
            else:
                color_buf.append((bag_ts, msg))

            # A colour frame is ready to pair only once some depth is newer than it,
            # so both the earlier and the later candidate are on hand.
            if not depth_buf or not color_buf or depth_buf[-1][0] <= color_buf[0][0]:
                continue
            c_ts, msg = color_buf.popleft()
            d_ts, d_img = min(depth_buf, key=lambda p: abs(p[0] - c_ts))
            if abs(c_ts - d_ts) > max_skew_ns:
                self.n_drop += 1
                continue
            bag_ts = c_ts
            self.skew_ms.append(abs(c_ts - d_ts) / 1e6)

            t_a = time.perf_counter()
            aligned = self.align_depth_to_color(d_img)
            self.align_ms.append((time.perf_counter() - t_a) * 1e3)

            # pace playback against the bag clock (or a forced rate)
            if t0_wall is None:
                t0_wall, t0_bag = time.time(), bag_ts
            if pub_period > 0:
                target = t0_wall + self.n_pub * pub_period
            else:
                target = t0_wall + (bag_ts - t0_bag) / 1e9
            sleep = target - time.time()
            if sleep > 0:
                time.sleep(sleep)

            if args.stamp_now:
                # Monotonic by construction. Reading the wall clock here instead
                # is unsafe on WSL2: the system clock steps backwards when the
                # host re-syncs, and ORB-SLAM3 treats a timestamp older than the
                # previous frame as a reason to start a new Atlas map. One
                # measured run took six such steps (worst -482.5 ms) and created
                # seven maps against one for clean runs.
                ns = self.t0_ns + int(round(self.n_pub * 1e9 / 30.0))
                stamp = Time(sec=ns // 1_000_000_000,
                             nanosec=ns % 1_000_000_000)
            else:
                stamp = msg.header.stamp
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id

            dm = Image()
            dm.header.stamp = stamp
            dm.header.frame_id = self.frame_id
            dm.height, dm.width = aligned.shape
            dm.encoding = "16UC1"
            dm.is_bigendian = 0
            dm.step = dm.width * 2
            dm.data = aligned.tobytes()

            # CameraInfo must go out with EVERY frame: rgbd_odometry synchronises
            # rgb + depth + camera_info together, so a latched-only CameraInfo
            # starves the ApproximateTime filter and almost no frame is processed.
            self.cam_info.header.stamp = stamp
            self.cam_info.header.frame_id = self.frame_id
            self.pub_rgb_ci.publish(self.cam_info)
            self.pub_depth_ci.publish(self.cam_info)
            self.pub_rgb.publish(msg)
            self.pub_depth.publish(dm)
            self.n_pub += 1

            if self.n_pub % 200 == 0:
                a = np.array(self.align_ms[-200:])
                self.get_logger().info(
                    f"{self.n_pub} pairs published (dropped {self.n_drop}) "
                    f"align {a.mean():.1f}ms avg / {np.percentile(a,95):.1f}ms p95")

            if args.limit and self.n_pub >= args.limit:
                break

            rclpy.spin_once(self, timeout_sec=0.0)

        a = np.array(self.align_ms) if self.align_ms else np.array([0.0])
        k = np.array(self.skew_ms) if self.skew_ms else np.array([0.0])
        tot = self.n_pub + self.n_drop
        self.get_logger().info(
            f"DONE: {self.n_pub} pairs published, {self.n_drop} dropped "
            f"({100.0*self.n_pub/max(tot,1):.1f}% paired)")
        self.get_logger().info(
            f"      align  avg {a.mean():.2f} / p95 {np.percentile(a,95):.2f} / max {a.max():.2f} ms")
        self.get_logger().info(
            f"      rgb-d skew avg {k.mean():.2f} / p95 {np.percentile(k,95):.2f} / max {k.max():.2f} ms")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="directory holding metadata.yaml + .db3")
    ap.add_argument("--rgb-topic", default="/camera/camera/color/image_raw")
    ap.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    ap.add_argument("--rgb-info-topic", default="/camera/camera/color/camera_info")
    ap.add_argument("--depth-info-topic",
                    default="/camera/camera/aligned_depth_to_color/camera_info")
    ap.add_argument("--frame-id", default="camera_color_optical_frame")
    ap.add_argument("--rate", type=float, default=0.0,
                    help="0 = replay at the recorded pace; else force this Hz")
    ap.add_argument("--limit", type=int, default=0, help="stop after N pairs (0 = all)")
    ap.add_argument("--queue", type=int, default=5)
    ap.add_argument("--reliable", action="store_true",
                    help="RELIABLE QoS (default BEST_EFFORT, like a real camera)")
    ap.add_argument("--stamp-now", action="store_true",
                    help="restamp with wall clock (recorded stamps are device-relative)")
    ap.add_argument("--max-skew-ms", type=float, default=20.0)
    ap.add_argument("--start-delay", type=float, default=3.0)
    ap.add_argument("--dump-config", action="store_true",
                    help="print the bag's intrinsics/extrinsics and exit")
    args = ap.parse_args()

    if args.dump_config:
        cfg = read_header(Path(args.bag))
        ci_c = Intr(parse_kv(cfg[COLOR_CI]))
        ci_d = Intr(parse_kv(cfg[DEPTH_CI]))
        Rc, tc = parse_tf(cfg[COLOR_TF])
        Rd, td = parse_tf(cfg[DEPTH_TF])
        R = Rc @ Rd.T          # tf/ref_0 is ref->stream; see module docstring
        t = tc - R @ td
        print(json.dumps({
            "color": {"w": ci_c.w, "h": ci_c.h, "fx": ci_c.fx, "fy": ci_c.fy,
                      "cx": ci_c.cx, "cy": ci_c.cy, "model": ci_c.model,
                      "coeffs": ci_c.coeffs},
            "depth": {"w": ci_d.w, "h": ci_d.h, "fx": ci_d.fx, "fy": ci_d.fy,
                      "cx": ci_d.cx, "cy": ci_d.cy},
            "depth_units_m": float(cfg[DEPTH_UNITS]),
            "depth_map_factor": 1.0 / float(cfg[DEPTH_UNITS]),
            "depth_to_color": {"R": R.tolist(), "t": t.tolist(),
                               "baseline_mm": float(np.linalg.norm(t) * 1000)},
        }, indent=2))
        return 0

    rclpy.init()
    node = AlignBridge(args)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
