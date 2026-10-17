"""live_session.py — live RealSense (ROS 2 topics) with the ConvSession interface, for `hub.py --sam-session live`.

The hub replays a recorded SAM session through `ConvSession`/`RsSession` (indexed frames, known
timeline). For a live camera there is no timeline: this class subscribes to the RealSense driver's
colour + aligned-depth topics (message_filters, same pairing as sam6d_receiver_node) in a background
rclpy thread and hands the hub the *newest* pair on request. Stamps are the colour header stamps
(driver `global_time_enabled:=true` = host clock) plus `offset_ns`, the SAM→SLAM clock offset
(the LiDAR host's clock minus this PC's clock, measured with objpose/tools/clock_offset.py).

Interface used by hub.py: K, kc, W, H, dir, t_ns (first/latest stamp), __len__, latest(), align().
"""
from __future__ import annotations

import threading
import time
from pathlib import Path

import numpy as np

from conv_session import Frame  # noqa: E402

COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"
CAMINFO = "/camera/camera/color/camera_info"


def _image_to_array(msg):
    h, w = int(msg.height), int(msg.width)
    enc = msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), np.uint8)
    if enc in ("bgr8", "rgb8"):
        img = buf.reshape(h, int(msg.step))[:, : w * 3].reshape(h, w, 3)
        if enc == "rgb8":
            img = img[:, :, ::-1]
        return np.ascontiguousarray(img)
    if enc in ("16uc1", "mono16"):
        return np.ascontiguousarray(buf.reshape(h, int(msg.step))[:, : w * 2].view("<u2").reshape(h, w))
    raise ValueError(f"unsupported encoding {msg.encoding}")


class LiveSamSession:
    def __init__(self, offset_ns: int = 0, topics=None, timeout_s: float = 120.0, slop_s: float = 0.02):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from message_filters import ApproximateTimeSynchronizer, Subscriber
        from sensor_msgs.msg import CameraInfo, Image

        topics = topics or {}
        self.dir = Path("live")
        self.offset_ns = int(offset_ns)
        self._K = None
        self.kc = [0.0] * 5
        self.W = self.H = 0
        self._lock = threading.Lock()
        self._latest = None            # (t_ns, bgr, depth_mm)
        self._consumed_t = -1
        self._first_ns = None
        self._latest_ns = None
        self.n_pairs = 0
        self._stop = threading.Event()

        rclpy.init(args=None)
        node = Node("objpose_live_sam")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        def on_info(m):
            if self._K is None:
                self._K = np.asarray(m.k, np.float64).reshape(3, 3)
                self.kc = [float(v) for v in (list(m.d) + [0.0] * 5)[:5]]
                self.W, self.H = int(m.width), int(m.height)

        def on_pair(color, depth):
            t = int(color.header.stamp.sec) * 1_000_000_000 + int(color.header.stamp.nanosec) + self.offset_ns
            try:
                bgr = _image_to_array(color)
                dep = _image_to_array(depth)
            except ValueError as e:
                node.get_logger().warn(str(e), throttle_duration_sec=5.0)
                return
            with self._lock:
                self._latest = (t, bgr, dep)
                self._latest_ns = t
                if self._first_ns is None:
                    self._first_ns = t
                self.n_pairs += 1

        node.create_subscription(CameraInfo, topics.get("caminfo", CAMINFO), on_info, qos)
        sync = ApproximateTimeSynchronizer(
            [Subscriber(node, Image, topics.get("rgb", COLOR), qos_profile=qos),
             Subscriber(node, Image, topics.get("depth", DEPTH), qos_profile=qos)],
            queue_size=10, slop=slop_s)
        sync.registerCallback(on_pair)
        self._node, self._sync = node, sync

        def spin():
            try:
                while not self._stop.is_set() and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.05)
            except Exception:              # ExternalShutdownException on Ctrl+C: the hub is going down anyway
                pass
        self._thread = threading.Thread(target=spin, daemon=True)
        self._thread.start()

        t_end = time.monotonic() + timeout_s
        while (self._K is None or self._latest is None) and time.monotonic() < t_end:
            time.sleep(0.05)
        if self._K is None or self._latest is None:
            raise RuntimeError("live camera: no camera_info / colour+depth pair received "
                               f"(topics {topics.get('rgb', COLOR)}, {topics.get('depth', DEPTH)}) within {timeout_s:.0f} s "
                               "— is the RealSense driver running in this ROS_DOMAIN_ID with align_depth.enable:=true?")

    # ── ConvSession-compatible surface ──────────────────────────────────────
    @property
    def K(self):
        return self._K.copy()

    @property
    def t_ns(self):
        """[first stamp, latest stamp] — the hub only reads t_ns[0] / t_ns[-1] in live mode"""
        with self._lock:
            f = self._first_ns if self._first_ns is not None else time.time_ns() + self.offset_ns
            l = self._latest_ns if self._latest_ns is not None else f
        return np.asarray([f, l], np.int64)

    def __len__(self):
        return 2

    def latest(self) -> Frame | None:
        """newest colour/depth pair not returned before, else None"""
        with self._lock:
            if self._latest is None or self._latest[0] == self._consumed_t:
                return None
            t, bgr, dep = self._latest
            self._consumed_t = t
        return Frame(self.n_pairs - 1, t, bgr, dep)

    def align(self, depth_mm):
        """aligned_depth_to_color is already on the colour grid"""
        return depth_mm

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self._node.destroy_node()
        except Exception:
            pass
