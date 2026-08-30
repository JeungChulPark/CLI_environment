#!/usr/bin/env python3
"""sam6d_multiobject_node.py — real-time multi-object SAM-6D (ISM + PEM) ROS2 node.

Runs in `sam6d_ros_humble`. All heavy data is loaded ONCE at startup:
  DINOv2 (semantic), MobileSAM (masked-appe), per-object template features,
  PEM model + per-object PEM template features + CAD point caches.
YOLO-World box proposals (only thing missing from this env) come from the
`sam_yolo` sidecar (tools/yoloworld_sidecar.py) over a Unix socket.

Per synchronized (RGB, aligned-depth) pair:
  sidecar -> boxes -> per-object ISM gate (recognize) -> for each accepted object
  build PEM input (observed cloud from depth+mask+K) -> coarse+fine pose.
Publishes geometry_msgs/PoseArray on ~/poses; optionally saves a unified overlay.
"""
import json
import os
import socket
import struct
import sys
import time

import cv2
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from scipy.spatial.transform import Rotation

REPO = "/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
PEM = os.path.join(REPO, "sam6d_master", "SAM-6D", "Pose_Estimation_Model")
ISM = os.path.join(REPO, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model")
sys.path.insert(0, REPO)
sys.path.insert(0, ISM)
import yolo_ism as yi
import yolo_ism_object_n as o_n
from segment_anything.utils.amg import mask_to_rle_pytorch

TMP = "/dev/shm/sam6d_rt"


class Sam6DMultiObjectNode(Node):
    def __init__(self):
        super().__init__("sam6d_multiobject")
        self.declare_parameter("config", o_n.DEFAULT_CONFIG)
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("caminfo_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("sidecar_sock", "/tmp/sam6d_yoloworld.sock")
        self.declare_parameter("det_score_thresh", 0.2)
        self.declare_parameter("save_overlay_dir", os.path.join(REPO, "outputs", "rt_node"))
        self.declare_parameter("save_every", 1)         # save overlay every Nth processed frame
        self.declare_parameter("device", "cuda:0")
        gp = lambda k: self.get_parameter(k).value

        self.device = gp("device") if torch.cuda.is_available() else "cpu"
        self.det_thresh = float(gp("det_score_thresh"))
        self.save_dir = gp("save_overlay_dir"); os.makedirs(self.save_dir, exist_ok=True)
        self.save_every = int(gp("save_every"))
        os.makedirs(TMP, exist_ok=True)
        self.bridge = CvBridge()
        self.K = None
        self.frame_i = 0

        # ---------- one-time loads ----------
        t0 = time.time()
        defaults, objs = o_n.load_config(gp("config"))
        objs = [o for o in objs if o.get("cad_ply") and
                os.path.isfile(o_n._abspath(o["cad_ply"]))]
        for o in objs:
            o["cad_abs"] = o_n._abspath(o["cad_ply"])
        self.get_logger().info(f"[load] DINOv2 + MobileSAM ...")
        self.model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, self.device)
        self.objs = o_n.prepare_objects(objs, self.model, self.device, rebuild=False)
        self.segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), self.device)
        self.pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
        self.unique_prompts, self.groups = o_n.build_prompt_groups(self.objs)
        self.min_score = min(float(o.get("score_threshold", 0.02)) for o in self.objs)

        # ---------- PEM (model + caches), loaded once ----------
        os.chdir(PEM)
        for sub in ("provider", "utils", "model", os.path.join("model", "pointnet2")):
            sys.path.append(os.path.join(PEM, sub))
        sys.path.insert(0, PEM)
        import gorilla, importlib
        import run_inference_custom as ric
        self.ric = ric
        self.cfg = gorilla.Config.fromfile(os.path.join(PEM, "config", "base.yaml"))
        self.cfg.model_name = "pose_estimation_model"
        self.get_logger().info("[load] PEM model ...")
        MODEL = importlib.import_module(self.cfg.model_name)
        self.pem = MODEL.Net(self.cfg.model).to(self.device).eval()
        gorilla.solver.load_checkpoint(model=self.pem, filename=os.path.join(PEM, "checkpoints", "sam-6d-pem-base.pth"))
        self._patch_cad_cache()       # CAD .npy point cache (shared with run_pem_batch)
        self._tem = {}                # PEM template features per object
        for o in self.objs:
            self._tem[o["name"]] = self._pem_templates(o["template_dir"])
        self.get_logger().info(f"[load] done in {time.time()-t0:.1f}s "
                               f"({len(self.objs)} objects)")

        # ---------- sidecar ----------
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect(gp("sidecar_sock"))
        self.get_logger().info("[sidecar] connected")

        # ---------- ROS I/O ----------
        self.pub = self.create_publisher(PoseArray, "~/poses", 10)
        self.create_subscription(CameraInfo, gp("caminfo_topic"), self._caminfo_cb, 10)
        sync = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, gp("rgb_topic")),
             Subscriber(self, Image, gp("depth_topic"))], queue_size=10, slop=0.05)
        sync.registerCallback(self._cb)
        self.get_logger().info("[ready] waiting for synced RGB + aligned-depth ...")

    # ---- PEM caches ----
    def _patch_cad_cache(self):
        import hashlib
        cdir = os.path.join(REPO, "outputs", "pem_inputs", "_model_pts_cache"); os.makedirs(cdir, exist_ok=True)
        orig = self.ric.trimesh.load_mesh
        class Stub:
            def __init__(s, p): s._p = p
            def sample(s, n):
                if n == len(s._p): return s._p
                idx = np.random.choice(len(s._p), n, replace=(n > len(s._p)))
                return s._p[idx]
        cache = {}
        def load(path, *a, **k):
            if path not in cache:
                cp = os.path.join(cdir, hashlib.md5(path.encode()).hexdigest()[:16] + ".npy")
                pts = np.load(cp) if os.path.isfile(cp) else None
                if pts is None:
                    pts = orig(path, *a, **k).sample(8192).astype(np.float32); np.save(cp, pts)
                cache[path] = Stub(pts)
            return cache[path]
        self.ric.trimesh.load_mesh = load

    def _pem_templates(self, tdir):
        a, b, c = self.ric.get_templates(tdir, self.cfg.test_dataset)
        with torch.inference_mode():
            tp, tf = self.pem.feature_extraction.get_obj_feats(a, b, c)
        return tp, tf

    def _caminfo_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(f"[caminfo] K fx={self.K[0,0]:.1f} received")

    # ---- sidecar query ----
    def _yolo_boxes(self, bgr):
        h, w, c = bgr.shape
        self.sock.sendall(struct.pack(">iii", h, w, c) + bgr.tobytes())
        n = struct.unpack(">i", self._recvall(4))[0]
        return json.loads(self._recvall(n).decode("utf-8"))

    def _recvall(self, n):
        buf = b""
        while len(buf) < n:
            ch = self.sock.recv(n - len(buf)); buf += ch
        return buf

    # ---- PEM on one detection ----
    def _pem_pose(self, o, bgr, depth_u16, mask):
        d = os.path.join(TMP, o["name"])
        os.makedirs(d, exist_ok=True)
        cv2.imwrite(f"{d}/rgb.png", bgr)
        cv2.imwrite(f"{d}/depth.png", depth_u16)
        json.dump({"cam_K": self.K.flatten().tolist(), "depth_scale": 1.0}, open(f"{d}/camera.json", "w"))
        rle = mask_to_rle_pytorch(torch.from_numpy(mask.astype(bool)).unsqueeze(0))[0]
        rle["counts"] = [int(x) for x in rle["counts"]]
        ys, xs = np.where(mask)
        det = [{"scene_id": 0, "image_id": 0, "category_id": 1,
                "bbox": [int(xs.min()), int(ys.min()), int(xs.max()-xs.min()), int(ys.max()-ys.min())],
                "score": 1.0, "segmentation": {"size": [int(mask.shape[0]), int(mask.shape[1])], "counts": rle["counts"]}}]
        json.dump(det, open(f"{d}/detection.json", "w"))
        inp, _img, _wp, mp, dets = self.ric.get_test_data(
            f"{d}/rgb.png", f"{d}/depth.png", f"{d}/camera.json", o["cad_abs"],
            f"{d}/detection.json", self.det_thresh, self.cfg.test_dataset)
        n = inp["pts"].size(0)
        tp, tf = self._tem[o["name"]]
        with torch.inference_mode():
            inp["dense_po"] = tp.repeat(n, 1, 1); inp["dense_fo"] = tf.repeat(n, 1, 1)
            out = self.pem(inp)
        ps = (out["pred_pose_score"] * out["score"]) if "pred_pose_score" in out else out["score"]
        ps = ps.detach().cpu().numpy()
        R = out["pred_R"].detach().cpu().numpy()[0]
        t = out["pred_t"].detach().cpu().numpy()[0] * 1000.0
        return R, t, float(ps[0]), mp

    # ---- main synced callback ----
    def _cb(self, rgb_msg, depth_msg):
        if self.K is None:
            return
        t0 = time.time()
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        try:
            boxes = self._yolo_boxes(bgr)
        except Exception as e:
            self.get_logger().warn(f"sidecar query failed: {e}"); return
        prompt_boxes = {i: [] for i in range(len(self.unique_prompts))}
        for b in boxes:
            if b["cls"] in prompt_boxes:
                prompt_boxes[b["cls"]].append((b["box"], b["conf"]))

        pa = PoseArray(); pa.header = rgb_msg.header
        overlay = rgb.copy(); drawn = []
        for pi, objs_here in self.groups.items():
            cand = sorted(prompt_boxes.get(pi, []), key=lambda t: t[1], reverse=True)
            for o in objs_here:
                r = o_n.recognize(o, cand, bgr, rgb, self.model, self.device, self.segmentor, self.pool)
                if not r["accepted"] or r["mask"] is None:
                    continue
                try:
                    R, t, score, mp = self._pem_pose(o, bgr, depth, r["mask"].astype(bool))
                except Exception as e:
                    self.get_logger().warn(f"PEM {o['name']} skipped: {type(e).__name__}")
                    continue
                p = Pose()
                p.position.x, p.position.y, p.position.z = float(t[0]/1000), float(t[1]/1000), float(t[2]/1000)
                q = Rotation.from_matrix(R).as_quat()
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
                pa.poses.append(p)
                col = self._color(o["name"])
                # get_test_data returns model_points in METERS; t is mm -> match units
                overlay = self.ric.draw_detections(overlay, R[None], t[None], mp * 1000.0, self.K[None], color=col)
                drawn.append((o["name"], score, col))
        self.pub.publish(pa)
        self.frame_i += 1
        if self.save_every and self.frame_i % self.save_every == 0 and drawn:
            self._save_overlay(overlay, drawn, self.frame_i)
        self.get_logger().info(f"frame {self.frame_i}: {len(pa.poses)} poses "
                               f"({', '.join(n for n,_,_ in drawn)})  {1000*(time.time()-t0):.0f}ms")

    _COLORS = [(0,0,255),(0,200,0),(255,0,0),(0,200,255),(255,0,255),(255,200,0),(128,0,255),(0,128,255)]
    def _color(self, name):
        return self._COLORS[sorted(o["name"] for o in self.objs).index(name) % len(self._COLORS)]

    def _save_overlay(self, rgb_img, drawn, idx):
        from PIL import Image as PImage
        out = rgb_img.copy(); y = 16
        for name, sc, col in drawn:
            cv2.rectangle(out, (6, y-10), (20, y), col, -1)
            cv2.putText(out, f"{name} {sc:.2f}", (24, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255,255,255), 1, cv2.LINE_AA)
            y += 18
        PImage.fromarray(np.uint8(out)).save(os.path.join(self.save_dir, f"frame_{idx:06d}.png"))


def main():
    rclpy.init()
    node = Sam6DMultiObjectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
