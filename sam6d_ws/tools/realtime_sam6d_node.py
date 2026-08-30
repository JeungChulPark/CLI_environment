#!/usr/bin/env python3
"""realtime_sam6d_node.py — `ros2 bag play` 를 실시간으로 받아 SAM-6D 를 돌리는 노드.

목적은 **PEM 회전 추정이 실제로 얼마나 흔들리는지 눈으로 보는 것**이다. 그래서 오프라인
배치처럼 모든 프레임을 처리하지 않고, **실시간 그대로** 동작한다.

  * 컬러/뎁스를 구독하되 **가장 최근 한 장만** 들고 있는다. 처리 중 들어온 프레임은 버린다
    (실제 로봇에서 일어나는 일 그대로).
  * 한 바퀴 = ISM(YOLO-World + DINOv2 + MobileSAM) -> 수락된 객체마다 PEM.
  * 결과는 JSONL 한 줄씩 append 한다. 렌더러가 이걸 읽어 좌우 비교 영상을 만든다.

ISM 과 PEM 을 한 프로세스에 두는 것이 핵심이다(2026-08-13 확인: `sam6d_ros_humble` 에
ultralytics 8.4.64 + clip 을 넣으면 둘 다 뜬다). 환경을 오가면 실시간이 성립하지 않는다.

실행:
    conda activate sam6d_ros_humble
    python tools/realtime_sam6d_node.py --out <기록 디렉터리>
그리고 다른 터미널에서
    ros2 bag play <bag> --clock
"""
import argparse, json, os, sys, threading, time
from collections import deque

import numpy as np
import cv2
import torch

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))

COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"
CAMINFO = "/camera/camera/color/camera_info"


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class RealtimeSam6D:
    """ROS 와 무관한 처리부. 프레임 하나를 받아 ISM+PEM 을 돌린다."""

    def __init__(self, device="cuda:0", scratch="/dev/shm/sam6d_rt"):
        import yolo_ism_object_n as o_n
        import yolo_ism as yi
        self.o_n, self.yi = o_n, yi
        self.dev = device if torch.cuda.is_available() else "cpu"
        self.scratch = scratch
        os.makedirs(scratch, exist_ok=True)

        defaults, cfg = o_n.load_config(o_n.DEFAULT_CONFIG)
        self.model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, self.dev)
        objs = o_n.prepare_objects(cfg, self.model, self.dev, False)
        # CAD 가 없는 객체는 PEM 을 못 돌리므로 뺀다
        keep = []
        for o in objs:
            cad = o_n._abspath(o.get("cad_ply", ""))
            if cad and os.path.isfile(cad):
                o["cad_abs"] = cad
                keep.append(o)
        self.objs = keep
        self.prompts, self.groups = o_n.build_prompt_groups(keep)
        self.tsim = o_n.template_similarity(keep)
        self.seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), self.dev)
        self.pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
        self.min_score = min(float(o.get("score_threshold", 0.02)) for o in keep)
        self.imgsz = int(defaults.get("imgsz", 640))
        from ultralytics import YOLOWorld
        self.yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
        self.yolo.set_classes(self.prompts)
        print(f"[ism] {len(keep)} objects, imgsz {self.imgsz}, multi_label "
              f"{o_n._multi_label_on(keep[0])}", flush=True)
        self._init_pem()

    def _init_pem(self):
        import importlib
        PEM = os.path.join(REPO, "sam6d_master", "SAM-6D", "Pose_Estimation_Model")
        self.PEM = PEM
        cwd = os.getcwd()
        os.chdir(PEM)
        for sub in ("provider", "utils", "model", os.path.join("model", "pointnet2")):
            sys.path.append(os.path.join(PEM, sub))
        sys.path.insert(0, PEM)
        import gorilla
        import run_inference_custom as ric
        self.ric = ric
        cfg = gorilla.Config.fromfile(os.path.join(PEM, "config", "base.yaml"))
        cfg.model_name = "pose_estimation_model"
        MODEL = importlib.import_module(cfg.model_name)
        self.pem = MODEL.Net(cfg.model).to(self.dev).eval()
        gorilla.solver.load_checkpoint(
            model=self.pem, filename=os.path.join(PEM, "checkpoints", "sam-6d-pem-base.pth"))
        self.pem_cfg = cfg
        os.chdir(cwd)
        self.tem_cache = {}
        print("[pem] model loaded", flush=True)

    def _templates(self, tdir):
        if tdir not in self.tem_cache:
            import hashlib
            cdir = os.path.join(REPO, "outputs", "pem_inputs", "_tem_feat_cache")
            os.makedirs(cdir, exist_ok=True)
            key = hashlib.md5(tdir.encode()).hexdigest()[:16]
            cpath = os.path.join(cdir, f"{key}.pt")
            if os.path.isfile(cpath):
                d = torch.load(cpath, map_location=self.dev)
                self.tem_cache[tdir] = (d["tp"].to(self.dev), d["tf"].to(self.dev))
            else:
                cwd = os.getcwd(); os.chdir(self.PEM)
                a, b, c = self.ric.get_templates(tdir, self.pem_cfg.test_dataset)
                with torch.inference_mode():
                    tp, tf = self.pem.feature_extraction.get_obj_feats(a, b, c)
                os.chdir(cwd)
                torch.save({"tp": tp.cpu(), "tf": tf.cpu()}, cpath)
                self.tem_cache[tdir] = (tp.to(self.dev), tf.to(self.dev))
        return self.tem_cache[tdir]

    def process(self, bgr, depth, K):
        """한 프레임 -> [{object, R, t_mm, score, bbox, ism_appe}]  + 단계별 소요시간"""
        from segment_anything.utils.amg import mask_to_rle_pytorch
        o_n, yi = self.o_n, self.yi
        t0 = time.perf_counter()
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        nf = yi.normalize_rgb(rgb)
        with o_n.multi_label_nms(o_n._multi_label_on(self.objs[0])):
            pr = self.yolo.predict(bgr, conf=self.min_score, imgsz=self.imgsz,
                                   verbose=False, device=self.dev)
        pb = {i: [] for i in range(len(self.prompts))}
        if len(pr) and pr[0].boxes is not None and len(pr[0].boxes):
            b = pr[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                ci = int(b.cls[j]) if b.cls is not None else 0
                if ci in pb:
                    pb[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
        for k in pb:
            pb[k].sort(key=lambda z: -z[1])
        res = o_n.recognize_frame_auto(self.groups, pb, bgr, rgb, nf, self.model,
                                       self.dev, self.seg, self.pool, self.tsim)
        t_ism = time.perf_counter() - t0

        acc = [(o, res[o["name"]]) for o in self.objs
               if res.get(o["name"], {}).get("accepted") and res[o["name"]].get("mask") is not None]
        out = []
        t1 = time.perf_counter()
        if acc:
            sd = self.scratch
            cv2.imwrite(f"{sd}/rgb.png", bgr)
            cv2.imwrite(f"{sd}/depth.png", depth)
            json.dump({"cam_K": [float(v) for v in np.asarray(K).ravel()], "depth_scale": 1.0},
                      open(f"{sd}/camera.json", "w"))
            cwd = os.getcwd(); os.chdir(self.PEM)
            for o, r in acc:
                try:
                    mask = r["mask"].astype(bool)
                    rle = mask_to_rle_pytorch(torch.from_numpy(mask).unsqueeze(0))[0]
                    x1, y1, x2, y2 = r["box"]
                    det = [{"scene_id": 0, "image_id": 0, "category_id": 1,
                            "bbox": [int(x1), int(y1), int(x2-x1), int(y2-y1)],
                            "score": float(r["masked_appe"]),
                            "segmentation": {"size": [int(h), int(w)],
                                             "counts": [int(c) for c in rle["counts"]]}}]
                    sp = f"{sd}/detection_{o['name']}.json"
                    json.dump(det, open(sp, "w"))
                    tp, tf = self._templates(o["template_dir"])
                    inp, _img, _wp, _mp, dets = self.ric.get_test_data(
                        f"{sd}/rgb.png", f"{sd}/depth.png", f"{sd}/camera.json",
                        o["cad_abs"], sp, 0.2, self.pem_cfg.test_dataset)
                    n = inp["pts"].size(0)
                    with torch.inference_mode():
                        inp["dense_po"] = tp.repeat(n, 1, 1)
                        inp["dense_fo"] = tf.repeat(n, 1, 1)
                        po = self.pem(inp)
                    sc = (po["pred_pose_score"] * po["score"]) if "pred_pose_score" in po else po["score"]
                    i = int(sc.detach().cpu().numpy().argmax())
                    out.append({"object": o["name"],
                                "R": po["pred_R"].detach().cpu().numpy()[i].tolist(),
                                "t_mm": (po["pred_t"].detach().cpu().numpy()[i]*1000).tolist(),
                                "score": float(sc.detach().cpu().numpy()[i]),
                                "bbox": [int(x1), int(y1), int(x2), int(y2)],
                                "ism_appe": float(r["masked_appe"])})
                except Exception as e:
                    print(f"  [pem-fail] {o['name']}: {type(e).__name__}: {e}", flush=True)
            os.chdir(cwd)
        return out, t_ism, time.perf_counter() - t1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="결과 JSONL 을 쓸 디렉터리")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--max-seconds", type=float, default=0.0, help="0 = 무제한")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, CameraInfo

    engine = RealtimeSam6D(a.device)
    jf = open(os.path.join(a.out, "results.jsonl"), "w")
    lock = threading.Lock()
    latest = {"color": None, "depth": None, "K": None}
    stats = {"color": 0, "processed": 0, "dropped": 0}
    stop = threading.Event()

    class N(Node):
        def __init__(self):
            super().__init__("sam6d_realtime")
            q = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(Image, COLOR, self.cb_color, q)
            self.create_subscription(Image, DEPTH, self.cb_depth, q)
            self.create_subscription(CameraInfo, CAMINFO, self.cb_info, q)

        def cb_color(self, m):
            buf = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
            img = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else buf.copy()
            with lock:
                if latest["color"] is not None:
                    stats["dropped"] += 1
                latest["color"] = (stamp_s(m.header), img)
                stats["color"] += 1

        def cb_depth(self, m):
            d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()
            with lock:
                latest["depth"] = (stamp_s(m.header), d)

        def cb_info(self, m):
            with lock:
                if latest["K"] is None:
                    latest["K"] = [float(v) for v in m.k]

    def worker():
        t_start = time.time()
        while not stop.is_set():
            with lock:
                c, d, K = latest["color"], latest["depth"], latest["K"]
                latest["color"] = None          # 소비 = 최신 것만 처리
            if c is None or d is None or K is None:
                time.sleep(0.005)
                continue
            ts, bgr = c
            _, depth = d
            if depth.shape != bgr.shape[:2]:
                continue
            t_recv = time.time()
            dets, t_ism, t_pem = engine.process(bgr, depth, K)
            rec = {"stamp": ts, "t_start": t_recv, "t_done": time.time(),
                   "t_ism": round(t_ism, 4), "t_pem": round(t_pem, 4),
                   "n": len(dets), "dets": dets}
            jf.write(json.dumps(rec) + "\n"); jf.flush()
            stats["processed"] += 1
            if stats["processed"] % 5 == 0:
                el = time.time() - t_start
                print(f"  처리 {stats['processed']}  버림 {stats['dropped']}  "
                      f"ISM {t_ism*1000:.0f}ms PEM {t_pem*1000:.0f}ms  "
                      f"({stats['processed']/max(el,1e-6):.2f} Hz)", flush=True)
            if a.max_seconds and time.time() - t_start > a.max_seconds:
                stop.set()

    rclpy.init()
    node = N()
    th = threading.Thread(target=worker, daemon=True)
    th.start()
    print("[node] 대기 중 — 다른 터미널에서 ros2 bag play 를 시작하라", flush=True)
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    stop.set(); th.join(timeout=30)
    jf.close()
    json.dump(stats, open(os.path.join(a.out, "stats.json"), "w"), indent=1)
    print(f"\n[node] 컬러 수신 {stats['color']} · 처리 {stats['processed']} · 버림 {stats['dropped']}")
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
