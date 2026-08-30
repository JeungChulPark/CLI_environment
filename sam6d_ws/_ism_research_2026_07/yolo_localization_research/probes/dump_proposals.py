#!/usr/bin/env python3
"""dump_proposals.py — 여러 proposal 설정의 raw 박스를 한 번에 덤프한다 (READ-ONLY).

운영 코드는 import 만 하고 수정하지 않는다. YOLO-World 만 다양한 설정으로 재실행해
**박스 좌표와 confidence** 를 저장한다. 평가는 전부 오프라인에서 한다.

설정 (config 이름 → 의미)
  cur          현행: 프롬프트 10개 각각 별도 forward, imgsz 960
  cur_1280     현행 프롬프트, imgsz 1280
  cur_1920     현행 프롬프트, imgsz 1920
  ml960        multi-label 단일 pass: set_classes(10개) 1회 forward, imgsz 960
  ml960_agn    위 + class-agnostic NMS
  generic      일반 범주 프롬프트 8개 (class-agnostic proposal 근사)
  ext          객체별 확장/외형 프롬프트 (prompt 개선)
  tile2x2      현행 프롬프트, 2x2 타일 (겹침 0.2) 후 좌표 복원
저장: candidate_dumps/<config>/<ds>.csv   (frame_id, prompt/class, x1..y2, conf)

⚠ 이 프로브는 박스만 만든다. DINOv2/MobileSAM 은 실행하지 않는다 — 이번 연구 범위가
proposal localization 이기 때문이다.
"""
import argparse, csv, os, sys, time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import yolo_ism_object_n as o_n          # noqa: E402

GT = os.path.join(RSRCH, "gt_input")
OUT = os.path.join(ROOT, "candidate_dumps")
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
COLOR_TOPIC = "/camera/camera/color/image_raw"
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
CONF = 0.005            # 관찰용으로 낮게 — 오프라인에서 임의 임계 재적용
MAXDET = 300

# 현행 프롬프트 (config 에서 읽음)
# 확장 프롬프트: 제품명 + 범주 + 외형. 객체당 3개.
EXT = {
    "milk": ["milk carton", "white paper milk carton", "rectangular beverage carton"],
    "choco_hazelnut_high": ["maroon box", "choco hazelnut snack box",
                            "rectangular packaged snack box with printed logo"],
    "Febreze_high": ["febreze spray bottle", "blue transparent spray bottle",
                     "plastic trigger spray bottle"],
    "Mugcup_high": ["mug cup", "pink ceramic mug with handle", "coffee mug"],
    "saffron": ["white jug", "beige plastic detergent jug with handle",
                "large plastic bottle with handle"],
    "Sauce_high": ["orange bottle", "brown sauce bottle with grey cap",
                   "small condiment bottle"],
    "Sikhye_high": ["yellow can", "gold aluminum beverage can", "drink can"],
    "Bear": ["brown bear doll", "brown teddy bear plush toy", "stuffed bear"],
    "Rabbit": ["white rabbit doll", "white bunny plush toy with long ears",
               "stuffed white rabbit"],
    "Dinosaur": ["green dinosaur doll", "green dinosaur plush toy", "stuffed green dinosaur"],
}
GENERIC = ["object", "box", "bottle", "can", "toy", "package", "container", "doll"]


def bag_frames(ds, want):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    hi = max(want)
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == COLOR_TOPIC]
        i = -1
        for conn, t, raw in reader.messages(connections=conns):
            i += 1
            if i > hi:
                break
            if i not in want:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            yield i, (cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
                      if msg.encoding.lower() == "rgb8" else buf.copy())


def run_pass(model, img, imgsz, device, agnostic=False, names=None, off=(0, 0)):
    r = model.predict(img, conf=CONF, imgsz=imgsz, max_det=MAXDET, verbose=False,
                      device=device, agnostic_nms=agnostic)
    out = []
    if len(r) and r[0].boxes is not None and len(r[0].boxes):
        b = r[0].boxes
        H, W = img.shape[:2]
        for j in range(len(b)):
            xy = b.xyxy[j].tolist()
            ci = int(b.cls[j]) if b.cls is not None else 0
            out.append((names[ci] if names else "?",
                        max(0, int(xy[0])) + off[0], max(0, int(xy[1])) + off[1],
                        min(W, int(xy[2])) + off[0], min(H, int(xy[3])) + off[1],
                        round(float(b.conf[j]), 4)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--configs", nargs="+",
                    default=["cur", "cur_1280", "cur_1920", "ml960", "ml960_agn",
                             "generic", "ext", "tile2x2"])
    ap.add_argument("--datasets", nargs="+", default=DATASETS)
    a = ap.parse_args()

    want = defaultdict(set)
    for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
        if r["user_reviewed"] == "yes":
            want[r["dataset_name"]].add(int(r["frame_id"]))
    print("GT 프레임:", {k: len(v) for k, v in sorted(want.items())})

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    W = defaults.get("weights", "yolov8m-worldv2.pt")
    CURP = [o["yolo_prompt"] for o in objs]
    P2O = {o["yolo_prompt"]: o["name"] for o in objs}
    print(f"weights={W} device={device}")

    from ultralytics import YOLOWorld
    cache = {}

    def M(classes, tag):
        if tag not in cache:
            y = YOLOWorld(W); y.set_classes(classes); cache[tag] = (y, classes)
        return cache[tag]

    for cfg in a.configs:
        os.makedirs(os.path.join(OUT, cfg), exist_ok=True)
        t0 = time.time()
        for ds in a.datasets:
            rows = []
            for fi, bgr in bag_frames(ds, want[ds]):
                h, w = bgr.shape[:2]
                if cfg.startswith("cur"):
                    sz = {"cur": 960, "cur_1280": 1280, "cur_1920": 1920}[cfg]
                    for p in CURP:
                        y, _ = M([p], f"single::{p}")
                        for nm, x1, y1, x2, y2, c in run_pass(y, bgr, sz, device, names=[p]):
                            rows.append({"frame_id": fi, "source": p, "target": P2O[p],
                                         "x1": x1, "y1": y1, "x2": x2, "y2": y2, "conf": c})
                elif cfg in ("ml960", "ml960_agn"):
                    y, cl = M(CURP, "multi::all")
                    for nm, x1, y1, x2, y2, c in run_pass(
                            y, bgr, 960, device, agnostic=(cfg == "ml960_agn"), names=cl):
                        rows.append({"frame_id": fi, "source": nm, "target": P2O.get(nm, ""),
                                     "x1": x1, "y1": y1, "x2": x2, "y2": y2, "conf": c})
                elif cfg == "generic":
                    for p in GENERIC:
                        y, _ = M([p], f"single::{p}")
                        for nm, x1, y1, x2, y2, c in run_pass(y, bgr, 960, device, names=[p]):
                            rows.append({"frame_id": fi, "source": p, "target": "",
                                         "x1": x1, "y1": y1, "x2": x2, "y2": y2, "conf": c})
                elif cfg == "ext":
                    for onm, ps in EXT.items():
                        for p in ps[1:]:                    # 현행(ps[0])은 cur 에 이미 있음
                            y, _ = M([p], f"single::{p}")
                            for nm, x1, y1, x2, y2, c in run_pass(y, bgr, 960, device, names=[p]):
                                rows.append({"frame_id": fi, "source": p, "target": onm,
                                             "x1": x1, "y1": y1, "x2": x2, "y2": y2, "conf": c})
                elif cfg == "tile2x2":
                    ov = 0.2
                    tw, th = int(w * (0.5 + ov / 2)), int(h * (0.5 + ov / 2))
                    for ox in (0, w - tw):
                        for oy in (0, h - th):
                            sub = bgr[oy:oy+th, ox:ox+tw]
                            for p in CURP:
                                y, _ = M([p], f"single::{p}")
                                for nm, x1, y1, x2, y2, c in run_pass(
                                        y, sub, 960, device, names=[p], off=(ox, oy)):
                                    rows.append({"frame_id": fi, "source": p, "target": P2O[p],
                                                 "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                                                 "conf": c})
            with open(os.path.join(OUT, cfg, f"{ds}.csv"), "w", newline="") as f:
                wt = csv.DictWriter(f, fieldnames=["frame_id", "source", "target",
                                                   "x1", "y1", "x2", "y2", "conf"])
                wt.writeheader(); wt.writerows(rows)
        n = sum(len(list(csv.DictReader(open(os.path.join(OUT, cfg, f"{d}.csv")))))
                for d in a.datasets)
        print(f"  {cfg:12s} 박스 {n:>8,}  ({time.time()-t0:.0f}s)")
    print(f"\n-> {OUT}")


if __name__ == "__main__":
    main()
