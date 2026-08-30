#!/usr/bin/env python3
"""dump_all_cls.py — 6개 SAM 데이터 전 후보 박스의 DINOv2 CLS 임베딩을 저장한다 (READ-ONLY).

왜 임베딩을 저장하는가:
  aggregation 변형(top5 / mean / min / harmonic / consistency / fusion)을 바꿀 때마다
  모델을 재실행하면 비용이 수십 분씩 든다. CLS(384차원)만 한 번 저장해 두면
  42-view 유사도는 `tcls @ cls` 한 번의 행렬곱이므로 이후 모든 변형이 오프라인 즉시 계산된다.
  22,436 박스 × 384 × 4B = 약 34 MB.

운영 모듈은 import 만 하고 수정하지 않는다. YOLO·MobileSAM 은 실행하지 않는다
(박스 좌표는 ism_accuracy_observation 의 이전 덤프에서 재사용).

산출:
  semantic_fusion/box_cls.npz     uid -> float32[384]  (+ 템플릿 CLS 42x384 동봉)
"""
import csv, os, sys, time
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                       # ism_fusion_research
RSRCH = os.path.dirname(ROOT)                      # _ism_research_2026_07
REPO = os.path.dirname(RSRCH)                      # sam6d_ws
sys.path.insert(0, REPO)
import yolo_ism as yi                    # noqa: E402
import yolo_ism_object_n as o_n          # noqa: E402

OBS = os.path.join(RSRCH, "ism_accuracy_observation")
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
COLOR_TOPIC = "/camera/camera/color/image_raw"
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
BATCH = 64


def bag_frames(ds):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == COLOR_TOPIC]
        i = -1
        for conn, t, raw in reader.messages(connections=conns):
            i += 1
            msg = reader.deserialize(raw, conn.msgtype)
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            yield i, (cv2.cvtColor(buf, cv2.COLOR_RGB2BGR)
                      if msg.encoding.lower() == "rgb8" else buf.copy())


def main():
    # 박스 좌표 적재 (이전 덤프 재사용)
    byframe = {}
    n_box = 0
    for ds in DATASETS:
        d = {}
        for r in csv.DictReader(open(os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv"))):
            d.setdefault(int(r["frame_id"]), []).append(
                (r["uid"], [int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])]))
            n_box += 1
        byframe[ds] = d
    print(f"박스 {n_box:,} / 프레임 {sum(len(v) for v in byframe.values()):,}")

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)

    out = {}
    for o in objs:                                  # 템플릿 CLS 동봉 (재현성)
        out[f"__tcls__{o['name']}"] = o["tcls"].cpu().numpy().astype(np.float32)

    t0 = time.time()
    done = 0
    for ds in DATASETS:
        buf_c, buf_u = [], []

        def flush():
            nonlocal buf_c, buf_u, done
            if not buf_c:
                return
            cls_all, _ = o_n.dinov2_blocks_forward(model, buf_c, device, [11])
            arr = cls_all.detach().cpu().numpy().astype(np.float32)
            for u, v in zip(buf_u, arr):
                out[u] = v
            done += len(buf_u)
            buf_c, buf_u = [], []

        for fi, bgr in bag_frames(ds):
            bx = byframe[ds].get(fi)
            if not bx:
                continue
            norm = yi.normalize_rgb(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            for uid, b in bx:
                c = yi.crop_resize_pad(norm, b)
                if c is None:
                    continue
                buf_c.append(c); buf_u.append(uid)
                if len(buf_c) >= BATCH:
                    flush()
        flush()
        print(f"  {ds}: 누적 {done:,} 박스  ({time.time()-t0:.0f}s)")

    p = os.path.join(HERE, "box_cls.npz")
    np.savez_compressed(p, **out)
    print(f"\n-> {p}  ({done:,} 박스, {os.path.getsize(p)/1e6:.1f} MB, {time.time()-t0:.0f}s)")


if __name__ == "__main__":
    main()
