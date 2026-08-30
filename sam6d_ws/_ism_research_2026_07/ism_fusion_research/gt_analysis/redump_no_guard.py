#!/usr/bin/env python3
"""redump_no_guard.py — GT 339 프레임에서 **객체별 score_threshold 가드를 제거**하고 재덤프.

왜 필요한가
  기존 덤프(pairs.csv)에는 이미 가드를 통과한 후보만 들어 있다 (Rabbit 최소 conf 정확히 0.3000).
  따라서 "가드를 낮추면 어떻게 되는가" 는 저장된 데이터로는 **원리적으로 측정할 수 없다**.
  여기서 YOLO 를 conf=0.02 (전 객체 동일) 로 다시 돌려 그 구간의 후보를 확보한다.

  가드(Bear .35 / Rabbit .30 / Dinosaur .30, 나머지 .02)는 원래 인형 간 cross-talk(FP)을
  막으려고 넣은 것인데, 사람 GT 검증에서 그 FP 는 HSV 게이트가 62% 제거했다.
  즉 가드의 존재 이유 자체를 재검토할 수 있게 됐다.

무엇이 운영과 같고 무엇이 다른가
  같음 : 프롬프트별 YOLO 패스, imgsz, MobileSAM, DINOv2 blocks, appe/sem 계산식,
         crop_resize_pad, masked_query_patches — 전부 운영 모듈을 import 해서 그대로 사용
  다름 : score_threshold 를 전 객체 0.02 로 통일, top_k 를 넉넉히(8) 저장
         → 오프라인에서 임의의 임계/top_k 를 재적용해 비교할 수 있다

운영 코드는 수정하지 않는다. 출력은 이 폴더에만 쓴다.

산출
  gt_analysis/noguard/<ds>_pairs.csv   박스×객체 (sem/appe/yolo_conf)
  gt_analysis/noguard/<ds>_boxes.csv   박스 메타
  gt_analysis/noguard/<ds>_hsv.npz     masked/bbox/background HSV 히스토그램
"""
import argparse, csv, os, sys, time
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RSRCH = os.path.dirname(ROOT)
REPO = os.path.dirname(RSRCH)
sys.path.insert(0, REPO)
import yolo_ism as yi                    # noqa: E402
import yolo_ism_object_n as o_n          # noqa: E402

GT = os.path.join(RSRCH, "gt_input")
OUT = os.path.join(HERE, "noguard")
os.makedirs(OUT, exist_ok=True)
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
COLOR_TOPIC = "/camera/camera/color/image_raw"
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
BLOCKS = [2, 9, 11]
UNIFORM_CONF = 0.02          # 전 객체 동일 (가드 제거)
KEEP_TOPK = 8                # 넉넉히 저장 — 오프라인에서 top_k 를 다시 자른다


def hists(bgr, mask):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m = (mask.astype(np.uint8) * 255) if mask is not None else None
    h1 = cv2.calcHist([hsv], [0], m, [32], [0, 180]).flatten()
    s1 = cv2.calcHist([hsv], [1], m, [32], [0, 256]).flatten()
    v1 = cv2.calcHist([hsv], [2], m, [32], [0, 256]).flatten()
    hs = cv2.calcHist([hsv], [0, 1], m, [16, 8], [0, 180, 0, 256]).flatten()
    out = []
    for a in (h1, s1, v1, hs):
        s = a.sum()
        out.append((a / s if s > 0 else a).astype(np.float32))
    return np.concatenate(out)


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--datasets", nargs="+", default=DATASETS)
    a = ap.parse_args()

    want = {}
    for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
        if r["user_reviewed"] == "yes":
            want.setdefault(r["dataset_name"], set()).add(int(r["frame_id"]))
    print("GT 프레임:", {k: len(v) for k, v in sorted(want.items())})

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    prompts, groups = o_n.build_prompt_groups(objs)
    img_sz = int(defaults.get("imgsz", 640))
    print(f"객체 {len(objs)} / 프롬프트 {len(prompts)} / imgsz {img_sz} / "
          f"통일 conf {UNIFORM_CONF} (원래 가드: "
          f"{ {o['name']: o.get('score_threshold') for o in objs if float(o.get('score_threshold',0))>0.02} })")

    for o in objs:
        o["_tcls"] = o["tcls"].to(device)
        o["_flat"] = {b: torch.cat(o["tappe_blocks"][b], 0).to(device) for b in BLOCKS}
        o["_seg"] = torch.cat([torch.full((t.shape[0],), i, dtype=torch.long)
                               for i, t in enumerate(o["tappe_blocks"][BLOCKS[0]])]).to(device)
        o["_nview"] = len(o["tappe_blocks"][BLOCKS[0]])

    from ultralytics import YOLOWorld
    yolos = []
    for p in prompts:
        y = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt")); y.set_classes([p])
        yolos.append(y)

    t0 = time.time()
    for ds in a.datasets:
        if ds not in want:
            continue
        brows, prows, H = [], [], []
        for fi, bgr in bag_frames(ds, want[ds]):
            h, w = bgr.shape[:2]
            norm = yi.normalize_rgb(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))

            pb = {}
            for pk, y in enumerate(yolos):
                r = y.predict(bgr, conf=UNIFORM_CONF, imgsz=img_sz, verbose=False, device=device)
                post = []
                if len(r) and r[0].boxes is not None and len(r[0].boxes):
                    b = r[0].boxes
                    for j in range(len(b)):
                        xy = b.xyxy[j].tolist()
                        x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                        x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                        post.append(((x1, y1, x2, y2), float(b.conf[j])))
                pb[pk] = sorted(post, key=lambda t: t[1], reverse=True)

            # 가드 없이 상위 KEEP_TOPK 만 남긴다
            cand_of = {}
            for pi, oh in groups.items():
                sel = pb[pi][:KEEP_TOPK]
                for o in oh:
                    for rank, (bx, sc) in enumerate(sel, start=1):
                        cand_of.setdefault(bx, {})[o["name"]] = (sc, rank)

            boxes = list(cand_of.keys())
            if not boxes:
                continue
            crops, keep = [], []
            for bx in boxes:
                c = yi.crop_resize_pad(norm, list(bx))
                if c is not None:
                    crops.append(c); keep.append(bx)
            if not crops:
                continue
            boxes = keep
            cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, device, BLOCKS)
            masks = yi.segment_boxes(seg, bgr, [list(b) for b in boxes], device)

            for bi, bx in enumerate(boxes):
                x1, y1, x2, y2 = bx
                uid = f"{ds}|{fi}|{x1}_{y1}_{x2}_{y2}"
                mask = masks[bi]
                sub = bgr[y1:y2, x1:x2]
                msub = mask[y1:y2, x1:x2] if mask is not None else None
                marea = int(msub.sum()) if msub is not None else 0
                barea = (x2-x1)*(y2-y1)
                H.append(np.concatenate([hists(sub, msub), hists(sub, None),
                                         hists(sub, ~msub if msub is not None else None)]))
                qfg = {}
                for b in BLOCKS:
                    qp = patch_all[b][bi]
                    qfg[b] = (yi.masked_query_patches(qp.cpu(), mask, list(bx), pool)[0].to(device)
                              if mask is not None else qp)
                brows.append({"uid": uid, "dataset": ds, "frame_id": fi,
                              "x1": x1, "y1": y1, "x2": x2, "y2": y2, "bbox_area": barea,
                              "mask_area": marea,
                              "mask_bbox_ratio": round(marea / max(1, barea), 4),
                              "n_valid_patch": int(qfg[11].shape[0]),
                              "candidate_for": ";".join(sorted(cand_of[bx].keys()))})

                for o in objs:
                    nm = o["name"]
                    sims = (o["_tcls"] @ cls_all[bi].to(device)).cpu()
                    order = torch.argsort(sims, descending=True)
                    ss = sims[order]; t1 = int(order[0])
                    ap = {}
                    for b in BLOCKS:
                        if qfg[b].shape[0] == 0:
                            ap[b] = (0.0, 0.0); continue
                        sim = qfg[b] @ o["_flat"][b].T
                        per = torch.full((qfg[b].shape[0], o["_nview"]), -1.0,
                                         device=device, dtype=sim.dtype)
                        per.scatter_reduce_(1,
                                            o["_seg"].unsqueeze(0).expand(qfg[b].shape[0], -1),
                                            sim, reduce="amax")
                        pv = per.mean(0).clamp(0, 1).cpu()
                        ap[b] = (float(pv[t1]), float(pv.max()))
                    cf = cand_of[bx].get(nm)
                    prows.append({"uid": uid, "dataset": ds, "frame_id": fi, "object": nm,
                                  "is_candidate": int(cf is not None),
                                  "yolo_conf": round(cf[0], 4) if cf else 0.0,
                                  "yolo_rank": cf[1] if cf else 0,
                                  "sem_top5": round(float(ss[:5].mean()), 4),
                                  "sem_mean": round(float(sims.mean()), 4),
                                  "appe11_clstop1": round(ap[11][0], 4),
                                  "appe11_max42": round(ap[11][1], 4),
                                  "sim_thr": o["similarity_threshold"],
                                  "appe_gate": o["appe_gate"],
                                  "orig_score_threshold": float(o.get("score_threshold", 0.02))})

        for name, rows in (("boxes", brows), ("pairs", prows)):
            with open(os.path.join(OUT, f"{ds}_{name}.csv"), "w", newline="") as f:
                wt = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                wt.writeheader(); wt.writerows(rows)
        np.savez_compressed(os.path.join(OUT, f"{ds}_hsv.npz"),
                            uid=np.array([r["uid"] for r in brows]),
                            hist=np.stack(H).astype(np.float32))
        print(f"  {ds}: 박스 {len(brows):,} / pair {len(prows):,}  ({time.time()-t0:.0f}s)")
    print(f"\n-> {OUT}")


if __name__ == "__main__":
    main()
