#!/usr/bin/env python3
"""phase37 — 190624 (darkest session): where does ISM break, stage by stage?

Runs the DEPLOYED recognition path on the 190624 SAM bag but with every gate
BYPASSED FOR LOGGING: for each frame and each object it scores EVERY YOLO
candidate (not just the semantic winner) through all four stages and writes the
raw numbers, so the failure can be attributed to exactly one stage:

  stage 1  YOLO-World proposal      yolo_conf   (deploy: >= 0.02, top_k 3)
  stage 2  DINOv2 cls  (whole obj)  sem         (deploy: >= 0.35)
  stage 4  DINOv2 patch (masked)    masked_appe (deploy: >= 0.605, blocks [2,9])
  stage 3  HSV colour               hsv         (deploy: >= 0.1214)

Read-only w.r.t. the operational code: imports yolo_ism / yolo_ism_object_n and
calls their functions, changes nothing. Parity with production is kept by using
the same shared YOLO pass at imgsz=640 that tools/build_pem_inputs.py uses.
"""
import argparse, bisect, csv, glob, json, os, sys, time
import cv2, numpy as np, torch

WS = "/home/ldh9501/temp_ws/CLI_environment/sam6d_ws"
sys.path.insert(0, WS)
sys.path.insert(0, os.path.join(WS, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism as yi
import yolo_ism_object_n as o_n
import ism_hsv

COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--max-frames", type=int, default=100000)
    ap.add_argument("--cand-per-object", type=int, default=5)
    ap.add_argument("--yolo-conf", type=float, default=0.005)
    ap.add_argument("--imgsz", type=int, default=640)
    a = ap.parse_args()
    dev = "cuda:0" if torch.cuda.is_available() else "cpu"

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, dev)
    objs = o_n.prepare_objects(objs, model, dev, rebuild=False)
    segmentor = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), dev)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    prompts, groups = o_n.build_prompt_groups(objs)
    for o in objs:
        print(f"[obj] {o['name']:22s} prompt={o['yolo_prompt']!r} thr={o.get('score_threshold')} "
              f"sim={o['similarity_threshold']} appe_gate={o_n._appe_gate_of(o)} "
              f"blocks={o_n._blocks_of(o)} hsv={o.get('hsv_gate_threshold')}")

    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(prompts)

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    ts_store = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([Path(a.bag)], default_typestore=ts_store) as rd:
        ccon = [c for c in rd.connections if c.topic == COLOR]
        dcon = [c for c in rd.connections if c.topic == DEPTH]
        color_ts = [t for _, t, _ in rd.messages(connections=ccon)]
        depth_ts = [t for _, t, _ in rd.messages(connections=dcon)]
        sel = list(range(0, len(color_ts), a.stride))[:a.max_frames]
        sel_set = set(sel)
        need = {}
        for ci in sel:
            j = bisect.bisect_left(depth_ts, color_ts[ci])
            cd = [k for k in (j - 1, j) if 0 <= k < len(depth_ts)]
            need[ci] = min(cd, key=lambda k: abs(depth_ts[k] - color_ts[ci])) if cd else None
        need_d = {v for v in need.values() if v is not None}
        print(f"[bag] ncolor={len(color_ts)} selected={len(sel)}")
        color_img, i = {}, -1
        for con, _t, raw in rd.messages(connections=ccon):
            i += 1
            if i in sel_set:
                m = rd.deserialize(raw, con.msgtype)
                buf = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                color_img[i] = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else buf.copy()
        depth_img, i = {}, -1
        for con, _t, raw in rd.messages(connections=dcon):
            i += 1
            if i in need_d:
                m = rd.deserialize(raw, con.msgtype)
                depth_img[i] = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()

    fields = ["frame_idx", "t", "object", "cand_rank", "n_cand_prompt",
              "x1", "y1", "x2", "y2", "yolo_conf", "sem", "masked_appe", "rank_appe",
              "appe_last", "hsv", "mask_area", "is_prod_winner",
              "frame_luma_mean", "frame_luma_p05", "frame_luma_p95", "frame_lapvar",
              "roi_luma_mean", "roi_lapvar"]
    fh = open(a.out, "w", newline="")
    wr = csv.DictWriter(fh, fieldnames=fields); wr.writeheader()

    LAST = o_n.model_last_block(model)
    t0 = time.time()
    for n, ci in enumerate(sel):
        bgr = color_img[ci]; h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        fl = dict(frame_luma_mean=round(float(gray.mean()), 2),
                  frame_luma_p05=int(np.percentile(gray, 5)),
                  frame_luma_p95=int(np.percentile(gray, 95)),
                  frame_lapvar=round(float(cv2.Laplacian(gray, cv2.CV_64F).var()), 1))
        norm_full = yi.normalize_rgb(rgb)
        res = yolo.predict(bgr, conf=a.yolo_conf, imgsz=a.imgsz, verbose=False, device=dev)
        pb = {k: [] for k in range(len(prompts))}
        if len(res) and res[0].boxes is not None and len(res[0].boxes):
            b = res[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
                x2 = max(x1 + 1, min(int(xy[2]), w)); y2 = max(y1 + 1, min(int(xy[3]), h))
                k = int(b.cls[j]) if b.cls is not None else 0
                if k in pb:
                    pb[k].append(([x1, y1, x2, y2], float(b.conf[j])))

        entries = []          # (obj, box, conf, rank, n_cand, crop)
        for pi, here in groups.items():
            cand = sorted(pb.get(pi, []), key=lambda t: t[1], reverse=True)
            for o in here:
                for rank, (box, sc) in enumerate(cand[:a.cand_per_object], start=1):
                    crop = yi.crop_resize_pad(norm_full, box)
                    if crop is not None:
                        entries.append([o, box, sc, rank, len(cand), crop])
        if not entries:
            for o in objs:
                wr.writerow(dict(frame_idx=ci, t=round(color_ts[ci] / 1e9, 6), object=o["name"],
                                 cand_rank=0, n_cand_prompt=0, **fl))
            continue

        want = sorted({b for o in objs for b in o["_need_blocks"]})
        cls_all, patch_all = o_n.dinov2_blocks_forward(model, [e[5] for e in entries], dev, want)
        masks = yi.segment_boxes(segmentor, bgr, [e[1] for e in entries], dev)

        # production winner per object = argmax sem among candidates that pass
        # the deployed YOLO threshold + top_k slot
        prod = {}
        for idx, (o, box, sc, rank, ncand, _c) in enumerate(entries):
            if sc >= float(o.get("score_threshold", 0.0)) and rank <= int(o.get("top_k", 3)):
                s = float(yi.semantic_score(cls_all[idx], o["tcls"], o["match_topk"]))
                if o["name"] not in prod or s > prod[o["name"]][1]:
                    prod[o["name"]] = (idx, s)

        for idx, (o, box, sc, rank, ncand, _c) in enumerate(entries):
            sem = float(yi.semantic_score(cls_all[idx], o["tcls"], o["match_topk"]))
            mask = masks[idx]
            qb = {b: patch_all[b][idx].cpu() for b in o["_need_blocks"]}
            m_appe = r_appe = 0.0
            area = 0
            if mask is not None:
                x1, y1, x2, y2 = box
                area = int(mask[y1:y2, x1:x2].sum())
                bt = int(torch.argmax(o["tcls"] @ cls_all[idx]))
                m_appe = float(o_n.masked_appe_blocks(qb, mask, box, pool, o, bt))
                r_appe = m_appe if o["_rank_blocks"] == o["_blocks"] else \
                    float(o_n.masked_appe_blocks(qb, mask, box, pool, o, bt, o["_rank_blocks"]))
            hsv = float(ism_hsv.shadow_score(bgr, box, mask, o.get("_hsv_proto")))
            x1, y1, x2, y2 = box
            g = gray[y1:y2, x1:x2]
            wr.writerow(dict(
                frame_idx=ci, t=round(color_ts[ci] / 1e9, 6), object=o["name"],
                cand_rank=rank, n_cand_prompt=ncand, x1=x1, y1=y1, x2=x2, y2=y2,
                yolo_conf=round(sc, 5), sem=round(sem, 5), masked_appe=round(m_appe, 5),
                rank_appe=round(r_appe, 5),
                appe_last=round(float(yi.appearance_score(patch_all[LAST][idx].cpu(),
                                                          o["tappe"], o["match_topk"])), 5),
                hsv=round(hsv, 5), mask_area=area,
                is_prod_winner=int(prod.get(o["name"], (-1,))[0] == idx), **fl,
                roi_luma_mean=round(float(g.mean()), 2) if g.size else 0,
                roi_lapvar=round(float(cv2.Laplacian(g, cv2.CV_64F).var()), 1) if g.size > 16 else 0))
        if n % 25 == 0:
            el = time.time() - t0
            print(f"[{n+1}/{len(sel)}] frame {ci} entries={len(entries)} "
                  f"{el:.0f}s ({el/(n+1):.2f}s/frame)", flush=True)
    fh.close()
    print("wrote", a.out, f"{time.time()-t0:.0f}s")


if __name__ == "__main__":
    main()
