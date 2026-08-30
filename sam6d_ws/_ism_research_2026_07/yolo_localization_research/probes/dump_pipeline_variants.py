#!/usr/bin/env python3
"""dump_pipeline_variants.py — 후보 생성 방식을 바꿔 후단까지 전부 다시 계산한다 (READ-ONLY).

지금까지는 proposal 단계까지만 봤다. 여기서는 **semantic + appearance + HSV 까지 연결**해
실제 TP/FP/FN 을 낼 수 있도록 모든 후보 박스의 점수를 계산한다.

후보 집합 (박스 좌표는 이미 candidate_dumps 에 있다)
  cur          현행 프롬프트 10 pass
  yoloe_txt    YOLOE text prompt 1 pass
  union        위 둘의 합집합 (IoU 0.75 이상 중복 제거)

각 박스에 대해 계산 (운영 모듈 import, 수정 없음)
  MobileSAM mask → DINOv2 CLS/patch(block11) → 객체 10종 각각의
  sem_top5 / appe11_clstop1 / masked HSV 히스토그램

라우팅 정보도 함께 저장한다 (어느 프롬프트가 그 박스를 만들었는가) —
오프라인에서 '현행 라우팅' 과 '라우팅 제거' 를 모두 재현하기 위함이다.

산출: pipeline/<variant>/<ds>_pairs.csv, <ds>_hsv.npz
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
import yolo_ism as yi                    # noqa: E402
import yolo_ism_object_n as o_n          # noqa: E402

GT = os.path.join(RSRCH, "gt_input")
DUMP = os.path.join(ROOT, "candidate_dumps")
OUT = os.path.join(ROOT, "pipeline")
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
DATASETS = ["sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"]
CONF_MIN = 0.02
DEDUP_IOU = 0.75
MAX_BOX = 40            # 프레임당 상한 (conf 내림차순). 비용 폭주 방지


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    u = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / u if u > 0 else 0.0


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
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as rd:
        conns = [c for c in rd.connections if c.topic == "/camera/camera/color/image_raw"]
        i = -1
        for conn, t, raw in rd.messages(connections=conns):
            i += 1
            if i > hi:
                break
            if i not in want:
                continue
            m = rd.deserialize(raw, conn.msgtype)
            b = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            yield i, (cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8"
                      else b.copy())


def load_boxes(cfg, ds):
    p = os.path.join(DUMP, cfg, f"{ds}.csv")
    out = defaultdict(list)
    if not os.path.isfile(p):
        return out
    for r in csv.DictReader(open(p)):
        c = float(r["conf"])
        if c < CONF_MIN:
            continue
        out[int(r["frame_id"])].append(
            ((int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])), c, r["target"], cfg))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", nargs="+", default=["cur", "yoloe_txt", "union"])
    a = ap.parse_args()

    want = defaultdict(set)
    for r in csv.DictReader(open(os.path.join(GT, "user_visibility_gt.csv"), encoding="utf-8")):
        if r["user_reviewed"] == "yes":
            want[r["dataset_name"]].add(int(r["frame_id"]))

    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), device)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    for o in objs:
        o["_tcls"] = o["tcls"].to(device)
        o["_flat"] = torch.cat(o["tappe_blocks"][11], 0).to(device)
        o["_seg"] = torch.cat([torch.full((t.shape[0],), i, dtype=torch.long)
                               for i, t in enumerate(o["tappe_blocks"][11])]).to(device)
        o["_nview"] = len(o["tappe_blocks"][11])
    P2O = {o["yolo_prompt"]: o["name"] for o in objs}

    t0 = time.time()
    for var in a.variants:
        os.makedirs(os.path.join(OUT, var), exist_ok=True)
        for ds in DATASETS:
            if var == "union":
                A, B = load_boxes("cur", ds), load_boxes("yoloe_txt", ds)
                src = defaultdict(list)
                for fr in set(list(A) + list(B)):
                    src[fr] = A.get(fr, []) + B.get(fr, [])
            else:
                src = load_boxes(var, ds)

            prows, H, uids = [], [], []
            for fi, bgr in bag_frames(ds, want[ds]):
                bs = sorted(src.get(fi, []), key=lambda x: -x[1])
                # IoU 중복 제거 — 좌표가 거의 같으면 하나로 합치고 라우팅(target)은 모두 보존
                kept = []
                for b, c, tgt, cf in bs:
                    hit = None
                    for k in kept:
                        if iou(b, k["bbox"]) >= DEDUP_IOU:
                            hit = k; break
                    if hit:
                        hit["targets"].add(tgt); hit["conf"] = max(hit["conf"], c)
                        hit["srcs"].add(cf)
                    else:
                        kept.append({"bbox": b, "conf": c, "targets": {tgt}, "srcs": {cf}})
                kept = kept[:MAX_BOX]
                if not kept:
                    continue
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                norm = yi.normalize_rgb(rgb)
                crops, keep = [], []
                for k in kept:
                    c = yi.crop_resize_pad(norm, list(k["bbox"]))
                    if c is not None:
                        crops.append(c); keep.append(k)
                if not crops:
                    continue
                cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, device, [11])
                masks = yi.segment_boxes(seg, bgr, [list(k["bbox"]) for k in keep], device)

                for bi, k in enumerate(keep):
                    x1, y1, x2, y2 = k["bbox"]
                    uid = f"{ds}|{fi}|{x1}_{y1}_{x2}_{y2}"
                    mask = masks[bi]
                    sub = bgr[y1:y2, x1:x2]
                    msub = mask[y1:y2, x1:x2] if mask is not None else None
                    H.append(np.concatenate([hists(sub, msub), hists(sub, None)]))
                    uids.append(uid)
                    qp = patch_all[11][bi]
                    q = (yi.masked_query_patches(qp.cpu(), mask, list(k["bbox"]), pool)[0].to(device)
                         if mask is not None else qp)
                    for o in objs:
                        sims = (o["_tcls"] @ cls_all[bi].to(device)).cpu()
                        order = torch.argsort(sims, descending=True)
                        ss = sims[order]; t1 = int(order[0])
                        if q.shape[0] == 0:
                            ap11 = 0.0
                        else:
                            sim = q @ o["_flat"].T
                            per = torch.full((q.shape[0], o["_nview"]), -1.0,
                                             device=device, dtype=sim.dtype)
                            per.scatter_reduce_(1, o["_seg"].unsqueeze(0).expand(q.shape[0], -1),
                                                sim, reduce="amax")
                            ap11 = float(per.mean(0).clamp(0, 1)[t1])
                        prows.append({
                            "uid": uid, "dataset": ds, "frame_id": fi, "object": o["name"],
                            "routed": int(o["name"] in k["targets"]),
                            "yolo_conf": round(k["conf"], 4),
                            "sources": ";".join(sorted(k["srcs"])),
                            "sem_top5": round(float(ss[:5].mean()), 5),
                            "appe11_clstop1": round(ap11, 5),
                            "sim_thr": o["similarity_threshold"], "appe_gate": o["appe_gate"],
                            "score_threshold": float(o.get("score_threshold", 0.02))})
            with open(os.path.join(OUT, var, f"{ds}_pairs.csv"), "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(prows[0].keys()))
                w.writeheader(); w.writerows(prows)
            np.savez_compressed(os.path.join(OUT, var, f"{ds}_hsv.npz"),
                                uid=np.array(uids), hist=np.stack(H).astype(np.float32))
        n = sum(len(list(csv.DictReader(open(os.path.join(OUT, var, f"{d}_pairs.csv")))))
                for d in DATASETS)
        print(f"  {var:10s} pair {n:>8,}  ({time.time()-t0:.0f}s)")
    print(f"\n-> {OUT}")


if __name__ == "__main__":
    main()
