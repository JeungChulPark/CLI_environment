#!/usr/bin/env python3
"""190624 의 모든 YOLO 박스에 대해 '주인 선출 결과 + 세 점수 + 투영 관계'를 전부 덤프한다.

이 표 하나로 세 가지를 오프라인에서 답할 수 있다.
  (1) semantic 컷 — 임계를 낮추면 무엇이 얼마나 돌아오고 오검출은 얼마나 느는가
  (2) choco 의 'B(박스가 물체를 안 덮음)' 294건이 진짜 제안 실패인가, GT 위치가 틀린 것인가
  (3) C(빼앗김) 과대 계상 — 한 박스가 두 물체를 함께 덮은 경우를 분리

박스 한 줄마다 기록한다:
  frame, box, yolo_conf, roi_luma
  winner            색 결선까지 마친 주인
  win_sem/appe/hsv  주인의 세 점수 (게이트 통과 여부와 무관하게 항상 계산)
  covers            이 박스가 덮는 (미가림·1.5m 이내) 객체 목록  ';' 구분
  covers_all        가림/거리 조건 없이 in-FOV 투영이 이 박스 안에 들어오는 객체 목록
  sem_<obj>         모든 객체의 semantic 점수 (임계 스윕용)
"""
import argparse, csv, os, sys, time
from collections import defaultdict

import cv2, numpy as np, torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
P37 = os.path.join(RSRCH, "phase37_lowlight_190624")
sys.path.insert(0, HERE); sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism_object_n as o_n
import yolo_ism as yi
import ism_hsv
from eval_190624 import load_clean_cells, BAG, COLOR


def all_projections(stride):
    """가림/거리 조건 없이 in-FOV 인 모든 투영점 {frame: {obj: (u,v,z)}}."""
    out = defaultdict(dict)
    for r in csv.DictReader(open(os.path.join(P37, "presence_geom.csv"))):
        fi = int(r["frame_idx"])
        if fi % stride or int(r["in_fov"]) != 1:
            continue
        out[fi][r["object"]] = (float(r["u"]), float(r["v"]), float(r["z_m"]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--margin", type=int, default=16)
    ap.add_argument("--out", default=os.path.join(REPO, "outputs", "phase39_multilabel_relative",
                                                  "csv", "boxes_190624.csv"))
    a = ap.parse_args()

    clean = load_clean_cells(a.stride)
    proj = all_projections(a.stride)
    frames = sorted(set(clean) | set(proj))
    print(f"[gt] 프레임 {len(frames)} · clean 셀 {sum(len(v) for v in clean.values())}")

    import copy
    defaults, cfg = o_n.load_config(o_n.DEFAULT_CONFIG)
    dev = defaults.get("device", "cuda:0")
    dev = dev if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, dev)
    objs = o_n.prepare_objects(copy.deepcopy(cfg), model, dev, False)
    names = [o["name"] for o in objs]
    n2o = {o["name"]: o for o in objs}
    prompts, _ = o_n.build_prompt_groups(objs)
    tsim = o_n.template_similarity(objs)
    tau = o_n._color_tiebreak_tau(objs[0])
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), dev)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(prompts)
    ms = min(float(o.get("score_threshold", 0.02)) for o in objs)
    sz = int(defaults.get("imgsz", 640))
    print(f"[cfg] imgsz {sz} tau {tau}")

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    ts = get_typestore(Stores.ROS2_HUMBLE)
    want, img, i = set(frames), {}, -1
    with AnyReader([Path(BAG)], default_typestore=ts) as rd:
        ccon = [c for c in rd.connections if c.topic == COLOR]
        for con, _t, raw in rd.messages(connections=ccon):
            i += 1
            if i in want:
                m = rd.deserialize(raw, con.msgtype)
                b = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                img[i] = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()

    fields = (["frame_idx", "x1", "y1", "x2", "y2", "yolo_conf", "roi_luma", "frame_luma",
               "winner", "win_sem", "win_appe", "win_hsv", "covers", "covers_all"]
              + [f"sem_{n}" for n in names])
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    fh = open(a.out, "w", newline=""); wr = csv.DictWriter(fh, fieldnames=fields); wr.writeheader()
    t0 = time.time()
    for n, fi in enumerate(frames, 1):
        bgr = img.get(fi)
        if bgr is None:
            continue
        h, w = bgr.shape[:2]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        fl = round(float(gray.mean()), 2)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        norm_full = yi.normalize_rgb(rgb)
        with o_n.multi_label_nms(o_n._multi_label_on(objs[0])):
            pr = yolo.predict(bgr, conf=ms, imgsz=sz, verbose=False, device=dev)
        uniq = []
        if len(pr) and pr[0].boxes is not None and len(pr[0].boxes):
            b = pr[0].boxes
            for j in range(len(b)):
                xy = b.xyxy[j].tolist()
                x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                conf = float(b.conf[j])
                if conf < ms:
                    continue
                for u in uniq:
                    if o_n._iou_xyxy(u["box"], [x1, y1, x2, y2]) > 0.95:
                        u["conf"] = max(u["conf"], conf); break
                else:
                    uniq.append({"box": [x1, y1, x2, y2], "conf": conf})
        crops, keep = [], []
        for u in uniq:
            c = yi.crop_resize_pad(norm_full, u["box"])
            if c is not None:
                crops.append(c); keep.append(u)
        if not crops:
            continue
        need = sorted({x for o in objs for x in o["_need_blocks"]})
        cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, dev, need)
        masks = yi.segment_boxes(seg, bgr, [u["box"] for u in keep], dev)
        cln = {c[0] for c in clean.get(fi, [])}
        for k, u in enumerate(keep):
            bx = u["box"]
            sems = {o["name"]: float(yi.semantic_score(cls_all[k], o["tcls"], o["match_topk"]))
                    for o in objs}
            mask = masks[k]
            hsv = ({o["name"]: float(ism_hsv.shadow_score(bgr, bx, mask, o.get("_hsv_proto")))
                    for o in objs} if mask is not None else {})
            top = max(sems, key=sems.get)
            if tau > 0 and hsv:
                tie = [x for x in sems if tsim[top].get(x, 0.0) >= tau]
                if len(tie) > 1:
                    top = max(tie, key=lambda x: hsv[x])
            o = n2o[top]
            appe = 0.0
            if mask is not None:
                qb = {x: patch_all[x][k].cpu() for x in o["_need_blocks"]}
                bt = int(torch.argmax(o["tcls"] @ cls_all[k]))
                appe = float(o_n.masked_appe_blocks(qb, mask, bx, pool, o, bt))
            inside = [nm for nm, (uu, vv, _z) in proj.get(fi, {}).items()
                      if bx[0]-a.margin <= uu <= bx[2]+a.margin and bx[1]-a.margin <= vv <= bx[3]+a.margin]
            g = gray[bx[1]:bx[3], bx[0]:bx[2]]
            wr.writerow(dict(frame_idx=fi, x1=bx[0], y1=bx[1], x2=bx[2], y2=bx[3],
                             yolo_conf=round(u["conf"], 5),
                             roi_luma=round(float(g.mean()), 2) if g.size else 0, frame_luma=fl,
                             winner=top, win_sem=round(sems[top], 5), win_appe=round(appe, 5),
                             win_hsv=round(hsv.get(top, -1.0), 5),
                             covers=";".join(sorted(x for x in inside if x in cln)),
                             covers_all=";".join(sorted(inside)),
                             **{f"sem_{nm}": round(sems[nm], 5) for nm in names}))
        if n % 100 == 0:
            print(f"  {n}/{len(frames)}  ({time.time()-t0:.0f}s)", flush=True)
    fh.close()
    print(f"wrote {a.out}  ({time.time()-t0:.0f}s)")


if __name__ == "__main__":
    main()
