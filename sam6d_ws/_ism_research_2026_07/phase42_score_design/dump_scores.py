#!/usr/bin/env python3
"""phase42 — 점수 설계 실험용 표. 셀마다 '모든 객체에 대한 세 점수'를 전부 기록한다.

이 표 하나로 두 제안을 실측 검증할 수 있다.
  (1) CAD 렌더 대신/더해 **실사 exemplar** 로 semantic 을 재면 나아지는가
      → 각 행의 crop CLS 벡터를 같이 저장해 두면, 세션을 하나씩 빼는 LOSO 로
        '다른 세션의 실사 crop' 은행을 만들어 오프라인에서 바로 채점할 수 있다.
  (2) 순차 게이트(sem→appe→HSV) 대신 **하나로 합친 점수**가 나은가
      → 정답 객체(양성)와 나머지 9개(음성)의 세 점수가 같은 행에 있으므로
        어떤 결합식이든 오프라인에서 평가된다.

대상 = phase41 과 같은 '확인된 가시 셀'(투영 화소의 depth 가 예상 z 와 ±0.30 m 일치)
중 **투영점을 덮는 YOLO 박스가 있는** 셀. 즉 제안까지는 성공한 셀만 본다 — 점수 설계가
바꿀 수 있는 범위가 거기이기 때문이다.
"""
import argparse, bisect, csv, glob, json, os, sys, time

import cv2, numpy as np, torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
ROOT = os.path.dirname(REPO)
sys.path.insert(0, os.path.join(RSRCH, "phase41_distance_check"))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism_object_n as o_n
import yolo_ism as yi
import ism_hsv
from measure_all import session_cells, OUTR, DATR, COLOR, DEPTH


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stride", type=int, default=8)
    ap.add_argument("--tol", type=float, default=0.30)
    ap.add_argument("--margin", type=int, default=16)
    ap.add_argument("--out", default=os.path.join(REPO, "outputs", "phase42_score_design"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    ext = json.load(open(f"{ROOT}/integration/cad_extents.json"))
    size_m = {k: max(v["size_mm"])/1000.0 for k, v in ext.items()}

    import copy
    defaults, cfgo = o_n.load_config(o_n.DEFAULT_CONFIG)
    dev = defaults.get("device", "cuda:0")
    dev = dev if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, dev)
    objs = o_n.prepare_objects(copy.deepcopy(cfgo), model, dev, False)
    names = [o["name"] for o in objs]
    prompts, _ = o_n.build_prompt_groups(objs)
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), dev)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(prompts)
    ms = min(float(o.get("score_threshold", 0.02)) for o in objs)
    sz = int(defaults.get("imgsz", 640))

    sessions = sorted(d for d in os.listdir(OUTR)
                      if os.path.isfile(f"{OUTR}/{d}/fused/orb3/objects.json")
                      and not d.endswith("_phase39"))
    fields = (["session", "frame", "object", "z_m", "px", "roi_luma", "roi_std", "omega"]
              + [f"sem_{n}" for n in names] + [f"appe_{n}" for n in names]
              + [f"hsv_{n}" for n in names])
    fh = open(f"{a.out}/scores.csv", "w", newline="")
    wr = csv.DictWriter(fh, fieldnames=fields); wr.writeheader()
    CLS = []

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    ts = get_typestore(Stores.ROS2_HUMBLE)

    for sess in sessions:
        cells, omf, fx, W, H = session_cells(sess, a.stride, size_m, a.tol)
        frames = sorted(cells)
        want, img, dep, need = set(frames), {}, {}, {}
        with AnyReader([Path(f"{DATR}/{sess}/sam")], default_typestore=ts) as rd:
            ccon = [c for c in rd.connections if c.topic == COLOR]
            dcon = [c for c in rd.connections if c.topic == DEPTH]
            cts = [t for _, t, _ in rd.messages(connections=ccon)]
            dts = [t for _, t, _ in rd.messages(connections=dcon)]
            for fi in frames:
                k = bisect.bisect_left(dts, cts[fi])
                cd = [x for x in (k-1, k) if 0 <= x < len(dts)]
                if cd:
                    need[fi] = min(cd, key=lambda x: abs(dts[x]-cts[fi]))
            i = -1
            for con, _t, raw in rd.messages(connections=ccon):
                i += 1
                if i in want:
                    m = rd.deserialize(raw, con.msgtype)
                    b = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                    img[i] = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()
            dw = set(need.values()); i = -1
            for con, _t, raw in rd.messages(connections=dcon):
                i += 1
                if i in dw:
                    m = rd.deserialize(raw, con.msgtype)
                    dep[i] = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
        t0 = time.time(); n_rows = 0
        for fi in frames:
            bgr = img.get(fi); d = dep.get(need.get(fi, -1))
            if bgr is None or d is None:
                continue
            h, w = bgr.shape[:2]
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            vis = []
            for name, iid, u, v, z in cells[fi]:
                uu, vv = int(round(u)), int(round(v))
                if not (0 <= uu < d.shape[1] and 0 <= vv < d.shape[0]):
                    continue
                win = d[max(0, vv-4):vv+5, max(0, uu-4):uu+5].astype(np.float32).ravel()
                win = win[win > 0]
                if win.size and abs(float(np.median(win))/1000.0 - z) <= a.tol:
                    vis.append((name, u, v, z))
            if not vis:
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            nf = yi.normalize_rgb(rgb)
            with o_n.multi_label_nms(True):
                pr = yolo.predict(bgr, conf=ms, imgsz=sz, verbose=False, device=dev)
            uniq = []
            if len(pr) and pr[0].boxes is not None and len(pr[0].boxes):
                b = pr[0].boxes
                for j in range(len(b)):
                    xy = b.xyxy[j].tolist()
                    x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                    x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                    cf = float(b.conf[j])
                    if cf < ms:
                        continue
                    for uq in uniq:
                        if o_n._iou_xyxy(uq["box"], [x1, y1, x2, y2]) > 0.95:
                            uq["conf"] = max(uq["conf"], cf); break
                    else:
                        uniq.append({"box": [x1, y1, x2, y2], "conf": cf})
            crops, keep = [], []
            for uq in uniq:
                c = yi.crop_resize_pad(nf, uq["box"])
                if c is not None:
                    crops.append(c); keep.append(uq)
            if not crops:
                continue
            nb = sorted({x for o in objs for x in o["_need_blocks"]})
            cls_all, patch_all = o_n.dinov2_blocks_forward(model, crops, dev, nb)
            masks = yi.segment_boxes(seg, bgr, [uq["box"] for uq in keep], dev)
            for name, u, v, z in vis:
                hits = [k for k, uq in enumerate(keep)
                        if uq["box"][0]-a.margin <= u <= uq["box"][2]+a.margin
                        and uq["box"][1]-a.margin <= v <= uq["box"][3]+a.margin]
                if not hits:
                    continue
                # 물체를 덮는 박스 중 가장 작은 것 = 그 물체를 가장 잘 감싼 것
                k = min(hits, key=lambda k: (keep[k]["box"][2]-keep[k]["box"][0]) *
                                            (keep[k]["box"][3]-keep[k]["box"][1]))
                mask = masks[k]
                if mask is None:
                    continue
                box = keep[k]["box"]
                row = {"session": sess, "frame": fi, "object": name, "z_m": round(z, 3),
                       "px": round(fx*size_m.get(name, .12)/max(z, 1e-3), 1),
                       "omega": round(omf.get(fi, 0.0), 1)}
                r0 = max(6, int(row["px"]/2))
                g = gray[max(0, int(v)-r0):int(v)+r0, max(0, int(u)-r0):int(u)+r0]
                row["roi_luma"] = round(float(g.mean()), 1) if g.size else 0
                row["roi_std"] = round(float(g.std()), 1) if g.size else 0
                for o in objs:
                    nm = o["name"]
                    row[f"sem_{nm}"] = round(float(yi.semantic_score(cls_all[k], o["tcls"],
                                                                    o["match_topk"])), 5)
                    qb = {x: patch_all[x][k].cpu() for x in o["_need_blocks"]}
                    bt = int(torch.argmax(o["tcls"] @ cls_all[k]))
                    row[f"appe_{nm}"] = round(float(o_n.masked_appe_blocks(qb, mask, box, pool, o, bt)), 5)
                    row[f"hsv_{nm}"] = round(float(ism_hsv.shadow_score(bgr, box, mask,
                                                                       o.get("_hsv_proto"))), 5)
                wr.writerow(row)
                CLS.append(cls_all[k].numpy().astype(np.float16))
                n_rows += 1
        print(f"[{sess}] 행 {n_rows} ({time.time()-t0:.0f}s)", flush=True)
    fh.close()
    np.save(f"{a.out}/cls.npy", np.array(CLS, dtype=np.float16))
    print(f"wrote {a.out}/scores.csv  ({len(CLS)} 행) + cls.npy")


if __name__ == "__main__":
    main()
