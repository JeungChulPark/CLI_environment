#!/usr/bin/env python3
"""phase40 — 260804_office 는 되는데 0807/190624 는 왜 안 되는가. 원인 변수를 가른다.

두 세션을 **같은 잣대**로 잰다. 융합 객체맵을 SAM 카메라에 역투영해 '그 프레임에 그 물체가
어디 있는지'를 만들고, 셀마다 아래를 기록한다.

  z_m       카메라-물체 거리
  px        화면에서 물체가 차지할 세로 픽셀 = fx * CAD 최대변(m) / z  (기하 예측)
  roi_luma  투영점 주변 밝기
  roi_std   투영점 주변 대비(표준편차)
  omega     그 시각 카메라 각속도 [deg/s] (SLAM 궤적)
  occluded  투영점 측정 depth 가 예상보다 20cm 이상 앞
  proposed  YOLO 박스가 투영점을 덮었는가   <- ① 제안
  owned     최종 수락된 그 객체의 박스가 투영점을 덮었는가

한 변수를 맞춘 부분집합에서 proposed 비율이 같아지면 그 변수가 원인이다.
"""
import argparse, bisect, csv, glob, json, os, sqlite3, sys, time
from collections import defaultdict
from itertools import groupby

import cv2, numpy as np, torch

HERE = os.path.dirname(os.path.abspath(__file__))
RSRCH = os.path.dirname(HERE); REPO = os.path.dirname(RSRCH)
ROOT = os.path.dirname(REPO)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "sam6d_master", "SAM-6D", "Instance_Segmentation_Model"))
import yolo_ism_object_n as o_n
import yolo_ism as yi

DATASETS = {
    "190624": dict(out=f"{ROOT}/integration/output/0807_chungbuk/190624__irregular_multi_loop",
                   data=f"{ROOT}/integration/data/0807_chungbuk/190624__irregular_multi_loop",
                   stride=4),
    "260804": dict(out=f"{ROOT}/integration/output/260804_office/longcircle2",
                   data=f"{ROOT}/integration/data/260804_office/longcircle2",
                   stride=2),
}
COLOR = "/camera/camera/color/image_raw"
DEPTH = "/camera/camera/aligned_depth_to_color/image_raw"


def q2R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def build_presence(cfg, stride):
    tr = np.loadtxt(f"{cfg['out']}/orb3/trajectory.txt")
    tt = tr[:, 0]
    T = np.zeros((len(tr), 4, 4)); T[:, 3, 3] = 1
    for i, r in enumerate(tr):
        T[i, :3, :3] = q2R(r[4:8]); T[i, :3, 3] = r[1:4]
    om = np.zeros(len(tr))
    for i in range(1, len(tr)-1):
        dR = T[i-1, :3, :3].T @ T[i+1, :3, :3]
        c = np.clip((np.trace(dR)-1)/2, -1, 1)
        om[i] = np.degrees(np.arccos(c)) / max(tt[i+1]-tt[i-1], 1e-6)
    X = np.load(sorted(glob.glob(f"{cfg['data']}/calib/*.npy"))[0])
    info = json.load(open(f"{cfg['data']}/info.json"))
    K = np.array(info["sam"]["camera"]["K"]).reshape(3, 3)
    W, H = info["sam"]["camera"]["width"], info["sam"]["camera"]["height"]
    db = sorted(glob.glob(f"{cfg['data']}/sam/*.db3"))[0]
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True); cur = con.cursor()
    cur.execute("select id,name from topics"); tops = cur.fetchall()
    tid = [t for t, n in tops if n == COLOR][0]
    cur.execute("select timestamp from messages where topic_id=? order by timestamp", (tid,))
    cts = np.array([r[0] for r in cur.fetchall()])/1e9
    con.close()
    obj = [o for o in json.load(open(f"{cfg['out']}/fused/orb3/objects.json"))["objects"]
           if o["observations"] >= 5]
    keep = []
    for name, g in groupby(sorted(obj, key=lambda o: o["object_name"]),
                           key=lambda o: o["object_name"]):
        for o in sorted(g, key=lambda o: -o["observations"]):
            p = np.array(o["T_map_obj"])[:3, 3]
            if any(np.linalg.norm(p-np.array(k["T_map_obj"])[:3, 3]) < 0.4
                   for k in keep if k["object_name"] == name):
                continue
            keep.append(o)
    cells, omf = defaultdict(list), {}
    j = np.searchsorted(tt, cts)
    for fi, t in enumerate(cts):
        if fi % stride:
            continue
        k = j[fi]
        cand = [x for x in (k-1, k) if 0 <= x < len(tt)]
        if not cand:
            continue
        kk = min(cand, key=lambda x: abs(tt[x]-t))
        if abs(tt[kk]-t) > 0.10:
            continue
        omf[fi] = float(om[kk])
        Tcm = np.linalg.inv(T[kk] @ X)
        for o in keep:
            P = np.array(o["T_map_obj"])[:3, 3]
            p = Tcm[:3, :3] @ P + Tcm[:3, 3]
            if p[2] <= 0.05:
                continue
            u = K[0, 0]*p[0]/p[2] + K[0, 2]; v = K[1, 1]*p[1]/p[2] + K[1, 2]
            if 0 <= u < W and 0 <= v < H:
                cells[fi].append((o["object_name"], float(u), float(v), float(p[2])))
    return cells, omf, float(K[0, 0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(REPO, "outputs", "phase40_why_190624", "cells.csv"))
    ap.add_argument("--margin", type=int, default=16)
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)

    ext = json.load(open(f"{ROOT}/integration/cad_extents.json"))
    size_m = {k: max(v["size_mm"])/1000.0 for k, v in ext.items()}

    import copy
    defaults, cfgo = o_n.load_config(o_n.DEFAULT_CONFIG)
    dev = defaults.get("device", "cuda:0")
    dev = dev if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, dev)
    objs = o_n.prepare_objects(copy.deepcopy(cfgo), model, dev, False)
    prompts, groups = o_n.build_prompt_groups(objs)
    tsim = o_n.template_similarity(objs)
    seg = yi.build_segmentor(o_n._abspath(defaults.get("seg_weights", "mobile_sam.pt")), dev)
    pool = torch.nn.AvgPool2d(yi.PATCH, yi.PATCH)
    from ultralytics import YOLOWorld
    yolo = YOLOWorld(defaults.get("weights", "yolov8m-worldv2.pt"))
    yolo.set_classes(prompts)
    ms = min(float(o.get("score_threshold", 0.02)) for o in objs)
    sz = int(defaults.get("imgsz", 640))
    print(f"[cfg] imgsz {sz}")

    fields = ["dataset", "frame", "object", "u", "v", "z_m", "px", "roi_luma", "roi_std",
              "frame_luma", "omega", "occluded", "proposed", "owned"]
    fh = open(a.out, "w", newline=""); wr = csv.DictWriter(fh, fieldnames=fields); wr.writeheader()

    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    from pathlib import Path
    ts = get_typestore(Stores.ROS2_HUMBLE)

    for tag, cfg in DATASETS.items():
        cells, omf, fx = build_presence(cfg, cfg["stride"])
        frames = sorted(cells)
        print(f"[{tag}] 셀 {sum(len(v) for v in cells.values())} · 프레임 {len(frames)} · fx {fx:.1f}")
        want = set(frames)
        img, dep, need = {}, {}, {}
        with AnyReader([Path(f"{cfg['data']}/sam")], default_typestore=ts) as rd:
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
            dwant = set(need.values()); i = -1
            for con, _t, raw in rd.messages(connections=dcon):
                i += 1
                if i in dwant:
                    m = rd.deserialize(raw, con.msgtype)
                    dep[i] = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
        t0 = time.time()
        for n, fi in enumerate(frames, 1):
            bgr = img.get(fi)
            if bgr is None:
                continue
            h, w = bgr.shape[:2]
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            fl = round(float(gray.mean()), 2)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            nf = yi.normalize_rgb(rgb)
            d = dep.get(need.get(fi, -1))
            with o_n.multi_label_nms(o_n._multi_label_on(objs[0])):
                pr = yolo.predict(bgr, conf=ms, imgsz=sz, verbose=False, device=dev)
            boxes, pb = [], {k: [] for k in range(len(prompts))}
            if len(pr) and pr[0].boxes is not None and len(pr[0].boxes):
                b = pr[0].boxes
                for j in range(len(b)):
                    xy = b.xyxy[j].tolist()
                    x1 = max(0, min(int(xy[0]), w-1)); y1 = max(0, min(int(xy[1]), h-1))
                    x2 = max(x1+1, min(int(xy[2]), w)); y2 = max(y1+1, min(int(xy[3]), h))
                    ci = int(b.cls[j]) if b.cls is not None else 0
                    boxes.append([x1, y1, x2, y2])
                    if ci in pb:
                        pb[ci].append(([x1, y1, x2, y2], float(b.conf[j])))
            for k in pb:
                pb[k].sort(key=lambda z: -z[1])
            res = o_n.recognize_frame_auto(groups, pb, bgr, rgb, nf, model, dev, seg, pool, tsim)
            acc = {k: v["box"] for k, v in res.items() if v["accepted"] and v["box"]}
            for name, u, v, z in cells[fi]:
                px = fx * size_m.get(name, 0.12) / max(z, 1e-3)
                r = max(6, int(px/2))
                x1, y1 = max(0, int(u)-r), max(0, int(v)-r)
                x2, y2 = min(w, int(u)+r), min(h, int(v)+r)
                g = gray[y1:y2, x1:x2]
                occ = 0
                if d is not None and 0 <= int(v) < d.shape[0] and 0 <= int(u) < d.shape[1]:
                    win = d[max(0, int(v)-4):int(v)+5, max(0, int(u)-4):int(u)+5].astype(np.float32).ravel()
                    win = win[win > 0]
                    if win.size:
                        occ = int((np.median(win)/1000.0) < z - 0.20)
                prop = int(any(bx[0]-a.margin <= u <= bx[2]+a.margin
                               and bx[1]-a.margin <= v <= bx[3]+a.margin for bx in boxes))
                bx = acc.get(name)
                own = int(bx is not None and bx[0]-a.margin <= u <= bx[2]+a.margin
                          and bx[1]-a.margin <= v <= bx[3]+a.margin)
                wr.writerow(dict(dataset=tag, frame=fi, object=name, u=round(u, 1), v=round(v, 1),
                                 z_m=round(z, 3), px=round(px, 1),
                                 roi_luma=round(float(g.mean()), 1) if g.size else 0,
                                 roi_std=round(float(g.std()), 1) if g.size else 0,
                                 frame_luma=fl, omega=round(omf.get(fi, 0.0), 1),
                                 occluded=occ, proposed=prop, owned=own))
            if n % 200 == 0:
                print(f"  [{tag}] {n}/{len(frames)} ({time.time()-t0:.0f}s)", flush=True)
    fh.close()
    print("wrote", a.out)


if __name__ == "__main__":
    main()
