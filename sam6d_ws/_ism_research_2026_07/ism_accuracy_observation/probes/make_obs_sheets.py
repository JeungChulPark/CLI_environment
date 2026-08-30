#!/usr/bin/env python3
"""make_obs_sheets.py — 라벨링용 contact sheet 생성 (READ-ONLY).

mode=boxes  : 층화 추출한 박스 crop 격자 (박스 정체 라벨링용)
mode=frames : 전체 프레임 격자 (객체 가시성 라벨링용 → proposal miss 판정)
"""
import argparse, csv, glob, os, random, sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
COLOR = "/camera/camera/color/image_raw"


def tile(img, size, label, sub=""):
    h, w = img.shape[:2]
    s = size / max(h, w)
    r = cv2.resize(img, (max(1, int(w*s)), max(1, int(h*s))), interpolation=cv2.INTER_AREA)
    c = np.full((size + 26, size, 3), 30, np.uint8)
    oy, ox = (size - r.shape[0])//2, (size - r.shape[1])//2
    c[oy:oy+r.shape[0], ox:ox+r.shape[1]] = r
    cv2.putText(c, label, (3, size+12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    if sub:
        cv2.putText(c, sub, (3, size+23), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (150, 220, 150), 1, cv2.LINE_AA)
    return c


def grid(ts, cols):
    rows = []
    for i in range(0, len(ts), cols):
        ch = ts[i:i+cols]
        while len(ch) < cols:
            ch.append(np.full_like(ts[0], 30))
        rows.append(np.hstack(ch))
    return np.vstack(rows)


def load_frames(ds, want):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    out = {}
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == COLOR]
        i = -1
        for conn, t, raw in reader.messages(connections=conns):
            i += 1
            if i not in want:
                continue
            m = reader.deserialize(raw, conn.msgtype)
            b = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            out[i] = cv2.cvtColor(b, cv2.COLOR_RGB2BGR) if m.encoding.lower() == "rgb8" else b.copy()
            if len(out) == len(want):
                break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["boxes", "frames"], required=True)
    ap.add_argument("--per-dataset", type=int, default=60)
    ap.add_argument("--per-sheet", type=int, default=30)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--size", type=int, default=150)
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    random.seed(0)
    out = a.out or os.path.join(ROOT, "contact_sheets", a.mode)
    os.makedirs(out, exist_ok=True)

    if a.mode == "boxes":
        pairs = []
        for f in sorted(glob.glob(os.path.join(ROOT, "frame_dumps", "*_pairs.csv"))):
            pairs += list(csv.DictReader(open(f)))
        boxes = {}
        for f in sorted(glob.glob(os.path.join(ROOT, "frame_dumps", "*_boxes.csv"))):
            for r in csv.DictReader(open(f)):
                boxes[r["uid"]] = r
        cand = defaultdict(list)
        for r in pairs:
            if r["is_candidate"] == "1":
                cand[(r["dataset"], r["frame_id"], r["object"])].append(r)
        claims = defaultdict(set)
        for k, v in cand.items():
            b = max(v, key=lambda x: float(x["sem_top5"]))
            if float(b["sem_top5"]) < float(b["sim_thr"]):
                continue
            tag = "ACC" if float(b["appe11_clstop1"]) >= float(b["appe_gate"]) else "rej"
            claims[b["uid"]].add(f"{tag}:{b['object']}")
        # crop 파일이 있는 것만 (save-crop-every 로 저장된 프레임)
        cd = os.path.join(ROOT, "frame_dumps", "crops")
        avail = {u: c for u, c in claims.items()
                 if os.path.isfile(os.path.join(cd, u.replace("|", "__") + ".png"))}
        byds = defaultdict(list)
        for u, c in avail.items():
            byds[u.split("|")[0]].append((u, c))
        sel = []
        for ds, lst in sorted(byds.items()):
            acc = [x for x in lst if any(s.startswith("ACC") for s in x[1])]
            rej = [x for x in lst if not any(s.startswith("ACC") for s in x[1])]
            random.shuffle(acc); random.shuffle(rej)
            n_acc = min(len(acc), int(a.per_dataset * 0.7))
            sel += acc[:n_acc] + rej[:a.per_dataset - n_acc]
        sel.sort(key=lambda x: (x[0].split("|")[0], sorted(x[1])))
        ts, idx = [], []
        for i, (u, c) in enumerate(sel):
            img = cv2.imread(os.path.join(cd, u.replace("|", "__") + ".png"))
            if img is None or img.size == 0:
                img = np.full((10, 10, 3), 60, np.uint8)
            ts.append(tile(img, a.size, f"#{i}", "|".join(sorted(c))[:26]))
            idx.append({"idx": i, "uid": u, "dataset": u.split("|")[0],
                        "frame_id": u.split("|")[1], "claims": "|".join(sorted(c))})
        for s in range(0, len(ts), a.per_sheet):
            cv2.imwrite(os.path.join(out, f"boxes_{s//a.per_sheet:02d}.png"),
                        grid(ts[s:s+a.per_sheet], a.cols))
        with open(os.path.join(out, "index_map.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(idx[0].keys())); w.writeheader(); w.writerows(idx)
        print(f"boxes: {len(ts)} tiles -> {(len(ts)-1)//a.per_sheet+1} sheets in {out}")
        return

    # frames 모드: 데이터셋별 균등 간격 프레임
    import re
    dss = sorted({os.path.basename(f).split("_boxes")[0]
                  for f in glob.glob(os.path.join(ROOT, "frame_dumps", "*_boxes.csv"))})
    ts, idx = [], []
    n = 0
    for ds in dss:
        fr = sorted({int(r["frame_id"]) for r in csv.DictReader(
            open(os.path.join(ROOT, "frame_dumps", f"{ds}_yolo.csv")))})
        step = max(1, len(fr)//a.per_dataset)
        want = set(fr[::step][:a.per_dataset])
        imgs = load_frames(ds, want)
        for fi in sorted(imgs):
            ts.append(tile(imgs[fi], a.size, f"#{n}", f"{ds.replace('sam_','')} f{fi}"))
            idx.append({"idx": n, "dataset": ds, "frame_id": fi}); n += 1
    for s in range(0, len(ts), a.per_sheet):
        cv2.imwrite(os.path.join(out, f"frames_{s//a.per_sheet:02d}.png"),
                    grid(ts[s:s+a.per_sheet], a.cols))
    with open(os.path.join(out, "index_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(idx[0].keys())); w.writeheader(); w.writerows(idx)
    print(f"frames: {len(ts)} tiles -> {(len(ts)-1)//a.per_sheet+1} sheets in {out}")


if __name__ == "__main__":
    main()
