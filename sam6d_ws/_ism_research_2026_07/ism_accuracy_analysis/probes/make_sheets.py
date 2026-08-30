#!/usr/bin/env python3
"""make_sheets.py — 라벨링용 reference sheet + contact sheet 생성 (READ-ONLY).

reference sheet : 객체별 템플릿 렌더 3장 (라벨러가 대상 객체 외형을 확인)
contact sheet   : MobileSAM 까지 도달한 고유 박스 crop 을 격자로 배치, 각 칸에 인덱스
"""
import argparse, csv, glob, os, sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(ROOT))  # sam6d_ws
sys.path.insert(0, REPO)
import yolo_ism_object_n as o_n   # noqa: E402


def tile(img, size, label, sub=""):
    h, w = img.shape[:2]
    s = size / max(h, w)
    r = cv2.resize(img, (max(1, int(w*s)), max(1, int(h*s))), interpolation=cv2.INTER_CUBIC)
    canvas = np.full((size + 26, size, 3), 30, np.uint8)
    oy, ox = (size - r.shape[0]) // 2, (size - r.shape[1]) // 2
    canvas[oy:oy+r.shape[0], ox:ox+r.shape[1]] = r
    cv2.putText(canvas, label, (3, size + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    if sub:
        cv2.putText(canvas, sub, (3, size + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 220, 150), 1, cv2.LINE_AA)
    return canvas


def grid(tiles, cols):
    rows = []
    for i in range(0, len(tiles), cols):
        chunk = tiles[i:i+cols]
        while len(chunk) < cols:
            chunk.append(np.full_like(tiles[0], 30))
        rows.append(np.hstack(chunk))
    return np.vstack(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["reference", "contact"], required=True)
    ap.add_argument("--out-dir", default=os.path.join(ROOT, "datasets", "sheets"))
    ap.add_argument("--per-sheet", type=int, default=30)
    ap.add_argument("--cols", type=int, default=6)
    ap.add_argument("--size", type=int, default=150)
    ap.add_argument("--uid-list", default=os.path.join(ROOT, "datasets", "label_targets.csv"))
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)

    if a.mode == "reference":
        _, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
        tiles = []
        for o in objs:
            rgbs = sorted(glob.glob(os.path.join(o_n._abspath(o["template_dir"]), "rgb_*.png")))
            for k in (0, 14, 28):
                if k < len(rgbs):
                    tiles.append(tile(cv2.imread(rgbs[k]), a.size, o["name"][:20]))
        cv2.imwrite(os.path.join(a.out_dir, "reference.png"), grid(tiles, 6))
        print(f"reference: {len(tiles)} tiles -> {a.out_dir}/reference.png")
        return

    rows = list(csv.DictReader(open(a.uid_list)))
    boxes = {r["uid"]: r for r in csv.DictReader(open(os.path.join(ROOT, "features", "boxes.csv")))}
    tiles, idx_map = [], []
    for i, r in enumerate(rows):
        b = boxes[r["uid"]]
        img = cv2.imread(b["crop"])
        if img is None or img.size == 0:
            img = np.full((10, 10, 3), 60, np.uint8)
        tiles.append(tile(img, a.size, f"#{i}", r.get("claims", "")[:26]))
        idx_map.append({"idx": i, "uid": r["uid"], "source": b["source"], "frame": b["frame"],
                        "claims": r.get("claims", "")})
    for s in range(0, len(tiles), a.per_sheet):
        n = s // a.per_sheet
        cv2.imwrite(os.path.join(a.out_dir, f"contact_{n:02d}.png"),
                    grid(tiles[s:s+a.per_sheet], a.cols))
    with open(os.path.join(a.out_dir, "index_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(idx_map[0].keys())); w.writeheader(); w.writerows(idx_map)
    print(f"contact: {len(tiles)} tiles -> {(len(tiles)-1)//a.per_sheet+1} sheets")


if __name__ == "__main__":
    main()
