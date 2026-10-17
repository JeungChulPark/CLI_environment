#!/usr/bin/env python3
"""make_ycbv_scans.py — YCB-V 시험 영상 12개를 FM-Fusion 입력 형식(doc/DATA.md)으로 바꾼다.

  <out>/scans/<scene>/color/frame-XXXXXX.png   → rgb/<idx+1>.png 링크
                     /depth/frame-XXXXXX.png   → depth/<idx+1>.png 링크 (0.1 mm 단위 → config depth_scale 10000)
                     /pose/frame-XXXXXX.txt    카메라→월드 4x4 (Object Pose Map 과 같은 ORB-SLAM3 결과, SLAM 월드)
                     /prediction_gt/frame-XXXXXX_{label.json,mask.png}
                                               인식 정확 가정: 데이터셋 물체 영역(mask_visib)과 물체 이름, 점수 1.0
                                               (mask_visib 가 없는 데이터셋은 건너뜀. 비교에는 make_gt_box_sam.py 의 prediction_gtbox 를 쓴다)
BENCH_ROOT / BENCH_TABLE / BENCH_SLAM / BENCH_FPS 로 데이터셋을 바꾼다(build_memory_maps 와 같음).

FM-Fusion 은 자세가 없는 프레임을 건너뛴다(ORB-SLAM3 추적 실패 프레임).

    python3 objpose/tools/baselines/fmfusion/make_ycbv_scans.py --out ~/DeepLearning/Datasets/fmfusion_ycbv
"""
from __future__ import annotations

import argparse, json, os, sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM  # noqa: E402  (load_slam: ORB-SLAM3 프레임별 현재 위치)

MIN_PIX = 1   # 빈 영역만 뺀다. 작은 영역은 FM-Fusion 이 min_det_masks 로 거른다.


def label_of(o):
    return o["ycb"].split("_", 1)[1].replace("_", " ")          # 003_cracker_box → "cracker box"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--scenes", nargs="*", default=None)
    a = ap.parse_args()
    table = json.loads(BM.TABLE.read_text())["objects"]
    names = {o["obj_id"]: label_of(o) for o in table}
    scenes = a.scenes or sorted(p.name for p in BM.YCBV.iterdir() if p.is_dir())
    for scene in scenes:
        sc = BM.YCBV / scene
        dst = a.out / "scans" / scene
        for d in ("color", "depth", "pose", "prediction_gt"):
            (dst / d).mkdir(parents=True, exist_ok=True)
        gt = json.loads((sc / "scene_gt.json").read_text())
        n = len(gt)
        slam = BM.load_slam(scene, n)
        n_det = 0
        for i in range(n):
            f = f"frame-{i:06d}"
            for d, src in (("color", sc / "rgb" / f"{i + 1:06d}.png"), ("depth", sc / "depth" / f"{i + 1:06d}.png")):
                link = dst / d / f"{f}{src.resolve().suffix}"       # YCB-M 의 rgb/*.png 는 JPEG 링크 — Open3D 는 확장자로 읽는다
                if not link.exists():
                    os.symlink(src, link)
            if i in slam:
                np.savetxt(dst / "pose" / f"{f}.txt", slam[i], fmt="%.9f")
            if not (sc / "mask_visib").is_dir():          # YCB-M 등: 물체 마스크 없음 → make_gt_box_sam.py 로 만든다
                continue
            inst = np.zeros((480, 640), np.uint8)
            dets = [{"value": 0, "label": "background"}]
            for k, r in enumerate(gt[str(i + 1)]):
                m = cv2.imread(str(sc / "mask_visib" / f"{i + 1:06d}_{k:06d}.png"), cv2.IMREAD_GRAYSCALE) > 0
                if m.sum() < MIN_PIX:
                    continue
                v = len(dets)
                inst[m] = v
                ys, xs = np.nonzero(m)
                dets.append({"value": v, "labels": {names[r["obj_id"]]: 1.0},
                             "box": [float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())]})
            if len(dets) < 2:
                continue
            tags = ". ".join(sorted({next(iter(d["labels"])) for d in dets[1:]}))
            (dst / "prediction_gt" / f"{f}_label.json").write_text(json.dumps({"raw_tags": tags, "tags": tags, "mask": dets}))
            cv2.imwrite(str(dst / "prediction_gt" / f"{f}_mask.png"), inst)
            n_det += 1
        print(f"{scene}: {n} frames, {len(slam)} with SLAM pose, {n_det} with detections", flush=True)


if __name__ == "__main__":
    main()
