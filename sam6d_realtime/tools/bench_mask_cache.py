#!/usr/bin/env python3
"""bench_mask_cache.py — 마스크 재사용의 속도 이득과 정확도 비용을 같이 잰다.

재사용은 공짜가 아니다. 물체가 움직인 만큼 마스크가 어긋나고, 그 마스크가 그대로 PEM 의
입력 점군을 고른다. 그래서 "몇 ms 줄었나" 만 재면 안 되고 **"원래 마스크와 얼마나
다른가"** 를 같이 재야 한다.

방법
----
합성 프레임 위에서 박스를 프레임마다 `--speed` 픽셀씩 움직인다. 매 프레임:
  · 기준(ground truth) = 그 프레임에서 MobileSAM 을 실제로 돌린 마스크
  · 실제 = MaskCache 를 거친 마스크(재사용됐을 수도, 재계산됐을 수도)
둘의 IoU 를 기록한다. IoU 1.0 은 재계산된 프레임(또는 완전히 일치), 낮을수록 어긋난 것.

물체 속도를 바꿔 가며 돌리면 "얼마나 빨리 움직일 때까지 재사용이 안전한가" 가 나온다.

    conda activate sam6d
    python tools/bench_mask_cache.py --frames 40 --speed 0 4 12 30
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "realtime"))


def synth_frame(h, w, boxes, rng):
    """박스 자리에 밝은 사각형이 있는 프레임. MobileSAM 이 실제로 물체로 잡는다."""
    img = (rng.integers(0, 60, (h, w, 3), dtype=np.uint8))
    # 박스마다 모양을 다르게 준다(높이를 달리해 마스크가 서로 구별되게). 전부 같은
    # 사각형이면 서로의 마스크를 바꿔 받아도 IoU 가 1.0 으로 나와 버그가 숨는다.
    for k, (x1, y1, x2, y2) in enumerate(boxes):
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        shrink = int((y2 - y1) * 0.18 * (k % 3))
        y1s = min(y2 - 4, y1 + shrink)
        if x2 > x1 and y2 > y1s:
            img[y1s:y2, x1:x2] = rng.integers(170, 255,
                                              (y2 - y1s, x2 - x1, 3), dtype=np.uint8)
    return img


def iou_mask(a, b):
    if a is None or b is None:
        return float("nan")
    inter = np.logical_and(a, b).sum()
    uni = np.logical_or(a, b).sum()
    return float(inter / uni) if uni else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--speed", type=int, nargs="+", default=[0, 4, 12, 30],
                    help="프레임당 박스 이동 픽셀")
    ap.add_argument("--nbox", type=int, default=4)
    ap.add_argument("--iou-thresh", type=float, default=0.90)
    ap.add_argument("--max-age", type=int, default=5)
    ap.add_argument("--interval", type=int, default=0)
    ap.add_argument("--no-shift", action="store_true")
    args = ap.parse_args()

    from ultralytics import SAM
    import yolo_ism as yi
    from mask_cache import MaskCache

    dev = "cuda:0"
    seg = SAM(str(REPO / "mobile_sam.pt")); seg.to(dev)
    H, W = 480, 640
    p = torch.cuda.get_device_properties(0)
    print("=" * 86)
    print(f"mask cache — {p.name} · {args.nbox} boxes · {args.frames} frames "
          f"· iou>={args.iou_thresh} max_age={args.max_age} interval={args.interval or 'off'}"
          f" shift={not args.no_shift}")
    print("=" * 86)
    print(f"{'speed':>6} {'reuse%':>8} {'SAM calls':>10} {'ms/frame':>10} "
          f"{'vs always':>10} | {'IoU mean':>9} {'IoU p10':>8} {'IoU min':>8} {'IoU<0.9':>8}")
    print("-" * 86)

    for sp in args.speed:
        rng = np.random.default_rng(0)
        base = [[60 + i * 120, 90, 60 + i * 120 + 90, 90 + 140] for i in range(args.nbox)]
        cache = MaskCache(enabled=True, iou_thresh=args.iou_thresh,
                          max_age=args.max_age, interval=args.interval,
                          shift=not args.no_shift)

        ious, t_cached, t_always = [], [], []
        for f in range(args.frames):
            boxes = [[b[0] + sp * f, b[1], b[2] + sp * f, b[3]] for b in base]
            boxes = [b for b in boxes if b[0] < W - 10]
            if not boxes:
                break
            img = synth_frame(H, W, boxes, rng)

            torch.cuda.synchronize(); t0 = time.perf_counter()
            got = cache.segment_boxes(yi.segment_boxes, seg, img, boxes, dev)
            torch.cuda.synchronize(); t_cached.append((time.perf_counter() - t0) * 1000)

            torch.cuda.synchronize(); t0 = time.perf_counter()
            gt = yi.segment_boxes(seg, img, boxes, dev)
            torch.cuda.synchronize(); t_always.append((time.perf_counter() - t0) * 1000)

            for a, b in zip(got, gt):
                v = iou_mask(a, b)
                if not np.isnan(v):
                    ious.append(v)

        s = cache.stats
        reuse = 100.0 * s["reused"] / max(1, s["boxes"])
        ia = np.asarray(ious) if ious else np.array([np.nan])
        tc, ta = np.mean(t_cached), np.mean(t_always)
        print(f"{sp:>6} {reuse:>7.1f}% {s['sam_calls']:>10} {tc:>9.1f} "
              f"{100*(ta-tc)/ta:>9.1f}% | {np.nanmean(ia):>9.3f} "
              f"{np.nanpercentile(ia,10):>8.3f} {np.nanmin(ia):>8.3f} "
              f"{int((ia<0.9).sum()):>8}")

    print("\nspeed=0 은 정지 물체 — 재사용이 100% 이고 IoU 1.0 이어야 한다.")
    print("speed 가 커지면 IoU 문턱에 걸려 재계산이 늘고, 재사용률과 이득이 함께 준다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
