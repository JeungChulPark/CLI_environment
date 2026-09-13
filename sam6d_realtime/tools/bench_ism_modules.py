#!/usr/bin/env python3
"""bench_ism_modules.py — ISM 단계(87 ms)를 구성 요소로 쪼개 잰다.

PEM 을 최적화하고 나면 ISM 이 다음 병목이 된다. ISM 은 프레임마다
  MobileSAM  : 후보 박스 → 마스크
  DINOv2 ViT-S/14 @224 : 후보 크롭 → cls + patch 특징
  템플릿 매칭 / HSV 게이트 / NMS  (GPU 캐시 사용, 상대적으로 쌈)
를 돈다. 이 스크립트는 앞의 두 신경망만 잰다 — 나머지는 캐시 조회와 작은 행렬곱이다.

실시간 경로(`recognize_frame_auto` -> `assign_frame_relative`)는 이미 프레임당 1회로
배치화돼 있다 — `yolo_ism_object_n.py:579` 가 전 크롭을 한 번에 DINOv2 에 넣고,
`:580` 이 전 박스를 한 번에 MobileSAM 에 넣는다. 박스별 `segment_box()` 를 쓰는
`recognize_multi()` 는 실시간이 타지 않는 다른 경로다. 그래도 배치화의 실제 이득이
얼마인지(= 되돌리면 얼마나 손해인지)를 같이 재서 그 구조의 가치를 수치로 남긴다.

    conda activate sam6d
    python tools/bench_ism_modules.py --boxes 1 2 4 8 --iters 20
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch

REPO = Path(__file__).resolve().parents[1]
ISM_DIR = REPO / "sam6d_master" / "SAM-6D" / "Instance_Segmentation_Model"


def sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def timeit(fn, iters, warmup):
    for _ in range(warmup):
        fn()
    sync()
    ts = []
    for _ in range(iters):
        sync(); t0 = time.perf_counter()
        fn()
        sync(); ts.append((time.perf_counter() - t0) * 1000)
    a = np.asarray(ts)
    return dict(mean=float(a.mean()), p50=float(np.percentile(a, 50)),
                p90=float(np.percentile(a, 90)), max=float(a.max()))


def make_frame(h=480, w=640, seed=0):
    rng = np.random.default_rng(seed)
    # 균일 노이즈는 인코더 입장에서 실제 장면과 연산량이 같다(고정 형상 conv/attention).
    return rng.integers(0, 255, (h, w, 3), dtype=np.uint8)


def make_boxes(n, h=480, w=640, seed=1):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        bw, bh = rng.integers(60, 180), rng.integers(60, 220)
        x1 = int(rng.integers(0, max(1, w - bw))); y1 = int(rng.integers(0, max(1, h - bh)))
        out.append([x1, y1, int(x1 + bw), int(y1 + bh)])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--boxes", type=int, nargs="+", default=[1, 2, 4, 8])
    ap.add_argument("--iters", type=int, default=20)
    ap.add_argument("--warmup", type=int, default=6)
    ap.add_argument("--device", default="cuda:0")
    args = ap.parse_args()

    dev = args.device
    p = torch.cuda.get_device_properties(0)
    print("=" * 78)
    print(f"ISM module benchmark — {p.name}  ({p.multi_processor_count} SM)")
    print("=" * 78)

    frame = make_frame()

    # ---------------- MobileSAM ----------------
    from ultralytics import SAM
    seg = SAM(str(REPO / "mobile_sam.pt"))
    seg.to(dev)

    print("\n### MobileSAM — 박스별 호출(현재 경로) vs 프레임당 1회(미사용 코드) ###")
    print(f"{'boxes':>6} {'per-box (현재)':>16} {'batched (미사용)':>18} {'절감':>10}")
    print("-" * 56)
    sam_rows = []
    for n in args.boxes:
        boxes = make_boxes(n)

        def per_box():
            for b in boxes:
                seg(frame, bboxes=[b], verbose=False, device=0)

        def batched():
            seg(frame, bboxes=list(boxes), verbose=False, device=0)

        a = timeit(per_box, args.iters, args.warmup)
        b = timeit(batched, args.iters, args.warmup)
        save = 100.0 * (a["mean"] - b["mean"]) / a["mean"] if a["mean"] else 0
        print(f"{n:>6} {a['mean']:>13.2f} ms {b['mean']:>15.2f} ms {save:>9.1f}%")
        sam_rows.append((n, a, b))

    # ---------------- DINOv2 ViT-S/14 ----------------
    print("\n### DINOv2 ViT-S/14 @224 — 크롭별 호출 vs 배치 ###")
    sys.path.insert(0, str(REPO))
    import yolo_ism as yi
    import yolo_ism_object_n as o_n
    model = yi.build_dinov2(yi.DEFAULT_DINOV2_CKPT, dev)
    BLOCKS = [2, 9]          # 운영 config 의 appe 블록
    S = yi.PROPOSAL_SIZE
    print(f"{'crops':>6} {'per-crop':>14} {'batched':>14} {'절감':>10}")
    print("-" * 50)
    for n in args.boxes:
        crops = [torch.rand(3, S, S) for _ in range(n)]

        def per_crop():
            with torch.inference_mode():
                for c in crops:
                    o_n.dinov2_blocks_forward(model, [c], dev, BLOCKS)

        def batched():
            with torch.inference_mode():
                o_n.dinov2_blocks_forward(model, crops, dev, BLOCKS)

        a = timeit(per_crop, args.iters, args.warmup)
        try:
            b = timeit(batched, args.iters, args.warmup)
            save = 100.0 * (a["mean"] - b["mean"]) / a["mean"] if a["mean"] else 0
            print(f"{n:>6} {a['mean']:>11.2f} ms {b['mean']:>11.2f} ms {save:>9.1f}%")
        except Exception as e:
            print(f"{n:>6} {a['mean']:>11.2f} ms   batched 실패: {type(e).__name__}: {e}")

    # ---------------- 합산 전망 ----------------
    print("\n### 프레임 합산 (박스 N개, MobileSAM + DINOv2) ###")
    print(f"{'N':>3} {'현재(직렬)':>14} {'배치화 시':>14}")
    print("-" * 36)
    for n, a, b in sam_rows:
        print(f"{n:>3} {a['mean']:>11.2f} ms {b['mean']:>11.2f} ms   (MobileSAM 부분만)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
