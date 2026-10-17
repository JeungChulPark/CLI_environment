#!/usr/bin/env python3
"""prewarm_ycbv_ism.py — build the ISM template feature caches for an object set offline.

`o_n.prepare_objects` builds `<name>_cls.pt` / `<name>_appe*.pt` from the 42 renders the first
time an object is used. For 21 new objects that is minutes of DINOv2 inside the hub's startup,
which would stall a timed run, so it is done here instead.

    python objpose/tools/prewarm_ycbv_ism.py [--config configs/yolo_ism_ycbv.yaml] [--rebuild]
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SAM6D = REPO / "sam6d_realtime"
sys.path.insert(0, str(SAM6D))
os.chdir(SAM6D)

import torch                                     # noqa: E402
import yolo_ism as yi                            # noqa: E402
import yolo_ism_object_n as o_n                  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/yolo_ism_ycbv.yaml")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--rebuild", action="store_true")
    a = ap.parse_args()

    defaults, objs = o_n.load_config(str(SAM6D / a.config) if not os.path.isabs(a.config) else a.config)
    print(f"[prewarm] {len(objs)} objects from {a.config}")
    device = torch.device(a.device)
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    t0 = time.time()
    ready = o_n.prepare_objects(objs, model, device, rebuild=a.rebuild)
    print(f"[prewarm] {len(ready)}/{len(objs)} objects ready in {time.time() - t0:.1f} s")
    missing = sorted({o["name"] for o in objs} - {o["name"] for o in ready})
    if missing:
        print(f"[prewarm] NOT ready: {missing}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
