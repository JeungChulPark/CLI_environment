#!/usr/bin/env python3
"""run_ycbv_sam6d.py — YCB-V 장면을 개선된 SAM-6D 로 두 가지 입력 방식으로 돌린다.

SAM-6D 설정은 hub 가 BOP 레인에서 쓰는 것과 같다 (run_bop.sh → hub.start_sam6d):
run_split_example.yaml 의 verify 스택 + configs/yolo_ism_ycbv.yaml (HSV 게이트 off, per-box
owner election off) + 속도 프리셋 "fast"(fine_npoint 1024, mask cache) + 장면의 대상 객체 목록.
두 모드는 **입력 방식만** 다르다.

  all       모든 프레임을 순서대로 하나씩 넣는다. 프레임을 버리지 않는다(오프라인 상한).
  realtime  영상이 30 fps 로 벽시계에 맞춰 도착한다고 보고, 추론이 끝난 순간 이미 도착한
            프레임 가운데 **가장 최신 것 하나**를 다음 입력으로 넣는다. 그 사이 프레임은 버려진다.
            (실제 카메라에 붙였을 때와 같은 방식. hub 의 공유메모리 경로도 최신 프레임만 집는다.)

산출물은 eval_bop_run.py 가 그대로 읽는 형식이다:
  <out>/<scene>/sam6d_estimates.jsonl   추정 한 줄에 하나 (t_ns, object, score, R, t_mm, ms, ...)
  <out>/<scene>/sam6d_frames.jsonl      처리한 프레임 한 줄에 하나 (t_ns, idx, 도착·시작·완료 시각, dets)
  <out>/<scene>/run.json                설정, 처리 프레임 수, 소요 시간

    conda run -n sam6d python objpose/tools/ycbv_sam6d/run_ycbv_sam6d.py --mode realtime \
        --out objpose/output/ycbv_sam6d_realtime_260921
"""
from __future__ import annotations

import argparse
import copy
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[3]
SAM6D = REPO / "sam6d_realtime"
sys.path.insert(0, str(SAM6D / "realtime"))
sys.path.insert(0, str(REPO / "objpose" / "pc"))

import verify_config as VC                    # noqa: E402
from sam6d_core import Sam6DCore              # noqa: E402
from bop_session import BopSession            # noqa: E402

YCBV = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video/test")
TABLE = REPO / "objpose" / "rt" / "bop" / "ycbv_objects.json"
ISM_CONFIG = "configs/yolo_ism_ycbv.yaml"
# hub.SAM6D_SPEED_PRESETS["fast"] 와 같은 값 (hub 를 import 하면 서버 의존성이 딸려 온다)
FAST = {"fine_npoint": 1024, "mask_cache": {"enabled": True, "iou_thresh": 0.90, "max_age": 5, "interval": 30}}


def scene_targets(scene: Path) -> list[str]:
    gt = json.loads((scene / "scene_gt.json").read_text())
    name = {o["obj_id"]: o["name"] for o in json.loads(TABLE.read_text())["objects"]}
    return [name[i] for i in sorted({r["obj_id"] for v in gt.values() for r in v}) if i in name]


def build_core(objects: list[str]):
    cfg = yaml.safe_load((SAM6D / "realtime" / "run_split_example.yaml").read_text(encoding="utf-8"))
    rt = cfg["runtime"]
    rt.update(copy.deepcopy(FAST))
    core = Sam6DCore(ISM_CONFIG, objects, rt.get("device", "cuda:0"), rt.get("det_score_thresh", 0.2),
                     appe_rerank=rt.get("appe_rerank"), verify=rt.get("verify", VC.UNSET),
                     precision=rt.get("precision"), mask_cache=rt.get("mask_cache"),
                     pem_schedule=rt.get("pem_schedule"), coarse_npoint_override=rt.get("coarse_npoint"),
                     fine_npoint_override=rt.get("fine_npoint"))
    return core, {"ism_config": ISM_CONFIG, "objects": objects, "runtime": rt}


def run_scene(scene: Path, mode: str, out: Path, max_frames: int = 0):
    out.mkdir(parents=True, exist_ok=True)
    sess = BopSession(scene)
    n = len(sess) if max_frames <= 0 else min(len(sess), max_frames)
    K = sess.K
    objects = scene_targets(scene)
    core, used_cfg = build_core(objects)
    # 예열: 모델 적재 직후 첫 실제 프레임은 CUDA 커널 준비로 ~3 s 걸린다(내부 _warmup 은 빈 영상이라
    # PEM 까지 가지 않는다). 시계를 켜기 전에 한 번 돌리고 결과는 버린 뒤, 마스크 캐시를 비운다.
    w = sess.read(0, with_depth=True)
    for _ in range(2):
        core.process(w.color_bgr, w.depth_raw, K, want_mask=False)
    if getattr(core, "mask_cache", None) is not None:
        core.mask_cache.reset()
    f_est = open(out / "sam6d_estimates.jsonl", "w")
    f_frm = open(out / "sam6d_frames.jsonl", "w")
    n_proc = n_det = 0
    fps = sess.fps
    wall0 = time.monotonic()
    i = 0
    while i < n:
        if mode == "realtime":
            # 지금 시각까지 도착한 프레임 가운데 가장 최신 것
            i = max(i, int((time.monotonic() - wall0) * fps))
            if i >= n:
                break
            # 추론이 한 프레임 간격(33 ms)보다 빨리 끝났으면 다음 프레임은 아직 오지 않았다 — 도착까지 기다린다
            wait = i / fps - (time.monotonic() - wall0)
            if wait > 0:
                time.sleep(wait)
        t_start = time.monotonic() - wall0
        fr = sess.read(i, with_depth=True)
        rows, ms, n_boxes, _ = core.process(fr.color_bgr, fr.depth_raw, K, want_mask=False)
        t_done = time.monotonic() - wall0
        n_proc += 1
        n_det += len(rows)
        arrive = i / fps
        rec = {"t_ns": fr.t_ns, "idx": i, "arrive_s": round(arrive, 4), "start_s": round(t_start, 4),
               "done_s": round(t_done, 4), "latency_s": round(t_done - arrive, 4), "ms": ms, "n_boxes": n_boxes,
               "dets": [{k: r.get(k) for k in ("object", "score", "R", "t_mm")} for r in rows]}
        f_frm.write(json.dumps(rec, ensure_ascii=False) + "\n")
        for r in rows:
            f_est.write(json.dumps({"t_ns": fr.t_ns, "idx": i, "object": r["object"], "score": r["score"],
                                    "R": r["R"], "t_mm": r["t_mm"], "latency_s": rec["latency_s"], "ms": ms},
                                   ensure_ascii=False) + "\n")
        if n_proc % 50 == 0:
            f_est.flush(); f_frm.flush()
            print(f"[{scene.name} {mode}] frame {i + 1}/{n} processed {n_proc} dets {n_det} "
                  f"last {ms['total']} ms  elapsed {t_done:.0f}s", flush=True)
        i += 1
    el = time.monotonic() - wall0
    f_est.close(); f_frm.close()
    summ = {"scene": scene.name, "mode": mode, "frames": n, "processed": n_proc, "detections": n_det,
            "elapsed_s": round(el, 1), "video_s": round(n / fps, 1), "hz": round(n_proc / el, 2), "config": used_cfg}
    (out / "run.json").write_text(json.dumps(summ, indent=1, ensure_ascii=False))
    print(f"[{scene.name} {mode}] done: processed {n_proc}/{n} frames, {n_det} dets, {el:.0f}s", flush=True)
    del core
    import torch
    torch.cuda.empty_cache()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["all", "realtime"], required=True)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--scenes", nargs="*", default=None)
    ap.add_argument("--max-frames", type=int, default=0, help="smoke test: only the first N frames")
    a = ap.parse_args()
    scenes = a.scenes or sorted(p.name for p in YCBV.iterdir())
    for s in scenes:
        if (a.out / s / "run.json").exists():
            print(f"[{s}] already done, skip", flush=True)
            continue
        run_scene(YCBV / s, a.mode, a.out / s, a.max_frames)


if __name__ == "__main__":
    main()
