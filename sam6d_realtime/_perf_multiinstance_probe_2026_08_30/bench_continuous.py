#!/usr/bin/env python3
"""bench_continuous.py — SAM-6D 단일/다중 인스턴스 연속 추론 실측 (일회성 probe)

목적: 멀티 인스턴스 전략의 타당성 판단을 위한 1번 측정.
  1) bag 에서 프레임을 미리 메모리에 올려 두고 (디코딩 비용 배제)
  2) Sam6DCore.process() 를 쉼 없이 연속 호출하면서
  3) 단계별 ms (yolo/ism/pem) + GPU 사용률(50ms 샘플링) + VRAM 을 기록한다.

다중 인스턴스 스케일링 측정은 이 스크립트를 N개 프로세스로 동시에 띄우고
--barrier-dir 로 시작 시점을 정렬한다 (모델 로드+워밍업 후 전원 대기 → 동시 시작).

실행 (sam6d env):
  python _perf_multiinstance_probe_2026_08_30/bench_continuous.py \
      --config realtime/run_bag_260826_eightcircle_dark.yaml \
      --bag data/260826_eightcircle_dark --frames 60 --stride 15

원본 코드는 일절 수정하지 않는다. 이 폴더만 지우면 원상 복구.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics as st
import sys
import threading
import time
from pathlib import Path

import numpy as np

PROBE_DIR = Path(__file__).resolve().parent
REPO_DIR = PROBE_DIR.parent
sys.path.insert(0, str(REPO_DIR / "realtime"))


# ──────────────────────────────────────────────────────────────── bag 프레임 적재
def load_frames_from_bag(bag_path, topics, n_frames, stride, slop_ns=20_000_000):
    """RGB/depth 를 stamp 로 짝맞춰 stride 간격으로 n_frames 개 디코딩해 반환."""
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
                ConverterOptions("cdr", "cdr"))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_cls = {name: get_message(type_map[name]) for name in
               (topics["rgb"], topics["depth"], topics["caminfo"])
               if name in type_map}
    if len(msg_cls) < 3:
        raise SystemExit(f"[bench] bag 에 필요한 토픽이 없다: {list(type_map)}")

    def decode_image(msg):
        h, w, step = msg.height, msg.width, msg.step
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, step)
        if msg.encoding in ("rgb8", "bgr8"):
            img = buf[:, : w * 3].reshape(h, w, 3).copy()
            if msg.encoding == "bgr8":
                img = img[:, :, ::-1].copy()
            return img                                   # RGB 로 통일
        if msg.encoding in ("16UC1", "mono16"):
            dt = np.dtype(np.uint16).newbyteorder(">" if msg.is_bigendian else "<")
            return buf[:, : w * 2].view(dt).reshape(h, w).astype(np.uint16)
        raise SystemExit(f"[bench] 미지원 인코딩: {msg.encoding}")

    def stamp_ns(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    K = None
    buf = {"rgb": [], "depth": []}                       # (stamp, msg) 최근 8개씩
    frames, pair_idx = [], 0
    while reader.has_next() and len(frames) < n_frames:
        topic, raw, _ = reader.read_next()
        if topic == topics["caminfo"]:
            if K is None:
                m = deserialize_message(raw, msg_cls[topics["caminfo"]])
                K = np.array(m.k, dtype=np.float64).reshape(3, 3)
            continue
        key = ("rgb" if topic == topics["rgb"]
               else "depth" if topic == topics["depth"] else None)
        if key is None:
            continue
        m = deserialize_message(raw, msg_cls[topic])
        s = stamp_ns(m)
        other = buf["depth" if key == "rgb" else "rgb"]
        mate = min(other, key=lambda t: abs(t[0] - s)) if other else None
        if mate is not None and abs(mate[0] - s) <= slop_ns:
            other.remove(mate)
            rgb_m, dep_m = (m, mate[1]) if key == "rgb" else (mate[1], m)
            if pair_idx % stride == 0:
                frames.append({"stamp_ns": stamp_ns(rgb_m),
                               "rgb": decode_image(rgb_m),
                               "depth": decode_image(dep_m)})
            pair_idx += 1
        else:
            buf[key].append((s, m))
            buf[key] = buf[key][-8:]
    reader = None
    if not frames:
        raise SystemExit("[bench] 짝맞은 프레임이 없다")
    return frames, K


# ──────────────────────────────────────────────────────────────── GPU 샘플러
class GpuSampler(threading.Thread):
    def __init__(self, period_s=0.05):
        super().__init__(daemon=True)
        import pynvml
        self.nv = pynvml
        self.nv.nvmlInit()
        self.h = self.nv.nvmlDeviceGetHandleByIndex(0)
        self.period = period_s
        self.samples = []                                # (t, util%, mem_MiB)
        self._halt = threading.Event()

    def run(self):
        while not self._halt.is_set():
            u = self.nv.nvmlDeviceGetUtilizationRates(self.h)
            mem = self.nv.nvmlDeviceGetMemoryInfo(self.h)
            self.samples.append((time.monotonic(), u.gpu, mem.used // (1 << 20)))
            self._halt.wait(self.period)

    def stop(self):
        self._halt.set()
        self.join(timeout=2)

    def my_vram_mib(self):
        try:
            for p in self.nv.nvmlDeviceGetComputeRunningProcesses(self.h):
                if p.pid == os.getpid():
                    return p.usedGpuMemory // (1 << 20)
        except Exception:
            pass
        return None

    def window_stats(self, t0, t1):
        w = [s for s in self.samples if t0 <= s[0] <= t1]
        if not w:
            return {}
        util = sorted(s[1] for s in w)
        n = len(util)
        return {"n_samples": n,
                "util_mean": round(st.mean(util), 1),
                "util_p50": util[n // 2],
                "util_p90": util[int(0.9 * n)],
                "util_max": util[-1],
                "idle_ratio_lt50": round(sum(1 for u in util if u < 50) / n, 3),
                "mem_used_mib_max": max(s[2] for s in w)}


# ──────────────────────────────────────────────────────────────── main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="realtime/run_bag_260826_eightcircle_dark.yaml")
    ap.add_argument("--bag", default="data/260826_eightcircle_dark")
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--stride", type=int, default=15)
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--out", default=None, help="결과 JSON 경로")
    ap.add_argument("--tag", default="single")
    ap.add_argument("--barrier-dir", default=None)
    ap.add_argument("--barrier-count", type=int, default=1)
    ap.add_argument("--instance-id", type=int, default=0)
    a = ap.parse_args()

    import yaml
    cfg_path = a.config if os.path.isabs(a.config) else str(REPO_DIR / a.config)
    cfg = yaml.safe_load(open(cfg_path, encoding="utf-8")) or {}
    rt = cfg.get("runtime", {})
    topics = cfg.get("topics", {})
    bag_path = a.bag if os.path.isabs(a.bag) else str(REPO_DIR / a.bag)

    print(f"[bench:{a.instance_id}] bag 프레임 적재 중… ({a.frames}f, stride {a.stride})",
          flush=True)
    t = time.monotonic()
    frames, K = load_frames_from_bag(bag_path, topics, a.frames, a.stride)
    print(f"[bench:{a.instance_id}] {len(frames)} 프레임 적재 완료 "
          f"({time.monotonic()-t:.1f}s, {frames[0]['rgb'].shape})", flush=True)

    import cv2
    for f in frames:                                     # infer 프로세스와 동일 경로
        f["bgr"] = cv2.cvtColor(f["rgb"], cv2.COLOR_RGB2BGR)

    import verify_config as VC
    from sam6d_core import Sam6DCore
    t = time.monotonic()
    core = Sam6DCore(cfg.get("ism", {}).get("config", "configs/yolo_ism_objects.yaml"),
                     cfg.get("ism", {}).get("objects", []),
                     rt.get("device", "cuda:0"), rt.get("det_score_thresh", 0.2),
                     appe_rerank=rt.get("appe_rerank"),
                     verify=rt.get("verify", VC.UNSET))
    load_s = time.monotonic() - t
    print(f"[bench:{a.instance_id}] 모델 로드 {load_s:.1f}s", flush=True)

    sampler = GpuSampler()
    sampler.start()

    for f in frames[: a.warmup]:                         # 워밍업 (계측 제외)
        core.process(f["bgr"], f["depth"], K, want_mask=False)
    print(f"[bench:{a.instance_id}] 워밍업 {a.warmup} 프레임 완료", flush=True)

    if a.barrier_dir and a.barrier_count > 1:            # 다중 인스턴스 시작 정렬
        bdir = Path(a.barrier_dir)
        bdir.mkdir(parents=True, exist_ok=True)
        (bdir / f"{a.instance_id}.ready").write_text(str(time.time()))
        print(f"[bench:{a.instance_id}] barrier 대기 ({a.barrier_count}개)…", flush=True)
        while len(list(bdir.glob("*.ready"))) < a.barrier_count:
            time.sleep(0.05)

    per_frame = []
    t_loop0 = time.monotonic()
    for f in frames:
        t_a = time.perf_counter()
        rows, ms, n_boxes, _ = core.process(f["bgr"], f["depth"], K, want_mask=False)
        wall_ms = round(1e3 * (time.perf_counter() - t_a), 1)
        per_frame.append({"stamp_ns": f["stamp_ns"], "wall_ms": wall_ms, "ms": ms,
                          "n_boxes": n_boxes, "n_accept": len(rows)})
    t_loop1 = time.monotonic()
    my_vram = sampler.my_vram_mib()
    sampler.stop()

    el = t_loop1 - t_loop0
    fps = len(frames) / el

    def stage(key):
        v = sorted(pf["ms"][key] for pf in per_frame)
        n = len(v)
        return {"mean": round(st.mean(v), 1), "p50": v[n // 2],
                "p90": v[int(0.9 * n)], "max": v[-1]}

    gpu = sampler.window_stats(t_loop0, t_loop1)
    det_frames = sum(1 for pf in per_frame if pf["n_accept"] > 0)
    result = {
        "tag": a.tag, "instance_id": a.instance_id, "pid": os.getpid(),
        "config": cfg_path, "bag": bag_path,
        "n_frames": len(frames), "stride": a.stride,
        "elapsed_s": round(el, 2), "fps": round(fps, 2),
        "frames_with_detection": det_frames,
        "model_load_s": round(load_s, 1),
        "vram_this_process_mib": my_vram,
        "stages_ms": {k: stage(k) for k in ("yolo", "ism", "pem", "total")},
        "wall_ms_mean": round(st.mean(pf["wall_ms"] for pf in per_frame), 1),
        "gpu": gpu,
        "loop_window": [t_loop0, t_loop1],
        "per_frame": per_frame,
        "gpu_samples": [(round(s[0] - t_loop0, 3), s[1], s[2])
                        for s in sampler.samples],
    }
    out = a.out or str(PROBE_DIR / "outputs" / f"{a.tag}_i{a.instance_id}.json")
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    json.dump(result, open(out, "w"), indent=1)

    print(f"\n[bench:{a.instance_id}] ===== {a.tag} 결과 =====")
    print(f"  프레임 {len(frames)}개 / {el:.1f}s → {fps:.2f} FPS "
          f"(검출 있는 프레임 {det_frames}개)")
    for k in ("yolo", "ism", "pem", "total"):
        s_ = result["stages_ms"][k]
        print(f"  {k:>5}: mean {s_['mean']:6.1f}  p50 {s_['p50']:6.1f}  "
              f"p90 {s_['p90']:6.1f}  max {s_['max']:6.1f} ms")
    if gpu:
        print(f"  GPU util: mean {gpu['util_mean']}%  p50 {gpu['util_p50']}%  "
              f"p90 {gpu['util_p90']}%  (util<50% 비율 {gpu['idle_ratio_lt50']*100:.0f}%)")
    print(f"  VRAM(이 프로세스): {my_vram} MiB / 장치 전체 max {gpu.get('mem_used_mib_max')} MiB")
    print(f"  결과 저장: {out}", flush=True)


if __name__ == "__main__":
    main()
