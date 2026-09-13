#!/usr/bin/env python3
"""bench_pem_modules.py — PEM 내부를 모듈 단위로 재는 합성 벤치마크.

왜 합성인가
-----------
이 기계에는 CAD(.ply)가 없다(비공개). CAD 없이는 렌더 템플릿도 PEM 템플릿 캐시도
만들 수 없고, 따라서 실제 영상으로 PEM 을 돌릴 수 없다. 그러나 PEM 의 비용은
텐서 shape 에 지배되고 객체의 정체성과는 무관하다 — 같은 shape 이면 같은 커널이
같은 횟수로 돈다. 체크포인트도 필요 없다(랜덤 가중치도 FLOPs 동일).

그래서 실제 config(config/base.yaml)와 실제 입력 shape(test_dataset 절)으로
모델을 세우고, 합성 텐서를 흘려 **모듈별 지연을 이 GPU 에서 실측**한다.
잴 수 있는 것: 어디가 느린가, FP16 이 듣는가, 후보 수를 줄이면 얼마나 빨라지는가.
잴 수 없는 것: 정확도. 정확도는 CAD 가 생긴 뒤 실제 영상으로 따로 봐야 한다.

입력 shape (config/base.yaml)
-----------------------------
    img_size                 224
    n_sample_observed_point  2048   -> pts [B,2048,3], rgb_choose [B,2048]
    fine_npoint              2048   -> dense_po [B,2048,3], dense_fo [B,2048,256]
    coarse_npoint            196
    nproposal1 / nproposal2  6000 / 300

B 는 프레임 안의 검출 개수다. 실시간 노드는 프레임당 PEM 을 한 번만 부르고 검출을
배치로 묶으므로(sam6d_core.py), B=1..8 을 훑으면 "객체가 늘면 얼마나 느려지는가"가
바로 나온다.

사용
----
    conda activate sam6d
    python tools/bench_pem_modules.py --batch 1 2 4 8 --iters 30
    python tools/bench_pem_modules.py --batch 4 --precision fp32 fp16
    python tools/bench_pem_modules.py --batch 4 --sweep-nproposal2 300 128 64 32
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from contextlib import contextmanager
from pathlib import Path

import numpy as np
import torch

REPO = Path(__file__).resolve().parents[1]
PEM_DIR = REPO / "sam6d_master" / "SAM-6D" / "Pose_Estimation_Model"


def _add_paths():
    for sub in ("", "provider", "utils", "model", "model/pointnet2"):
        p = str(PEM_DIR / sub) if sub else str(PEM_DIR)
        if p not in sys.path:
            sys.path.insert(0, p)


@contextmanager
def cuda_timer(store, key):
    """CUDA 를 확실히 비우고 재는 타이머. 커널 큐가 남아 있으면 다음 단계에 비용이 섞인다."""
    torch.cuda.synchronize()
    t0 = time.perf_counter()
    yield
    torch.cuda.synchronize()
    store.setdefault(key, []).append((time.perf_counter() - t0) * 1000.0)


def build_model(cfg_overrides=None, device="cuda:0"):
    _add_paths()
    import gorilla
    import importlib
    cfg = gorilla.Config.fromfile(str(PEM_DIR / "config" / "base.yaml"))
    for k, v in (cfg_overrides or {}).items():
        # dotted key, e.g. "coarse_point_matching.nproposal2"
        node = cfg.model
        parts = k.split(".")
        for p in parts[:-1]:
            node = node[p]
        node[parts[-1]] = v
    MODEL = importlib.import_module("pose_estimation_model")
    net = MODEL.Net(cfg.model).to(device).eval()
    return net, cfg


def make_inputs(B, cfg, device="cuda:0", dtype=torch.float32):
    """실제 추론 경로(미리 구운 템플릿)와 같은 키 구성으로 합성 입력을 만든다."""
    td = cfg.test_dataset
    S = int(td.img_size)                     # 224
    N = int(cfg.model.fine_npoint)           # observed points must equal fine_npoint
    M = int(cfg.model.fine_npoint)           # 2048 (템플릿 점 수)
    g = torch.Generator(device="cpu").manual_seed(0)

    rgb = torch.randn(B, 3, S, S, generator=g).to(device=device, dtype=dtype)
    # rgb_choose 는 [0, S*S) 의 픽셀 인덱스. gather 의 메모리 패턴이 비용에 영향을
    # 주므로 무작위로 흩어 놓는다(실제 마스크 픽셀도 흩어져 있다).
    rgb_choose = torch.randint(0, S * S, (B, N), generator=g).to(device)
    # 관측 점군: 대략 30cm 앞의 10cm 크기 물체
    pts = (torch.randn(B, N, 3, generator=g) * 0.05).to(device=device, dtype=dtype)
    pts[..., 2] += 0.3
    dense_po = (torch.randn(B, M, 3, generator=g) * 0.05).to(device=device, dtype=dtype)
    dense_fo = torch.randn(B, M, 256, generator=g).to(device=device, dtype=dtype)

    return {
        "rgb": rgb,
        "rgb_choose": rgb_choose,
        "pts": pts,
        "dense_po": dense_po,
        "dense_fo": dense_fo,
        "score": torch.ones(B, device=device, dtype=dtype),
        "model": (torch.randn(B, int(td.n_sample_model_point), 3, generator=g) * 0.05
                  ).to(device=device, dtype=dtype),
        "K": torch.tensor([[572.4, 0, 325.3], [0, 573.6, 242.0], [0, 0, 1]],
                          dtype=dtype, device=device).expand(B, 3, 3).contiguous(),
    }


def instrument(net, store):
    """모듈별 forward 를 감싸 CUDA 동기 타이밍을 심는다. 원본 코드는 건드리지 않는다."""
    handles = []
    targets = {
        "feature_extraction (ViT-Base)": net.feature_extraction,
        "geo_embedding": net.geo_embedding,
        "coarse_point_matching": net.coarse_point_matching,
        "fine_point_matching": net.fine_point_matching,
    }
    state = {}

    def pre(name):
        def hook(mod, inp):
            torch.cuda.synchronize()
            state[name] = time.perf_counter()
        return hook

    def post(name):
        def hook(mod, inp, out):
            torch.cuda.synchronize()
            store.setdefault(name, []).append((time.perf_counter() - state[name]) * 1000.0)
        return hook

    for name, mod in targets.items():
        handles.append(mod.register_forward_pre_hook(pre(name)))
        handles.append(mod.register_forward_hook(post(name)))
    return handles


def run_once(net, inp, store, precision):
    with torch.inference_mode():
        if precision == "fp16":
            with torch.autocast(device_type="cuda", dtype=torch.float16):
                with cuda_timer(store, "TOTAL"):
                    net(dict(inp))
        elif precision == "bf16":
            with torch.autocast(device_type="cuda", dtype=torch.bfloat16):
                with cuda_timer(store, "TOTAL"):
                    net(dict(inp))
        else:
            with cuda_timer(store, "TOTAL"):
                net(dict(inp))


def summarise(store, iters):
    out = {}
    for k, v in store.items():
        a = np.asarray(v[-iters:], dtype=float)
        out[k] = dict(mean=float(a.mean()), p50=float(np.percentile(a, 50)),
                      p90=float(np.percentile(a, 90)), max=float(a.max()), n=len(a))
    return out


def gpu_info():
    p = torch.cuda.get_device_properties(0)
    return dict(name=p.name, sm=p.multi_processor_count,
                vram_gb=round(p.total_memory / 1e9, 1),
                cc=f"{p.major}.{p.minor}", torch=torch.__version__,
                cuda=torch.version.cuda)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--batch", type=int, nargs="+", default=[1, 2, 4, 8],
                    help="프레임당 검출 개수")
    ap.add_argument("--iters", type=int, default=30)
    ap.add_argument("--warmup", type=int, default=8)
    ap.add_argument("--precision", nargs="+", default=["fp32"],
                    choices=["fp32", "fp16", "bf16"])
    ap.add_argument("--sweep-nproposal2", type=int, nargs="*", default=None)
    ap.add_argument("--sweep-coarse-npoint", type=int, nargs="*", default=None)
    ap.add_argument("--sweep-fine-npoint", type=int, nargs="*", default=None,
                    help="fine_npoint. ViTEncoder asserts rgb_choose.size(1)==npoint, "
                         "so n_sample_observed_point moves with it.")
    ap.add_argument("--set", nargs="*", default=None, metavar="KEY=VAL",
                    help="임의 조합 실험. 예: --set coarse_npoint=96 fine_npoint=1024 "
                         "coarse_point_matching.nproposal2=64")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    assert torch.cuda.is_available(), "CUDA 없음"
    info = gpu_info()
    print("=" * 74)
    print(f"PEM module benchmark — {info['name']}  ({info['sm']} SM, {info['vram_gb']} GB, "
          f"cc {info['cc']})")
    print(f"torch {info['torch']} / cuda {info['cuda']}")
    print("=" * 74)

    configs = [({}, "baseline")]
    if args.sweep_nproposal2:
        configs = [({"coarse_point_matching.nproposal2": n}, f"nproposal2={n}")
                   for n in args.sweep_nproposal2]
    if args.sweep_coarse_npoint:
        configs = [({"coarse_npoint": n}, f"coarse_npoint={n}")
                   for n in args.sweep_coarse_npoint]
    if args.sweep_fine_npoint:
        configs = [({"fine_npoint": n}, f"fine_npoint={n}")
                   for n in args.sweep_fine_npoint]
    if args.set:
        ov = {}
        for kv in args.set:
            k, v = kv.split("=", 1)
            ov[k] = int(v) if v.lstrip("-").isdigit() else v
        configs = [(ov, "+".join(args.set))]

    results = {"gpu": info, "runs": []}
    for overrides, tag in configs:
        net, cfg = build_model(overrides)
        for prec in args.precision:
            for B in args.batch:
                inp = make_inputs(B, cfg)
                store = {}
                handles = instrument(net, store)
                try:
                    for _ in range(args.warmup):
                        run_once(net, inp, store, prec)
                    store.clear()
                    for _ in range(args.iters):
                        run_once(net, inp, store, prec)
                finally:
                    for h in handles:
                        h.remove()
                s = summarise(store, args.iters)
                mem = torch.cuda.max_memory_allocated() / 1e9
                torch.cuda.reset_peak_memory_stats()

                total = s["TOTAL"]["mean"]
                print(f"\n--- {tag} · {prec} · B={B} detections "
                      f"({args.iters} iters) ---")
                order = ["feature_extraction (ViT-Base)", "geo_embedding",
                         "coarse_point_matching", "fine_point_matching"]
                acc = 0.0
                for k in order:
                    if k not in s:
                        continue
                    m = s[k]["mean"]
                    acc += m
                    print(f"  {k:<32} {m:7.2f} ms  ({100*m/total:4.1f}%)"
                          f"   p90 {s[k]['p90']:6.2f}")
                print(f"  {'(unattributed)':<32} {total-acc:7.2f} ms  "
                      f"({100*(total-acc)/total:4.1f}%)")
                print(f"  {'TOTAL':<32} {total:7.2f} ms   p90 {s['TOTAL']['p90']:6.2f}"
                      f"   peak VRAM {mem:.2f} GB")
                print(f"  {'per detection':<32} {total/B:7.2f} ms")
                results["runs"].append(dict(tag=tag, precision=prec, batch=B,
                                            stages=s, peak_vram_gb=round(mem, 3)))
        del net
        torch.cuda.empty_cache()

    if args.out:
        Path(args.out).write_text(json.dumps(results, indent=1))
        print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
