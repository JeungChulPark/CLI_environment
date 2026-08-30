#!/usr/bin/env python3
"""bench_patch_sim.py — cost of query<->template patch similarity at K=1..42.

Read-only: loads the EXISTING cached template appe features
(outputs/yolo_ism_object_n/template_features/<obj>_appe.pt) and times the
similarity math only (no DINOv2, no MobileSAM, no source modification).

Measures four variants, each on CPU (current production placement) and GPU:
  ragged-loop  : per-template python loop over [Np_i,384] tensors (current code)
  padded-batch : one [K*Np,384] matmul with a segment-max reduction (proposed)
Reported: median / p90 over N repeats after warm-up, with cuda synchronize.
"""
import argparse, json, os, statistics, sys, time
import torch

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))  # sam6d_ws
FEAT = os.path.join(REPO, "outputs", "yolo_ism_object_n", "template_features")


def load_obj(name):
    blob = torch.load(os.path.join(FEAT, f"{name}_appe.pt"), map_location="cpu")
    return blob["patch_features"]           # list of [Np_i, 384]


def masked_appe_ragged(q, tpl_list):
    """Exactly yolo_ism.masked_appe_score applied per template, max over templates."""
    best = 0.0
    for tp in tpl_list:
        sim = q @ tp.T
        best = max(best, float(sim.max(dim=1).values.mean().clamp(0, 1)))
    return best


def masked_appe_flat(q, flat, seg_id, n_seg):
    """One matmul against ALL templates concatenated, then segment reduce.

    flat   : [sum(Np_i), 384]   template patches of every view, concatenated
    seg_id : [sum(Np_i)]        which template each row belongs to
    """
    sim = q @ flat.T                                     # [Nq, Ntot]
    # per (query patch, template) max  ->  scatter_reduce amax
    out = torch.full((q.shape[0], n_seg), -1.0, device=q.device, dtype=q.dtype)
    out.scatter_reduce_(1, seg_id.expand(q.shape[0], -1), sim, reduce="amax")
    per_t = out.mean(dim=0)                              # [n_seg] mean over query fg patches
    return float(per_t.max().clamp(0, 1))


def timeit(fn, device, repeats, warmup=5):
    for _ in range(warmup):
        fn()
    if device == "cuda":
        torch.cuda.synchronize()
    ts = []
    for _ in range(repeats):
        if device == "cuda":
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        fn()
        if device == "cuda":
            torch.cuda.synchronize()
        ts.append((time.perf_counter() - t0) * 1000.0)
    ts.sort()
    return {"median_ms": statistics.median(ts),
            "p90_ms": ts[int(0.9 * (len(ts) - 1))],
            "mean_ms": statistics.fmean(ts)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", default="Bear")
    ap.add_argument("--nq", type=int, nargs="+", default=[64, 128, 256],
                    help="query foreground patch counts to test")
    ap.add_argument("--ks", type=int, nargs="+", default=[1, 3, 5, 10, 42])
    ap.add_argument("--repeats", type=int, default=50)
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results",
        "bench_patch_sim.json"))
    a = ap.parse_args()

    tpl = load_obj(a.object)
    npatch = [t.shape[0] for t in tpl]
    meta = {"object": a.object, "n_templates": len(tpl),
            "patches_per_template": {"min": min(npatch), "median": statistics.median(npatch),
                                     "max": max(npatch), "total": sum(npatch)},
            "dim": tpl[0].shape[1],
            "template_cache_MB_fp32": sum(npatch) * tpl[0].shape[1] * 4 / 1e6,
            "torch": torch.__version__,
            "gpu": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None}
    print(json.dumps(meta, indent=2))

    rows = []
    devices = ["cpu"] + (["cuda"] if torch.cuda.is_available() else [])
    for dev in devices:
        for nq in a.nq:
            q = torch.nn.functional.normalize(torch.randn(nq, 384), dim=-1).to(dev)
            for k in a.ks:
                sub = [t.to(dev) for t in tpl[:k]]
                flat = torch.cat(sub, 0)
                seg = torch.cat([torch.full((t.shape[0],), i, dtype=torch.long, device=dev)
                                 for i, t in enumerate(sub)]).unsqueeze(0)
                gflop = 2 * nq * flat.shape[0] * 384 / 1e9
                r1 = timeit(lambda: masked_appe_ragged(q, sub), dev, a.repeats)
                r2 = timeit(lambda: masked_appe_flat(q, flat, seg, k), dev, a.repeats)
                rows.append({"device": dev, "nq": nq, "K": k,
                             "template_patches": flat.shape[0], "GFLOP": round(gflop, 4),
                             "ragged_loop": {kk: round(v, 4) for kk, v in r1.items()},
                             "flat_matmul": {kk: round(v, 4) for kk, v in r2.items()}})
                print(f"{dev:4s} nq={nq:4d} K={k:3d} pat={flat.shape[0]:5d} "
                      f"{gflop:7.4f} GFLOP  ragged={r1['median_ms']:8.3f}ms  "
                      f"flat={r2['median_ms']:8.3f}ms")

    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump({"meta": meta, "rows": rows}, open(a.out, "w"), indent=2)
    print(f"\n-> {a.out}")


if __name__ == "__main__":
    main()
