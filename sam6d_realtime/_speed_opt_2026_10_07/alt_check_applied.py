"""after the change is in model_utils.py: run the ORIGINAL function (text taken from the backup copy) next to the
in-file one on every call and compare the selected index and every row."""
import atexit, collections, os, torch
STATS = collections.Counter()
HERE = os.path.dirname(os.path.abspath(__file__))


def install(mu, orig, timed):
    src = open(os.path.join(HERE, "model_utils_before_261012.py")).read()
    body = src[src.index("def sequential_candidate_select("):src.index("def _count_modes(")]
    ns = dict(mu.__dict__); exec(compile(body, "reference_select", "exec"), ns); ref = ns["sequential_candidate_select"]
    new = orig["sequential_candidate_select"]
    assert ref.__code__.co_code != new.__code__.co_code, "the in-file function is still the original"
    fuzz(ref, new)
    def both(*a, **k):
        s0, r0 = ref(*a, **k); s1, r1 = new(*a, **k)
        STATS["calls"] += 1; STATS["same_selected"] += int(torch.equal(s0, s1)); STATS["same_rows"] += int(r0 == r1)
        STATS["objects"] += len(r0); STATS["accepted"] += sum(1 for r in r0 if r["accepted"]); STATS["candidate_rows"] += sum(len(r["candidates"]) for r in r0)
        for r in r0: STATS["reason:" + str(r["rejection_reason"])] += 1
        return s1, r1
    mu.sequential_candidate_select = both
    atexit.register(lambda: print("compare:", dict(STATS)))


def fuzz(ref, new, trials=400):
    """random inputs that reach every branch: invalid poses, NaN scores, no mask / texture survivor, one survivor,
    tight and scattered clusters, symmetric objects, shape None, explicit proposal ids"""
    import math
    g = torch.Generator(device="cuda").manual_seed(7); dev = "cuda"
    ver = {"mask_iou_min": 0.420998, "texture_min_score": 0.449562, "cluster_rotation_deg": 20.0, "cluster_translation_mm": 25.0,
           "cluster_min_occupancy": 0.5, "sym_step_deg": 10,
           "symmetry_axes": {"Sikhye_high": [0.0, 0.0, 1.0], "Mugcup_high": [0.1057, 0.0337, 0.9938]}}
    names_pool = ["Bear", "Sikhye_high", "Mugcup_high", None, "milk"]
    same = collections.Counter(); reasons = collections.Counter()
    def rot(n, spread):
        base = torch.linalg.qr(torch.randn(3, 3, device=dev, generator=g))[0]
        if torch.linalg.det(base) < 0: base[:, 0] = -base[:, 0]
        w = torch.randn(n, 3, device=dev, generator=g) * spread; th = w.norm(dim=1, keepdim=True).clamp_min(1e-9); k = w / th
        K = torch.zeros(n, 3, 3, device=dev); K[:, 0, 1], K[:, 0, 2], K[:, 1, 0], K[:, 1, 2], K[:, 2, 0], K[:, 2, 1] = -k[:, 2], k[:, 1], k[:, 2], -k[:, 0], -k[:, 1], k[:, 0]
        Rw = torch.eye(3, device=dev) + torch.sin(th)[..., None] * K + (1 - torch.cos(th))[..., None] * (K @ K)
        return base @ Rw
    for i in range(trials):
        B = int(torch.randint(1, 4, (1,), generator=g, device=dev)); P = [300, 300, 40, 7][i % 4]
        mode = i % 8
        R = torch.stack([rot(P, [0.05, 0.3, 1.5][(i + b) % 3]) for b in range(B)])
        t = torch.randn(B, P, 3, device=dev, generator=g) * [0.005, 0.02, 0.2][(i // 3) % 3] + torch.tensor([0.0, 0.0, 1.2], device=dev)
        geo = torch.rand(B, P, device=dev, generator=g)
        tex = torch.rand(B, P, device=dev, generator=g) * [1.0, 0.6, 0.44][mode % 3]
        miou = torch.rand(B, P, device=dev, generator=g) * [1.0, 0.6, 0.41][(mode // 3) % 3]
        pv = torch.rand(B, P, device=dev, generator=g) > [0.0, 0.2, 1.0][(i // 5) % 3] if mode != 7 else torch.zeros(B, P, dtype=torch.bool, device=dev)
        if mode == 1: R[:, ::7] = float("nan")
        if mode == 2: geo[:, ::5] = float("nan"); tex[:, 1::9] = float("nan"); miou[:, 2::11] = float("nan")
        if mode == 3: R[:, ::4] = R[:, ::4] * 1.3                      # not orthonormal
        if mode == 4:                                                  # exactly one survivor
            tex[:] = 0.1; miou[:] = 0.9; pv[:] = True; tex[:, 3] = 0.9
        if mode == 5: t[:, :, 0] = float("inf") * (torch.rand(B, P, device=dev, generator=g) > 0.9)
        shape = None if i % 13 == 0 else {"projection_valid": pv, "mask_iou": miou, "coverage": torch.rand(B, P, device=dev, generator=g),
                                           "size_ratio": torch.rand(B, P, device=dev, generator=g) * 2, "rendered_mask_area_px": torch.randint(0, 5000, (B, P), device=dev, generator=g).float()}
        if shape is not None and i % 17 == 0: shape.pop("coverage")
        names = [names_pool[(i + b) % len(names_pool)] for b in range(B)]
        pid = None if i % 2 else torch.stack([torch.randperm(6000, device=dev, generator=g)[:P] for _ in range(B)])
        s0, r0 = ref(R, t, geo, tex, shape, names, ver, pid); s1, r1 = new(R, t, geo, tex, shape, names, ver, pid)
        same["selected"] += int(torch.equal(s0, s1)); same["rows"] += int(r0 == r1); same["trials"] += 1
        for r in r0: reasons[str(r["rejection_reason"]) if not r["accepted"] else "accepted(cluster %s)" % ("1" if r["cluster_size"] == 1 else ">1")] += 1
    print("fuzz:", dict(same), "| branches reached:", dict(reasons))
