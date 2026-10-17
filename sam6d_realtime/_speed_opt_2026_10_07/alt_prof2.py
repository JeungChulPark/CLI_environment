import time, collections, atexit, importlib.util, torch
sp = importlib.util.spec_from_file_location("alt_select", __file__.replace("alt_prof2", "alt_select")); AS = importlib.util.module_from_spec(sp); sp.loader.exec_module(AS)
T = collections.defaultdict(float); N = collections.Counter(); inside = [0]; shapes = collections.Counter()
def install(mu, orig, timed):
    AS.install(mu, orig, timed)
    icv = mu.independent_candidate_verify
    def w(*a, **k):
        inside[0] += 1
        try: return icv(*a, **k)
        finally: inside[0] -= 1
    mu.independent_candidate_verify = w
    def wrap(name):
        f = getattr(torch, name)
        def g(*a, **k):
            if not inside[0]: return f(*a, **k)
            torch.cuda.synchronize(); t = time.perf_counter(); r = f(*a, **k); torch.cuda.synchronize()
            T[name] += time.perf_counter() - t; N[name] += 1
            if name == 'cdist': shapes[(tuple(a[0].shape), tuple(a[1].shape))] += 1
            return r
        setattr(torch, name, g)
    for n in ('cdist', 'einsum'): wrap(n)
    f_rle = mu._mask_rle
    def rle(*a, **k):
        t = time.perf_counter(); r = f_rle(*a, **k); T['_mask_rle'] += time.perf_counter() - t; N['_mask_rle'] += 1; return r
    mu._mask_rle = rle
    atexit.register(lambda: print('inside verify (18 timed frames + 6 warm-up):', {k: (round(v / 24 * 1e3, 1), N[k] // 24) for k, v in T.items()}, '| cdist shapes', shapes.most_common(4)))
