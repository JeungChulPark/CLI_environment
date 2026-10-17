import sys, os, time, glob, json, collections, yaml, numpy as np, torch
REPO = "/home/jucpark/DeepLearning/CLI_environment_server/sam6d_realtime"
os.chdir(REPO); sys.path.insert(0, REPO + "/realtime")
from sam6d_core import Sam6DCore
import verify_config as VC
cfg = yaml.safe_load(open(sys.argv[1])); rt = cfg.get("runtime", {})
if os.environ.get("VERIFY_OVR"): rt["verify"] = dict(rt.get("verify") or {}, **json.loads(os.environ["VERIFY_OVR"])); print("verify override", os.environ["VERIFY_OVR"])
core = Sam6DCore(cfg.get("ism", {}).get("config", "configs/yolo_ism_objects.yaml"), cfg.get("ism", {}).get("objects", []),
                 rt.get("device", "cuda:0"), rt.get("det_score_thresh", 0.2), appe_rerank=rt.get("appe_rerank"),
                 verify=rt.get("verify", VC.UNSET), pem_diagnostic=rt.get("pem_diagnostic"), precision=rt.get("precision"),
                 mask_cache=None, pem_schedule=rt.get("pem_schedule"),
                 coarse_npoint_override=rt.get("coarse_npoint"), fine_npoint_override=rt.get("fine_npoint"))
mu = next(m for n, m in sys.modules.items() if n.endswith("model_utils") and hasattr(m, "sequential_candidate_select"))
T = collections.defaultdict(float); CNT = collections.Counter()
def timed(name):
    f = getattr(mu, name)
    def w(*a, **k):
        torch.cuda.synchronize(); t = time.perf_counter(); r = f(*a, **k); torch.cuda.synchronize()
        T[name] += time.perf_counter() - t; CNT[name] += 1; return r
    setattr(mu, name, w); return f
orig = {n: timed(n) for n in ("independent_candidate_verify", "_candidate_shape_metrics", "sequential_candidate_select")}
alt = sys.argv[3] if len(sys.argv) > 3 else ""
if alt:
    import importlib.util
    sp = importlib.util.spec_from_file_location("alt", alt); am = importlib.util.module_from_spec(sp); sp.loader.exec_module(am); am.install(mu, orig, timed)
frames = [np.load(p) for p in sorted(glob.glob(sys.argv[2] + "/*.npz"))]
out = []
for rep in range(4):
    for i, f in enumerate(frames):
        if rep == 1 and i == 0: T.clear(); CNT.clear(); MS = collections.defaultdict(list)      # rep 0 = warm-up
        rows, ms, nb, _ = core.process(f["bgr"], f["depth"], f["K"])
        if rep >= 1:
            for k, v in ms.items(): MS[k].append(v)
        if rep == 3: out.append([(r.get("name") or r.get("object"), [round(float(x), 2) for x in np.asarray(r.get("t_mm", r.get("t", [0,0,0]))).reshape(-1)[:3]], np.round(np.asarray(r.get("R", np.eye(3))), 4).tolist()) for r in rows])
nfr = len(frames) * 3
print("objects per frame:", [len(o) for o in out], [o[0] for o in out[0]] if out and out[0] else None)
print("per-frame ms:", {k: round(float(np.median(v)), 1) for k, v in MS.items()})
for k in T: print(f"  {k:32s} {T[k] / nfr * 1e3:7.1f} ms/frame  ({CNT[k] / nfr:.1f} calls/frame)")
json.dump(out, open(sys.argv[4] if len(sys.argv) > 4 else "/dev/null", "w"))
