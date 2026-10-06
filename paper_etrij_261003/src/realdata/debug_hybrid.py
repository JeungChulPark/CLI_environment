import os, sys, json, yaml, numpy as np
sys.path.insert(0, os.path.expanduser("~/DeepLearning/CLI_environment/objpose/pc"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from conv_session import ConvSession
S = ConvSession(os.path.expanduser("~/DeepLearning/Dataset/260901_cbnu_bigeightcircle/SAM"))
t = 1788249666315801748
i = int(np.argmin(np.abs(np.asarray(S.t_ns if hasattr(S, "t_ns") else [S.read(k, False).t_ns for k in range(len(S))]) - t)))
fr = S.read(i, True); bgr = fr.color_bgr; dep = S.align(fr.depth_raw).astype(np.float32); K = S.K
print("frame", i, bgr.shape, dep.shape, dep.dtype, float(np.median(dep[dep > 0])), K.tolist())
cfg = yaml.safe_load(open(os.path.expanduser("~/DeepLearning/CLI_environment/objpose/output/real_260901_hybS05/sam6d_config.yaml")))
os.environ.update(OBJPOSE_LIVE_MODE=sys.argv[1], OBJPOSE_ORIG_DESC="dinov2_vitl14", OBJPOSE_ORIG_THRESH="0.2")
import live_cores
c = live_cores.LiveCore(cfg)
rows, ms, nb, _ = c.process(bgr, dep, K)
print("rows", [(r["object"], r["score"], r.get("t_mm")) for r in rows])
d = c.last_frame_diag
print("cands", [(x["object"], x.get("mask_px"), x.get("valid_depth_px"), x.get("radius_inliers"), x.get("input_rejection"), x.get("rejection_reason"), x.get("mask_iou"), x.get("texture_score")) for x in d.get("pem_candidates", [])])
print("rej", d.get("rejections"), d.get("pem_error"))
