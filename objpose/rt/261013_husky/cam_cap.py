# PC: the SAM camera through the RealSense ROS driver (ROS_DOMAIN_ID 72) -> median of 45 depth frames + colour + intrinsics
import sys, time, os, numpy as np
sys.path.insert(0, "/home/jucpark/DeepLearning/CLI_environment_server/objpose/pc")
from live_session import LiveSamSession
s = LiveSamSession(timeout_s=25); deps = []; bgr = None; t0 = time.time()
while len(deps) < 45 and time.time() - t0 < 15:
    f = s.latest()
    if f is None: time.sleep(0.01); continue
    deps.append(np.asarray(f.depth_raw).copy()); bgr = np.asarray(f.color_bgr).copy()
D = np.stack(deps).astype(np.float32); D[D == 0] = np.nan
with np.errstate(all="ignore"):
    med = np.nanmedian(D, 0); cnt = np.isfinite(D).sum(0); sd = np.nanstd(D, 0)
med[(cnt < len(deps) * 0.7) | (sd > 0.02 * med + 5)] = 0
np.savez_compressed(sys.argv[1], depth_mm=np.nan_to_num(med).astype(np.uint16), bgr=bgr, K=s.K, D=np.array(s.kc), t_wall=time.time())
print("frames", len(deps), "valid px", round(float((med > 0).mean()), 3)); sys.stdout.flush(); os._exit(0)
