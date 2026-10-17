# laptop: its own D455f (PyPI pyrealsense2) -> median of 45 aligned depth frames + one colour frame + intrinsics
import sys, time, numpy as np, pyrealsense2 as rs
pipe = rs.pipeline(); cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30); cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
prof = pipe.start(cfg); align = rs.align(rs.stream.color)
for _ in range(30): pipe.wait_for_frames()
deps = []; bgr = None
while len(deps) < 45:
    f = align.process(pipe.wait_for_frames()); d, c = f.get_depth_frame(), f.get_color_frame()
    if not d or not c: continue
    deps.append(np.asanyarray(d.get_data()).copy()); bgr = np.asanyarray(c.get_data()).copy()
i = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics(); pipe.stop()
D = np.stack(deps).astype(np.float32); D[D == 0] = np.nan
with np.errstate(all="ignore"):
    med = np.nanmedian(D, 0); cnt = np.isfinite(D).sum(0); sd = np.nanstd(D, 0)
med[(cnt < len(deps) * 0.7) | (sd > 0.02 * med + 5)] = 0
K = np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1]])
np.savez_compressed(sys.argv[1], depth_mm=np.nan_to_num(med).astype(np.uint16), bgr=bgr, K=K, D=np.array(i.coeffs), t_wall=time.time())
print("frames", len(deps), "valid px", round(float((med > 0).mean()), 3))
