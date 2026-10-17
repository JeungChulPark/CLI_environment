#!/usr/bin/env python3
"""eval_lmo_drift.py — controlled test on LM-O (scene 000002): the ground-truth camera trajectory with an artificial,
realistic SLAM drift added, corrected by the object anchor (SAM-6D results, optionally depth ICP).

LM-O cannot run a real SLAM: its 1214 frames are snapshots (median 11 cm / 9 deg between frames), ORB-SLAM3 loses
tracking. So:
  camera GT   frame-0 camera = world; frame i from the best-visible static object: T_w_ci = T_c0_o . T_ci_o^-1
  "SLAM"      T_slam_i = D_i . T_gt_i, D a drift that grows with time to about 6 cm / 1.5 deg at the end (seeded),
              the size ORB-SLAM3 drifted on YCB-V
  SAM-6D      the pipeline's per-image results for the 200 BOP images (verified poses only), arriving at done_s,
              frame index = video frame at 30 fps
  depth ICP   every 3rd frame (10 Hz), 50 ms after the frame
Scores: camera position error vs GT (frame 0 shared), RMSE / end / p90.
    python eval_lmo_drift.py [--icp-hz 10] [--seed 0] --tag T
"""
import argparse, json, sys, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/pc"))
from obj_anchor import ObjectAnchor, AnchorConfig, inv            # noqa: E402
from anchor_memory import WorldPoseMemory, KEEP                   # noqa: E402

LMO = Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/bop/lmo/test/000002")
RUN = REPO / "objpose/output/cmp_260924_gt/proposedscore_lmo_bop19/000002/sam6d_frames.jsonl"
PTS = REPO / "sam6d_realtime/assets/model_points"
NAMES = {1: "lmo_ape", 5: "lmo_can", 6: "lmo_cat", 8: "lmo_driller", 9: "lmo_duck", 10: "lmo_eggbox", 11: "lmo_glue", 12: "lmo_holepuncher"}
FPS, T0 = 30.0, 1_600_000_000_000_000_000
STEP = int(round(1e9 / FPS))


def T_of(r):
    M = np.eye(4); M[:3, :3] = np.reshape(r["cam_R_m2c"], (3, 3)); M[:3, 3] = np.asarray(r["cam_t_m2c"]) / 1000.0
    return M


def kabsch(A, B):
    """T with B ~ R A + t"""
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    R = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = cb - R @ ca
    return T


def gt_cameras(a, b, static):
    """camera of frames a..b-1 in the frame-a camera, fitted to the centres of the static objects (Kabsch, the
    object layout re-estimated as the median 3x). LM-O is not a static scene: objects are moved during the capture
    (the layout changes completely at frames 396, 703, 1005; ape / duck / holepuncher move in between), so only a
    segment and the objects that stay put there are usable. -> ids, cams, gt, per-object layout residual [m]"""
    gt = json.loads((LMO / "scene_gt.json").read_text())
    ids = [str(k) for k in range(a, b)]
    F = [{r["obj_id"]: T_of(r) for r in gt[k] if r["obj_id"] in static} for k in ids]
    lay = {o: T[:3, 3] for o, T in F[0].items()}
    for _ in range(3):
        cams = []
        for rs in F:
            os_ = [o for o in rs if o in lay]
            cams.append(kabsch(np.array([rs[o][:3, 3] for o in os_]), np.array([lay[o] for o in os_])))
        W = {}
        for c, rs in zip(cams, F):
            for o, T in rs.items():
                W.setdefault(o, []).append((c @ T)[:3, 3])
        lay = {o: np.median(v, 0) for o, v in W.items()}
    c0 = inv(cams[0]); cams = [c0 @ c for c in cams]
    resid = {NAMES[o]: float(np.median(np.linalg.norm(np.array(v) - lay[o], axis=1))) for o, v in W.items()}
    return ids, np.array(cams), gt, resid


def drift(cams, seed, end_m=0.06, end_deg=1.5):
    """drift that grows with TIME (LM-O frames jump ~11 cm apart, so a path-length drift would be metres):
    each frame adds end_m / n of translation along a slowly turning direction and end_deg / n of rotation about a
    slowly turning axis (about the current camera), i.e. about end_m / end_deg at the end — the size ORB-SLAM3
    drifted on YCB-V (25-92 mm end error)."""
    rng = np.random.default_rng(seed)
    n = len(cams)
    D = np.eye(4); out = [D.copy()]
    u = rng.normal(size=3); u /= np.linalg.norm(u)
    w = rng.normal(size=3); w /= np.linalg.norm(w)
    for i in range(1, n):
        u += 0.02 * rng.normal(size=3); u /= np.linalg.norm(u)
        w += 0.02 * rng.normal(size=3); w /= np.linalg.norm(w)
        Rd = Rotation.from_rotvec(w * np.deg2rad(end_deg) / n).as_matrix()
        pc = cams[i][:3, 3]
        Rp = np.eye(4); Rp[:3, :3] = Rd; Rp[:3, 3] = pc - Rd @ pc + u * end_m / n
        D = Rp @ D
        out.append(D.copy())
    return [out[i] @ cams[i] for i in range(n)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--icp-hz", type=float, default=0.0); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tag", required=True); ap.add_argument("--sig", type=float, default=None, help="object noise [m] (default: measured)")
    ap.add_argument("--seg", type=int, nargs=2, default=(1005, 1214)); ap.add_argument("--static", default="5,6,8,9,10,11")
    ap.add_argument("--min-lm-rot", type=int, default=3, help="99 = never correct the rotation")
    ap.add_argument("--set", action="append", default=[], metavar="key=value", help="AnchorConfig override")
    ap.add_argument("--end-m", type=float, default=0.06); ap.add_argument("--end-deg", type=float, default=1.5)
    a = ap.parse_args()
    static = {int(x) for x in a.static.split(",")}
    ids, cams, gt, resid = gt_cameras(a.seg[0], a.seg[1], static)
    print(a.tag, "GT layout residual [mm]:", {k[4:]: round(v * 1000, 1) for k, v in resid.items()})
    s0 = a.seg[0]
    slam = drift(cams, a.seed, a.end_m, a.end_deg)
    K = np.asarray(json.loads((LMO / "scene_camera.json").read_text())["0"]["cam_K"], float).reshape(3, 3)
    frames = [json.loads(l) for l in open(RUN)]
    # the BOP images were run back to back (done_s runs far ahead of the video time) -> result at frame time + its processing time (queue wait excluded)
    frames = [dict(f, idx=f["idx"] - s0, done_s=(f["idx"] - s0) / FPS + f["ms"]["total"] / 1000.0,
                   dets=[d for d in f["dets"] if d["object"] in {NAMES[o] for o in static}]) for f in frames if s0 <= f["idx"] < a.seg[1]]
    # measured SAM-6D translation error on this scene (verified poses) -> object noise
    errs = []
    for f in frames:
        g = {NAMES[r["obj_id"]]: T_of(r) for r in gt[str(f["idx"] + s0)] if r["obj_id"] in static}
        for d in f["dets"]:
            if d.get("verified", True) and d["object"] in g:
                errs.append(np.linalg.norm(np.asarray(d["t_mm"]) / 1000.0 - g[d["object"]][:3, 3]))
    sig = a.sig if a.sig is not None else float(np.median(errs))
    cfg = AnchorConfig(use_rotation=False, max_rot_sigma=2.0, sig_obs_t0=sig, sig_obs_tz=0.0, deadband_m=sig, min_obs=5, min_lm_rot=a.min_lm_rot)
    for kv in a.set:
        k, v = kv.split("="); cur = getattr(cfg, k)
        setattr(cfg, k, (v.lower() in ("1", "true")) if isinstance(cur, bool) else type(cur)(v))
    res = {"sam6d_err_median_mm": float(np.median(errs) * 1000), "sigma_m": sig, "seg": list(a.seg), "n_sam_frames": len(frames),
           "layout_resid_mm": {k: v * 1000 for k, v in resid.items()}}
    for mode in ("slam", "anchor", "anchor_icp"):
        if mode == "anchor_icp" and a.icp_hz <= 0:
            continue
        anc = ObjectAnchor(cfg)
        mem = WorldPoseMemory(K, (640, 480), depth_check=None, assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None)
        ev = [(f["done_s"], 0, f) for f in frames]
        icp = None
        if mode == "anchor_icp":
            from depth_icp import DepthICP
            mp = {}
            for nm in {NAMES[o] for o in static}:
                q = np.load(PTS / f"{nm}.npy").astype(np.float64) / 1000.0
                mp[nm] = q[np.random.default_rng(0).choice(len(q), min(1500, len(q)), replace=False)]
            icp = DepthICP(mp)
            st = max(1, int(round(FPS / a.icp_hz)))
            ev += [(k / FPS + 0.05, 1, k) for k in range(0, len(ids), st)]
        ev.sort(key=lambda e: (e[0], e[1]))
        hist, icp_ms, nal = [], [], [0, 0]
        for t_ev, kind, f in ev:
            if kind == 1:
                k = f
                lms = [l for l in mem.anchor_landmarks() if l[3] >= 5]
                if not lms:
                    continue
                t0 = time.perf_counter()
                import cv2
                dm = cv2.imread(str(LMO / "depth" / f"{k + s0:06d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float64) / 1000.0
                Tc = anc.correction() @ slam[k]
                dd = []
                for _, nm, Tw, _ in lms:
                    Tco = inv(Tc) @ Tw
                    r = icp.align(Tco, nm, dm, K)
                    nal[0] += 1
                    if r is None or r["inlier"] < 0.5 or np.linalg.norm(r["shift"]) > 0.25:
                        continue
                    nal[1] += 1
                    T2 = Tco.copy(); T2[:3, 3] += r["shift"]; dd.append((nm, T2))
                icp_ms.append((time.perf_counter() - t0) * 1e3)
                if dd:
                    anc.observe(T0 + k * STEP, slam[k], dd, lms, src="icp")
                    hist.append((t_ev, anc.correction()))
                continue
            k = f["idx"]
            dets = [(d["object"], d["score"], d["R"], d["t_mm"]) for d in f["dets"] if d.get("verified", True)]
            if mode == "slam":
                x = slam[k]
            else:
                dd = []
                for nm, _, R, t in dets:
                    T = np.eye(4); T[:3, :3] = np.reshape(R, (3, 3)); T[:3, 3] = np.asarray(t) / 1000.0
                    dd.append((nm, T))
                x = anc.observe(T0 + k * STEP, slam[k], dd, mem.anchor_landmarks())
                hist.append((f["done_s"], anc.correction()))
            mem.add_frame_world(T0 + k * STEP, x, dets)
            mem.update()
        done = [h[0] for h in hist]
        live = []
        for k in range(len(ids)):
            j = int(np.searchsorted(done, k / FPS, side="right")) - 1
            live.append((hist[j][1] @ slam[k]) if j >= 0 else slam[k])
        e = np.array([np.linalg.norm(live[k][:3, 3] - cams[k][:3, 3]) for k in range(len(ids))]) * 1000
        n5 = max(1, len(e) // 20)
        res[mode] = {"rmse_mm": float(np.sqrt((e ** 2).mean())), "median_mm": float(np.median(e)), "p90_mm": float(np.percentile(e, 90)),
                     "end_mm": float(e[-n5:].mean()), "max_mm": float(e.max()), "series": e[::5].round(1).tolist()}
        if icp_ms:
            res[mode]["icp_ms_mean"] = float(np.mean(icp_ms)); res[mode]["icp_aligned"] = nal
        print(f"{a.tag} {mode:10s} RMSE {res[mode]['rmse_mm']:6.1f}  median {res[mode]['median_mm']:6.1f}  p90 {res[mode]['p90_mm']:6.1f}  end {res[mode]['end_mm']:6.1f} mm"
              + (f"  (icp {np.mean(icp_ms):.1f} ms, aligned {nal[1]}/{nal[0]})" if icp_ms else ""), flush=True)
    print(f"{a.tag} SAM-6D translation error median {res['sam6d_err_median_mm']:.1f} mm -> sigma {sig * 1000:.1f} mm")
    out = REPO / "objpose/output/obj_anchor_260924"
    (out / f"lmo_drift_{a.tag}.json").write_text(json.dumps(res))


if __name__ == "__main__":
    main()
