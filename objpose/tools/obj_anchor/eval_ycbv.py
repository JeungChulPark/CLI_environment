#!/usr/bin/env python3
"""eval_ycbv.py — does the Object Pose Map make the live SLAM camera pose more accurate? (YCB-V, GT camera poses)

Real-time replay, same clock as the SAM-6D real-time runs: frame k is shown at k/30 s, a SAM-6D result of frame
idx becomes available at its done_s. The live camera pose shown at frame k is
    baseline   T_slam(k)                        (ORB-SLAM3 RGB-D, pose returned right after tracking frame k)
    anchored   C(k/30) . T_slam(k)              C = correction from the latest result with done_s <= k/30
Both are scored against GT camera poses:
    drift      aligned at frame 0 only (what a live system sees)       -> RMSE / max / end error
    ATE        best SE(3) alignment over the whole run (TUM standard)  -> RMSE
The anchored run feeds the object memory with its own corrected poses (the two directions close the loop).

    /home/jucpark/anaconda3/envs/sam6d/bin/python objpose/tools/obj_anchor/eval_ycbv.py --run <sam6d run dir> --tag gt
"""
from __future__ import annotations

import argparse, json, sys
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/pc"))
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM                                   # noqa: E402
from core.depth_check import DepthCheckConfig                     # noqa: E402
from obj_anchor import ObjectAnchor, AnchorConfig, inv, rotvec    # noqa: E402
from anchor_memory import WorldPoseMemory                          # noqa: E402
from build_memory_maps import T0_NS, STEP_NS                         # noqa: E402

FPS = 30.0
OUT = REPO / "objpose/output/obj_anchor_260924"


def symmetric_names(by_id):
    info = json.loads((BM.YCBV.parent / "models_eval/models_info.json").read_text())
    return frozenset(by_id[int(k)] for k, v in info.items()
                     if int(k) in by_id and ("symmetries_continuous" in v or "symmetries_discrete" in v))


ICP_HZ = 0.0          # depth-ICP camera-object observations per second (set by --icp-hz)
ICP_GATE = (0.5, 0.25)  # accept an ICP result: inlier ratio >= , shift <= [m] (--icp-inlier, --icp-max-shift)


def replay(scene, run, anchored, cfg, by_id):
    sc = BM.YCBV / scene
    ids, Twc_gt, gobj, K = BM.gt_scene(sc, by_id)
    slam = BM.load_slam(scene, len(ids))
    mem = WorldPoseMemory(K, (640, 480),
                          depth_check=DepthCheckConfig(free_margin_m=0.15, confirm_frames=3, max_speed_mps=0.6, max_rot_dps=45.0),
                          assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None)
    anc = ObjectAnchor(cfg)
    frames = [json.loads(l) for l in (run / scene / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()]
    ev = [(f["done_s"], 0, f) for f in frames]
    icp = None
    if anchored and ICP_HZ > 0:                         # depth ICP at video frames, ~50 ms after capture
        from depth_icp import DepthICP
        mp = {}
        for nm in by_id.values():
            q = np.load(BM.PTS / f"{nm}.npy").astype(np.float64) / 1000.0
            mp[nm] = q[np.random.default_rng(0).choice(len(q), min(1500, len(q)), replace=False)]
        icp = DepthICP(mp)
        step = max(1, int(round(FPS / ICP_HZ)))
        ev += [(k / FPS + 0.05, 1, k) for k in range(0, len(ids), step)]
    ev.sort(key=lambda e: (e[0], e[1]))
    hist = []                                           # (time, C)
    snaps = []                                          # (done_s, [(name, T_w_obj)]) what the map holds
    icp_stat = [0, 0, []]
    for t_ev, kind, f in ev:
        if kind == 1:
            k = f
            if k not in slam:
                continue
            lms = [l for l in mem.anchor_landmarks() if l[3] >= 5]
            if not lms:
                continue
            import time as _t
            t0 = _t.perf_counter()
            raw = cv2.imread(str(sc / "depth" / f"{k + 1:06d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float64) * 1e-4
            Tc = anc.correction() @ slam[k]
            dd = []
            for _, nm, Tw, _ in lms:
                Tco = inv(Tc) @ Tw
                r = icp.align(Tco, nm, raw, K)
                icp_stat[0] += 1
                if r is None or r["inlier"] < ICP_GATE[0] or np.linalg.norm(r["shift"]) > ICP_GATE[1]:
                    continue
                icp_stat[1] += 1
                T2 = Tco.copy(); T2[:3, 3] += r["shift"]
                dd.append((nm, T2))
            icp_stat[2].append((_t.perf_counter() - t0) * 1e3)
            if dd:
                anc.observe(T0_NS + k * STEP_NS, slam[k], dd, lms, src="icp")
                hist.append((t_ev, anc.correction()))
            continue
        k = f["idx"]
        if k not in slam:
            continue
        dets = [(d["object"], d["score"], d["R"], d["t_mm"]) for d in f["dets"]]
        if anchored:
            dd = []
            for nm, _, R, t in dets:
                T = np.eye(4); T[:3, :3] = np.reshape(R, (3, 3)); T[:3, 3] = np.asarray(t) / 1000.0
                dd.append((nm, T))
            x = anc.observe(int(f["t_ns"]), slam[k], dd, mem.anchor_landmarks())
            hist.append((f["done_s"], anc.correction()))
        else:
            x = slam[k]
        raw = cv2.imread(str(sc / "depth" / f"{k + 1:06d}.png"), cv2.IMREAD_UNCHANGED)
        depth = np.rint(raw.astype(np.float32) * 0.1).astype(np.uint16)[::8, ::8]
        mem.add_frame_world(int(f["t_ns"]), x, dets, depth=depth)
        mem.update()
        snaps.append((f["done_s"], [(m["name"], m["T_w_obj"]) for m in mem.landmarks() if m["status"] in ("active", "lost", "remembered")]))
    anc.stats["icp"] = {"attempts": icp_stat[0], "aligned": icp_stat[1],
                        "ms_mean": float(np.mean(icp_stat[2])) if icp_stat[2] else None}
    # live pose per video frame
    done = [h[0] for h in hist]
    live = {}
    for k, T in slam.items():
        j = int(np.searchsorted(done, k / FPS, side="right")) - 1
        live[k] = (hist[j][1] @ T) if j >= 0 else T
    return ids, Twc_gt, gobj, live, mem, anc, snaps


_PTS = {}


def display_match(scene, ids, live, snaps, by_id):
    """Object Pose Map display agreement (as baselines/display_eval.py): at every frame, the map objects drawn with
    the live camera pose; a visible GT object counts when the drawn one is within ADD-S < 0.1 d."""
    from scipy.spatial import cKDTree
    info = json.loads((BM.YCBV.parent / "models_eval/models_info.json").read_text())
    gt = json.loads((BM.YCBV / scene / "scene_gt.json").read_text())
    gi = json.loads((BM.YCBV / scene / "scene_gt_info.json").read_text())
    name_id = {v: k for k, v in by_id.items()}
    done = [s[0] for s in snaps]
    inst = match = 0
    for k, fid in enumerate(ids):
        G = {}
        for r, i in zip(gt[fid], gi[fid]):
            if i.get("visib_fract", 1) >= 0.1 and r["obj_id"] in by_id:
                T = np.eye(4); T[:3, :3] = np.reshape(r["cam_R_m2c"], (3, 3)); T[:3, 3] = np.asarray(r["cam_t_m2c"]) / 1000.0
                G[by_id[r["obj_id"]]] = T
        inst += len(G)
        j = int(np.searchsorted(done, k / FPS, side="right")) - 1
        if j < 0 or k not in live:
            continue
        Tcw = inv(live[k])
        best = {}
        for nm, Tw in snaps[j][1]:
            if nm not in G:
                continue
            if nm not in _PTS:
                p = np.load(BM.PTS / f"{nm}.npy").astype(float)
                if len(p) > 1500: p = p[np.random.default_rng(0).choice(len(p), 1500, replace=False)]
                _PTS[nm] = (p, cKDTree(p), 0.1 * info[str(name_id[nm])]["diameter"])
            p, tree, thr = _PTS[nm]
            D = inv(G[nm]) @ Tcw @ Tw
            e = tree.query(p @ D[:3, :3].T + D[:3, 3] * 1000.0)[0].mean()
            best[nm] = min(best.get(nm, 1e9), e)
        match += sum(1 for nm, e in best.items() if e < _PTS[nm][2])
    return match, inst


def score(Twc_gt, live):
    ks = sorted(live)
    P = np.array([live[k][:3, 3] for k in ks]); G = Twc_gt[ks, :3, 3]
    # drift: frame-0 alignment
    A0 = Twc_gt[ks[0]] @ inv(live[ks[0]])
    e0 = np.linalg.norm(P @ A0[:3, :3].T + A0[:3, 3] - G, axis=1) * 1000
    r0 = np.array([np.linalg.norm(rotvec(Twc_gt[k, :3, :3].T @ A0[:3, :3] @ live[k][:3, :3])) for k in ks]) * 180 / np.pi
    # ATE: pose-based SE(3) alignment (as build_memory_maps)
    M = sum(Twc_gt[k, :3, :3] @ live[k][:3, :3].T for k in ks)
    U, _, Vt = np.linalg.svd(M); R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    t = np.mean([Twc_gt[k, :3, 3] - R @ live[k][:3, 3] for k in ks], 0)
    ea = np.linalg.norm(P @ R.T + t - G, axis=1) * 1000
    n5 = max(1, len(ks) // 20)
    return {"drift_rmse_mm": float(np.sqrt((e0 ** 2).mean())), "drift_max_mm": float(e0.max()), "drift_end_mm": float(e0[-n5:].mean()),
            "drift_rot_mean_deg": float(r0.mean()), "ate_rmse_mm": float(np.sqrt((ea ** 2).mean())),
            "series_drift_mm": e0[::10].round(1).tolist()}


def map_error(gobj, mem, Twc_gt, live):
    """final map: object position error after the same frame-0 alignment (what the live map shows)"""
    ks = sorted(live)
    A0 = Twc_gt[ks[0]] @ inv(live[ks[0]])
    errs, used = [], set()
    for m in sorted(mem.landmarks(), key=lambda m: -m["n_obs"]):
        if m["status"] not in ("active", "lost", "remembered") or m["name"] in used or m["name"] not in gobj:
            continue
        used.add(m["name"])
        errs.append(float(np.linalg.norm((A0 @ m["T_w_obj"])[:3, 3] - gobj[m["name"]][:3, 3]) * 1000))
    return errs


def one(args):
    scene, run, cfg = args
    table = json.loads(BM.TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    cfg.symmetric = symmetric_names(by_id)
    out = {"scene": scene}
    for mode in ("baseline", "anchored"):
        ids, Twc_gt, gobj, live, mem, anc, snaps = replay(scene, run, mode == "anchored", cfg, by_id)
        s = score(Twc_gt, live)
        s["display_match"], s["display_inst"] = display_match(scene, ids, live, snaps, by_id)
        me = map_error(gobj, mem, Twc_gt, live)
        s.update(map_err_median_mm=float(np.median(me)) if me else None, map_err_max_mm=float(max(me)) if me else None,
                 map_registered=len(me))
        if mode == "anchored":
            s["anchor"] = anc.summary()
        out[mode] = s
    b, a = out["baseline"], out["anchored"]
    print(f"{scene}: drift RMSE {b['drift_rmse_mm']:.1f} -> {a['drift_rmse_mm']:.1f} mm, end {b['drift_end_mm']:.1f} -> {a['drift_end_mm']:.1f}, "
          f"ATE {b['ate_rmse_mm']:.1f} -> {a['ate_rmse_mm']:.1f}, map med {b['map_err_median_mm']} -> {a['map_err_median_mm']}, "
          f"display {b['display_match'] / b['display_inst']:.3f} -> {a['display_match'] / a['display_inst']:.3f}, "
          f"solve {a['anchor']['solve_ms_median']:.1f}/{a['anchor']['solve_ms_p95']:.1f} ms", flush=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, default=REPO / "objpose/output/cmp_260924_gt/proposed_ycbv_realtime")
    ap.add_argument("--tag", default="gt")
    ap.add_argument("--scenes", nargs="*", default=None)
    ap.add_argument("--set", action="append", default=[], metavar="key=value", help="AnchorConfig override")
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--icp-hz", type=float, default=0.0)
    ap.add_argument("--slam-dir", type=Path, default=None, help="ORB-SLAM3 runs (<scene>_frames.txt), default build_memory_maps.SLAM")
    ap.add_argument("--icp-inlier", type=float, default=0.5); ap.add_argument("--icp-max-shift", type=float, default=0.25)
    a = ap.parse_args()
    global ICP_HZ, ICP_GATE
    ICP_HZ = a.icp_hz; ICP_GATE = (a.icp_inlier, a.icp_max_shift)
    if a.slam_dir is not None:
        BM.SLAM = a.slam_dir.resolve()
    cfg = AnchorConfig()
    for kv in a.set:
        k, v = kv.split("=")
        cur = getattr(cfg, k)
        setattr(cfg, k, tuple(float(x) for x in v.split(",")) if isinstance(cur, tuple) else (v.lower() in ("1", "true")) if isinstance(cur, bool) else type(cur)(v))
    scenes = a.scenes or sorted(p.name for p in BM.YCBV.iterdir())
    with Pool(a.jobs) as p:
        res = p.map(one, [(s, a.run, cfg) for s in scenes])
    tot = {}
    for mode in ("baseline", "anchored"):
        tot[mode] = {k: float(np.mean([r[mode][k] for r in res])) for k in ("drift_rmse_mm", "drift_max_mm", "drift_end_mm", "drift_rot_mean_deg", "ate_rmse_mm")}
        me = [r[mode]["map_err_median_mm"] for r in res if r[mode]["map_err_median_mm"] is not None]
        tot[mode]["map_err_median_mm_mean"] = float(np.mean(me))
        tot[mode]["display_match_rate"] = sum(r[mode]["display_match"] for r in res) / sum(r[mode]["display_inst"] for r in res)
    ms = np.concatenate([[r["anchored"]["anchor"]["solve_ms_median"]] for r in res])
    tot["solve_ms_median_mean"] = float(ms.mean()); tot["solve_ms_max"] = float(max(r["anchored"]["anchor"]["solve_ms_max"] for r in res))
    print(json.dumps(tot, indent=1))
    OUT.mkdir(parents=True, exist_ok=True)
    cfgd = {k: (sorted(v) if isinstance(v, frozenset) else v) for k, v in vars(cfg).items()}
    (OUT / f"ycbv_{a.tag}.json").write_text(json.dumps({"run": str(a.run), "config": cfgd, "mean": tot, "scenes": res}, indent=1))


if __name__ == "__main__":
    main()
