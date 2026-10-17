#!/usr/bin/env python3
"""eval_real.py — object-anchored SLAM correction on recorded live runs (real robot data, no GT camera).

A live hub run logs everything the hub received, with data-clock stamps: every SLAM pose message
(slam_poses.jsonl: live T_wc + reference keyframe), every keyframe update (kf_updates.jsonl, periodic local-BA
refreshes and loop closures), every SAM-6D result (sam6d_frames.jsonl) and its age when it arrived
(sam6d_estimates.jsonl: result_age_s). Replaying them in arrival order reproduces what the hub knew at each moment.

Camera pose shown live at time t (SAM camera = SLAM pose . X):
    raw        T_wc streamed by ORB-SLAM3
    hub        keyframe-anchored pose with the keyframe poses known at t (what the Object Pose Map uses now)
    anchored   C(t) . hub(t), C from the object anchor fed with results that have arrived by t
Reference: the same trajectory re-expressed with the final keyframe poses (after the run's loop closures / BA).

Object map: hub = ObjectMemory on keyframe-anchored poses (as live), anchored = memory fed with the corrected poses.

    /home/jucpark/anaconda3/envs/sam6d/bin/python objpose/tools/obj_anchor/eval_real.py <run dir> --x <X json> --session <SAM session>
"""
from __future__ import annotations

import argparse, json, sys, time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/pc"))
from fusion import Fusion, PoseBuffer, mat                       # noqa: E402
from memory import ObjectMemory                                   # noqa: E402
from core.depth_check import DepthCheckConfig                     # noqa: E402
from obj_anchor import ObjectAnchor, AnchorConfig, inv           # noqa: E402
from anchor_memory import WorldPoseMemory, CorrectedSlamMemory, KEEP                   # noqa: E402

OUT = REPO / "objpose/output/obj_anchor_260924"
DEPTH_DS = 8


def jl(p):
    return [json.loads(l) for l in Path(p).read_text().splitlines() if l.strip()]


class Depths:
    """aligned depth of the SAM session by data-clock stamp (hub: depth[::8, ::8] of every frame sent to SAM-6D)"""
    def __init__(self, session, tau_ns):
        from conv_session import ConvSession
        self.s = ConvSession(session, offset_ns=tau_ns, cache_dir=REPO / "objpose/output/.clock_cache")
        self.K, self.W, self.H = self.s.K, self.s.W, self.s.H

    def at(self, t_ns):
        i = int(np.argmin(np.abs(self.s.t_ns - t_ns)))
        if abs(int(self.s.t_ns[i]) - t_ns) > 5_000_000:
            return None
        return np.ascontiguousarray(self.s.read(i, True).depth_raw[::DEPTH_DS, ::DEPTH_DS]).astype(np.uint16)


def oracle_landmarks(run_dir, X, scale):
    """diagnosis only: every object's pose from ALL its estimates placed with the FINAL trajectory (median position,
    rotation of the estimate nearest to it). Not available live — used to test whether the frozen anchors are the problem."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from fit_extrinsic import final_poses
    pb = final_poses(Path(run_dir))
    by = {}
    for e in jl(Path(run_dir) / "sam6d_estimates.jsonl"):
        T, _ = pb.at(int(e["t_ns"]))
        if T is None:
            continue
        Tco = np.eye(4); Tco[:3, :3] = np.asarray(e["R"]); Tco[:3, 3] = np.asarray(e["t_mm"]) * scale / 1000.0
        by.setdefault(e["object"], []).append(T @ X @ Tco)
    out = []
    for k, (nm, Ts) in enumerate(sorted(by.items())):
        if len(Ts) < 5:
            continue
        P = np.array([T[:3, 3] for T in Ts]); med = np.median(P, 0)
        Tm = Ts[int(np.argmin(np.linalg.norm(P - med, axis=1)))].copy(); Tm[:3, 3] = med
        out.append((10000 + k, nm, Tm, len(Ts)))
    return out


def run(a, cfg):
    d = Path(a.run)
    X = np.asarray(json.loads(Path(a.x).read_text())["T_slam_sam"], float).reshape(4, 4)
    poses_msgs = jl(d / "slam_poses.jsonl")
    kfu = jl(d / "kf_updates.jsonl")
    frames = jl(d / "sam6d_frames.jsonl")
    age = {}
    for e in jl(d / "sam6d_estimates.jsonl"):
        if e.get("result_age_s") is not None:
            age[int(e["t_ns"])] = float(e["result_age_s"])
    if a.session:
        dep = Depths(a.session, int(round(a.tau_s * 1e9)))
    else:                                   # session not reachable: K from the frame-clock cache, no depth check
        z = np.load(a.k_npz)
        dep = type("K", (), {"K": np.asarray(z["K"], float).reshape(3, 3), "W": int(z["W"]), "H": int(z["H"])})()
        a.no_depth_check = True
    dc = None if a.no_depth_check else DepthCheckConfig(free_margin_m=0.15, confirm_frames=3, max_speed_mps=0.6, max_rot_dps=45.0)

    # events on the data clock: pose (arrives at its stamp), kf update, SAM result (arrives at stamp + age)
    ev = [(int(m["t_ns"]), 0, "pose", m) for m in poses_msgs]
    ev += [(int(u["t_ns"]), 1, "kf", u) for u in kfu]
    med_age = float(np.median(list(age.values()))) if age else 0.5
    for f in frames:
        if f["dets"]:
            ev.append((int(f["t_ns"]) + int(age.get(int(f["t_ns"]), med_age) * 1e9), 2, "sam", f))
    if getattr(a, "icp_hz", 0) > 0:          # depth-ICP observations at SAM-camera frames, every 1/icp_hz s
        nxt = ev[0][0]
        for i_, tf in enumerate(dep.s.t_ns):
            if tf >= nxt:
                ev.append((int(tf) + 50_000_000, 3, "icp", (int(tf), i_)))   # ~ processing latency; SLAM poses after tf have arrived
                nxt = tf + 1e9 / a.icp_hz
    ev.sort(key=lambda e: (e[0], e[1]))

    pb = PoseBuffer(max_gap_s=0.25)
    fus = Fusion(X, pb)
    mem_hub = ObjectMemory(fus, dep.K, (dep.W, dep.H), depth_check=dc, assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None)
    mem_anc = WorldPoseMemory(dep.K, (dep.W, dep.H), depth_check=dc, assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None)
    # slamcorr: the keyframe-anchored map, each frame registered at the corrected camera pose
    mem_cor = (CorrectedSlamMemory(fus, dep.K, (dep.W, dep.H), depth_check=dc, assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None)
               if a.anchor_map == "slamcorr" else None)
    dups = []                                          # (t, duplicates in the SLAM-pose map, duplicates in the anchored map)
    anc = ObjectAnchor(cfg)
    icp, icp_ms, icp_n = None, [], [0, 0]
    if getattr(a, "icp_hz", 0) > 0:
        from depth_icp import DepthICP
        mp = {}
        for nm in json.loads((REPO / "integration/cad_extents.json").read_text()):
            f = REPO / "sam6d_realtime/assets/model_points" / f"{nm}.npy"
            if f.exists():
                q = np.load(f).astype(np.float64) / 1000.0
                mp[nm] = q[np.random.default_rng(0).choice(len(q), min(1500, len(q)), replace=False)]
        icp = DepthICP(mp)
    if a.kf_anchors:
        from fusion import inv_se3

        def attach(T_w, t_ns):
            kf = pb.anchor_at(t_ns)
            T_w_kf, _ = pb.kfs.resolve(kf) if kf is not None else (None, None)
            return ("w", T_w) if T_w_kf is None else ("kf", kf, inv_se3(T_w_kf) @ T_w)

        def resolve(h):
            if h[0] == "w":
                return h[1]
            T_w_kf, _ = pb.kfs.resolve(h[1])
            return None if T_w_kf is None else T_w_kf @ h[2]
        anc.attach, anc.resolve = attach, resolve
    oracle = oracle_landmarks(d, X, a.range_scale) if a.landmarks == "oracle" else None
    oracle_ref = {nm: T for _, nm, T, _ in oracle_landmarks(d, X, a.range_scale)}
    live_raw, live_hub, live_anc = {}, {}, {}
    prev_T, n_jumps = None, [0]
    n_sam = 0
    pred = {"hub": [], "anchored": []}                 # (mm in camera, px) of each new detection vs the map's prediction
    Kc = np.asarray(dep.K, float)

    def predict(lms, T_w_cam, dets, key):
        Tcw = inv(T_w_cam)
        for nm, _, R, tm in dets:
            c = [m for m in lms if m["name"] == nm and m["n_obs"] >= 5 and m["status"] in KEEP]
            if not c:
                continue
            meas = np.asarray(tm, float) / 1000.0
            P = np.array([(Tcw @ m["T_w_obj"])[:3, 3] for m in c])
            j = int(np.argmin(np.linalg.norm(P - meas, axis=1)))
            p = P[j]
            if p[2] < 0.1 or meas[2] < 0.1:
                continue
            uv = lambda q: Kc[:2, :2] @ (q[:2] / q[2]) + Kc[:2, 2]
            pred[key].append((float(np.linalg.norm(p - meas) * 1000), float(np.linalg.norm(uv(p) - uv(meas)))))
    for t, _, kind, m in ev:
        if kind == "pose":
            if m.get("T_wc") is None or m.get("state") not in ("OK", "OK_KLT"):
                continue
            pb.add(t, m.get("state"), m.get("T_wc"), m.get("map_id"), kf=m.get("ref_kf"), T_w_kf=m.get("T_w_kf"),
                   kf_map_id=m.get("kf_map_id"), map_changed=bool(m.get("map_changed")))
            T, _ = pb.at(t)
            if T is None:
                continue
            live_raw[t] = mat(m["T_wc"])
            # a jump of the SLAM pose itself (loop closure applied to tracking, relocalisation, a tracking glitch):
            # faster than the camera can move -> absorb it so the display stays continuous, converge over loop_blend_s
            if cfg.loop_blend_s > 0 and prev_T is not None and (t - prev_T[0]) < 100_000_000 \
                    and np.linalg.norm(T[:3, 3] - prev_T[1][:3, 3]) > getattr(a, "jump_m", 0.05):
                anc.reset_smooth(prev_T[1] @ X, T @ X, t)
                n_jumps[0] += 1
            prev_T = (t, T)
            live_hub[t] = T
            live_anc[t] = anc.correct(T @ X, t) @ inv(X)
            # optional snapshots (for figures): the map objects the display would draw at this moment
            snap_ns = getattr(a, "snap_ns", None)
            while snap_ns and snap_ns[0] <= t:
                tq = snap_ns.pop(0)
                Tq, _ = pb.at(tq)                      # camera exactly at the requested time (e.g. a SAM frame)
                if Tq is None:
                    continue
                amem = mem_cor if mem_cor is not None else mem_hub
                a.snaps.append({"t": tq, "T_hub": Tq, "T_anc": anc.correct(Tq @ X, t) @ inv(X),
                                "hub_lms": [(m_["name"], m_["T_w_obj"]) for m_ in mem_hub.landmarks() if m_["status"] in KEEP],
                                "anc_lms": [(m_["name"], m_["T_w_obj"]) for m_ in amem.landmarks() if m_["status"] in KEEP]})
        elif kind == "icp":
            s_, fidx = m
            T, _ = pb.at(s_)
            amap = mem_cor if mem_cor is not None else mem_hub
            if T is None:
                continue
            t_i = time.perf_counter()
            depth_m = dep.s.read(fidx, True).depth_raw.astype(np.float64) / 1000.0
            Tc = anc.correct(T @ X, t)
            lms = [(m_["object_id"], m_["name"], m_["T_w_obj"], m_["n_obs"]) for m_ in amap.landmarks()
                   if m_["status"] in KEEP and m_["n_obs"] >= 5 and m_["name"] in icp.models]
            t_c = time.perf_counter()
            dd = []
            for _, nm, Tw, _ in lms:
                Tco = inv(Tc) @ Tw
                res_ = icp.align(Tco, nm, depth_m, dep.K)
                icp_n[0] += 1
                if res_ is None or res_["inlier"] < 0.5 or np.linalg.norm(res_["shift"]) > 0.25:
                    continue
                icp_n[1] += 1
                Tco2 = Tco.copy(); Tco2[:3, 3] += res_["shift"]
                dd.append((nm, Tco2))
            icp_ms.append((time.perf_counter() - t_c) * 1e3)
            if dd:
                anc.now_ns = t
                anc.observe(s_, T @ X, dd, lms, src="icp")
        elif kind == "kf":
            loop_ = a.reset_on_loop and m.get("reason") not in ("periodic", "end")
            tl_ = pb.latest_t()
            Tb_, _ = pb.at(tl_) if (loop_ and cfg.loop_blend_s > 0) else (None, None)
            pb.apply_kf_update(m.get("map_id"), m.get("kfs", []))
            if loop_:
                Ta_, _ = pb.at(tl_) if Tb_ is not None else (None, None)
                if Tb_ is not None and Ta_ is not None:
                    anc.reset_smooth(Tb_ @ X, Ta_ @ X, t)
                else:
                    anc.reset()
                if mem_cor is not None:           # anchors and frames registered with the old corrections: drop them
                    anc.frozen.clear(); anc.views.clear(); anc.clusters.clear()
                    mem_cor.new_epoch()
        else:
            s = int(m["t_ns"])
            T, _ = pb.at(s)
            depth = dep.at(s) if dc is not None else None
            dets = [(x["object"], x.get("score", 0.0), x["R"], [v * a.range_scale for v in x["t_mm"]]) for x in m["dets"]]
            if T is not None:
                # before this result is used anywhere: where do the two maps say the objects are in this frame?
                predict(mem_hub.landmarks(), T @ X, dets, "hub")
                amap = mem_cor if mem_cor is not None else (mem_hub if a.anchor_map == "slam" else mem_anc)
                predict(amap.landmarks(), anc.correct(T @ X), dets, "anchored")
            mem_hub.add_frame(s, dets, depth=depth)
            mem_hub.update()
            if T is None:
                continue
            n_sam += 1
            dd = []
            for nm, _, R, tm in dets:
                Tco = np.eye(4); Tco[:3, :3] = np.reshape(R, (3, 3)); Tco[:3, 3] = np.asarray(tm) / 1000.0
                dd.append((nm, Tco))
            if oracle is not None:
                lms = oracle
            elif mem_cor is not None:
                lms = [(m_["object_id"], m_["name"], m_["T_w_obj"], m_["n_obs"]) for m_ in mem_cor.landmarks() if m_["status"] in KEEP]
            elif a.anchor_map == "slam":       # the keyframe-anchored map built from SLAM poses (follows loop closures)
                lms = [(m_["object_id"], m_["name"], m_["T_w_obj"], m_["n_obs"]) for m_ in mem_hub.landmarks() if m_["status"] in KEEP]
            else:
                lms = mem_anc.anchor_landmarks()
            anc.now_ns = t                      # arrival time of this result (for blend_s)
            x = anc.observe(s, T @ X, dd, lms)
            if mem_cor is not None:
                mem_cor.add_frame(s, dets, depth=depth, C=anc.C_applied)
                mem_cor.update()
            else:
                mem_anc.add_frame_world(s, x, dets, depth=depth)
                mem_anc.update()

            def ndup(mm):
                ls = [m_["name"] for m_ in mm.landmarks() if m_["status"] in KEEP]
                return len(ls) - len(set(ls))
            dups.append((t, ndup(mem_hub), ndup(mem_cor if mem_cor is not None else (mem_hub if a.anchor_map == "slam" else mem_anc))))
    mem_hub.update()
    # reference: every pose sample under the final keyframe poses
    ref = {t: pb.corrected(s_) for t, s_ in zip(pb._t, pb._s)}
    # how far the anchors that were actually used sit from the final-trajectory object positions
    used_mem = mem_cor if mem_cor is not None else (mem_hub if a.anchor_map == "slam" else mem_anc)
    lm_by_id = {m["object_id"]: m["name"] for m in used_mem.landmarks()}
    anchors = {k: (anc.resolve(v) if anc.resolve is not None else v) for k, v in anc.frozen.items()} if anc.frozen else {m["object_id"]: m["T_w_obj"] for m in used_mem.landmarks() if m["status"] in KEEP}
    frozen_err = []
    for lid, Tf in anchors.items():
        nm = lid[0] if isinstance(lid, tuple) else lm_by_id.get(lid)
        if nm in oracle_ref:
            frozen_err.append({"name": nm, "err_mm": float(np.linalg.norm(Tf[:3, 3] - oracle_ref[nm][:3, 3]) * 1000),
                               "vec_mm": ((Tf[:3, 3] - oracle_ref[nm][:3, 3]) * 1000).round(1).tolist()})
    if mem_cor is not None:
        mem_cor.update(); mem_anc = mem_cor
    print(f"pose jumps absorbed: {n_jumps[0]}")
    return dict(icp_ms=icp_ms, icp_n=icp_n, dups=dups, frozen_err=frozen_err, live_raw=live_raw, live_hub=live_hub, live_anc=live_anc, ref=ref, mem_hub=mem_hub, mem_anc=mem_anc,
                anc=anc, n_sam=n_sam, kf_max_corr_m=pb.kfs.max_correction_m, pred=pred)


def err_stats(live, ref, X):
    ts = sorted(set(live) & set(ref))
    # SAM-camera positions (where the objects are seen from)
    e = np.array([np.linalg.norm((live[t] @ X)[:3, 3] - (ref[t] @ X)[:3, 3]) for t in ts]) * 1000
    return {"rmse_mm": float(np.sqrt((e ** 2).mean())), "median_mm": float(np.median(e)), "p90_mm": float(np.percentile(e, 90)),
            "max_mm": float(e.max()), "n": len(ts), "series": [(round((t - ts[0]) / 1e9, 2), round(float(v), 1)) for t, v in zip(ts[::15], e[::15])]}


def pred_stats(v):
    if not v:
        return None
    a = np.array(v)
    return {"n": len(a), "mm_median": float(np.median(a[:, 0])), "mm_p90": float(np.percentile(a[:, 0], 90)),
            "px_median": float(np.median(a[:, 1])), "px_p90": float(np.percentile(a[:, 1], 90)),
            "within_50mm": float((a[:, 0] < 50).mean())}


def witness(run_dir, trajs, X, scale):
    """objects as witnesses: place every SAM-6D estimate with each camera trajectory (nearest pose within 20 ms);
    the trajectory under which each object's estimates collapse best is the more consistent one."""
    est = jl(Path(run_dir) / "sam6d_estimates.jsonl")
    out = {}
    for key, tr in trajs.items():
        ts = np.array(sorted(tr))
        by = {}
        for e in est:
            s_ = int(e["t_ns"]); j = int(np.searchsorted(ts, s_))
            cand = [k for k in (j - 1, j) if 0 <= k < len(ts)]
            if not cand:
                continue
            k = min(cand, key=lambda k: abs(ts[k] - s_))
            if abs(ts[k] - s_) > 20_000_000:
                continue
            p = (tr[ts[k]] @ X)[:3, :3] @ (np.asarray(e["t_mm"]) * scale / 1000.0) + (tr[ts[k]] @ X)[:3, 3]
            by.setdefault(e["object"], []).append(p)
        dev = np.concatenate([np.linalg.norm(np.array(v) - np.median(np.array(v), 0), axis=1) for v in by.values() if len(v) >= 5]) * 1000
        out[key] = {"median_mm": float(np.median(dev)), "p90_mm": float(np.percentile(dev, 90)), "n": int(len(dev))}
    return out


def map_stats(mem):
    lms = [m for m in mem.landmarks() if m["status"] in KEEP]
    per = {}
    for m in lms:
        per[m["name"]] = per.get(m["name"], 0) + 1
    return {"instances": len(lms), "classes": len(per), "duplicates": sum(v - 1 for v in per.values()), "per_class": per,
            "objects": [{"name": m["name"], "n_obs": m["n_obs"], "p": np.round(m["T_w_obj"][:3, 3], 3).tolist()} for m in lms]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--x", required=True, help="rig extrinsic json (T_slam_sam)")
    ap.add_argument("--session", default=None, help="SAM session (converted bag) for K and depth")
    ap.add_argument("--k-npz", default=None, help="without --session: conv_*.npz frame-clock cache holding K, W, H")
    ap.add_argument("--tau-s", type=float, default=0.0, help="the run's SAM tau")
    ap.add_argument("--tag", required=True)
    ap.add_argument("--no-depth-check", action="store_true")
    ap.add_argument("--landmarks", choices=["memory", "oracle"], default="memory",
                    help="oracle (diagnosis): anchors = object poses from all estimates with the final trajectory")
    ap.add_argument("--anchor-map", choices=["own", "slam", "slamcorr"], default="own",
                    help="own: anchors from a memory fed with the corrected poses; slam: from the keyframe-anchored SLAM map")
    ap.add_argument("--icp-hz", type=float, default=0.0, help="depth-ICP camera-object observations per second (0 = off)")
    ap.add_argument("--kf-anchors", action="store_true", help="frozen anchors stored relative to their SLAM keyframe")
    ap.add_argument("--reset-on-loop", action="store_true", help="reset the correction when SLAM re-optimises its map")
    ap.add_argument("--range-scale", type=float, default=1.0, help="multiply SAM-6D translations (fitted range scale)")
    ap.add_argument("--set", action="append", default=[], metavar="key=value")
    a = ap.parse_args()
    cfg = AnchorConfig()
    for kv in a.set:
        k, v = kv.split("=")
        cur = getattr(cfg, k)
        setattr(cfg, k, tuple(float(x) for x in v.split(",")) if isinstance(cur, tuple) else (v.lower() in ("1", "true")) if isinstance(cur, bool) else type(cur)(v))
    r = run(a, cfg)
    X = np.asarray(json.loads(Path(a.x).read_text())["T_slam_sam"], float).reshape(4, 4)
    wit = witness(a.run, {"raw": r["live_raw"], "anchored": r["live_anc"], "final": r["ref"]}, X, a.range_scale)
    print(f"{a.tag} witness spread (median/p90 mm): " + ", ".join(f"{k} {v['median_mm']:.1f}/{v['p90_mm']:.1f}" for k, v in wit.items()))
    dd_ = np.array([(x[1], x[2]) for x in r["dups"]]) if r["dups"] else np.zeros((1, 2))
    print(f"{a.tag} duplicates during the run: SLAM-pose map max {int(dd_[:, 0].max())} mean {dd_[:, 0].mean():.2f} | anchored map max {int(dd_[:, 1].max())} mean {dd_[:, 1].mean():.2f}")
    if r["icp_ms"]:
        im = np.array(r["icp_ms"])
        print(f"{a.tag} depth ICP: {len(im)} frames, {im.mean():.1f} ms mean / {np.percentile(im, 95):.1f} p95 per frame, objects aligned {r['icp_n'][1]}/{r['icp_n'][0]}")
    res = {"dups": [[(x[0]), x[1], x[2]] for x in r["dups"]], "witness": wit, "frozen_err": r["frozen_err"], "run": a.run, "sam_results_used": r["n_sam"], "kf_max_correction_m": r["kf_max_corr_m"],
           "raw": err_stats(r["live_raw"], r["ref"], X), "hub": err_stats(r["live_hub"], r["ref"], X),
           "anchored": err_stats(r["live_anc"], r["ref"], X),
           "pred_hub": pred_stats(r["pred"]["hub"]), "pred_anchored": pred_stats(r["pred"]["anchored"]),
           "map_hub": map_stats(r["mem_hub"]), "map_anchored": map_stats(r["mem_anc"]), "anchor": r["anc"].summary(),
           "config": {k: (sorted(v) if isinstance(v, frozenset) else v) for k, v in vars(cfg).items()}}
    for k in ("raw", "hub", "anchored"):
        s = res[k]
        print(f"{a.tag} {k:9s} RMSE {s['rmse_mm']:6.1f}  median {s['median_mm']:6.1f}  p90 {s['p90_mm']:6.1f}  max {s['max_mm']:6.1f} mm")
    for k in ("pred_hub", "pred_anchored"):
        q = res[k]
        if q:
            print(f"{a.tag} {k:13s} n {q['n']}  {q['mm_median']:.1f} / p90 {q['mm_p90']:.1f} mm   {q['px_median']:.1f} / p90 {q['px_p90']:.1f} px   <50mm {q['within_50mm']:.3f}")
    for k in ("map_hub", "map_anchored"):
        print(f"{a.tag} {k:13s} instances {res[k]['instances']} classes {res[k]['classes']} duplicates {res[k]['duplicates']}")
    fe = res["frozen_err"]
    if fe:
        print(f"{a.tag} anchors vs final-trajectory objects: " + ", ".join(f"{x['name']} {x['err_mm']:.0f}" for x in fe) + " mm")
    an = res["anchor"]
    print(f"{a.tag} anchor updates {an['updates']}/{an['frames']}  rejected {an['rejected']}  solve {an['solve_ms_median']:.1f} / p95 {an['solve_ms_p95']:.1f} / max {an['solve_ms_max']:.1f} ms")
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"real_{a.tag}.json").write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
