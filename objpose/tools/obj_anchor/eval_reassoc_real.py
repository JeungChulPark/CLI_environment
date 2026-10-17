#!/usr/bin/env python3
"""eval_reassoc_real.py — re-association of objects that leave the view and come back, on a recorded live run (no GT).

Replays a live hub run like eval_real.py (SLAM pose messages, keyframe updates, SAM-6D results in arrival order) into the
plain Object Pose Map (ObjectMemory on keyframe-anchored poses, hub defaults, no object->SLAM correction), and a baseline
memory that gets no SLAM (camera-frame positions: identity camera pose).

Each scene holds one object per class, so a class detection after a gap is right when it joins the map object that
existed before the gap and wrong when it spawns a new one. Event: consecutive accepted detections of a class GAP_S
or more apart. 'out' when the map object's centre left the image (or went behind the camera) during the gap, 'in'
otherwise (in view but not detected). Displacement: re-detected position vs the map object's position before it.

Also stores per-video-frame snapshots (camera pose, map objects with ids, recent events) for render_reassoc_real.py.

    /home/jucpark/anaconda3/envs/sam6d/bin/python objpose/tools/obj_anchor/eval_reassoc_real.py <run> --x <X json> \
        --session <SAM session> --tau-s <tau> --out <dir>
"""
from __future__ import annotations

import argparse, json, pickle, sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/pc"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from fusion import Fusion, PoseBuffer, mat                       # noqa: E402
from memory import ObjectMemory                                   # noqa: E402
from core.depth_check import DepthCheckConfig                     # noqa: E402
from eval_real import Depths, jl                                  # noqa: E402

KEEP = ("active", "lost", "remembered")
GAP_S = 2.0


class RecMemory(ObjectMemory):
    """keeps each step's decisions and the landmarks' (status, first_seen) right after it, per memory build"""

    def __init__(self, *a, **k):
        self.log = {}
        super().__init__(*a, **k)

    def _step(self, mem, f, idx):
        fr = super()._step(mem, f, idx)
        st = {lm.object_id: (getattr(lm.status, "value", str(lm.status)), lm.first_seen_time) for lm in mem.store.all_landmarks()}
        self.log.setdefault(id(mem), {})[f.t_ns] = (f.dets, fr, st)
        return fr

    def last(self, t_ns):
        return self.log.get(id(self._mem), {}).get(t_ns)


def in_view(T_w_cam, p, K, W, H, margin=0):
    q = np.linalg.inv(T_w_cam)
    c = q[:3, :3] @ p + q[:3, 3]
    if c[2] < 0.2:
        return False
    u = K[0, 0] * c[0] / c[2] + K[0, 2]; v = K[1, 1] * c[1] / c[2] + K[1, 2]
    return margin <= u < W - margin and margin <= v < H - margin


def run(a):
    d = Path(a.run)
    X = np.asarray(json.loads(Path(a.x).read_text())["T_slam_sam"], float).reshape(4, 4)
    poses_msgs = jl(d / "slam_poses.jsonl"); kfu = jl(d / "kf_updates.jsonl"); frames = jl(d / "sam6d_frames.jsonl")
    age = {int(e["t_ns"]): float(e["result_age_s"]) for e in jl(d / "sam6d_estimates.jsonl") if e.get("result_age_s") is not None}
    dep = Depths(a.session, int(round(a.tau_s * 1e9)))
    K = np.asarray(dep.K, float); W, H = dep.W, dep.H
    dc = DepthCheckConfig(free_margin_m=0.15, confirm_frames=3, max_speed_mps=0.6, max_rot_dps=45.0)
    ev = [(int(m["t_ns"]), 0, "pose", m) for m in poses_msgs] + [(int(u["t_ns"]), 1, "kf", u) for u in kfu]
    med_age = float(np.median(list(age.values()))) if age else 0.5
    for f in frames:
        if f["dets"]:
            ev.append((int(f["t_ns"]) + int(age.get(int(f["t_ns"]), med_age) * 1e9), 2, "sam", f))
    ev.sort(key=lambda e: (e[0], e[1]))
    t0 = ev[0][0]
    pb = PoseBuffer(max_gap_s=0.25); pb0 = PoseBuffer(max_gap_s=0.25)
    mems = {"slam": RecMemory(Fusion(X, pb), dep.K, (W, H), depth_check=dc, assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None),
            "noslam": RecMemory(Fusion(X, pb0), dep.K, (W, H), depth_check=dc, assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None)}
    dets_log = {k: [] for k in mems}       # (t, name, decision, oid, first_seen, lm position before [or None], measured position)
    cams = []                              # (t, T_w_samcam) live
    snaps, snap_t = [], (np.arange(t0, ev[-1][0], int(1e9 / a.video_hz)) if a.video_hz > 0 else np.array([], np.int64))
    si = 0
    recent = []                            # event texts for the video
    for t, _, kind, m in ev:
        if kind == "pose":
            if m.get("T_wc") is None or m.get("state") not in ("OK", "OK_KLT"):
                continue
            pb.add(t, m.get("state"), m.get("T_wc"), m.get("map_id"), kf=m.get("ref_kf"), T_w_kf=m.get("T_w_kf"),
                   kf_map_id=m.get("kf_map_id"), map_changed=bool(m.get("map_changed")))
            pb0.add(t, m.get("state"), np.eye(4).tolist(), 0)
            T, _ = pb.at(t)
            if T is not None:
                cams.append((t, T @ X))
            while si < len(snap_t) and snap_t[si] <= t:
                tq = int(snap_t[si]); si += 1
                Tq, _ = pb.at(tq)
                if Tq is None:
                    continue
                snaps.append({"t": tq, "T_cam": Tq @ X,
                              "lms": [(m_["object_id"], m_["name"], m_["T_w_obj"], m_["status"], m_["n_obs"]) for m_ in mems["slam"].landmarks()
                                      if m_["status"] in KEEP + ("tentative",)],
                              "events": [e for e in recent if tq - e[0] < 3e9]})
        elif kind == "kf":
            pb.apply_kf_update(m.get("map_id"), m.get("kfs", []))
        else:
            s = int(m["t_ns"])
            depth = dep.at(s)
            dets = [(x["object"], x.get("score", 0.0), x["R"], x["t_mm"]) for x in m["dets"]]
            for key, mem in mems.items():
                before = {mm["object_id"]: mm for mm in mem.landmarks()}
                mem.add_frame(s, dets, depth=depth)
                mem.update()
                rec = mem.last(s)
                if rec is None:
                    continue
                fdets, fr, st = rec
                Tws, _ = mem.fusion.T_w_sam(s)
                for dec in fr.decisions:
                    if dec.object_id is None:
                        continue
                    nm, _, Tco = fdets[dec.detection_id % 100]
                    pm = (Tws @ Tco)[:3, 3] if Tws is not None else None
                    b = before.get(dec.object_id)
                    dets_log[key].append((s, nm, getattr(dec.decision_type, "value", str(dec.decision_type)), dec.object_id,
                                          st.get(dec.object_id, (None, None))[1], None if b is None else b["T_w_obj"][:3, 3].copy(), pm,
                                          {i: mm["T_w_obj"][:3, 3].copy() for i, mm in before.items() if mm["name"] == nm}))
    cam_t = np.array([c[0] for c in cams]); cam_T = [c[1] for c in cams]

    ref_pos = {m_["name"]: m_["T_w_obj"][:3, 3].copy() for m_ in sorted(mems["slam"].landmarks(), key=lambda z: z["n_obs"])
               if m_["status"] in KEEP}              # SLAM-map position per class (one object per class): decides in/out of view

    def events(log):
        out = []
        by = {}
        for r in log:
            by.setdefault(r[1], []).append(r)
        for nm, seq in by.items():
            for r1, r2 in zip(seq, seq[1:]):
                t1, t2 = r1[0], r2[0]
                if (t2 - t1) / 1e9 < GAP_S:
                    continue
                p_old = ref_pos.get(nm)
                lo, hi = np.searchsorted(cam_t, t1), np.searchsorted(cam_t, t2)
                out_view = any(not in_view(cam_T[k], p_old, K, W, H) for k in range(lo, hi, 3)) if p_old is not None else False
                existing = [p for oid, p in r2[7].items()]
                ok = r2[2] != "new_tentative" and r2[4] is not None and r2[4] * 1e9 <= t1 + 1
                disp = None
                if r2[6] is not None and existing:
                    disp = float(min(np.linalg.norm(r2[6] - p) for p in existing)) * 100
                out.append({"obj": nm, "t1": round((t1 - t0) / 1e9, 2), "t2": round((t2 - t0) / 1e9, 2), "gap_s": round((t2 - t1) / 1e9, 2),
                            "cause": "out" if out_view else "in", "ok": bool(ok), "decision": r2[2], "had_map_object": bool(existing),
                            "disp_cm": disp})
        return sorted(out, key=lambda e: e["t2"])

    res = {"run": str(d), "gap_s": GAP_S, "duration_s": round((ev[-1][0] - t0) / 1e9, 1)}
    for key, mem in mems.items():
        evs = events(dets_log[key])
        lms = [m_ for m_ in mem.landmarks() if m_["status"] in KEEP]
        per = {}
        for m_ in lms:
            per[m_["name"]] = per.get(m_["name"], 0) + 1
        summ = {}
        for c in ("all", "out", "in"):
            e = [x for x in evs if (c == "all" or x["cause"] == c) and x["had_map_object"]]
            summ[c] = {"n": len(e), "ok": sum(x["ok"] for x in e), "rate": round(sum(x["ok"] for x in e) / len(e), 4) if e else None,
                       "disp_cm_median": float(np.median([x["disp_cm"] for x in e if x["ok"] and x["disp_cm"] is not None])) if any(x["ok"] for x in e) else None}
        res[key] = {"events": evs, "summary": summ, "final_instances": len(lms), "final_duplicates": sum(v - 1 for v in per.values()),
                    "ids_created": len({r[3] for r in dets_log[key]})}
    # event marks for the video: the SLAM map's re-detections after an out-of-view gap
    for e in res["slam"]["events"]:
        if e["cause"] == "out" and e["had_map_object"]:
            tt = t0 + int(e["t2"] * 1e9)
            for sn in snaps:
                if 0 <= sn["t"] - tt < 3e9:
                    sn["events"].append((tt, e["obj"], e["gap_s"], e["ok"], e["disp_cm"]))
    return res, snaps, t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run"); ap.add_argument("--x", required=True); ap.add_argument("--session", required=True)
    ap.add_argument("--tau-s", type=float, default=0.0); ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--video-hz", type=float, default=10.0)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    res, snaps, t0 = run(a)
    (a.out / "reassoc_real.json").write_text(json.dumps(res, indent=1, ensure_ascii=False))
    with open(a.out / "snaps.pkl", "wb") as f:
        pickle.dump({"snaps": snaps, "t0": t0, "session": a.session, "tau_s": a.tau_s, "x": a.x}, f)
    for k in ("slam", "noslam"):
        print(k, json.dumps(res[k]["summary"], ensure_ascii=False), "final dup", res[k]["final_duplicates"], "ids", res[k]["ids_created"])


if __name__ == "__main__":
    main()
