"""anchor_ablation.py — which time and which keyframe a late SAM-6D estimate should be tied to.

Replays a finished live run (slam_poses.jsonl, kf_updates.jsonl, sam6d_estimates.jsonl) in the
order the hub learned each piece — a pose when it crossed the tunnel (t_ns + recv_lag_s), a keyframe
update when it was sent, an estimate when SAM-6D answered (t_ns + result_age_s) — and places the
same estimates in the map five ways:

  A     arrival pose, world-fixed      camera pose at the moment the result arrives
  A_kf  arrival pose, arrival keyframe as A, but stored against that pose's reference keyframe
  B     capture pose, world-fixed      pose buffer interpolated at the frame's capture time, as
                                       streamed (what a tf2-style lookup at the stamp returns)
  C     capture pose, arrival keyframe B stored against the keyframe the camera tracks when the
                                       result arrives (keyframe anchoring picked at arrival)
  C2    corrected capture pose,        as the hub computes it, but anchored to the arrival
        arrival keyframe               keyframe instead of the capture keyframe
  D     capture keyframe (the hub)     capture pose stored against the capture-time reference
                                       keyframe, restored from its final pose

An estimate whose capture-time poses have not arrived waits for them (A needs no capture pose).
Each variant is then read in the final map (all keyframe updates applied, culled keyframes
redirected) and scored by how tightly the estimates of each object agree: the distance of each
estimate from its object's median position. Every variant scores the same estimates.

    python objpose/pc/anchor_ablation.py objpose/output/live_260901_cbnu_bigeightcircle_orbslam3 ...
"""
from __future__ import annotations

import argparse
import bisect
import glob
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fusion import KeyframeMap, TRACKING_OK, inv_se3, interp_se3, mat  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
VARIANTS = ("A", "A_kf", "B", "C", "C2", "D")
MAX_GAP_NS = 250_000_000


def jsonl(p: Path):
    with open(p) as f:
        return [json.loads(l) for l in f if l.strip()]


def load_X(run: Path) -> np.ndarray:
    s = json.load(open(run / "summary.json"))
    if s.get("X_slam_sam") is not None:
        return mat(s["X_slam_sam"])
    p = Path(s["extrinsic_path"])
    p = p if p.is_absolute() else REPO / p
    x = json.load(open(p))
    return mat(x.get("X_slam_sam") or x.get("T") or x["X"])


def run_one(run: Path, extra_delay_s: float = 0.0) -> dict:
    """extra_delay_s: hold every recognition result this much longer before it arrives (latency sweep)."""
    X = load_X(run)
    poses = [p for p in jsonl(run / "slam_poses.jsonl") if p.get("state") in TRACKING_OK and p.get("T_wc")]
    kfu = jsonl(run / "kf_updates.jsonl")
    ests = [e for e in jsonl(run / "sam6d_estimates.jsonl") if e.get("result_age_s") is not None]

    # a keyframe update leaves the Mac with the pose it follows; it arrives with that pose's lag
    pt = np.array([p["t_ns"] for p in poses])
    lag = np.array([p.get("recv_lag_s", 0.0) for p in poses])

    def lag_at(t):
        k = min(max(int(np.searchsorted(pt, t)), 0), len(pt) - 1)
        return lag[k]

    events = []
    for i, p in enumerate(poses):
        events.append((p["t_ns"] + int(p.get("recv_lag_s", 0.0) * 1e9), 0, i))
    for i, u in enumerate(kfu):
        events.append((u["t_ns"] + int(lag_at(u["t_ns"]) * 1e9), 1, i))
    for i, e in enumerate(ests):
        events.append((e["t_ns"] + int((e["result_age_s"] + extra_delay_s) * 1e9), 2, i))
    events.sort()

    km = KeyframeMap()
    kt: list[int] = []                    # known pose samples, by capture time
    ks: list[tuple] = []                  # (t_ns, T_wc, kf, T_kf_c, map_id)
    latest_arrived = None                 # newest pose to have arrived (the camera "now")
    waiting: list[tuple[int, int]] = []   # (estimate index, arrival time)
    placed: dict[int, dict] = {}
    upd_log: list[tuple[int, float]] = []  # (arrival, max keyframe move in that update)

    def bracket(t):
        k = bisect.bisect_left(kt, t)
        if k < len(kt) and kt[k] == t:
            return ks[k], ks[k], 0.0
        if 0 < k < len(kt) and kt[k] - kt[k - 1] <= MAX_GAP_NS:
            return ks[k - 1], ks[k], (t - kt[k - 1]) / (kt[k] - kt[k - 1])
        return None

    def corrected(s):
        if s[2] is None:
            return s[1]
        T, _ = km.resolve(s[2])
        return s[1] if T is None else T @ s[3]

    def T_obj(e):
        T = np.eye(4)
        T[:3, :3] = np.asarray(e["R"], float).reshape(3, 3)
        T[:3, 3] = np.asarray(e["t_mm"], float) / 1000.0
        return X @ T

    def try_place(i, t_arr, now):
        e = ests[i]
        br = bracket(e["t_ns"])
        if br is None:
            return False
        a, b, u = br
        Tso = T_obj(e)
        raw = a[1] if a is b else interp_se3(a[1], b[1], u)
        cor = corrected(a) if a is b else interp_se3(corrected(a), corrected(b), u)
        cap = a if u < 0.5 else b
        arr = arrival_state[i]
        r = {"obj": e["object"], "t_s": e["t_ns"], "t_arr": t_arr, "t_place": now, "map": cap[4],
             "A": ("world", arr["T_wc"] @ Tso),
             "B": ("world", raw @ Tso)}
        r["A_kf"] = ("kf", arr["kf"], inv_se3(arr["T_w_kf"]) @ arr["T_wc"] @ Tso) if arr["kf"] is not None else r["A"]
        Tka, _ = km.resolve(arr["kf"]) if arr["kf"] is not None else (None, None)
        r["C"] = ("kf", arr["kf"], inv_se3(Tka) @ raw @ Tso) if Tka is not None else r["B"]
        r["C2"] = ("kf", arr["kf"], inv_se3(Tka) @ cor @ Tso) if Tka is not None else ("world", cor @ Tso)
        Tkc, _ = km.resolve(cap[2]) if cap[2] is not None else (None, None)
        r["D"] = ("kf", cap[2], inv_se3(Tkc) @ cor @ Tso) if Tkc is not None else ("world", cor @ Tso)
        placed[i] = r
        return True

    arrival_state: dict[int, dict] = {}
    for t, kind, i in events:
        if kind == 0:
            p = poses[i]
            T = mat(p["T_wc"])
            kf = p.get("ref_kf")
            T_kf_c = None
            if kf is not None and p.get("T_w_kf") is not None:
                Tk = mat(p["T_w_kf"])
                km.set_pose(int(kf), Tk, p.get("kf_map_id", p.get("map_id")))
                T_kf_c = inv_se3(Tk) @ T
            s = (p["t_ns"], T, None if T_kf_c is None else int(kf), T_kf_c, p.get("map_id"))
            k = bisect.bisect_left(kt, p["t_ns"])
            kt.insert(k, p["t_ns"]); ks.insert(k, s)
            if latest_arrived is None or p["t_ns"] >= latest_arrived[0]:
                latest_arrived = (p["t_ns"], T, s[2], None if T_kf_c is None else T @ inv_se3(T_kf_c))
        elif kind == 1:
            u = kfu[i]
            before = {k: km.T[k][:3, 3].copy() for k in km.T}
            km.apply_update(u["map_id"], u["kfs"])
            moved = [np.linalg.norm(km.T[k][:3, 3] - before[k]) for k in before if k in km.T]
            upd_log.append((t, max(moved) if moved else 0.0))
        else:
            if latest_arrived is None:
                continue
            ta, Ta, kfa, Tka = latest_arrived
            arrival_state[i] = {"T_wc": Ta, "kf": kfa, "T_w_kf": Tka}
            if not try_place(i, t, t):
                waiting.append((i, t))
        if waiting:
            waiting = [(j, ta) for j, ta in waiting if not try_place(j, ta, t)]

    # read everything in the final map
    main_map = max(set(s[4] for s in ks), key=[s[4] for s in ks].count)
    out = defaultdict(lambda: defaultdict(list))   # variant -> obj -> [(pos, idx)]
    positions = defaultdict(list)                  # variant -> [(obj, xyz in the final map, idx)]
    meta = {}
    upd_t = np.array([u[0] for u in upd_log]); upd_m = np.array([u[1] for u in upd_log])
    for i, r in placed.items():
        if r["map"] != main_map:
            continue
        pos = {}
        for v in VARIANTS:
            rep = r[v]
            if rep[0] == "world":
                pos[v] = rep[1][:3, 3]
            else:
                T, m = km.resolve(rep[1])
                if T is None:
                    break
                pos[v] = (T @ rep[2])[:3, 3]
        else:
            for v in VARIANTS:
                out[v][r["obj"]].append((pos[v], i))
                positions[v].append((r["obj"], pos[v].tolist(), i))
            win = (upd_t > r["t_s"]) & (upd_t <= r["t_place"])
            meta[i] = {"delay_s": (r["t_arr"] - r["t_s"]) / 1e9, "waited_s": (r["t_place"] - r["t_arr"]) / 1e9,
                       "corr_between_m": float(upd_m[win].max()) if win.any() else 0.0}
    res = {}
    for v in VARIANTS:
        d = {}
        for obj, rows in out[v].items():
            P = np.array([p for p, _ in rows])
            c = np.median(P, axis=0)
            for (p, i) in rows:
                d[i] = float(np.linalg.norm(p - c))
        res[v] = d
    return {"run": run.name, "n_placed": len(meta), "n_est": len(ests), "dist": res, "meta": meta,
            "positions": dict(positions), "kf_final": {int(k): km.T[k][:3, 3].tolist() for k in km.T},
            "kf_max_correction_cm": json.load(open(run / "summary.json")).get("kf_max_correction_cm")}


def summarize(results, subset=None):
    rows = {}
    for v in VARIANTS:
        ds = [r["dist"][v][i] for r in results for i in r["meta"] if subset is None or subset(r["meta"][i])]
        if ds:
            a = np.array(ds) * 100
            rows[v] = (len(a), np.median(a), np.percentile(a, 90), np.mean(a > 10) * 100)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--json", help="write per-run results here")
    a = ap.parse_args()
    runs = [Path(p) for g in a.runs for p in sorted(glob.glob(g))]
    results = []
    for r in runs:
        try:
            results.append(run_one(r))
        except Exception as e:  # a run without keyframe data cannot be compared
            print(f"skip {r.name}: {e}", file=sys.stderr)
    for title, sub in (("all estimates", None),
                       ("map corrected >2 cm between capture and placement", lambda m: m["corr_between_m"] > 0.02),
                       ("result arrived >1 s after capture", lambda m: m["delay_s"] > 1.0)):
        rows = summarize(results, sub)
        print(f"\n{title}   (distance from the object's median position, cm)")
        print(f"  {'variant':6} {'n':>5} {'median':>7} {'p90':>7} {'>10cm %':>8}")
        for v, (n, med, p90, big) in rows.items():
            print(f"  {v:6} {n:5d} {med:7.2f} {p90:7.2f} {big:8.1f}")
    if a.json:
        json.dump([{k: v for k, v in r.items() if k not in ("dist", "positions", "kf_final")} | {"dist": {v: {str(i): d for i, d in r["dist"][v].items()} for v in VARIANTS}}
                   for r in results], open(a.json, "w"))


if __name__ == "__main__":
    main()
