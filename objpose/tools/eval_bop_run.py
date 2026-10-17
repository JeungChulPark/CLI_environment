#!/usr/bin/env python3
"""eval_bop_run.py — score a hub run on a BOP scene against its ground truth.

Two different things are measured, because the system produces two different outputs:

  per-frame pose      every SAM-6D estimate (`sam6d_estimates.jsonl`) against `scene_gt.json`
                      in the camera frame — ADD / ADD-S, and recall at 0.1 x object diameter,
                      the usual YCB-Video operating point. Those rates are conditional on an
                      estimate existing, so they are reported beside **instance recall**: of all
                      ground-truth instances visible in the frames SAM-6D actually processed
                      (`sam6d_frames.jsonl` — the pipeline runs at --sam-feed-hz, not at the
                      video's frame rate), how many got a correct pose. A detector that fires
                      rarely but precisely looks perfect on the first number and poor on this one.
  object memory       the fused world landmarks from `summary.json` against the object's
                      ground-truth world pose. YCB-V objects do not move inside a scene, so a
                      scene has ONE true world pose per object and the memory should converge
                      to it; this is what the per-frame metric cannot show.

The second one needs a world frame. Scenes converted from LineMOD (LM, LM-O) carry no
`cam_R_w2c` and their ground truth is per image, so that section is skipped there and only the
per-frame pose is scored — which is all the BOP protocol asks of those datasets.

    python objpose/tools/eval_bop_run.py --run objpose/output/<run> \
        --scene /.../YCB_Video/test/000048
    python objpose/tools/eval_bop_run.py --run <run> --scene /.../lmo/test/000002 \
        --table objpose/rt/bop/lmo_objects.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
PTS_DIR = REPO / "sam6d_realtime" / "assets" / "model_points"
T0_NS = 1_600_000_000_000_000_000
FPS = 30.0


def se3(R, t):
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float).reshape(3, 3)
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def add_err(pts, T_est, T_gt):
    """mean distance between the two posed point sets [mm]"""
    a = pts @ T_est[:3, :3].T + T_est[:3, 3]
    b = pts @ T_gt[:3, :3].T + T_gt[:3, 3]
    return float(np.linalg.norm(a - b, axis=1).mean())


def adds_err(pts, T_est, T_gt, sub=2000):
    """symmetry-aware: mean nearest-neighbour distance [mm]"""
    from scipy.spatial import cKDTree
    p = pts if len(pts) <= sub else pts[np.random.default_rng(0).choice(len(pts), sub, replace=False)]
    a = p @ T_est[:3, :3].T + T_est[:3, 3]
    b = p @ T_gt[:3, :3].T + T_gt[:3, 3]
    return float(cKDTree(b).query(a)[0].mean())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--table", default=None, help="rt/bop/<dataset>_objects.json "
                                                  "(default: guessed from the scene path)")
    ap.add_argument("--visib-min", type=float, default=0.1, help="ignore GT instances below this visible fraction")
    ap.add_argument("--out", default=None, help="write the report as JSON here too")
    a = ap.parse_args()

    run, scene = Path(a.run), Path(a.scene)
    tpath = Path(a.table) if a.table else (
        REPO / "objpose" / "rt" / "bop" /
        (("lmo" if "/lmo/" in str(scene) else "ycbv") + "_objects.json"))
    meta = json.loads(tpath.read_text())
    table = meta["objects"]
    fps = float(meta.get("fps", FPS))
    by_name = {o["name"]: o for o in table}
    by_id = {o["obj_id"]: o for o in table}

    models_info = json.loads((Path(meta["models_dir"]).parent / "models" / "models_info.json").read_text())
    gt = json.loads((scene / "scene_gt.json").read_text())
    info = json.loads((scene / "scene_gt_info.json").read_text())
    cam = json.loads((scene / "scene_camera.json").read_text())
    ids = sorted(cam, key=int)
    idx_of_frame = {fid: i for i, fid in enumerate(ids)}
    step_ns = int(round(1e9 / fps))
    has_world = "cam_R_w2c" in cam[ids[0]]

    pts = {}
    for o in table:
        p = PTS_DIR / f"{o['name']}.npy"
        if p.exists():
            pts[o["name"]] = np.load(p).astype(np.float64)

    # ── ground truth, per frame index and per object ─────────────────────────
    gt_cam = {}          # (frame_idx, obj_id) -> T_c_m
    gt_world = {}        # obj_id -> T_w_m  (median over frames; YCB-V objects are static)
    world_samples = {}
    for fid, rows in gt.items():
        i = idx_of_frame.get(fid)
        if i is None:
            continue
        T_wc = (np.linalg.inv(se3(cam[fid]["cam_R_w2c"], np.asarray(cam[fid]["cam_t_w2c"], float)))
                if has_world else None)
        for j, r in enumerate(rows):
            if info.get(fid, [{}] * len(rows))[j].get("visib_fract", 1.0) < a.visib_min:
                continue
            T_cm = se3(r["cam_R_m2c"], np.asarray(r["cam_t_m2c"], float))
            gt_cam[(i, r["obj_id"])] = T_cm
            if has_world:
                world_samples.setdefault(r["obj_id"], []).append((T_wc @ T_cm)[:3, 3])
    for oid, v in world_samples.items():
        gt_world[oid] = np.median(np.asarray(v), axis=0)          # mm, in the BOP world frame

    # ── per-frame estimates ──────────────────────────────────────────────────
    est_path = run / "sam6d_estimates.jsonl"
    if not est_path.exists():
        cand = sorted(run.glob("sam6d_estimates*.jsonl"))
        est_path = cand[0] if cand else est_path
    # frames the pipeline actually processed — instance recall is over these, not all 30 fps
    processed = set()
    fpath = run / "sam6d_frames.jsonl"
    if fpath.exists():
        for line in fpath.read_text().splitlines():
            if line.strip():
                t = int(json.loads(line)["t_ns"])
                processed.add(int(round((t - T0_NS) / step_ns)))

    per_obj = {}
    n_rows = 0
    for line in est_path.read_text().splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        n_rows += 1
        name = r["object"]
        o = by_name.get(name)
        if o is None or name not in pts:
            continue
        i = int(round((int(r["t_ns"]) - T0_NS) / step_ns))
        T_gt = gt_cam.get((i, o["obj_id"]))
        d = per_obj.setdefault(name, {"obj_id": o["obj_id"], "n_est": 0, "n_scored": 0,
                                      "n_false": 0, "add": [], "adds": []})
        d["n_est"] += 1
        if T_gt is None:
            d["n_false"] += 1                    # estimate on a frame where this object is not GT-visible
            continue
        T_est = se3(r["R"], np.asarray(r["t_mm"], float))
        d["n_scored"] += 1
        d["add"].append(add_err(pts[name], T_est, T_gt))
        d["adds"].append(adds_err(pts[name], T_est, T_gt))
        d.setdefault("hit_frames", set())
        thr = 0.1 * models_info[str(o["obj_id"])]["diameter"]
        if d["adds"][-1] < thr:
            d["hit_frames"].add(i)

    # ── report ───────────────────────────────────────────────────────────────
    gt_count, gt_proc = {}, {}
    for (i, oid) in gt_cam:
        gt_count[oid] = gt_count.get(oid, 0) + 1
        if not processed or i in processed:
            gt_proc[oid] = gt_proc.get(oid, 0) + 1

    rep = {"run": str(run), "scene": str(scene), "dataset": meta["dataset"], "frames": len(ids),
           "estimate_rows": n_rows, "visib_min": a.visib_min, "has_world": has_world, "objects": {}}
    print(f"\nscene {scene.name}: {len(ids)} frames, {len(gt_count)} objects, "
          f"{n_rows} SAM-6D estimate rows\n")
    hdr = (f"{'object':26s} {'GTfr':>5s} {'proc':>5s} {'est':>5s} {'ADD<0.1d':>9s} "
           f"{'ADD-S<0.1d':>11s} {'inst.rec':>9s} {'ADD-S mm':>9s}")
    print(hdr); print("-" * len(hdr))
    tot = {"scored": 0, "add_ok": 0, "adds_ok": 0, "inst": 0, "inst_hit": 0}
    for name, d in sorted(per_obj.items(), key=lambda kv: kv[1]["obj_id"]):
        diam = models_info[str(d["obj_id"])]["diameter"]
        thr = 0.1 * diam
        add = np.asarray(d["add"]); adds = np.asarray(d["adds"])
        add_ok = int((add < thr).sum()) if len(add) else 0
        adds_ok = int((adds < thr).sum()) if len(adds) else 0
        n = max(len(add), 1)
        rep["objects"][name] = {
            "obj_id": d["obj_id"], "diameter_mm": diam, "gt_frames": gt_count.get(d["obj_id"], 0),
            "estimates": d["n_est"], "scored": d["n_scored"], "off_gt_frames": d["n_false"],
            "add_recall": add_ok / n if len(add) else None,
            "adds_recall": adds_ok / n if len(adds) else None,
            "add_mean_mm": float(add.mean()) if len(add) else None,
            "adds_mean_mm": float(adds.mean()) if len(adds) else None,
            "add_median_mm": float(np.median(add)) if len(add) else None,
            "gt_instances_processed": gt_proc.get(d["obj_id"], 0),
            "instance_recall": (len(d.get("hit_frames", ()))
                                / max(gt_proc.get(d["obj_id"], 0), 1)),
        }
        tot["scored"] += d["n_scored"]; tot["add_ok"] += add_ok; tot["adds_ok"] += adds_ok
        tot["inst"] += gt_proc.get(d["obj_id"], 0); tot["inst_hit"] += len(d.get("hit_frames", ()))
        gp = gt_proc.get(d["obj_id"], 0)
        print(f"{name:26s} {gt_count.get(d['obj_id'],0):5d} {gp:5d} {d['n_est']:5d} "
              f"{(add_ok/n*100 if len(add) else 0):8.1f}% {(adds_ok/n*100 if len(adds) else 0):10.1f}% "
              f"{len(d.get('hit_frames',()))/max(gp,1)*100:8.1f}% "
              f"{(adds.mean() if len(adds) else float('nan')):9.1f}")
    for oid in sorted(gt_count):
        nm = by_id[oid]["name"]
        if nm not in per_obj:
            print(f"{nm:26s} {gt_count[oid]:5d} {gt_proc.get(oid,0):5d} {0:5d} "
                  f"{'—':>9s} {'—':>11s} {0.0:8.1f}% {'—':>9s}")
            rep["objects"].setdefault(nm, {"obj_id": oid, "gt_frames": gt_count[oid], "estimates": 0,
                                           "scored": 0, "add_recall": None, "adds_recall": None,
                                           "gt_instances_processed": gt_proc.get(oid, 0),
                                           "instance_recall": 0.0})
    s = max(tot["scored"], 1)
    rep["overall"] = {"scored": tot["scored"], "add_recall": tot["add_ok"] / s,
                      "adds_recall": tot["adds_ok"] / s,
                      "gt_instances_processed": tot["inst"],
                      "instance_recall": tot["inst_hit"] / max(tot["inst"], 1),
                      "processed_frames": len(processed)}
    print(f"\noverall: {tot['scored']} scored estimates, "
          f"ADD<0.1d {tot['add_ok']/s*100:.1f}%, ADD-S<0.1d {tot['adds_ok']/s*100:.1f}%")
    print(f"instance recall: {tot['inst_hit']}/{tot['inst']} = "
          f"{tot['inst_hit']/max(tot['inst'],1)*100:.1f}% of ground-truth instances in the "
          f"{len(processed)} processed frames got a pose within 0.1d (ADD-S)")

    # ── object memory ────────────────────────────────────────────────────────
    summ_path = run / "summary.json"
    if not has_world:
        print("\nobject memory: not scored — this scene has no world frame "
              "(per-image ground truth, no cam_R_w2c)")
    elif summ_path.exists():
        summ = json.loads(summ_path.read_text())
        lms = summ.get("memory_landmarks") or []
        print(f"\nobject memory: {len(lms)} landmarks")
        rep["memory"] = []
        for m in lms:
            T = np.asarray(m["T_w_obj"], float).reshape(4, 4)
            name = m.get("name") or m.get("object") or m.get("cls")
            o = by_name.get(name)
            gtw = gt_world.get(o["obj_id"]) if o else None
            err_mm = float(np.linalg.norm(T[:3, 3] * 1000.0 - gtw)) if gtw is not None else None
            rep["memory"].append({"name": name, "obs": m.get("n_obs") or m.get("observations"),
                                  "pos_err_mm": err_mm})
            print(f"  {str(name):26s} obs={m.get('n_obs') or m.get('observations')}  "
                  f"world position error = {'—' if err_mm is None else f'{err_mm:7.1f} mm'}")
        errs = [r["pos_err_mm"] for r in rep["memory"] if r["pos_err_mm"] is not None]
        if errs:
            print(f"  median landmark position error: {np.median(errs):.1f} mm  (n={len(errs)})")
            rep["memory_median_err_mm"] = float(np.median(errs))

    if a.out:
        Path(a.out).write_text(json.dumps(rep, indent=1))
        print(f"\nreport -> {a.out}")


if __name__ == "__main__":
    main()
