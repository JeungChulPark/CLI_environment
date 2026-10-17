#!/usr/bin/env python3
"""display_eval.py — how well the Object-Pose-Map display matches the ground truth, frame by frame.

For one real-time run (sam6d_frames.jsonl with done_s): the object memory is replayed exactly as
build_memory_maps.py does (ORB-SLAM3 poses, hub defaults). At every video frame k (30 fps) the map
state available at k/30 s (results whose done_s <= k/30) is drawn with the SLAM camera pose of frame k,
i.e. what the screen shows. Each drawn object is compared with the GT pose of that frame:
  match       ADD-S < 0.1 d          (per visible GT instance, visib_fract >= 0.1)
  pos_err     |t_est - t_gt| [mm]    (drawn objects that have a GT instance)
  px_err      projected centre offset [px]
  extra       drawn boxes with no GT counterpart (duplicates, wrong object) per frame
    python3 display_eval.py <name> <run_dir> [--scenes ...]
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM                                     # noqa: E402
from build_memory_maps import (Fusion, PoseBuffer, ObjectMemory, DepthCheckConfig, YCBV, T0_NS, STEP_NS, FPS, KEEP, map_occlusion_kwargs)  # noqa

OUT = Path(__import__("os").environ.get("DISPLAY_OUT", REPO / "objpose/output/cmp_260924_gt/display"))


def replay(scene, run_dir, by_id):
    sc = YCBV / scene
    ids, Twc_gt, gobj, K = BM.gt_scene(sc, by_id)
    n = len(ids)
    slam = BM.load_slam(scene, n)
    poses = PoseBuffer(max_gap_s=0.25)
    for i in sorted(slam):
        poses.add(T0_NS + i * STEP_NS, "OK", slam[i], map_id=0)
    mem = ObjectMemory(Fusion(np.eye(4), poses), K, (640, 480),
                       depth_check=DepthCheckConfig(free_margin_m=0.15, confirm_frames=3, max_speed_mps=0.6, max_rot_dps=45.0),
                       assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None, **map_occlusion_kwargs(by_id.values()))
    import cv2
    frames = [json.loads(l) for l in (run_dir / scene / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()]
    frames.sort(key=lambda f: f.get("done_s", f["idx"]))
    snaps = []                                   # (done_s, [(name, T_w_obj)])
    for f in frames:
        raw = cv2.imread(str(sc / "depth" / f"{f['idx'] + 1:06d}.png"), cv2.IMREAD_UNCHANGED)
        depth = np.rint(raw.astype(np.float32) * 0.1).astype(np.uint16)[::8, ::8]
        mem.add_frame(int(f["t_ns"]), [(d["object"], d["score"], d["R"], d["t_mm"]) for d in f["dets"]], depth=depth)
        mem.update()
        snaps.append((f.get("done_s", f["idx"] / FPS),
                      [(m["name"], np.asarray(m["T_w_obj"], float)) for m in mem.landmarks() if m["status"] in KEEP]))
    return ids, slam, K, snaps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name"); ap.add_argument("run_dir", type=Path)
    ap.add_argument("--scenes", nargs="*", default=None)
    a = ap.parse_args()
    table = json.loads(BM.TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    info = json.loads((YCBV.parent / "models_eval/models_info.json").read_text())
    pts, thr = {}, {}
    for o in table:
        p = np.load(BM.PTS / f"{o['name']}.npy").astype(float)
        if len(p) > 1500: p = p[np.random.default_rng(0).choice(len(p), 1500, replace=False)]
        pts[o["name"]] = (p, cKDTree(p)); thr[o["name"]] = 0.1 * info[str(o["obj_id"])]["diameter"]
    tot = {"inst": 0, "match": 0, "shown": 0, "extra": 0, "frames": 0}
    pos, pix, per_scene, series = [], [], {}, {}
    for scene in (a.scenes or sorted(p.name for p in YCBV.iterdir())):
        ids, slam, K, snaps = replay(scene, a.run_dir, by_id)
        gt = json.loads((YCBV / scene / "scene_gt.json").read_text()); gi = json.loads((YCBV / scene / "scene_gt_info.json").read_text())
        done = [s[0] for s in snaps]
        s_i, s_m, s_n = 0, 0, 0
        ser = []
        for k, fid in enumerate(ids):
            G = {}
            for r, i in zip(gt[fid], gi[fid]):
                if i.get("visib_fract", 1) >= 0.1 and r["obj_id"] in by_id:
                    T = np.eye(4); T[:3, :3] = np.reshape(r["cam_R_m2c"], (3, 3)); T[:3, 3] = np.asarray(r["cam_t_m2c"]) / 1000.0
                    G[by_id[r["obj_id"]]] = T
            s_i += len(G); tot["inst"] += len(G)
            j = int(np.searchsorted(done, k / FPS, side="right")) - 1
            if j < 0 or k not in slam:
                continue
            tot["frames"] += 1
            Tcw = np.linalg.inv(slam[k])
            used = set()
            best = {}
            for nm, Tw in snaps[j][1]:
                Tc = Tcw @ Tw
                if nm not in G:
                    tot["extra"] += 1; continue
                p, tree = pts[nm]
                D = np.linalg.inv(G[nm]) @ Tc
                e = tree.query(p @ D[:3, :3].T + D[:3, 3] * 1000.0)[0].mean()
                if nm in best:
                    tot["extra"] += 1               # a second box for the same object
                    if e >= best[nm][0]:
                        continue
                best[nm] = (e, Tc)
            for nm, (e, Tc) in best.items():
                tot["shown"] += 1
                ok = e < thr[nm]
                ok = bool(ok); tot["match"] += int(ok); s_m += int(ok)
                pos.append(np.linalg.norm(Tc[:3, 3] - G[nm][:3, 3]) * 1000.0)
                def pj(T):
                    c = T[:3, 3]; return np.array([K[0, 0] * c[0] / c[2] + K[0, 2], K[1, 1] * c[1] / c[2] + K[1, 2]])
                if Tc[2, 3] > 0.05:
                    pix.append(float(np.linalg.norm(pj(Tc) - pj(G[nm]))))
                ser.append((k / FPS, nm, float(e)))
        per_scene[scene] = {"inst": s_i, "match": s_m, "match_rate": s_m / max(s_i, 1)}
        series[scene] = ser
        print(scene, per_scene[scene], flush=True)
    res = {"name": a.name, "run": str(a.run_dir), "match_rate": tot["match"] / max(tot["inst"], 1),
           "inst": tot["inst"], "shown_per_frame": tot["shown"] / max(tot["frames"], 1),
           "extra_per_frame": tot["extra"] / max(tot["frames"], 1),
           "pos_err_median_mm": float(np.median(pos)) if pos else None, "pos_err_p90_mm": float(np.percentile(pos, 90)) if pos else None,
           "px_err_median": float(np.median(pix)) if pix else None, "px_err_p90": float(np.percentile(pix, 90)) if pix else None,
           "per_scene": per_scene}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{a.name}.json").write_text(json.dumps(res, indent=1))
    (OUT / f"{a.name}_series.json").write_text(json.dumps(series))
    print(json.dumps({k: v for k, v in res.items() if k != "per_scene"}, indent=1))


if __name__ == "__main__":
    main()
