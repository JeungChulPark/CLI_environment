"""Proposed method on the YCB-V sequences: the live object-pose hub with GT camera poses in place
of SLAM, then its final object memory written as two common-format maps.

run     per scene: objpose/pc/hub.py replays the YCB-V bag at 1x and runs the recogniser (improvement 3
        + cluster-first: FastSAM-x > ViT-S > ISM 0.5 > our PEM > pose verification, cluster-first,
        consensus 0.3) exactly as ycbv_live/run_hybS05_r2.sh; gt_pose_stream.py replaces ORB-SLAM3 and
        streams gt/ycbv/<scene>/cameras.json. Identity extrinsic, so the hub world = GT world.
        Output: out/ours_runs/<scene>/ (hub run dir) + <scene>.log
export  out/ours_cad/ycbv/<scene>/: one object per final memory landmark (shown statuses, not
        dup-suppressed): label + T_w_o (the CAD surface is placed at that pose when scored).
        out/ours_obs/ycbv/<scene>/: per landmark, the depth inside the ISM mask of every accepted
        detection associated with it, back-projected with the GT camera of that frame. The
        memory does not log which detection fed which landmark, so a detection is assigned to the
        nearest landmark of its class within the memory's association gate (0.15 m).

    python run_ours.py run --scenes 52           # GPU part, wrap in flock when the GPU is shared
    python run_ours.py export --scenes 48 ... 59

Code: OBJPOSE_CODE (default: this checkout) is the CLI_environment tree whose objpose/ and
sam6d_realtime/ run; cluster-first needs commit 0c539f3 or later.
"""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
from PIL import Image

from common import OUT, fps, save_map, seq_cameras, seq_name, transform
import seq_bags

HERE = Path(__file__).resolve().parent
CODE = Path(os.environ.get("OBJPOSE_CODE", HERE.parents[2]))
BAGS = Path(os.environ.get("YCBV_BAGS", Path.home() / "DeepLearning/Dataset/bop/ycbv_bags"))
ISM = Path(os.environ.get("YCBV_ISM", Path.home() / "DeepLearning/Dataset/bop/ycbv_work/ycbv_objects.yaml"))
PY = os.environ.get("SAM6D_PY", str(Path.home() / "anaconda3/envs/sam6d/bin/python"))
RUNS = OUT / "ours_runs"
T0 = 1_700_000_000_000_000_000
SHOWN = ("active", "lost", "remembered")
ASSOC_GATE_M = 0.15
# improvement 3 (ycbv_live/run_hybS05_r2.sh) + cluster-first
RECOG_ENV = {"OBJPOSE_LIVE_MODE": "hybrid", "OBJPOSE_ORIG_DESC": "dinov2_vits14", "OBJPOSE_ORIG_THRESH": "0.5",
             "FASTSAM_X": os.environ.get("FASTSAM_X", str(Path.home() / "DeepLearning/Dataset/bop/ycbv_work/FastSAM-x.pt"))}


def snapshot_times(dataset, s):
    """Capture time of the last frame of every visit (continuous ObChange run), on the replay clock."""
    if not dataset.endswith("_all"):
        return []
    cams = seq_cameras(dataset, s)
    fr = sorted(cams["frames"], key=lambda f: f["id"])
    last = {}
    for n, f in enumerate(fr):
        last[f["visit"]] = n
    return [(v, T0 + int(round(n * 1e9 / fps(cams)))) for v, n in sorted(last.items())]


def run_scene(s, http_port, slam_port, dataset="ycbv"):
    sc = seq_name(dataset, s)
    out = RUNS / sc
    out.mkdir(parents=True, exist_ok=True)
    bags = BAGS / f"ycbv_{s:06d}" if dataset == "ycbv" else seq_bags.write(dataset, s)
    env = {**os.environ, **RECOG_ENV}
    hub_cmd = [PY, "-u", str(CODE / "objpose/pc/hub.py"), "--slam", "orbslam3",
               "--extrinsic", str(CODE / "objpose/rt/ycbv/X_identity.json"),
               "--sam-session", str(bags / "SAM"), "--slam-session", str(bags / "SLAM"),
               "--out", str(out), "--no-mac", "--ism-config", str(ISM), "--cluster-occupancy", "0.3",
               "--start-s", "0.0", "--tail-s", "4", "--recognizer", str(CODE / "paper_etrij_261003/src/realdata/live_cores.py"),
               "--verify-cluster-first", "--exit-when-done",
               "--http-port", str(http_port), "--slam-port", str(slam_port)]
    snaps = snapshot_times(dataset, s)
    cams = seq_cameras(dataset, s) if dataset != "ycbv" else None
    wait_s = 3600 if cams is None else max(3600, 1.3 * len(cams["frames"]) / fps(cams) + 600)
    if snaps:
        hub_cmd += ["--memory-snapshots", ",".join(str(t) for _, t in snaps)]
    t = time.time()
    with open(RUNS / f"{sc}.log", "w") as lf:
        hub = subprocess.Popen(hub_cmd, stdout=lf, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        gt = subprocess.Popen([PY, "-u", str(HERE / "gt_pose_stream.py"), "--scene", str(s), "--dataset", dataset,
                               "--connect", f"127.0.0.1:{slam_port}"], stdout=lf, stderr=subprocess.STDOUT)
        try:
            hub.wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            os.killpg(hub.pid, signal.SIGINT)
            hub.wait(60)
        gt.wait(30) if gt.poll() is None else None
    ok = (out / "summary.json").exists()
    print(f"{sc} {'ok' if ok else 'NO SUMMARY'} {time.time() - t:.0f} s", flush=True)
    return ok


def ism_mask(det, hw):
    sm = det["verify"]["shape_mask"]
    h, w = sm["size"]
    flat = np.zeros(h * w, bool)
    for a, n in sm["rle"]:
        flat[a:a + n] = True
    m224 = flat.reshape(h, w)
    y1, y2, x1, x2 = [int(round(v)) for v in sm["crop_bbox_yxyx"]]
    ch, cw = y2 - y1, x2 - x1
    if ch <= 0 or cw <= 0:
        return None
    yy = (np.arange(ch) * h // ch).clip(0, h - 1)       # nearest, as cv2.INTER_NEAREST
    xx = (np.arange(cw) * w // cw).clip(0, w - 1)
    crop = m224[yy][:, xx]
    full = np.zeros(hw, bool)
    Y1, X1, Y2, X2 = max(y1, 0), max(x1, 0), min(y2, hw[0]), min(x2, hw[1])
    full[Y1:Y2, X1:X2] = crop[Y1 - y1:Y2 - y1, X1 - x1:X2 - x1]
    return full


def export_scene(s, step_px=2, dataset="ycbv"):
    """One run -> the two common-format maps. A continuous ObChange run (obchange_<room>_all, scene = last
    visit) is exported once per visit snapshot, to out/<form>/obchange_<room>_all/<visit>."""
    run = RUNS / seq_name(dataset, s)
    summ = json.load(open(run / "summary.json"))
    if dataset.endswith("_all"):
        snaps = dict((t, v) for v, t in snapshot_times(dataset, s))
        return [export_landmarks(run, summ, sn["landmarks"], dataset, snaps[int(sn["t_ns"])], s, step_px, sn["t_ns"])
                for sn in summ["memory_snapshots"] if snaps[int(sn["t_ns"])] >= 2]
    return export_landmarks(run, summ, summ.get("memory_landmarks", []), dataset, s, s, step_px)


def export_landmarks(run, summ, landmarks, dataset, out_scene, run_scene_, step_px, until_ns=None):
    lms = [l for l in landmarks if l["status"] in SHOWN and not l["dup_suppressed"]]
    cams = seq_cameras(dataset, run_scene_)
    frames = sorted(cams["frames"], key=lambda f: f["id"])
    K = np.array(cams["K"]).reshape(3, 3)
    seq = Path(cams["seq_dir"])
    sc = f"{out_scene:06d}"

    T_w = {l["object_id"]: np.array(l["T_w_obj"]).reshape(4, 4) for l in lms}
    save_map(OUT / "ours_cad" / dataset / sc,
             [{"label": l["name"], "T_w_o": T_w[l["object_id"]], "n_obs": l["n_obs"], "score": l["score"]}
              for l in lms], {"method": "ours", "form": "cad", "run": str(run)})

    pts = {l["object_id"]: [] for l in lms}
    n_det = n_used = 0
    for line in open(run / "sam6d" / "detections.jsonl"):
        d = json.loads(line)
        if not d.get("verify", {}).get("accepted", True) or "shape_mask" not in d.get("verify", {}):
            continue
        n_det += 1
        if until_ns is not None and int(d["stamp_ns"]) > int(until_ns):
            continue
        n = int(round((int(d["stamp_ns"]) - T0) * fps(cams) / 1e9))
        if not 0 <= n < len(frames):
            continue
        T_wc = np.array(frames[n]["T_wc"])
        c_w = transform(T_wc, np.array(d["t_mm"])[None] / 1000.0)[0]
        cand = [(np.linalg.norm(T_w[l["object_id"]][:3, 3] - c_w), l["object_id"]) for l in lms if l["name"] == d["object"]]
        cand = [c for c in cand if c[0] <= ASSOC_GATE_M]
        if not cand:
            continue
        oid = min(cand)[1]
        depth = np.array(Image.open(seq / "depth" / f"{frames[n]['id']:06d}.png"), np.float64) * cams["depth_scale_mm"] / 1000.0
        m = ism_mask(d, depth.shape)
        if m is None:
            continue
        sub = np.zeros_like(m)
        sub[::step_px, ::step_px] = True
        v, u = np.nonzero(m & sub & (depth > 0))
        z = depth[v, u]
        P = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)
        pts[oid].append(transform(T_wc, P))
        n_used += 1
    save_map(OUT / "ours_obs" / dataset / sc,
             [{"label": l["name"], "points": np.concatenate(pts[l["object_id"]]) if pts[l["object_id"]] else np.zeros((0, 3)),
               "n_obs": l["n_obs"]} for l in lms if pts[l["object_id"]]],
             {"method": "ours", "form": "observed", "run": str(run)})
    fr = [json.loads(x) for x in open(run / "sam6d_frames.jsonl")]
    stat = {"dataset": dataset, "scene": out_scene, "landmarks": len(lms), "accepted_dets": n_det, "dets_assigned": n_used,
            "processed_frames": len(fr), "video_frames": len(frames),
            "sam6d_hz": summ.get("sam6d_frames", len(fr)) / max(summ.get("duration_s") or len(frames) / fps(cams), 1e-9)}
    print(json.dumps(stat), flush=True)
    return stat


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["run", "export"])
    ap.add_argument("--scenes", type=int, nargs="*", default=list(range(48, 60)))
    ap.add_argument("--dataset", default="ycbv", help="ycbv | obchange_<room> | obchange_<room>_all (see common.seq_cameras)")
    ap.add_argument("--http-port", type=int, default=8775)
    ap.add_argument("--slam-port", type=int, default=17011)
    a = ap.parse_args()
    if a.cmd == "run":
        for s in a.scenes:
            if (RUNS / seq_name(a.dataset, s) / "summary.json").exists():   # resumable (2026-10-12)
                print(f"{seq_name(a.dataset, s)} skip (summary.json exists)", flush=True)
                continue
            run_scene(s, a.http_port, a.slam_port, a.dataset)
            time.sleep(5)     # let the ports and the shared-memory channel go
    else:
        stats = [export_scene(s, dataset=a.dataset) for s in a.scenes]
        json.dump(stats, open(OUT / "ours_runs" / f"export_stats_{a.dataset}.json", "w"), indent=1)


if __name__ == "__main__":
    main()
