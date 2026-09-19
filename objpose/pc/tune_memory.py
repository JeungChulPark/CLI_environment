#!/usr/bin/env python3
"""tune_memory.py — replay a finished run through the object memory with other settings.

The memory is a Bernoulli existence filter: a landmark that is inside the camera view and
not re-detected has its existence probability pushed down every frame, and once it falls far
enough the landmark is retired. How fast that happens is set by P_D, the detection rate the
filter EXPECTS of a visible object, so P_D has to match what the detector actually does on
this data or healthy objects are thrown away as phantoms.

A run already stores everything the memory consumed — the SAM-6D detections, their frames,
and the SLAM poses with the final keyframe corrections — so the whole lifecycle can be
replayed offline in a second, without the Mac, SAM-6D or the GPU. That makes the settings
measurable instead of guessed: this replays a run under each setting and reports how long
each object stays displayable, and how many landmarks appear that belong to no real object.

    python objpose/pc/tune_memory.py objpose/output/<run> --pd 0.5 0.15 0.08
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "objectmemory_ws" / "object_memory" / "src"))
from core.models import Sam6DDetection, SlamCameraPose, TrackingStatus  # noqa: E402
from fusion import inv_se3  # noqa: E402
from pipeline.object_memory_runner import StreamingObjectMemory  # noqa: E402

DISPLAYED = ("active", "lost", "remembered")     # what hub.memory_rows puts on screen


def mat(v) -> np.ndarray:
    """4x4 from 16 floats, or from the 12 of a keyframe row (bottom row implied)."""
    a = np.asarray(v, np.float64).ravel()
    if a.size == 12:
        a = np.concatenate([a, [0.0, 0.0, 0.0, 1.0]])
    return a.reshape(4, 4)


def load_run(run: Path, extrinsic: str | None = None):
    """(frames, K, img_size) — one entry per SAM-6D processed frame, in time order."""
    S = json.loads((run / "summary.json").read_text())
    if extrinsic:
        X = mat(json.loads(Path(extrinsic).read_text())["T_slam_sam"])
    elif S.get("X_slam_sam"):
        X = mat(S["X_slam_sam"])
    elif S.get("extrinsic_path"):
        p = Path(S["extrinsic_path"])
        X = mat(json.loads((p if p.is_absolute() else REPO / p).read_text())["T_slam_sam"])
    else:                      # runs from before the hub recorded the rig it was given
        raise SystemExit(f"{run.name}: summary.json has no rig; pass --extrinsic <X_*.json>")

    kfu_path = run / "kf_updates.jsonl"
    kfu = [json.loads(l) for l in open(kfu_path)] if kfu_path.exists() else []
    last_kf = {int(r[0]): mat(r[1:]) for r in kfu[-1]["kfs"]} if kfu else {}
    t_p, T_p = [], []
    for line in open(run / "slam_poses.jsonl"):
        m = json.loads(line)
        if m.get("state") not in ("OK", "OK_KLT") or m.get("T_wc") is None:
            continue
        T = mat(m["T_wc"])
        ref = m.get("ref_kf")
        if ref is not None and m.get("T_w_kf") is not None and int(ref) in last_kf:
            T = last_kf[int(ref)] @ np.linalg.inv(mat(m["T_w_kf"])) @ T
        t_p.append(int(m["t_ns"]))
        T_p.append(T @ X)                       # SAM-camera pose in the SLAM world
    order = np.argsort(t_p)
    t_p, T_p = np.asarray(t_p)[order], np.asarray(T_p)[order]

    dets: dict[int, list] = {}
    for line in open(run / "sam6d_estimates.jsonl"):
        e = json.loads(line)
        T = np.eye(4)
        T[:3, :3] = np.asarray(e["R"], float)
        T[:3, 3] = np.asarray(e["t_mm"], float) / 1000.0
        dets.setdefault(int(e["t_ns"]), []).append((e["object"], float(e.get("score") or 0.0), T))

    stamps = sorted({int(json.loads(l)["t_ns"]) for l in open(run / "sam6d_frames.jsonl")})
    frames = []
    for t in stamps:
        i = int(np.argmin(np.abs(t_p - t)))
        pose = T_p[i] if abs(t_p[i] - t) <= 60_000_000 else None
        frames.append((t, pose, dets.get(t, [])))
    K = np.asarray(S.get("K") or [[386.813, 0, 322.784], [0, 386.242, 250.28], [0, 0, 1]], float)
    return frames, [list(map(float, r)) for r in K], (640, 480), S


def replay(frames, K, img_size, **kw) -> dict:
    mem = StreamingObjectMemory(cam_K=K, img_size=img_size,
                                assoc_trans_gate_m=0.15, assoc_rot_gate_deg=None, **kw)
    shown: dict[str, int] = {}
    spans: dict[str, list] = {}
    for idx, (t, pose, rows) in enumerate(frames):
        stamp = t / 1e9
        ds = [Sam6DDetection(stamp=stamp, frame_id="sam_camera", detection_id=idx * 100 + j,
                             object_name=n, T_cam_obj=tuple(map(tuple, T)), score=s)
              for j, (n, s, T) in enumerate(rows)]
        if pose is None:
            mem.step_no_pose(ds, stamp, idx)
        else:
            mem.step(ds, SlamCameraPose(stamp=stamp, frame_id="map", child_frame_id="sam_camera",
                                        T_map_cam=tuple(map(tuple, pose)),
                                        tracking_status=TrackingStatus.OK,
                                        source_slam_id="replay"), stamp, idx)
        live = {lm.object_name for lm in mem.store.all_landmarks()
                if getattr(lm.status, "value", lm.status) in DISPLAYED}
        for n in live:
            shown[n] = shown.get(n, 0) + 1
            spans.setdefault(n, []).append(idx)
    lms = [{"name": lm.object_name, "status": getattr(lm.status, "value", lm.status),
            "n_obs": lm.observation_count, "r": round(float(lm.confidence), 3),
            "p": [round(float(lm.T_map_obj[i][3]), 3) for i in range(3)]}
           for lm in mem.store.all_landmarks()]
    return {"shown": shown, "spans": spans, "landmarks": lms, "n_frames": len(frames)}


def count_spans(idxs: list[int], gap: int = 3) -> int:
    n = 1
    for a, b in zip(idxs, idxs[1:]):
        if b - a > gap:
            n += 1
    return n


def phantoms(lms: list[dict], truth: dict[str, np.ndarray], tol=0.25) -> int:
    """Kept landmarks whose position matches no estimate cluster of their own class."""
    bad = 0
    for l in lms:
        if l["status"] not in DISPLAYED:
            continue
        p = truth.get(l["name"])
        if p is None or np.linalg.norm(np.asarray(l["p"]) - p) > tol:
            bad += 1
    return bad


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--pd", type=float, nargs="+", default=[0.5, 0.25, 0.15, 0.10, 0.05])
    ap.add_argument("--clutter", type=float, nargs="+", default=[0.1],
                    help="false-alarm likelihood. A detection is evidence FOR existence only "
                         "while P_D > clutter, so lowering P_D without this weakens every hit")
    ap.add_argument("--min-obs", type=int, nargs="+", default=[5])
    ap.add_argument("--max-range", type=float, nargs="+", default=[0.0],
                    help="depth (m) beyond which a miss is no evidence (P_D = 0); 0 = no limit")
    ap.add_argument("--extrinsic", default="",
                    help="rig json, for runs made before the hub recorded it")
    a = ap.parse_args()
    run = Path(a.run)
    frames, K, size, S = load_run(run, a.extrinsic or None)
    truth = {}
    for line in open(run / "sam6d_estimates.jsonl"):
        e = json.loads(line)
        if e.get("placed") and e.get("T_w_obj"):      # unplaced estimates carry no world pose
            truth.setdefault(e["object"], []).append(mat(e["T_w_obj"])[:3, 3])
    truth = {k: np.median(np.asarray(v), 0) for k, v in truth.items()}
    names = sorted(truth)
    print(f"{run.name}: SAM-6D 처리 {len(frames)} 프레임, 객체 {len(names)}종\n")
    head = f"{'설정':<26}" + "".join(f"{n[:10]:>11}" for n in names) + f"{'유령':>6}{'랜드마크':>9}"
    print(head)
    for mo in a.min_obs:
        for cl in a.clutter:
            for pd in a.pd:
                for mr in a.max_range:
                    r = replay(frames, K, size, pd_base=pd, clutter_ratio=cl, min_obs_longterm=mo,
                               pd_max_range_m=mr or None)
                    cells = ""
                    for n in names:
                        sh = r["shown"].get(n, 0)
                        sp = count_spans(r["spans"][n]) if n in r["spans"] else 0
                        cells += f"{100*sh/r['n_frames']:8.0f}%/{sp:<2d}"
                    kept = sum(1 for l in r["landmarks"] if l["status"] in DISPLAYED)
                    label = f"P_D {pd:.2f} c {cl:.2f} obs≥{mo}" + (f" ≤{mr:.1f}m" if mr else "")
                    print(f"{label:<26}{cells}{phantoms(r['landmarks'], truth):6d}{kept:9d}")
    print("\n칸 = 표시된 프레임 비율 / 끊긴 구간 수   (hub 기본값: P_D 0.08, c 0.01, obs≥5)")
    print(f"객체 {len(names)}종이므로 랜드마크가 {len(names)}개를 넘으면 중복/유령이 생긴 것")


if __name__ == "__main__":
    main()
