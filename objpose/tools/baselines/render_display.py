#!/usr/bin/env python3
"""render_display.py — what the Object-Pose-Map screen shows at a given frame, per method.
Green = ground-truth box, colour = box drawn from the map with that frame's SLAM pose.
    python3 render_display.py <scene> <out.jpg> <frame> [<frame> ...]
"""
import json, os, sys
from pathlib import Path
import numpy as np, cv2

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import display_eval as DE                                          # noqa: E402
BM = DE.BM
RUNS = [("원본 SAM-6D", Path(os.environ.get("ORIG_RUN", REPO / "objpose/output/cmp_260924/orig_ycbv_realtime_top1")), (43, 130, 217)),
        ("제안 방법", Path(os.environ.get("PROP_RUN", REPO / "objpose/output/cmp_260924_gt/proposed_ycbv_realtime")), (180, 111, 31))]
EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def corners(nm):
    p = np.load(BM.PTS / f"{nm}.npy") / 1000.0
    lo, hi = p.min(0), p.max(0)
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def draw(img, T, nm, K, col, th):
    c = corners(nm) @ T[:3, :3].T + T[:3, 3]
    if np.any(c[:, 2] < 0.05): return
    uv = (c[:, :2] / c[:, 2:3]) @ K[:2, :2].T + K[:2, 2]
    for a, b in EDGES:
        cv2.line(img, tuple(np.int32(uv[a])), tuple(np.int32(uv[b])), col, th, cv2.LINE_AA)


def main():
    scene, out, frames = sys.argv[1], sys.argv[2], [int(v) for v in sys.argv[3:]]
    table = json.loads(BM.TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    gt = json.loads((BM.YCBV / scene / "scene_gt.json").read_text()); gi = json.loads((BM.YCBV / scene / "scene_gt_info.json").read_text())
    reps = [(nm, DE.replay(scene, run, by_id), col) for nm, run, col in RUNS]
    hold_frames = sorted((json.loads(l) for l in (RUNS[0][1] / scene / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()),
                         key=lambda f: f["done_s"])
    rows = []
    for k in frames:
        tiles = []
        ids0, _, K0, _ = reps[0][1]
        img = cv2.imread(str(BM.YCBV / scene / "rgb" / f"{int(ids0[k]):06d}.png"))
        for r, i in zip(gt[ids0[k]], gi[ids0[k]]):
            if i.get("visib_fract", 1) >= 0.1 and r["obj_id"] in by_id:
                T = np.eye(4); T[:3, :3] = np.reshape(r["cam_R_m2c"], (3, 3)); T[:3, 3] = np.asarray(r["cam_t_m2c"]) / 1000.0
                draw(img, T, by_id[r["obj_id"]], K0, (60, 200, 60), 3)
        last = [f for f in hold_frames if f["done_s"] <= k / DE.FPS]
        if last:
            for d in last[-1]["dets"]:
                T = np.eye(4); T[:3, :3] = np.reshape(d["R"], (3, 3)); T[:3, 3] = np.asarray(d["t_mm"]) / 1000.0
                draw(img, T, d["object"], K0, RUNS[0][2], 2)
        tiles.append(img)
        for nm, (ids, slam, K, snaps), col in reps:
            fid = ids[k]
            img = cv2.imread(str(BM.YCBV / scene / "rgb" / f"{int(fid):06d}.png"))
            for r, i in zip(gt[fid], gi[fid]):
                if i.get("visib_fract", 1) >= 0.1 and r["obj_id"] in by_id:
                    T = np.eye(4); T[:3, :3] = np.reshape(r["cam_R_m2c"], (3, 3)); T[:3, 3] = np.asarray(r["cam_t_m2c"]) / 1000.0
                    draw(img, T, by_id[r["obj_id"]], K, (60, 200, 60), 3)
            done = [s[0] for s in snaps]
            j = int(np.searchsorted(done, k / DE.FPS, side="right")) - 1
            if j >= 0 and k in slam:
                Tcw = np.linalg.inv(slam[k])
                for onm, Tw in snaps[j][1]:
                    draw(img, Tcw @ Tw, onm, K, col, 2)
            tiles.append(img)
        rows.append(np.hstack(tiles))
    cv2.imwrite(out, np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 90])


if __name__ == "__main__":
    main()
