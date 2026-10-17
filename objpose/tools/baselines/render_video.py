#!/usr/bin/env python3
"""render_video.py — mp4 of what the real-time screen showed, frame by frame, from the real-time run records
(each SAM-6D result appears at its done_s, the map at the result it came from, the camera at that frame's SLAM pose).
Green = ground-truth box.
    python3 render_video.py modes   <scene> <t0_s> <t1_s> <out.mp4>   proposed: last result held | moved by SLAM | Object Pose Map
    python3 render_video.py compare <scene> <t0_s> <t1_s> <out.mp4>   Object Pose Map: original SAM-6D | proposed
"""
import json, subprocess, sys
from pathlib import Path
import numpy as np, cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))
import render_display as RD                                        # noqa: E402
DE, BM = RD.DE, RD.BM
GT_COL = (60, 200, 60)


def gt_draw(img, gt, gi, fid, by_id, K):
    for r, i in zip(gt[fid], gi[fid]):
        if i.get("visib_fract", 1) >= 0.1 and r["obj_id"] in by_id:
            T = np.eye(4); T[:3, :3] = np.reshape(r["cam_R_m2c"], (3, 3)); T[:3, 3] = np.asarray(r["cam_t_m2c"]) / 1000.0
            RD.draw(img, T, by_id[r["obj_id"]], K, GT_COL, 2)


def Tco(d):
    T = np.eye(4); T[:3, :3] = np.reshape(d["R"], (3, 3)); T[:3, 3] = np.asarray(d["t_mm"]) / 1000.0
    return T


def main():
    mode, scene, t0, t1, out = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
    table = json.loads(BM.TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    gt = json.loads((BM.YCBV / scene / "scene_gt.json").read_text()); gi = json.loads((BM.YCBV / scene / "scene_gt_info.json").read_text())
    (_, orun, ocol), (_, prun, pcol) = RD.RUNS
    prop = DE.replay(scene, prun, by_id)
    ids, slam, K, psnaps = prop
    pframes = sorted((json.loads(l) for l in (prun / scene / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()), key=lambda f: f["done_s"])
    pdone = [f["done_s"] for f in pframes]
    if mode == "compare":
        _, _, _, osnaps = DE.replay(scene, orun, by_id)
    fps = min(15, int(DE.FPS))
    frames = []
    for k in range(int(t0 * DE.FPS), int(t1 * DE.FPS), max(1, int(DE.FPS / fps))):
        if k not in slam:
            continue
        fid = ids[k]
        base = cv2.imread(str(BM.YCBV / scene / "rgb" / f"{int(fid):06d}.png"))
        tiles = []
        if mode == "modes":
            j = int(np.searchsorted(pdone, k / DE.FPS, side="right")) - 1
            last = pframes[j] if j >= 0 else None
            # 1 last result held in camera coordinates
            img = base.copy(); gt_draw(img, gt, gi, fid, by_id, K)
            if last:
                for d in last["dets"]:
                    RD.draw(img, Tco(d), d["object"], K, pcol, 2)
            tiles.append(img)
            # 2 last result moved by the SLAM motion since its frame
            img = base.copy(); gt_draw(img, gt, gi, fid, by_id, K)
            if last and last["idx"] in slam:
                M = np.linalg.inv(slam[k]) @ slam[last["idx"]]
                for d in last["dets"]:
                    RD.draw(img, M @ Tco(d), d["object"], K, pcol, 2)
            tiles.append(img)
            snap_sets = [(psnaps, pcol)]
        else:
            snap_sets = [(osnaps, ocol), (psnaps, pcol)]
        for snaps, col in snap_sets:
            img = base.copy(); gt_draw(img, gt, gi, fid, by_id, K)
            done = [s[0] for s in snaps]
            j = int(np.searchsorted(done, k / DE.FPS, side="right")) - 1
            if j >= 0:
                Tcw = np.linalg.inv(slam[k])
                for onm, Tw in snaps[j][1]:
                    RD.draw(img, Tcw @ Tw, onm, K, col, 2)
            tiles.append(img)
        sep = np.full((480, 8, 3), 255, np.uint8)
        row = tiles[0]
        for t in tiles[1:]:
            row = np.hstack([row, sep, t])
        frames.append(row)
    h, w = frames[0].shape[:2]
    w2 = w - (w % 2)
    p = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{w}x{h}', '-r', str(fps), '-i', '-',
                          '-vf', f'crop={w2}:{h}:0:0', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-profile:v', 'main', '-crf', '22',
                          '-movflags', '+faststart', out], stdin=subprocess.PIPE)
    for f in frames:
        p.stdin.write(f.tobytes())
    p.stdin.close(); p.wait()
    mid = frames[len(frames) // 2][:, :w2]
    sheet = [cv2.resize(f[:, :w2], (w2 // 3, h // 3)) for f in frames[::max(1, len(frames) // 12)]][:12]
    cv2.imwrite(out.replace('.mp4', '_sheet.jpg'), np.vstack([np.hstack(sheet[i:i + 3]) for i in range(0, len(sheet) - 2, 3)]))
    np.save(out.replace('.mp4', '_frames.npy'), np.stack(frames[::max(1, len(frames) // 12)][:12]))
    cv2.imwrite(out.replace('.mp4', '_poster.jpg'), mid, [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(len(frames), 'frames', f'{w2}x{h}', out)


if __name__ == "__main__":
    main()
