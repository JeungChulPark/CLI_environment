#!/usr/bin/env python3
"""render_overlay.py — YCB-V 영상 위에 GT 포즈와 SAM-6D 추정 포즈를 3D 박스로 겹쳐 그린다.

  초록  GT 객체 포즈
  파랑  모든 프레임 입력 — 그 프레임의 추정
  주황  실시간 입력 — 그 시각 화면에 떠 있는 결과(가장 최근 결과를 ORB-SLAM3 카메라 이동으로 보정)

산출물:
  <out>/overlay_grid.png            12장면 × (모든 프레임 | 실시간) 한 장씩
  <out>/overlay_<scene>.mp4          좌: 모든 프레임 입력, 우: 실시간 화면 — 영상 전체

    python3 objpose/tools/ycbv_sam6d/render_overlay.py --all objpose/output/ycbv_sam6d_all_260921 \
        --rt objpose/output/ycbv_sam6d_realtime_260921 --slam objpose/output/ycbv_slam_orb_260921 \
        --out objpose/docs/figs_ycbv_sam6d_gt --video 000052 000048
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compare_modes import FPS, PTS_DIR, TABLE, YCBV, load_frames, load_slam, se3  # noqa: E402

GREEN = (60, 200, 60)
BLUE = (178, 111, 31)       # BGR of #1f6fb2
ORANGE = (30, 138, 224)     # BGR of #e08a1e
EDGES = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), (1, 5), (2, 6), (3, 7)]


def corners(pts):
    lo, hi = pts.min(0), pts.max(0)
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def draw_box(img, K, T, cor, color, width, label=None):
    P = cor @ T[:3, :3].T + T[:3, 3]
    if (P[:, 2] <= 10).any():
        return
    uv = (P @ K.T)[:, :2] / P[:, 2:3]
    uv = np.round(uv).astype(int)
    for i, j in EDGES:
        cv2.line(img, tuple(uv[i]), tuple(uv[j]), color, width, cv2.LINE_AA)
    if label:
        x, y = uv[:, 0].min(), uv[:, 1].min() - 6
        cv2.putText(img, label, (int(x), int(max(12, y))), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, label, (int(x), int(max(12, y))), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)


class Scene:
    def __init__(self, s, a, cor, by_id):
        self.dir = YCBV / "test" / s
        cam = json.loads((self.dir / "scene_camera.json").read_text())
        self.ids = sorted(cam, key=int)
        self.K = np.asarray(cam[self.ids[0]]["cam_K"], float).reshape(3, 3)
        gt = json.loads((self.dir / "scene_gt.json").read_text())
        info = json.loads((self.dir / "scene_gt_info.json").read_text())
        self.gt = [[(by_id[r["obj_id"]], se3(r["cam_R_m2c"], r["cam_t_m2c"])) for j, r in enumerate(gt[f])
                    if r["obj_id"] in by_id and info[f][j]["visib_fract"] >= 0.1] for f in self.ids]
        self.all = {f["idx"]: f["dets"] for f in load_frames(a.all / s)}
        rt = sorted(load_frames(a.rt / s), key=lambda f: f["done_s"])
        slam = load_slam(a.slam / f"{s}_frames.txt", len(self.ids))
        self.rt_screen, self.rt_src = [], []
        j = -1
        for k in range(len(self.ids)):
            while j + 1 < len(rt) and rt[j + 1]["done_s"] <= k / FPS:
                j += 1
            if j < 0 or slam[rt[j]["idx"]] is None or slam[k] is None:
                self.rt_screen.append([]); self.rt_src.append(None)
                continue
            C = np.linalg.inv(slam[k]) @ slam[rt[j]["idx"]]
            self.rt_screen.append([(d["object"], C @ se3(d["R"], d["t_mm"])) for d in rt[j]["dets"]])
            self.rt_src.append(rt[j]["idx"])
        self.rt_input = {f["idx"] for f in rt}
        self.cor = cor

    def image(self, k):
        return cv2.imread(str(self.dir / "rgb" / f"{int(self.ids[k]):06d}.png"))

    def panel(self, k, mode, label=True):
        img = self.image(k)
        for nm, T in self.gt[k]:
            draw_box(img, self.K, T, self.cor[nm], GREEN, 3)
        if mode == "all":
            est = [(d["object"], se3(d["R"], d["t_mm"])) for d in self.all.get(k, [])]
            color, title = BLUE, "all frames input"
        else:
            est = self.rt_screen[k]
            color = ORANGE
            src = self.rt_src[k]
            title = "real-time: " + ("new result" if k in self.rt_input and src == k else
                                     f"showing result of {(k - src) / FPS * 1000:.0f} ms ago" if src is not None else "no result yet")
        for nm, T in est:
            draw_box(img, self.K, T, self.cor[nm], color, 2, nm.replace("ycbv_", "") if label else None)
        cv2.rectangle(img, (0, 0), (640, 26), (255, 255, 255), -1)
        cv2.putText(img, f"{title}   t={k / FPS:5.2f}s", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 30, 30), 1, cv2.LINE_AA)
        return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--all", required=True, type=Path)
    ap.add_argument("--rt", required=True, type=Path)
    ap.add_argument("--slam", required=True, type=Path)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--video", nargs="*", default=[])
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    table = json.loads(TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    cor = {o["name"]: corners(np.load(PTS_DIR / f"{o['name']}.npy")) for o in table if (PTS_DIR / f"{o['name']}.npy").exists()}

    # 격자: 장면마다 한가운데 근처에서 실시간 결과가 새로 나오지 않은(= 보정된 결과를 보여 주는) 프레임
    scenes = sorted(p.name for p in (YCBV / "test").iterdir())
    tiles = []
    for s in scenes:
        sc = Scene(s, a, cor, by_id)
        k = len(sc.ids) // 2
        while k in sc.rt_input:
            k += 1
        row = np.concatenate([sc.panel(k, "all"), np.full((480, 8, 3), 255, np.uint8), sc.panel(k, "rt")], 1)
        cv2.putText(row, s, (8, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(row, s, (8, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        tiles.append(row)
    pad = np.full((10, tiles[0].shape[1], 3), 255, np.uint8)
    cols = []
    for c in range(2):
        cols.append(np.concatenate(sum([[t, pad] for t in tiles[c * 6:(c + 1) * 6]], [])[:-1], 0))
    grid = np.concatenate([cols[0], np.full((cols[0].shape[0], 20, 3), 255, np.uint8), cols[1]], 1)
    grid = cv2.resize(grid, None, fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(a.out / "overlay_grid.png"), grid)
    print("overlay_grid.png", grid.shape)

    for s in a.video:
        sc = Scene(s, a, cor, by_id)
        path = a.out / f"overlay_{s}.mp4"
        vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (1288, 520))
        for k in range(len(sc.ids)):
            fr = np.full((520, 1288, 3), 255, np.uint8)
            fr[40:520, 0:640] = sc.panel(k, "all")
            fr[40:520, 648:1288] = sc.panel(k, "rt")
            cv2.putText(fr, f"YCB-Video {s}   green = GT pose   blue = all-frames SAM-6D   orange = real-time SAM-6D (on screen, SLAM-compensated)",
                        (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1, cv2.LINE_AA)
            vw.write(fr)
        vw.release()
        print(path)


if __name__ == "__main__":
    main()
