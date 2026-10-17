#!/usr/bin/env python3
"""render_examples.py — crops showing what pose verification rejected / kept (GT box runs).
Green outline = GT silhouette, orange dots = the unverified geometric top-1 pose."""
import json, os, sys
from pathlib import Path
import numpy as np, cv2

REPO = Path(__file__).resolve().parents[3]
OUT = Path(os.environ["CMP_OUT"])
ROOT = {"ycbv": Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video/test"),
        "lmo": Path("/home/jucpark/DeepLearning/Datasets/paper_benchmarks/bop/lmo/test")}
TAB = {"ycbv": "ycbv_objects.json", "lmo": "lmo_objects.json"}
PTS = REPO / "sam6d_realtime/assets/model_points"


def pose_of(ds, sc, im, nm, run="proposednoverify"):
    for l in (OUT / f"{run}_{ds}_bop19" / sc / "sam6d_frames.jsonl").read_text().splitlines():
        f = json.loads(l)
        if f["im_id"] == im:
            for d in f["dets"]:
                if d["object"] == nm:
                    T = np.eye(4); T[:3, :3] = np.reshape(d["R"], (3, 3)); T[:3, 3] = d["t_mm"]; return T


def render(ds, sc, im, nm, size=320):
    scene = ROOT[ds] / sc
    cam = json.loads((scene / "scene_camera.json").read_text())[str(im)]
    K = np.reshape(cam["cam_K"], (3, 3))
    img = cv2.imread(str(next((scene / "rgb").glob(f"{im:06d}.*"))))
    oid = {o["name"]: o["obj_id"] for o in json.loads((REPO / "objpose/rt/bop" / TAB[ds]).read_text())["objects"]}[nm]
    gt = [r for r in json.loads((scene / "scene_gt.json").read_text())[str(im)] if r["obj_id"] == oid][0]
    G = np.eye(4); G[:3, :3] = np.reshape(gt["cam_R_m2c"], (3, 3)); G[:3, 3] = gt["cam_t_m2c"]
    E = pose_of(ds, sc, im, nm)
    p = np.load(PTS / f"{nm}.npy").astype(float)
    def proj(T):
        c = p @ T[:3, :3].T + T[:3, 3]; uv = c[:, :2] / c[:, 2:3]
        return (uv @ K[:2, :2].T + K[:2, 2]).astype(int)
    ug = proj(G); ue = proj(E)
    h, w = img.shape[:2]
    m = np.zeros((h, w), np.uint8)
    for u, v in ug:
        if 0 <= u < w and 0 <= v < h: cv2.circle(m, (u, v), 3, 255, -1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    ov = img.copy()
    for u, v in ue[::2]:
        if 0 <= u < w and 0 <= v < h: cv2.circle(ov, (u, v), 2, (0, 140, 255), -1)
    img = cv2.addWeighted(ov, 0.55, img, 0.45, 0)
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, cs, -1, (60, 200, 60), 2, cv2.LINE_AA)
    x, y, bw, bh = cv2.boundingRect(m)
    s = int(max(bw, bh) * 1.5); cx, cy = x + bw // 2, y + bh // 2
    x0, y0 = max(0, cx - s // 2), max(0, cy - s // 2)
    crop = img[y0:min(h, y0 + s), x0:min(w, x0 + s)]
    return cv2.resize(crop, (size, size))


if __name__ == "__main__":
    ss = json.loads((OUT / "soft_split.json").read_text())
    ds = sys.argv[1]; cat = sys.argv[2]; n = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    ex = ss[f"examples_{ds}"][cat]
    rng = np.random.default_rng(1); pick = [ex[i] for i in rng.choice(len(ex), min(n, len(ex)), replace=False)]
    tiles = []
    for sc, im, nm, vis in pick:
        t = render(ds, sc, im, nm, 240)
        cv2.putText(t, f"{sc} {im} {nm[:14]} v{vis:.2f}", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        tiles.append(t)
    while len(tiles) % 4: tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(sys.argv[4], np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]))
