#!/usr/bin/env python3
"""render_overlay.py — ObChange 방문마다 실제 카메라 영상 위에 정답과 세 방법의 지도 객체를 겹쳐 그린다.

  프레임   채점 대상 정답 물체가 가장 많이 보이는 프레임 하나 + 거기서 안 보인 물체가 가장 많이 보이는 프레임 하나
  칸       정답 | 제안 방법 | FM-Fusion | DualMap  (같은 프레임)
  객체     대상 물체 이름이 붙은 지도 객체를 그 프레임에 투영한 윤곽. 제안 방법은 저장된 자세의 CAD 점, 나머지는 복원 점(깊이 영상과 3 cm 안에서 맞는 점만).
  색       초록 = 정답과 같은 이름이고 정답 중심 10 cm 안, 주황 = 정답 자리(10 cm 안)인데 다른 이름, 빨강 = 정답이 없는 자리
    ~/anaconda3/envs/dualmap/bin/python objpose/tools/obchange/render_overlay.py --maps <out>/maps_final --room kitchen --out <dir>
"""
import argparse, json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parents[3]
OB = Path("/home/jucpark/DeepLearning/Datasets/ObChange/bop"); PTS = REPO / "sam6d_realtime/assets/model_points"
CATALOG = ["002_master_chef_can", "003_cracker_box", "004_sugar_box", "005_tomato_soup_can", "008_pudding_box", "009_gelatin_box",
           "019_pitcher_base", "024_bowl", "025_mug", "040_large_marker", "061_foam_brick"]
KO = {"002_master_chef_can": "커피 캔", "003_cracker_box": "크래커 상자", "004_sugar_box": "설탕 상자", "005_tomato_soup_can": "수프 캔", "008_pudding_box": "푸딩 상자",
      "009_gelatin_box": "젤라틴 상자", "019_pitcher_base": "피처", "024_bowl": "그릇", "025_mug": "머그컵", "040_large_marker": "마커", "061_foam_brick": "스펀지 벽돌",
      "027_skillet": "프라이팬", "029_plate": "접시"}
METHODS = [("제안 방법", "proposed"), ("FM-Fusion", "fmfusion"), ("DualMap", "dualmap")]
G, O_, R_ = (46, 125, 50), (224, 138, 30), (198, 40, 40)
F = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 17); FT = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 22)
dist = lambda a, b: float(np.linalg.norm(np.asarray(a) - np.asarray(b)))


def cam_of(cam, k):
    return np.array(cam[k]["cam_K"]).reshape(3, 3), np.array(cam[k]["cam_R_w2c"]).reshape(3, 3), np.array(cam[k]["cam_t_w2c"]) / 1000


def project(P, K, R, t):
    X = np.asarray(P) @ R.T + t; z = X[:, 2]; ok = z > 0.2
    uv = np.c_[K[0, 0] * X[:, 0] / np.maximum(z, 1e-6) + K[0, 2], K[1, 1] * X[:, 1] / np.maximum(z, 1e-6) + K[1, 2]]
    return uv, z, ok & (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480)


def visible(gts, K, R, t):
    uv, z, ok = project([g["center"] for g in gts], K, R, t)
    return {i for i in range(len(gts)) if ok[i] and 25 < uv[i, 0] < 615 and 25 < uv[i, 1] < 455}


def pick_frames(scene, gts):
    cam = json.loads((OB / "test" / scene / "scene_camera.json").read_text()); keys = sorted(cam, key=int)[::5]
    vis = {k: visible(gts, *cam_of(cam, k)) for k in keys}
    k1 = max(keys, key=lambda k: len(vis[k])); rest = set(range(len(gts))) - vis[k1]
    k2 = max(keys, key=lambda k: (len(vis[k] & rest), len(vis[k]))) if rest else None
    return cam, [k1] + ([k2] if k2 is not None and vis[k2] & rest else [])


def label(d, xy, text, col):
    w = d.textlength(text, font=F); x = min(max(xy[0] - w / 2, 2), 638 - w); y = min(max(xy[1], 2), 456)
    d.rectangle([x - 3, y - 1, x + w + 3, y + 21], fill=(255, 255, 255)); d.text((x, y - 2), text, fill=col, font=F)


def panel(rgb, title):
    im = Image.fromarray(rgb).convert("RGBA"); return im, Image.new("RGBA", im.size, (0, 0, 0, 0)), title


def finish(im, ov, title, notes):
    out = Image.alpha_composite(im, ov).convert("RGB"); d = ImageDraw.Draw(out)
    for xy, text, col in notes: label(d, xy, text, col)
    w = d.textlength(title, font=FT); d.rectangle([0, 0, w + 14, 30], fill=(255, 255, 255)); d.text((7, -1), title, fill=(0, 0, 0), font=FT)
    return np.asarray(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps", type=Path, required=True); ap.add_argument("--room", required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
    table = {o["ycb"]: o["name"] for o in json.loads((REPO / "objpose/rt/bop/obchange_objects.json").read_text())["objects"]}
    model = {y: np.load(PTS / f"{n}.npy")[::8] / 1000.0 for y, n in table.items()}
    scenes = sorted((p.stem for p in (OB / "gt").glob(f"{a.room}_s*.json")), key=lambda s: int(s.rsplit("_s", 1)[1]))
    for sc in scenes:
        gts = [o for o in json.loads((OB / "gt" / f"{sc}.json").read_text())["objects"] if o.get("center")]
        gc = [{"name": o["ycb"], "center": o["center"]} for o in gts if o["ycb"] in CATALOG]
        if not gc: continue
        cam, keys = pick_frames(sc, gc); rows = []
        for k in keys:
            K, R, t = cam_of(cam, k); rgb = cv2.imread(str(OB / "test" / sc / "rgb" / f"{int(k):06d}.png"))[:, :, ::-1]
            dep = cv2.imread(str(OB / "test" / sc / "depth" / f"{int(k):06d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32) * float(cam[k].get("depth_scale", 1.0)) / 1000.0
            im, ov, title = panel(rgb, "정답"); d = ImageDraw.Draw(ov); notes = []
            uv, z, ok = project([o["center"] for o in gts], K, R, t)
            for o, p, v in zip(gts, uv, ok):
                if not v: continue
                col = (0, 0, 0) if o["ycb"] in CATALOG else (120, 120, 120)
                d.ellipse([p[0] - 11, p[1] - 11, p[0] + 11, p[1] + 11], outline=col + (255,), width=3); notes.append(((p[0], p[1] + 13), KO.get(o["ycb"], o["ycb"]), col))
            tiles = [finish(im, ov, title, notes)]
            for name, key in METHODS:
                M = json.loads((a.maps / key / f"{sc}.json").read_text()) if (a.maps / key / f"{sc}.json").exists() else []
                im, ov, _ = panel(rgb, name); d = ImageDraw.Draw(ov); notes = []; n_ok = 0; used = set()
                for g in gc:                      # 정답마다 같은 이름의 가장 가까운 객체 하나가 맞음
                    c = sorted((dist(g["center"], o["center"]), i) for i, o in enumerate(M) if o["name"] == g["name"] and dist(g["center"], o["center"]) < 0.10)
                    if c: used.add(c[0][1]); n_ok += 1
                for i, o in enumerate(M):
                    if o["name"] not in CATALOG: continue
                    if "R" in o: P = model[o["name"]] @ np.asarray(o["R"]).T + np.asarray(o["center"]); surf = False
                    else: P = np.asarray(o.get("pts", []), float).reshape(-1, 3); surf = True
                    if len(P) < 4: continue
                    q, zz, v = project(P, K, R, t)
                    if surf and v.any():          # 복원 점은 그 프레임에서 실제로 보이는 것만
                        ui, vi = q[v, 0].astype(int), q[v, 1].astype(int); dd = dep[vi, ui]; keep = (dd <= 0) | (np.abs(dd - zz[v]) < 0.03); idx = np.where(v)[0][keep]
                    else: idx = np.where(v)[0]
                    if len(idx) < max(4, 0.15 * len(P)): continue
                    near = any(dist(g["center"], o["center"]) < 0.10 for g in gts)
                    col = G if i in used else (O_ if near else R_)
                    hull = cv2.convexHull(q[idx].astype(np.float32)).reshape(-1, 2)
                    if len(hull) >= 3:
                        d.polygon([tuple(p) for p in hull], fill=col + (70,), outline=col + (255,)); d.line([tuple(p) for p in hull] + [tuple(hull[0])], fill=col + (255,), width=3)
                    notes.append(((float(q[idx, 0].mean()), float(q[idx, 1].max()) + 3), KO.get(o["name"], o["name"]), col))
                tiles.append(finish(im, ov, f"{name}  (이 방문에서 맞음 {n_ok}/{len(gc)})", notes))
            rows.append(np.concatenate([np.pad(x, ((0, 0), (0, 8), (0, 0)), constant_values=255) for x in tiles], 1)[:, :-8])
        fig = np.concatenate([np.pad(r, ((0, 8), (0, 0), (0, 0)), constant_values=255) for r in rows], 0)[:-8]
        Image.fromarray(fig).save(a.out / f"overlay_{sc}.png"); print(sc, "frames", [int(k) for k in keys], fig.shape, flush=True)


if __name__ == "__main__":
    main()
