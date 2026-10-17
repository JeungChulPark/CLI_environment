#!/usr/bin/env python3
"""render_objmap3d.py — 방법마다 최종 객체 지도를 그린다. 카메라 사진 위에 겹쳐 그리는 것이 아니라, 영상 전체로 복원한 장면 점군을 비스듬히 내려다본 그림 위에 지도에 등록된 객체를 놓는다.

  배경     그 방문(영상)의 깊이 영상을 카메라 자세로 쌓은 색 점군(8 mm 로 솎음, 옅게). 모든 방법이 같은 배경을 쓴다.
  시점     물체가 모여 있는 곳(가까운 물체끼리 1 m 안에서 묶은 무리)마다 하나. 무리를 가장 잘 본 카메라 쪽에서, 뒤로 물러나 35° 위에서 내려다본다.
  객체     지도에 등록된 객체(대상 물체 이름이 붙은 것)의 점을 투영한 윤곽. 배경에 가려져도 그린다(지도에 있는 것을 모두 보이기 위해).
  칸       기준(정답 위치 또는 실제 장면) + 방법 11가지, 4열 3행
  --data obchange   objpose/output/combo_obchange_261009/objlist.json. 색: 초록 맞음 / 노랑 중복 / 보라 다른 물체 자리 / 빨강 정답 없는 자리. 검은 원 = 정답 물체
  --data lab        objpose/output/lab_combo_261010/objlist.json.     색: 초록 = 그 종류에서 관측이 가장 많은 객체 / 빨강 = 같은 종류의 초과 등록
    <sam6d python> objpose/tools/analysis/render_objmap3d.py --data obchange --out objpose/docs/combo_obchange_261009/media_map
"""
import argparse, json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parents[3]
ap = argparse.ArgumentParser(); ap.add_argument("--data", choices=["obchange", "lab"], required=True); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--scenes", nargs="*", default=None); ap.add_argument("--max-labels", type=int, default=8)
a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
FB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"; F = ImageFont.truetype(FB, 17); FT = ImageFont.truetype(FB, 20); W, H, FOC = 640, 480, 520.0
if a.data == "obchange":
    BOP = Path("/home/jucpark/DeepLearning/Datasets/ObChange/bop/test"); L = json.loads((REPO / "objpose/output/combo_obchange_261009/objlist.json").read_text()); L = {r["scene"]: r for r in L}
    KO = {"002_master_chef_can": "커피 캔", "003_cracker_box": "크래커 상자", "004_sugar_box": "설탕 상자", "005_tomato_soup_can": "수프 캔", "008_pudding_box": "푸딩 상자", "009_gelatin_box": "젤라틴 상자",
          "019_pitcher_base": "피처", "024_bowl": "그릇", "025_mug": "머그컵", "040_large_marker": "마커", "061_foam_brick": "스펀지 벽돌"}
    ORDER = ["A", "B", "C_on", "C_off", "D", "E_on", "E_off", "FM-Fusion", "FM-Fusion + CAD", "DualMap", "DualMap + CAD"]; UP = np.array([0, 0, 1.0])
    COL = {"맞음": (46, 125, 50), "중복": (214, 160, 0), "다른 물체 자리": (123, 31, 162), "정답 없는 자리": (198, 40, 40)}
else:
    BOP = Path.home() / "DeepLearning/Datasets/lab_sessions_bop/test"; LO = REPO / "objpose/output/lab_combo_261010"; L = json.loads((LO / "objlist.json").read_text())
    KO = {"milk": "우유", "choco_hazelnut_high": "초코 상자", "Febreze_high": "페브리즈", "Mugcup_high": "머그컵", "saffron": "세제 통", "Sikhye_high": "식혜 캔", "Bear": "곰 인형", "Dinosaur": "공룡 인형"}
    ORDER = ["A", "B", "C", "C_off", "D", "E", "E_off", "FM-Fusion", "FM-Fusion + CAD", "DualMap", "DualMap + CAD"]; UP = np.array([0, -1.0, 0])     # SLAM 월드 = 첫 카메라(아래가 +y)
    COL = {False: (46, 125, 50), True: (198, 40, 40)}
LAB = {"A": "A. 원본 SAM-6D", "B": "B. 전체 분할+원본 판정+검증", "C": "C. 문구 상자+원본 판정+검증", "C_on": "C. 문구 상자+원본 판정+검증", "C_off": "C′. C에서 검증 끔", "D": "D. 제안 방법, 완화",
       "E": "E. 제안 방법", "E_on": "E. 제안 방법", "E_off": "E′. E에서 검증 끔", "FM-Fusion": "FM-Fusion", "FM-Fusion + CAD": "FM-Fusion + CAD 이름", "DualMap": "DualMap", "DualMap + CAD": "DualMap + CAD 이름"}


def quat_to_rot(q):
    x, y, z, w = q; return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def scene_frames(s):
    """→ [(프레임 키, K, Tcw(w2c), depth_scale)]"""
    cam = json.loads((BOP / s / "scene_camera.json").read_text()); keys = sorted(cam, key=int); out = []
    if a.data == "obchange":
        for k in keys:
            T = np.eye(4); T[:3, :3] = np.array(cam[k]["cam_R_w2c"]).reshape(3, 3); T[:3, 3] = np.array(cam[k]["cam_t_w2c"]) / 1000.0; out.append((k, np.array(cam[k]["cam_K"]).reshape(3, 3), T, float(cam[k]["depth_scale"])))
    else:
        for r in np.loadtxt(LO / "slam" / f"{s}_frames.txt", ndmin=2):
            i = int(round(r[0] * 10))
            if i >= len(keys): continue
            T = np.eye(4); T[:3, :3] = quat_to_rot(r[4:8]); T[:3, 3] = r[1:4]; k = keys[i]; out.append((k, np.array(cam[k]["cam_K"]).reshape(3, 3), np.linalg.inv(T), float(cam[k]["depth_scale"])))
    return out


def cloud(s, frames, step):
    P, C = [], []
    for k, K, T, ds in frames[::step]:
        d = cv2.imread(str(BOP / s / "depth" / f"{int(k):06d}.png"), cv2.IMREAD_UNCHANGED); c = cv2.imread(str(BOP / s / "rgb" / f"{int(k):06d}.png"))
        if d is None or c is None: continue
        z = d[::4, ::4].astype(np.float32) * ds / 1000.0; v, u = np.mgrid[0:d.shape[0]:4, 0:d.shape[1]:4]; m = (z > 0.25) & (z < 3.5); z = z[m]
        X = np.c_[(u[m] - K[0, 2]) * z / K[0, 0], (v[m] - K[1, 2]) * z / K[1, 1], z]; Twc = np.linalg.inv(T); P.append(X @ Twc[:3, :3].T + Twc[:3, 3]); C.append(c[::4, ::4][m][:, ::-1])
    P = np.concatenate(P); C = np.concatenate(C); _, idx = np.unique(np.floor(P / 0.008).astype(np.int64), axis=0, return_index=True)
    return P[idx], C[idx]


def clusters(cen, link=1.0):
    n = len(cen); lab = list(range(n))
    for i in range(n):
        for j in range(i):
            if np.linalg.norm(np.asarray(cen[i]) - np.asarray(cen[j])) < link:
                x, y = lab[i], lab[j]; lab = [y if v == x else v for v in lab]
    return [[i for i in range(n) if lab[i] == g] for g in sorted(set(lab), key=lambda g: -lab.count(g))]


def view_for(cen, frames):
    """무리(중심들)를 가장 잘 본 카메라 쪽에서 물러나 35° 위에서 내려다보는 가상 카메라 (R, t: 월드→카메라)"""
    c = np.mean(cen, 0); r = max(0.30, max(np.linalg.norm(np.asarray(x) - c) for x in cen)); best = None
    for k, K, T, ds in frames[::5]:
        X = np.asarray(cen) @ T[:3, :3].T + T[:3, 3]; u = K[0, 0] * X[:, 0] / np.maximum(X[:, 2], 1e-6) + K[0, 2]; v = K[1, 1] * X[:, 1] / np.maximum(X[:, 2], 1e-6) + K[1, 2]
        vis = (X[:, 2] > 0.3) & (X[:, 2] < 3.0) & (u > 20) & (u < 620) & (v > 20) & (v < 460); score = (vis.sum(), -float(np.median(X[:, 2])))
        if best is None or score > best[0]: best = (score, np.linalg.inv(T)[:3, 3])
    hdir = best[1] - c; hdir = hdir - UP * (hdir @ UP); hdir = hdir / max(np.linalg.norm(hdir), 1e-6); d = max(0.85 if a.data == "obchange" else 1.25, 2.9 * r); el = np.deg2rad(35)
    eye = c + hdir * d * np.cos(el) + UP * d * np.sin(el); fwd = c - eye; fwd /= np.linalg.norm(fwd); right = np.cross(fwd, UP); right /= np.linalg.norm(right); down = np.cross(fwd, right)
    R = np.stack([right, down, fwd]); return R, -R @ eye, c, d


def proj(P, R, t):
    X = np.asarray(P, float) @ R.T + t; z = X[:, 2]; return np.c_[FOC * X[:, 0] / np.maximum(z, 1e-6) + W / 2, FOC * X[:, 1] / np.maximum(z, 1e-6) + H / 2], z


def background(P, C, R, t, c, d):
    m = np.linalg.norm(P - c, axis=1) < 2.2 * d; uv, z = proj(P[m], R, t); col = C[m]; ok = (z > 0.15) & (uv[:, 0] >= 0) & (uv[:, 0] < W - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < H - 1)
    uv, z, col = uv[ok].astype(int), z[ok], col[ok]; o = np.argsort(-z); uv, col, z = uv[o], col[o], z[o]; img = np.full((H, W, 3), 255, np.uint8); size = np.clip((FOC * 0.013 / z), 1, 6).astype(int)
    for dy in range(6):
        for dx in range(6):
            s = (size > dy) & (size > dx); img[np.minimum(uv[s, 1] + dy, H - 1), np.minimum(uv[s, 0] + dx, W - 1)] = col[s]
    return (img.astype(np.float32) * 0.8 + 255 * 0.2).astype(np.uint8)        # 조금 옅게


def label(dr, xy, text, col, taken):
    w = dr.textlength(text, font=F); x = min(max(xy[0] - w / 2, 2), W - 2 - w); y = min(max(xy[1], 32), H - 24)
    for _ in range(8):
        if all(not (x < bx + bw and bx < x + w and abs(y - by) < 21) for bx, by, bw in taken): break
        y = min(y + 21, H - 24)
    taken.append((x, y, w)); dr.rectangle([x - 3, y - 1, x + w + 3, y + 21], fill=(255, 255, 255)); dr.text((x, y - 2), text, fill=col, font=F)


def panel(bg, title, objs, R, t, rings):
    ov = Image.new("RGBA", (W, H), (0, 0, 0, 0)); d = ImageDraw.Draw(ov); drawn = []
    for o, col, primary in objs:
        uv, z = proj(o["pts"], R, t); ok = (z > 0.15)
        if ok.sum() < 4: continue
        q = uv[ok]; ins = (q[:, 0] >= 0) & (q[:, 0] < W) & (q[:, 1] >= 0) & (q[:, 1] < H)
        if ins.sum() < 0.5 * len(q): continue
        hull = cv2.convexHull(q.astype(np.float32)).reshape(-1, 2)
        if len(hull) < 3: continue
        d.polygon([tuple(p) for p in hull], fill=col + (70,)); d.line([tuple(p) for p in hull] + [tuple(hull[0])], fill=col + (255,), width=3)
        drawn.append((o, col, primary, float(cv2.contourArea(hull)), (float(q[:, 0].mean()), float(q[:, 1].max()) + 3)))
    for p_, nm in rings:
        uv, z = proj([p_], R, t)
        if z[0] > 0.15 and 0 <= uv[0, 0] < W and 0 <= uv[0, 1] < H: d.ellipse([uv[0, 0] - 9, uv[0, 1] - 9, uv[0, 0] + 9, uv[0, 1] + 9], outline=(0, 0, 0, 255), width=3)
    out = Image.alpha_composite(Image.fromarray(bg).convert("RGBA"), ov).convert("RGB"); dr = ImageDraw.Draw(out); taken = []
    first = [x for x in drawn if x[2]]; rest = sorted((x for x in drawn if not x[2]), key=lambda x: -x[3])
    for o, col, _, _, xy in first + rest[: a.max_labels]: label(dr, xy, KO.get(o["name"], o["name"]), col, taken)
    if not objs:
        for p_, nm in rings:
            uv, z = proj([p_], R, t)
            if z[0] > 0.15 and 0 <= uv[0, 0] < W and 0 <= uv[0, 1] < H: label(dr, (uv[0, 0], uv[0, 1] + 12), nm, (0, 0, 0), taken)
    w = dr.textlength(title, font=FT); dr.rectangle([0, 0, w + 14, 29], fill=(255, 255, 255)); dr.text((7, -1), title, fill=(0, 0, 0), font=FT)
    return np.asarray(out)


index = {}
for s in (a.scenes or sorted(L)):
    frames = scene_frames(s)
    if a.data == "obchange":
        rec = L[s]; gts = [g for g in rec["gt"] if g["center"] and g["target"]]; ref = [g["center"] for g in gts]; rings = [(g["center"], KO[g["name"]]) for g in gts]; meth = rec["methods"]
    else:
        meth = L[s]; e = [o for o in meth.get("E", []) if not o["extra"]]; ref = [o["center"] for o in e]; rings = []
    if not ref: continue
    P, C = cloud(s, frames, 4 if a.data == "obchange" else 6); index[s] = []
    for ci, g in enumerate(clusters(ref)):
        if ci >= 2 and len(g) < 2: break
        R, t, c, d = view_for([ref[i] for i in g], frames); bg = background(P, C, R, t, c, d)
        tiles = [panel(bg, f"정답 위치 (이 시점에 {len(g)}개)" if a.data == "obchange" else "복원한 장면", [], R, t, rings)]
        for m in ORDER:
            if m not in meth: continue
            if a.data == "obchange":
                objs = [(o, COL[o["verdict"]], o["verdict"] == "맞음") for o in sorted(meth[m]["objects"], key=lambda o: o["verdict"] == "맞음") if o["verdict"] != "이름 없음"]
                n_ok = sum(x[2] for x in objs); title = f"{LAB[m]} · 맞음 {n_ok}, 잘못 {len(objs) - n_ok}"
            else:
                objs = [(o, COL[o["extra"]], not o["extra"]) for o in sorted(meth[m], key=lambda o: not o["extra"]) if o["name"]]; title = f"{LAB[m]} · 등록 {len(objs)}, 초과 {sum(not x[2] for x in objs)}"
            tiles.append(panel(bg, title, objs, R, t, rings))
        while len(tiles) % 4: tiles.append(np.full((H, W, 3), 255, np.uint8))
        rows = [np.concatenate([np.pad(x, ((0, 0), (0, 8), (0, 0)), constant_values=255) for x in tiles[i:i + 4]], 1)[:, :-8] for i in range(0, len(tiles), 4)]
        fig = np.concatenate([np.pad(x, ((0, 8), (0, 0), (0, 0)), constant_values=255) for x in rows], 0)[:-8]
        name = f"map_{s}_c{ci + 1}.jpg"; Image.fromarray(fig).save(a.out / name, quality=88); index[s].append({"file": name, "cluster": ci + 1, "objects": len(g)})
    print(s, [(x["cluster"], x["objects"]) for x in index[s]], len(P), flush=True)
(a.out / f"index_{a.data}.json").write_text(json.dumps(index))
