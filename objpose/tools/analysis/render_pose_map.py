#!/usr/bin/env python3
"""render_pose_map.py — 한 조합(자세 추정 구성 + 지도 생성 방식)이 실제로 만든 객체 지도를 그린다: 복원한 장면 위에, 지도에 저장된 6D 자세대로 CAD 모형과 좌표축을 놓는다.

  배경     영상 전체의 깊이를 카메라 자세로 쌓아 복원한 장면(조금 옅게)
  객체     지도에 저장된 자세(위치 + 방향)로 놓은 CAD 점(물체마다 다른 색. 배경 위에 그리고 객체끼리는 가려짐 처리) + 그 자세의 좌표축(빨강 x, 초록 y, 파랑 z, 길이 6 cm) + 이름
  시점     물체가 모인 곳마다 한 장(가까운 물체끼리 1 m 안에서 묶음, 많은 순서로 --clusters 개까지). 그 무리를 가장 잘 본 카메라 쪽에서 물러나 35° 위에서 내려다본다.
  --data obchange  objpose/output/six_obchange_261011 (designs/, scores.json, gt_centers.json). 이름표에 맞음/잘못 등록을 함께 적는다.
  --data lab       objpose/output/live_six_261010 (designs/). 이름표에 깊이 지지율을 함께 적는다.
  <out>/pose_<장면>_c<무리>.jpg, <out>/poses_<data>.json (장면 → 객체마다 이름, 위치, 방향(롤 · 피치 · 요, 사원수), 관측 수, 판정/지지율)
    <sam6d python> objpose/tools/analysis/render_pose_map.py --data obchange --config 6 --variant 1n --out <dir> [--scenes …]
"""
import argparse, json, sys
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
REPO = Path(__file__).resolve().parents[3]
ap = argparse.ArgumentParser(); ap.add_argument("--data", choices=["obchange", "lab"], required=True); ap.add_argument("--config", type=int, required=True); ap.add_argument("--variant", default="1n"); ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--scenes", nargs="*", default=None); ap.add_argument("--clusters", type=int, default=2)
a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True); _argv = sys.argv; sys.argv = [_argv[0]]
if a.data == "obchange":
    sys.path.insert(0, str(REPO / "objpose/tools/obchange")); import designs_obchange as DO; MD = DO.MD; ROOT = (REPO / "objpose/output" / __import__("os").environ.get("OBJPOSE_OB_OUT", "six_obchange_261011"))
    KO = {"ycbv_master_chef_can": "커피 캔", "ycbv_cracker_box": "크래커 상자", "ycbv_sugar_box": "설탕 상자", "ycbv_tomato_soup_can": "수프 캔", "ycbv_pudding_box": "푸딩 상자", "ycbv_gelatin_box": "젤라틴 상자", "ycbv_pitcher_base": "피처",
          "ycbv_bowl": "그릇", "ycbv_mug": "머그컵", "ycbv_large_marker": "마커", "ycbv_foam_brick": "스펀지 벽돌"}
    SCORE = {(r["config"], r["scene"], r["variant"]): r for r in json.loads((ROOT / "scores.json").read_text())}; GT = json.loads((ROOT / "gt_centers.json").read_text())
else:
    sys.path.insert(0, str(REPO / "objpose/tools/lab")); import map_designs as MD; ROOT = (REPO / "objpose/output" / __import__("os").environ.get("OBJPOSE_LAB_OUT", "live_six_261010"))
    KO = {"milk": "우유", "choco_hazelnut_high": "초코 상자", "Febreze_high": "페브리즈", "Mugcup_high": "머그컵", "saffron": "세제 통", "Sikhye_high": "식혜 캔", "Bear": "곰 인형", "Dinosaur": "공룡 인형"}
sys.argv = _argv
FB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"; F = ImageFont.truetype(FB, 24); W, H, FOC = 1280, 960, 1040.0
PAL = [(31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40), (148, 103, 189), (140, 86, 75), (227, 119, 194), (23, 190, 207), (188, 189, 34), (127, 127, 127), (255, 187, 120)]


def proj(P, R, t):
    X = np.asarray(P, float) @ R.T + t; z = X[:, 2]; return np.c_[FOC * X[:, 0] / np.maximum(z, 1e-6) + W / 2, FOC * X[:, 1] / np.maximum(z, 1e-6) + H / 2], z


def rpy(Rm):
    sy = float(np.hypot(Rm[0, 0], Rm[1, 0])); return [float(np.degrees(v)) for v in ((np.arctan2(Rm[2, 1], Rm[2, 2]), np.arctan2(-Rm[2, 0], sy), np.arctan2(Rm[1, 0], Rm[0, 0])) if sy > 1e-6 else (np.arctan2(-Rm[1, 2], Rm[1, 1]), np.arctan2(-Rm[2, 0], sy), 0.0))]


def quat(Rm):
    qw = np.sqrt(max(0.0, 1 + np.trace(Rm))) / 2
    return [1.0, 0, 0, 0] if qw < 1e-6 else [float(qw), float((Rm[2, 1] - Rm[1, 2]) / (4 * qw)), float((Rm[0, 2] - Rm[2, 0]) / (4 * qw)), float((Rm[1, 0] - Rm[0, 1]) / (4 * qw))]


def clusters(cen, link=1.0):
    n = len(cen); lab = list(range(n))
    for i in range(n):
        for j in range(i):
            if np.linalg.norm(cen[i] - cen[j]) < link: x, y = lab[i], lab[j]; lab = [y if q == x else q for q in lab]
    return [[i for i in range(n) if lab[i] == g] for g in sorted(set(lab), key=lambda g: -lab.count(g))]


scenes = a.scenes or sorted({f.stem.split("_", 1)[1] for f in (ROOT / "designs").glob(f"c{a.config}_*.json")}); POSES = {}; index = {}
for s in scenes:
    f = ROOT / "designs" / f"c{a.config}_{s}.json"
    if not f.exists(): continue
    L = json.loads(f.read_text())[a.variant]
    if a.data == "obchange":
        sc = DO.ObScene(ROOT / f"c{a.config}", s); frames = []; UP = np.array([0, 0, 1.0]); ver = SCORE[(a.config, s, a.variant)]["verdict"]
        for k in sc.keys[:: max(1, len(sc.keys) // 180)]:
            cm = sc.cam[k]; T = np.eye(4); T[:3, :3] = np.array(cm["cam_R_w2c"]).reshape(3, 3); T[:3, 3] = np.array(cm["cam_t_w2c"]) / 1000.0
            frames.append((np.linalg.inv(T), cv2.imread(str(sc.bop / "depth" / f"{int(k):06d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32) * float(cm["depth_scale"]) / 1000.0, cv2.imread(str(sc.bop / "rgb" / f"{int(k):06d}.png"))))
        extra = ["맞음" if v else "잘못 등록" for v in ver]
    else:
        sc = MD.Scene(ROOT / f"c{a.config}" / s, s); Tws = [(st, T) for st, T in sc.Tws.items() if T is not None]; frames = []
        for st, Tw in Tws[:: max(1, len(Tws) // 160)]:
            fr = sc.ses.read(int(np.argmin(np.abs(sc.ses.t_ns - st))), True); frames.append((Tw, fr.depth_raw.astype(np.float32) / 1000.0, fr.color_bgr))
        UP = -np.mean([T[:3, 1] for _, T in Tws], axis=0); UP /= np.linalg.norm(UP); extra = ["" if m["support"] is None else f"지지율 {m['support']:.2f}" for m in L]
    POSES[s] = [{"name": m["name"], "label": KO[m["name"]], "xyz_m": [round(float(v), 4) for v in np.asarray(m["T_w_obj"])[:3, 3]], "rpy_deg": [round(v, 1) for v in rpy(np.asarray(m["T_w_obj"])[:3, :3])], "quat_wxyz": [round(v, 4) for v in quat(np.asarray(m["T_w_obj"])[:3, :3])],
                 "n_obs": m.get("n_obs"), "note": e} for m, e in zip(L, extra)]
    if not L: index[s] = []; continue
    P, C = [], []
    for Tw, d, col in frames:
        z = d[::4, ::4]; v, u = np.mgrid[0:d.shape[0]:4, 0:d.shape[1]:4]; m = (z > 0.25) & (z < 3.5); z = z[m]
        X = np.c_[(u[m] - sc.K[0, 2]) * z / sc.K[0, 0], (v[m] - sc.K[1, 2]) * z / sc.K[1, 1], z]; P.append(X @ Tw[:3, :3].T + Tw[:3, 3]); C.append(col[::4, ::4][m][:, ::-1])
    P = np.concatenate(P); C = np.concatenate(C); _, idx = np.unique(np.floor(P / 0.006).astype(np.int64), axis=0, return_index=True); P, C = P[idx], C[idx]
    cen = np.array([np.asarray(m["T_w_obj"])[:3, 3] for m in L]); index[s] = []; color = {n: PAL[i % len(PAL)] for i, n in enumerate(KO)}
    for ci, g in enumerate(clusters(cen)[: a.clusters]):
        ref = cen[g]; c0 = ref.mean(0); r = max(0.25, np.linalg.norm(ref - c0, axis=1).max()); seen = []
        for Tw, _, _ in frames:
            Tcw = np.linalg.inv(Tw); X = ref @ Tcw[:3, :3].T + Tcw[:3, 3]; u = sc.K[0, 0] * X[:, 0] / np.maximum(X[:, 2], 1e-6) + sc.K[0, 2]; vv = sc.K[1, 1] * X[:, 1] / np.maximum(X[:, 2], 1e-6) + sc.K[1, 2]
            seen.append((int(((X[:, 2] > 0.3) & (X[:, 2] < 3.0) & (u > 20) & (u < 620) & (vv > 20) & (vv < 460)).sum()), Tw[:3, 3]))
        top = max(n for n, _ in seen); best = (top, np.mean([c for n, c in seen if n >= max(1, 0.8 * top)], axis=0))      # 무리를 잘 본 카메라들의 평균 위치 쪽에서 본다
        hd = best[1] - c0; hd = hd - UP * (hd @ UP); hd /= max(np.linalg.norm(hd), 1e-6); dist = max(0.8, 2.7 * r); el = np.deg2rad(32); eye = c0 + hd * dist * np.cos(el) + UP * dist * np.sin(el)
        fwd = c0 - eye; fwd /= np.linalg.norm(fwd); right = np.cross(fwd, UP); right /= np.linalg.norm(right); R = np.stack([right, np.cross(fwd, right), fwd]); t = -R @ eye
        # 배경 + 객체 점을 한 번에 깊이 순으로 칠한다(가려짐 처리)
        m = np.linalg.norm(P - c0, axis=1) < 2.2 * dist; pts = [P[m]]; cols = [(C[m].astype(np.float32) * 0.75 + 255 * 0.25).astype(np.uint8)]; big = [np.zeros(int(m.sum()), bool)]
        for mm in L:
            T = np.asarray(mm["T_w_obj"], float); M = np.load(MD.PTS / f"{mm['name']}.npy") / 1000.0; M = M[np.random.default_rng(0).choice(len(M), min(len(M), 6000), replace=False)]
            pts.append(M @ T[:3, :3].T + T[:3, 3]); cols.append(np.tile(np.array(color[mm["name"]], np.uint8), (len(M), 1))); big.append(np.ones(len(M), bool))
        Q = np.concatenate(pts); Cc = np.concatenate(cols); Bg = np.concatenate(big); uv, z = proj(Q, R, t); ok = (z > 0.15) & (uv[:, 0] >= 0) & (uv[:, 0] < W - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < H - 1)
        uv, z, Cc, Bg = uv[ok].astype(int), z[ok], Cc[ok], Bg[ok]; o = np.argsort(-(z - np.where(Bg, 100.0, 0.0)));         # 배경을 먼저 깊이 순으로 칠하고, 객체는 그 위에 객체끼리의 깊이 순으로 칠한다(복원 점군의 잡음에 CAD 가 묻히지 않게)
        uv, z, Cc, Bg = uv[o], z[o], Cc[o], Bg[o]; img = np.full((H, W, 3), 255, np.uint8); size = np.clip(FOC * np.where(Bg, 0.006, 0.008) / z, 2, 7).astype(int)
        for layer in (False, True):                # 배경을 다 칠한 뒤에 객체를 칠한다
            for dy in range(9):
                for dx in range(9):
                    q = (size > dy) & (size > dx) & (Bg == layer); img[np.minimum(uv[q, 1] + dy, H - 1), np.minimum(uv[q, 0] + dx, W - 1)] = Cc[q]
        im = Image.fromarray(img); dr = ImageDraw.Draw(im); taken = []
        for mm, e in zip(L, extra):
            T = np.asarray(mm["T_w_obj"], float); ax = np.array([[0, 0, 0], [0.06, 0, 0], [0, 0.06, 0], [0, 0, 0.06]]) @ T[:3, :3].T + T[:3, 3]; q, zz = proj(ax, R, t)
            if zz[0] < 0.15 or not (0 <= q[0, 0] < W and 0 <= q[0, 1] < H): continue
            for k, col in ((1, (220, 30, 30)), (2, (30, 170, 30)), (3, (30, 60, 230))):
                dr.line([tuple(q[0]), tuple(q[k])], fill=(255, 255, 255), width=9); dr.line([tuple(q[0]), tuple(q[k])], fill=col, width=5)
            body, zb = proj(np.load(MD.PTS / f"{mm['name']}.npy")[::40] / 1000.0 @ T[:3, :3].T + T[:3, 3], R, t); txt = KO[mm["name"]] + (f" · {e}" if e else ""); w = dr.textlength(txt, font=F)
            x = min(max(float(body[:, 0].mean()) - w / 2, 4), W - 4 - w); y = min(max(float(body[:, 1].max()) + 6, 6), H - 34)
            for _ in range(10):
                if all(not (x < bx + bw and bx < x + w and abs(y - by) < 30) for bx, by, bw in taken): break
                y = min(y + 30, H - 34)
            taken.append((x, y, w)); dr.rectangle([x - 4, y - 1, x + w + 4, y + 30], fill=(255, 255, 255), outline=color[mm["name"]], width=3); dr.text((x, y - 3), txt, fill=(0, 0, 0), font=F)
        if a.data == "obchange":
            for gt in GT[s]:
                if not gt["target"]: continue
                q, zz = proj([gt["center"]], R, t)
                if zz[0] > 0.15 and 0 <= q[0, 0] < W and 0 <= q[0, 1] < H: dr.ellipse([q[0, 0] - 13, q[0, 1] - 13, q[0, 0] + 13, q[0, 1] + 13], outline=(0, 0, 0), width=4)
        name = f"pose_{s}_c{ci + 1}.jpg"; im.save(a.out / name, quality=88); index[s].append({"file": name, "objects": len(g)})
    print(s, len(L), [x["objects"] for x in index[s]], flush=True)
(a.out / f"poses_{a.data}.json").write_text(json.dumps({"config": a.config, "variant": a.variant, "poses": POSES, "index": index}, ensure_ascii=False))
