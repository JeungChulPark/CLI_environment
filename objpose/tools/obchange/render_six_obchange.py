#!/usr/bin/env python3
"""render_six_obchange.py — ObChange 방문마다 최종 객체 지도를 그린다. 열 = 여섯 구성, 행 = 지도 방식(--rows). 정답으로 채점한 결과에 따라 색을 칠한다.

  배경   그 방문의 깊이 영상을 데이터셋 카메라 자세로 쌓아 복원한 장면(8 mm 로 솎음)
  시점   정답 물체가 가장 많이 모인 곳을 가장 잘 본 카메라 쪽에서 물러나 35° 위에서 내려다본 가상 시점
  객체   지도 객체의 CAD 점을 투영한 윤곽. 초록 = 맞음(TP), 빨강 = 잘못 등록(FP). 검은 원 = 정답 물체의 위치
  입력   <root>/designs/c<구성>_<방문>.json, scores.json(객체별 판정), gt_centers.json  (designs_obchange.py, score_designs.py)
    <sam6d python> objpose/tools/obchange/render_six_obchange.py --root objpose/output/six_obchange_261011 --out <dir> --rows V0 1n 1n+3
"""
import argparse, json, sys
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, str(Path(__file__).resolve().parent))
_argv = sys.argv; sys.argv = [_argv[0]]
import designs_obchange as DO      # noqa: E402
sys.argv = _argv
ap = argparse.ArgumentParser(); ap.add_argument("--root", type=Path, required=True); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--rows", nargs="+", default=["V0", "1n"]); ap.add_argument("--scenes", nargs="*", default=None)
ap.add_argument("--configs", nargs="*", type=int, default=[1, 2, 3, 4, 5, 6]); ap.add_argument("--wrap", type=int, default=6); ap.add_argument("--suffix", default="")
a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True); a.root = a.root.resolve()
KO = {"ycbv_master_chef_can": "커피 캔", "ycbv_cracker_box": "크래커 상자", "ycbv_sugar_box": "설탕 상자", "ycbv_tomato_soup_can": "수프 캔", "ycbv_pudding_box": "푸딩 상자", "ycbv_gelatin_box": "젤라틴 상자", "ycbv_pitcher_base": "피처",
      "ycbv_bowl": "그릇", "ycbv_mug": "머그컵", "ycbv_large_marker": "마커", "ycbv_foam_brick": "스펀지 벽돌"}
VN = {"V0": "지금 방식", "1": "설계 1", "1n": "설계 1′", "2": "설계 2", "3": "설계 3", "1+2": "설계 1+2", "1n+2": "설계 1′+2", "1+3": "설계 1+3", "1n+3": "설계 1′+3", "2+3": "설계 2+3", "1+2+3": "설계 1+2+3", "1n+2+3": "설계 1′+2+3"}
FB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"; F = ImageFont.truetype(FB, 17); FT = ImageFont.truetype(FB, 20); W, H, FOC = 640, 480, 520.0; G, R_ = (46, 125, 50), (198, 40, 40); UP = np.array([0, 0, 1.0])
SC = {(r["config"], r["scene"], r["variant"]): r for r in json.loads((a.root / "scores.json").read_text())}; GT = json.loads((a.root / "gt_centers.json").read_text())


def proj(P, R, t):
    X = np.asarray(P, float) @ R.T + t; z = X[:, 2]; return np.c_[FOC * X[:, 0] / np.maximum(z, 1e-6) + W / 2, FOC * X[:, 1] / np.maximum(z, 1e-6) + H / 2], z


def label(dr, xy, text, col, taken):
    w = dr.textlength(text, font=F); x = min(max(xy[0] - w / 2, 2), W - 2 - w); y = min(max(xy[1], 30), H - 24)
    for _ in range(8):
        if all(not (x < bx + bw and bx < x + w and abs(y - by) < 21) for bx, by, bw in taken): break
        y = min(y + 21, H - 24)
    taken.append((x, y, w)); dr.rectangle([x - 3, y - 1, x + w + 3, y + 21], fill=(255, 255, 255)); dr.text((x, y - 2), text, fill=col, font=F)


scenes = a.scenes or sorted({s for (c, s, v) in SC}); index = {}
for s in scenes:
    cfgs = [c for c in a.configs if (a.root / "designs" / f"c{c}_{s}.json").exists()]
    if not cfgs: continue
    D = {c: json.loads((a.root / "designs" / f"c{c}_{s}.json").read_text()) for c in cfgs}; sc = DO.ObScene(a.root / f"c{cfgs[-1]}", s); keys = sc.keys; P, C = [], []
    for k in keys[:: max(1, len(keys) // 180)]:
        cm = sc.cam[k]; T = np.eye(4); T[:3, :3] = np.array(cm["cam_R_w2c"]).reshape(3, 3); T[:3, 3] = np.array(cm["cam_t_w2c"]) / 1000.0; Tw = np.linalg.inv(T)
        d = cv2.imread(str(sc.bop / "depth" / f"{int(k):06d}.png"), cv2.IMREAD_UNCHANGED); col = cv2.imread(str(sc.bop / "rgb" / f"{int(k):06d}.png"))
        z = d[::4, ::4].astype(np.float32) * float(cm["depth_scale"]) / 1000.0; v, u = np.mgrid[0:d.shape[0]:4, 0:d.shape[1]:4]; m = (z > 0.25) & (z < 3.5); z = z[m]
        X = np.c_[(u[m] - sc.K[0, 2]) * z / sc.K[0, 0], (v[m] - sc.K[1, 2]) * z / sc.K[1, 1], z]; P.append(X @ Tw[:3, :3].T + Tw[:3, 3]); C.append(col[::4, ::4][m][:, ::-1])
    P = np.concatenate(P); C = np.concatenate(C); _, idx = np.unique(np.floor(P / 0.008).astype(np.int64), axis=0, return_index=True); P, C = P[idx], C[idx]
    gts = [g for g in GT[s] if g["target"]]; cen = np.array([g["center"] for g in gts])
    if not len(cen): continue
    lab = list(range(len(cen)))                               # 가까운 정답끼리 묶어 가장 큰 무리
    for i in range(len(cen)):
        for j in range(i):
            if np.linalg.norm(cen[i] - cen[j]) < 1.0: x, y = lab[i], lab[j]; lab = [y if q == x else q for q in lab]
    g0 = max(set(lab), key=lab.count); ref = cen[[i for i in range(len(cen)) if lab[i] == g0]]; c0 = ref.mean(0); r = max(0.30, np.linalg.norm(ref - c0, axis=1).max()); best = None
    for k in keys[::5]:
        cm = sc.cam[k]; Rw = np.array(cm["cam_R_w2c"]).reshape(3, 3); tw = np.array(cm["cam_t_w2c"]) / 1000.0; X = ref @ Rw.T + tw; u = sc.K[0, 0] * X[:, 0] / np.maximum(X[:, 2], 1e-6) + sc.K[0, 2]; vv = sc.K[1, 1] * X[:, 1] / np.maximum(X[:, 2], 1e-6) + sc.K[1, 2]
        score = (int(((X[:, 2] > 0.3) & (X[:, 2] < 3.0) & (u > 20) & (u < 620) & (vv > 20) & (vv < 460)).sum()), -float(np.median(X[:, 2])))
        if best is None or score > best[0]: best = (score, -Rw.T @ tw)
    hd = best[1] - c0; hd = hd - UP * (hd @ UP); hd /= max(np.linalg.norm(hd), 1e-6); dist = max(0.85, 2.9 * r); el = np.deg2rad(35); eye = c0 + hd * dist * np.cos(el) + UP * dist * np.sin(el)
    fwd = c0 - eye; fwd /= np.linalg.norm(fwd); right = np.cross(fwd, UP); right /= np.linalg.norm(right); R = np.stack([right, np.cross(fwd, right), fwd]); t = -R @ eye
    m = np.linalg.norm(P - c0, axis=1) < 2.2 * dist; uv, z = proj(P[m], R, t); col = C[m]; ok = (z > 0.15) & (uv[:, 0] >= 0) & (uv[:, 0] < W - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < H - 1)
    uv, z, col = uv[ok].astype(int), z[ok], col[ok]; o = np.argsort(-z); uv, col, z = uv[o], col[o], z[o]; bg = np.full((H, W, 3), 255, np.uint8); size = np.clip(FOC * 0.013 / z, 1, 6).astype(int)
    for dy in range(6):
        for dx in range(6):
            q = (size > dy) & (size > dx); bg[np.minimum(uv[q, 1] + dy, H - 1), np.minimum(uv[q, 0] + dx, W - 1)] = col[q]
    bg = (bg.astype(np.float32) * 0.8 + 255 * 0.2).astype(np.uint8); rows = []
    for v in a.rows:
        tiles = []
        for c in cfgs:
            L = D[c][v]; ver = SC[(c, s, v)]["verdict"]; ov = Image.new("RGBA", (W, H), (0, 0, 0, 0)); dr = ImageDraw.Draw(ov); notes = []
            for i, m_ in sorted(enumerate(L), key=lambda x: ver[x[0]]):
                T = np.asarray(m_["T_w_obj"], float); q, zz = proj(sc.pts(m_["name"])[::4] @ T[:3, :3].T + T[:3, 3], R, t); q = q[zz > 0.15]
                if len(q) < 4 or ((q[:, 0] >= 0) & (q[:, 0] < W) & (q[:, 1] >= 0) & (q[:, 1] < H)).sum() < 0.5 * len(q): continue
                colr = G if ver[i] else R_; hull = cv2.convexHull(q.astype(np.float32)).reshape(-1, 2)
                if len(hull) < 3: continue
                dr.polygon([tuple(p) for p in hull], fill=colr + (70,)); dr.line([tuple(p) for p in hull] + [tuple(hull[0])], fill=colr + (255,), width=3)
                notes.append(((float(q[:, 0].mean()), float(q[:, 1].max()) + 3), KO[m_["name"]], colr, ver[i], float(cv2.contourArea(hull))))
            for g in gts:
                q, zz = proj([g["center"]], R, t)
                if zz[0] > 0.15 and 0 <= q[0, 0] < W and 0 <= q[0, 1] < H: dr.ellipse([q[0, 0] - 9, q[0, 1] - 9, q[0, 0] + 9, q[0, 1] + 9], outline=(0, 0, 0, 255), width=3)
            im = Image.alpha_composite(Image.fromarray(bg).convert("RGBA"), ov).convert("RGB"); d2 = ImageDraw.Draw(im); taken = []
            for xy, tx, cl, okk, ar in [n for n in notes if n[3]] + sorted((n for n in notes if not n[3]), key=lambda n: -n[4])[:7]: label(d2, xy, tx, cl, taken)
            x = SC[(c, s, v)]; title = f"구성 {c} · {VN[v]} · 맞음 {x['tp']}/{x['gt']}, 잘못 {x['named'] - x['tp']}"; w = d2.textlength(title, font=FT); d2.rectangle([0, 0, w + 12, 28], fill=(255, 255, 255)); d2.text((6, -2), title, fill=(0, 0, 0), font=FT)
            tiles.append(np.asarray(im))
        for k in range(0, len(tiles), a.wrap):
            tl = tiles[k:k + a.wrap]
            while len(tl) < a.wrap: tl.append(np.full((H, W, 3), 255, np.uint8))
            rows.append(np.concatenate([np.pad(x, ((0, 0), (0, 8), (0, 0)), constant_values=255) for x in tl], 1)[:, :-8])
    fig = np.concatenate([np.pad(x, ((0, 8), (0, 0), (0, 0)), constant_values=255) for x in rows], 0)[:-8]
    name = f"ob_{s}{a.suffix}.jpg"; Image.fromarray(fig).save(a.out / name, quality=86); index[s] = name; print(s, fig.shape, flush=True)
(a.out / f"index_ob{a.suffix}.json").write_text(json.dumps(index))
