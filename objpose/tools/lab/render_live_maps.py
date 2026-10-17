#!/usr/bin/env python3
"""render_live_maps.py — 실제 환경 실행의 최종 객체 지도를 그린다. 영상마다 한 장: 열 = 여섯 구성, 행 = 지도 방식(V0 지금 방식, V1 설계 1, V2 설계 1+2, V3 설계 1+2+3).

  배경   그 영상의 인식 카메라 깊이를 실행 때의 카메라 자세로 쌓아 복원한 장면(구성 6 실행의 자세 사용, 8 mm 로 솎음)
  시점   물체가 모인 곳을 가장 잘 본 카메라 쪽에서 물러나 35° 위에서 내려다본 가상 시점. 물체가 두 군데 이상에 모여 있으면 가장 많은 곳.
  객체   지도 객체의 CAD 점을 투영한 윤곽. 초록 = 그 종류에서 하나(지지율이 가장 높은 것), 빨강 = 같은 종류의 초과 등록. 이름표 옆 숫자는 깊이 지지율.
    <sam6d python> objpose/tools/lab/render_live_maps.py --out objpose/docs/live_six_261010/media [--rows V0 V3]
"""
import argparse, json, sys
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, str(Path(__file__).resolve().parent))
_argv = sys.argv; sys.argv = [_argv[0]]
import map_designs as MD      # noqa: E402
sys.argv = _argv
REPO = MD.REPO; O = (REPO / "objpose/output" / __import__("os").environ.get("OBJPOSE_LAB_OUT", "live_six_261010"))
ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--rows", nargs="+", default=["V0", "V1", "V2", "V3"]); ap.add_argument("--sessions", nargs="*", default=None); ap.add_argument("--suffix", default=""); ap.add_argument("--configs", nargs="*", type=int, default=[1, 2, 3, 4, 5, 6]); ap.add_argument("--wrap", type=int, default=6, help="한 줄에 넣을 구성 수(6 = 한 줄, 3 = 두 줄로 나눔)")
a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
KO = {"milk": "우유", "choco_hazelnut_high": "초코 상자", "Febreze_high": "페브리즈", "Mugcup_high": "머그컵", "saffron": "세제 통", "Sikhye_high": "식혜 캔", "Bear": "곰 인형", "Dinosaur": "공룡 인형"}
CFG = {1: "1. FastSAM-x + ViT-L", 2: "2. FastSAM-x + ViT-S", 3: "3. YOLO-W + FastSAM-x + ViT-L", 4: "4. YOLO-W + FastSAM-x + ViT-S", 5: "5. YOLO-W + MobileSAM + ViT-L", 6: "6. YOLO-W + MobileSAM + ViT-S", 7: "7. 제안 관문 완화", 8: "8. 제안 관문"}
VN = {"V0": "지금 방식", "V1": "설계 1", "V2": "설계 1+2", "V3": "설계 1+2+3", "1": "설계 1", "1n": "설계 1′", "2": "설계 2", "3": "설계 3", "1+2": "설계 1+2", "1n+2": "설계 1′+2", "1+3": "설계 1+3", "1n+3": "설계 1′+3", "2+3": "설계 2+3", "1+2+3": "설계 1+2+3", "1n+2+3": "설계 1′+2+3"}
FB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"; F = ImageFont.truetype(FB, 17); FT = ImageFont.truetype(FB, 19); W, H, FOC = 640, 480, 520.0; G, R_ = (46, 125, 50), (198, 40, 40)


def proj(P, R, t):
    X = np.asarray(P, float) @ R.T + t; z = X[:, 2]; return np.c_[FOC * X[:, 0] / np.maximum(z, 1e-6) + W / 2, FOC * X[:, 1] / np.maximum(z, 1e-6) + H / 2], z


def clusters(cen, link=1.0):
    n = len(cen); lab = list(range(n))
    for i in range(n):
        for j in range(i):
            if np.linalg.norm(cen[i] - cen[j]) < link:
                x, y = lab[i], lab[j]; lab = [y if v == x else v for v in lab]
    return [[i for i in range(n) if lab[i] == g] for g in sorted(set(lab), key=lambda g: -lab.count(g))]


def label(dr, xy, text, col, taken):
    w = dr.textlength(text, font=F); x = min(max(xy[0] - w / 2, 2), W - 2 - w); y = min(max(xy[1], 30), H - 24)
    for _ in range(8):
        if all(not (x < bx + bw and bx < x + w and abs(y - by) < 21) for bx, by, bw in taken): break
        y = min(y + 21, H - 24)
    taken.append((x, y, w)); dr.rectangle([x - 3, y - 1, x + w + 3, y + 21], fill=(255, 255, 255)); dr.text((x, y - 2), text, fill=col, font=F)


sessions = a.sessions or sorted({p.name.split("_", 1)[1][:-5] for p in (O / "designs").glob("c*_26*.json")}); index = {}
for s in sessions:
    D = {c: json.loads(f.read_text()) for c in a.configs for f in [O / "designs" / f"c{c}_{s}.json"] if f.exists()}
    if not D: continue
    cref = max(D); sc = MD.Scene(O / f"c{cref}" / s, s); Tws = [(st, T) for st, T in sc.Tws.items() if T is not None]
    # 배경 점군
    P, C = [], []
    for st, Tw in Tws[:: max(1, len(Tws) // 160)]:
        i = int(np.argmin(np.abs(sc.ses.t_ns - st))); fr = sc.ses.read(i, True); z = fr.depth_raw[::4, ::4].astype(np.float32) / 1000.0; v, u = np.mgrid[0:fr.depth_raw.shape[0]:4, 0:fr.depth_raw.shape[1]:4]; m = (z > 0.25) & (z < 3.5); z = z[m]
        X = np.c_[(u[m] - sc.K[0, 2]) * z / sc.K[0, 0], (v[m] - sc.K[1, 2]) * z / sc.K[1, 1], z]; P.append(X @ Tw[:3, :3].T + Tw[:3, 3]); C.append(fr.color_bgr[::4, ::4][m][:, ::-1])
    P = np.concatenate(P); C = np.concatenate(C); _, idx = np.unique(np.floor(P / 0.008).astype(np.int64), axis=0, return_index=True); P, C = P[idx], C[idx]
    UP = -np.mean([T[:3, 1] for _, T in Tws], axis=0); UP /= np.linalg.norm(UP)
    # 시점: 모든 구성 · 방식의 객체 중심을 모아 가장 큰 무리
    cen = np.array([np.asarray(m["T_w_obj"])[:3, 3] for c in D for v in ("V0", "V3") for m in D[c][v]])
    if not len(cen): continue
    g = clusters(cen)[0]; ref = cen[g]; c0 = np.median(ref, 0); ref = ref[np.linalg.norm(ref - c0, axis=1) < 1.2]; c0 = ref.mean(0); r = max(0.30, np.percentile(np.linalg.norm(ref - c0, axis=1), 90)); best = None
    for st, Tw in Tws[::5]:
        Tcw = np.linalg.inv(Tw); X = ref @ Tcw[:3, :3].T + Tcw[:3, 3]; u = sc.K[0, 0] * X[:, 0] / np.maximum(X[:, 2], 1e-6) + sc.K[0, 2]; vv = sc.K[1, 1] * X[:, 1] / np.maximum(X[:, 2], 1e-6) + sc.K[1, 2]
        score = (int(((X[:, 2] > 0.3) & (X[:, 2] < 3.0) & (u > 20) & (u < 620) & (vv > 20) & (vv < 460)).sum()), -float(np.median(X[:, 2])))
        if best is None or score > best[0]: best = (score, Tw[:3, 3])
    hd = best[1] - c0; hd = hd - UP * (hd @ UP); hd /= max(np.linalg.norm(hd), 1e-6); d = max(1.25, 2.9 * r); el = np.deg2rad(35); eye = c0 + hd * d * np.cos(el) + UP * d * np.sin(el)
    fwd = c0 - eye; fwd /= np.linalg.norm(fwd); right = np.cross(fwd, UP); right /= np.linalg.norm(right); R = np.stack([right, np.cross(fwd, right), fwd]); t = -R @ eye
    m = np.linalg.norm(P - c0, axis=1) < 2.2 * d; uv, z = proj(P[m], R, t); col = C[m]; ok = (z > 0.15) & (uv[:, 0] >= 0) & (uv[:, 0] < W - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < H - 1)
    uv, z, col = uv[ok].astype(int), z[ok], col[ok]; o = np.argsort(-z); uv, col, z = uv[o], col[o], z[o]; bg = np.full((H, W, 3), 255, np.uint8); size = np.clip(FOC * 0.013 / z, 1, 6).astype(int)
    for dy in range(6):
        for dx in range(6):
            q = (size > dy) & (size > dx); bg[np.minimum(uv[q, 1] + dy, H - 1), np.minimum(uv[q, 0] + dx, W - 1)] = col[q]
    bg = (bg.astype(np.float32) * 0.8 + 255 * 0.2).astype(np.uint8); rows = []
    for v in a.rows:
        tiles = []
        for c in a.configs:
            if c not in D: tiles.append(np.full((H, W, 3), 255, np.uint8)); continue
            L = D[c][v]; bestk = {}
            for i, m_ in enumerate(L):
                k = (m_["support"] if m_["support"] is not None else -1, m_.get("n_obs") or 0)
                if m_["name"] not in bestk or k > bestk[m_["name"]][0]: bestk[m_["name"]] = (k, i)
            ov = Image.new("RGBA", (W, H), (0, 0, 0, 0)); dr = ImageDraw.Draw(ov); notes = []
            for i, m_ in sorted(enumerate(L), key=lambda x: bestk[x[1]["name"]][1] == x[0]):
                T = np.asarray(m_["T_w_obj"], float); q, zz = proj(sc.pts(m_["name"])[::4] @ T[:3, :3].T + T[:3, 3], R, t); q = q[zz > 0.15]
                if len(q) < 4 or ((q[:, 0] >= 0) & (q[:, 0] < W) & (q[:, 1] >= 0) & (q[:, 1] < H)).sum() < 0.5 * len(q): continue
                colr = G if bestk[m_["name"]][1] == i else R_; hull = cv2.convexHull(q.astype(np.float32)).reshape(-1, 2)
                if len(hull) < 3: continue
                dr.polygon([tuple(p) for p in hull], fill=colr + (70,)); dr.line([tuple(p) for p in hull] + [tuple(hull[0])], fill=colr + (255,), width=3)
                notes.append(((float(q[:, 0].mean()), float(q[:, 1].max()) + 3), KO[m_["name"]] + ("" if m_["support"] is None else f" {m_['support']:.1f}"), colr))
            im = Image.alpha_composite(Image.fromarray(bg).convert("RGBA"), ov).convert("RGB"); d2 = ImageDraw.Draw(im); taken = []
            for xy, tx, cl in notes: label(d2, xy, tx, cl, taken)
            names = [m_["name"] for m_ in L]; title = f"{CFG[c]} · {VN[v]} · 등록 {len(L)}, 초과 {len(names) - len(set(names))}"; w = d2.textlength(title, font=FT); d2.rectangle([0, 0, w + 12, 27], fill=(255, 255, 255)); d2.text((6, -2), title, fill=(0, 0, 0), font=FT)
            tiles.append(np.asarray(im))
        for k in range(0, len(tiles), a.wrap): rows.append(np.concatenate([np.pad(x, ((0, 0), (0, 8), (0, 0)), constant_values=255) for x in tiles[k:k + a.wrap]], 1)[:, :-8])
    fig = np.concatenate([np.pad(x, ((0, 8), (0, 0), (0, 0)), constant_values=255) for x in rows], 0)[:-8]
    name = f"live_{s}{a.suffix}.jpg"; Image.fromarray(fig).save(a.out / name, quality=86); index[s] = name; print(s, fig.shape, flush=True)
(a.out / f"index{a.suffix}.json").write_text(json.dumps(index))
