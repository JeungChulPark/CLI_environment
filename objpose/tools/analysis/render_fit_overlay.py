#!/usr/bin/env python3
"""render_fit_overlay.py — fit_pose_from_points.py 의 자세(제안: 저장된 자세, 나머지: 점군에 CAD 를 맞춘 자세)로 CAD 를 실제 영상에 겹쳐 그린다.
검은 선 = 정답 자세의 CAD 윤곽, 초록 = ADD(-S) 가 지름의 10 % 미만, 빨강 = 그 이상. 물체 좌표축(x 빨강, y 초록, z 파랑)을 함께 그려 방향을 보인다.
    <sam6d python> objpose/tools/analysis/render_fit_overlay.py --fit <fit_pose.json> --scene 000054 --at 0.15 --out <png>"""
import argparse, json, sys
from pathlib import Path
import cv2, numpy as np
from PIL import Image, ImageDraw, ImageFont
REPO = Path(__file__).resolve().parents[3]; sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM   # noqa: E402
ap = argparse.ArgumentParser(); ap.add_argument("--fit", type=Path, required=True); ap.add_argument("--dump", type=Path, required=True); ap.add_argument("--scene", required=True)
ap.add_argument("--at", type=float, default=0.15); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args()
F = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 16); FT = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 22)
tab = json.loads(BM.TABLE.read_text())["objects"]; by_id = {o["obj_id"]: o["name"] for o in tab}; model = {o["name"]: np.load(BM.PTS / f"{o['name']}.npy")[::8] / 1000.0 for o in tab}
sd = BM.YCBV / a.scene; ids, Twc, gobj, K = BM.gt_scene(sd, by_id); k = int(a.at * (len(ids) - 1)); Tcw = np.linalg.inv(Twc[k])
rgb = cv2.imread(str(sd / "rgb" / f"{int(ids[k]):06d}.png"))[:, :, ::-1].copy(); D = json.loads((a.dump / f"{a.scene}.json").read_text()); rows = [r for r in json.loads(a.fit.read_text())["rows"] if r["scene"] == a.scene]


def proj(P):
    X = np.asarray(P) @ Tcw[:3, :3].T + Tcw[:3, 3]; return np.c_[K[0, 0] * X[:, 0] / X[:, 2] + K[0, 2], K[1, 1] * X[:, 1] / X[:, 2] + K[1, 2]]


def hull(P): return [tuple(p) for p in cv2.convexHull(proj(P).astype(np.float32)).reshape(-1, 2)]


def axes(d, T, s=0.06):
    o = proj(T[:3, 3][None])[0]
    for i, c in enumerate([(220, 30, 30), (30, 170, 30), (40, 80, 230)]):
        e = proj((T[:3, 3] + T[:3, i] * s)[None])[0]; d.line([tuple(o), tuple(e)], fill=c + (255,), width=4)


tiles = []
for m in ["정답", "제안 방법", "FM-Fusion", "DualMap"]:
    im = Image.fromarray(rgb).convert("RGBA"); ov = Image.new("RGBA", im.size, (0, 0, 0, 0)); d = ImageDraw.Draw(ov); notes = []
    for g in D["gt"]:
        Tg = np.asarray(g["T"], float); h = hull(model[g["name"]] @ Tg[:3, :3].T + Tg[:3, 3]); d.line(h + [h[0]], fill=(0, 0, 0, 255), width=2)
        if m == "정답": axes(d, Tg)
    n_ok = 0
    for r in [r for r in rows if r["method"] == m]:
        T = np.asarray(r["T"], float); col = (46, 125, 50) if r["pass"] else (198, 40, 40); n_ok += r["pass"]
        h = hull(model[r["object"]] @ T[:3, :3].T + T[:3, 3]); d.polygon(h, fill=col + (60,)); d.line(h + [h[0]], fill=col + (255,), width=3); axes(d, T)
        q = proj(T[:3, 3][None])[0]; notes.append((q, f"{r['rot_sym_deg']:.0f}°", col))
    out = Image.alpha_composite(im, ov).convert("RGB"); d = ImageDraw.Draw(out)
    for q, txt, col in notes:
        w = d.textlength(txt, font=F); d.rectangle([q[0] + 6, q[1] + 6, q[0] + 12 + w, q[1] + 28], fill=(255, 255, 255)); d.text((q[0] + 9, q[1] + 5), txt, fill=col, font=F)
    title = "정답 자세" if m == "정답" else f"{m}  (통과 {n_ok}/{sum(1 for r in rows if r['method'] == m)})"
    w = d.textlength(title, font=FT); d.rectangle([0, 0, w + 14, 30], fill=(255, 255, 255)); d.text((7, -1), title, fill=(0, 0, 0), font=FT); tiles.append(np.asarray(out))
fig = np.concatenate([np.pad(x, ((0, 0), (0, 8), (0, 0)), constant_values=255) for x in tiles], 1)[:, :-8]; Image.fromarray(fig).save(a.out); print(a.out, fig.shape)
