#!/usr/bin/env python3
"""render_overlay_ycb.py — YCB 장면의 실제 카메라 영상 위에 정답과 세 방법의 지도 객체를 겹쳐 그린다 (ObChange 의 render_overlay.py 와 같은 형식).

  객체   eval_maps_f1.py --dump 의 장면 json (정답 좌표계). 제안 방법은 추정 자세의 CAD 점, 나머지는 복원 점(깊이 영상과 3 cm 안에서 맞는 점만).
  프레임 장면 길이의 --at 지점(기본 0.15, 0.65)의 영상. 카메라 자세는 데이터셋의 정답 자세.
  색     초록 = 맞음, 주황 = 정답 자리(10 cm 안)에 다른 이름, 빨강 = 중복 · 먼 위치 · 정답 없는 자리의 다른 이름
BENCH_ROOT / BENCH_TABLE 로 데이터셋을 바꾼다.
    <sam6d python> objpose/tools/baselines/fmfusion/render_overlay_ycb.py --dump <dump dir> --scenes 000054 --out <dir>
"""
import argparse, json, sys
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM   # noqa: E402

METHODS = ["제안 방법", "FM-Fusion", "DualMap"]
G, O_, R_ = (46, 125, 50), (224, 138, 30), (198, 40, 40)
F = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 16); FT = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 22)
TAG = {"duplicate": "중복", "far": "먼 위치", "wrong_name": "다른 이름"}
short = lambda n: n.replace("ycbv_", "").replace("_", " ")


def project(P, K, Tcw):
    X = np.asarray(P) @ Tcw[:3, :3].T + Tcw[:3, 3]; z = X[:, 2]
    uv = np.c_[K[0, 0] * X[:, 0] / np.maximum(z, 1e-6) + K[0, 2], K[1, 1] * X[:, 1] / np.maximum(z, 1e-6) + K[1, 2]]
    return uv, z, (z > 0.2) & (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480)


def label(d, xy, text, col):
    w = d.textlength(text, font=F); x = min(max(xy[0] - w / 2, 2), 638 - w); y = min(max(xy[1], 32), 458)
    d.rectangle([x - 3, y - 1, x + w + 3, y + 20], fill=(255, 255, 255)); d.text((x, y - 2), text, fill=col, font=F)


def finish(rgb, ov, title, notes):
    out = Image.alpha_composite(Image.fromarray(rgb).convert("RGBA"), ov).convert("RGB"); d = ImageDraw.Draw(out)
    for xy, text, col in notes: label(d, xy, text, col)
    w = d.textlength(title, font=FT); d.rectangle([0, 0, w + 14, 30], fill=(255, 255, 255)); d.text((7, -1), title, fill=(0, 0, 0), font=FT)
    return np.asarray(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", type=Path, required=True); ap.add_argument("--scenes", nargs="+", required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--at", type=float, nargs="+", default=[0.15, 0.65])
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
    table = json.loads(BM.TABLE.read_text())["objects"]; by_id = {o["obj_id"]: o["name"] for o in table}
    model = {o["name"]: np.load(BM.PTS / f"{o['name']}.npy")[::8] / 1000.0 for o in table}
    for sc in a.scenes:
        D = json.loads((a.dump / f"{sc}.json").read_text()); sd = BM.YCBV / sc
        ids, Twc, gobj, K = BM.gt_scene(sd, by_id); cam = json.loads((sd / "scene_camera.json").read_text())
        gts = [(g["name"], np.asarray(g["T"], float)) for g in D["gt"]]; gcen = [np.asarray(g["center"]) for g in D["gt"]]; rows = []
        for fr in a.at:
            k = int(fr * (len(ids) - 1)); Tcw = np.linalg.inv(Twc[k])
            rgb = cv2.imread(str(sd / "rgb" / f"{int(ids[k]):06d}.png"))[:, :, ::-1].copy()
            dep = cv2.imread(str(sd / "depth" / f"{int(ids[k]):06d}.png"), cv2.IMREAD_UNCHANGED).astype(np.float32) * float(cam[ids[k]].get("depth_scale", 1.0)) / 1000.0
            ov = Image.new("RGBA", (640, 480), (0, 0, 0, 0)); d = ImageDraw.Draw(ov); notes = []
            for nm, T in gts:
                q, z, v = project(model[nm] @ T[:3, :3].T + T[:3, 3], K, Tcw)
                if v.sum() < 10: continue
                hull = cv2.convexHull(q[v].astype(np.float32)).reshape(-1, 2); d.line([tuple(p) for p in hull] + [tuple(hull[0])], fill=(0, 0, 0, 255), width=3)
                notes.append(((float(q[v, 0].mean()), float(q[v, 1].max()) + 3), short(nm), (0, 0, 0)))
            tiles = [finish(rgb, ov, f"정답 (물체 {len(gts)}개)", notes)]
            for m in METHODS:
                objs = D["methods"].get(m, []); ov = Image.new("RGBA", (640, 480), (0, 0, 0, 0)); d = ImageDraw.Draw(ov); notes = []
                for o in objs:
                    if o["kind"] == "unnamed": continue
                    P = np.asarray(o["pts"], float); q, z, v = project(P, K, Tcw)
                    if o["R"] is None and v.any():
                        ui, vi = q[v, 0].astype(int), q[v, 1].astype(int); dd = dep[vi, ui]; idx = np.where(v)[0][(dd <= 0) | (np.abs(dd - z[v]) < 0.03)]
                    else: idx = np.where(v)[0]
                    if len(idx) < max(10, 0.1 * len(P)): continue
                    near = min(np.linalg.norm(np.asarray(o["center"]) - c) for c in gcen) < 0.10
                    col = G if o["kind"] == "match" else (O_ if (o["kind"] == "wrong_name" and near) else R_)
                    hull = cv2.convexHull(q[idx].astype(np.float32)).reshape(-1, 2)
                    if len(hull) >= 3:
                        d.polygon([tuple(p) for p in hull], fill=col + (70,)); d.line([tuple(p) for p in hull] + [tuple(hull[0])], fill=col + (255,), width=3)
                    txt = short(o["name"]) if o["kind"] in ("match", "wrong_name") else f"{TAG[o['kind']]}: {short(o['name'])}"
                    notes.append(((float(q[idx, 0].mean()), float(q[idx, 1].max()) + 3), txt, col))
                n_ok = sum(o["kind"] == "match" for o in objs)
                tiles.append(finish(rgb, ov, f"{m}  (맞음 {n_ok}/{len(gts)})", notes))
            rows.append(np.concatenate([np.pad(x, ((0, 0), (0, 8), (0, 0)), constant_values=255) for x in tiles], 1)[:, :-8])
        fig = np.concatenate([np.pad(r, ((0, 8), (0, 0), (0, 0)), constant_values=255) for r in rows], 0)[:-8]
        Image.fromarray(fig).save(a.out / f"overlay_{sc}.png"); print(sc, fig.shape, flush=True)


if __name__ == "__main__":
    main()
