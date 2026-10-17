#!/usr/bin/env python3
"""render_reassoc_real.py — video of the Object Pose Map re-associating objects on a recorded live run.

Left: the recognition camera image with every map object (status active/lost/remembered) drawn as its CAD box at the
live camera pose, coloured and numbered by map id. Right: top view of the camera path and the map objects.
A banner marks each re-detection after the object left the view (from eval_reassoc_real.py).

    python render_reassoc_real.py <eval out dir> --out video.mp4 [--speed 2] [--t0 s --t1 s]
"""
from __future__ import annotations

import argparse, json, pickle, subprocess, sys, tempfile
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/pc"))
from conv_session import ConvSession                              # noqa: E402

KO = {"milk": "우유팩", "saffron": "사프란", "Febreze_high": "탈취제", "Sikhye_high": "식혜캔", "Mugcup_high": "머그컵",
      "Bear": "곰인형", "choco_hazelnut_high": "초코과자", "Dinosaur": "공룡인형"}
PAL = [(31, 111, 178), (224, 138, 30), (58, 157, 93), (184, 52, 42), (123, 94, 167), (140, 86, 75), (212, 160, 23),
       (23, 190, 207), (227, 119, 194), (100, 100, 100)]
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONTB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"


def font(sz, bold=False):
    try:
        return ImageFont.truetype(FONTB if bold else FONT, sz)
    except OSError:
        return ImageFont.truetype(FONT, sz)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", type=Path); ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--speed", type=float, default=2.0); ap.add_argument("--t0", type=float, default=0); ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--sheet", type=Path, default=None, help="also save a contact sheet of the event frames")
    a = ap.parse_args()
    S = pickle.load(open(a.dir / "snaps.pkl", "rb"))
    res = json.loads((a.dir / "reassoc_real.json").read_text())
    snaps, t0 = S["snaps"], S["t0"]
    sess = ConvSession(S["session"], offset_ns=int(round(S["tau_s"] * 1e9)), cache_dir=REPO / "objpose/output/.clock_cache")
    K = sess.K; W, H = sess.W, sess.H
    ext = json.loads((REPO / "integration/cad_extents.json").read_text())
    corners = {n: np.array([[x, y, z] for x in (v["min_mm"][0], v["max_mm"][0]) for y in (v["min_mm"][1], v["max_mm"][1])
                            for z in (v["min_mm"][2], v["max_mm"][2])]) / 1000.0 for n, v in ext.items()}
    edges = [(0, 1), (0, 2), (1, 3), (2, 3), (4, 5), (4, 6), (5, 7), (6, 7), (0, 4), (1, 5), (2, 6), (3, 7)]
    # top view frame: plane of the camera path (smallest principal axis = up)
    P = np.array([s["T_cam"][:3, 3] for s in snaps]); c0 = P.mean(0)
    U, sv, Vt = np.linalg.svd(P - c0); B = Vt[:2]
    allp = np.vstack([P] + [np.array([l[2][:3, 3] for l in s["lms"]]) for s in snaps[-1:] if s["lms"]])
    uv = (allp - c0) @ B.T; lo, hi = uv.min(0) - 0.6, uv.max(0) + 0.6; span = max(hi - lo)
    MW, MH = 520, 600
    sc = (min(MW, MH) - 40) / span; mid = (lo + hi) / 2
    to_px = lambda q: (int(MW / 2 + ((q - c0) @ B[0] - mid[0]) * sc), int(MH / 2 - ((q - c0) @ B[1] - mid[1]) * sc))
    ev_all = [e for e in res["slam"]["events"] if e["had_map_object"] and e["cause"] == "out"]
    color = {}
    tmp = Path(tempfile.mkdtemp())
    fps = 10.0 * a.speed / 2.0 * 2  # snaps at 10 Hz
    step = max(1, int(round(a.speed * 10 / 15)))          # output ~15 fps
    sel = [s for s in snaps if a.t0 <= (s["t"] - t0) / 1e9 <= a.t1][::step]
    f_big, f_mid, f_small = font(26, True), font(18, True), font(15)
    sheet_frames = []
    for k, s in enumerate(sel):
        tt = (s["t"] - t0) / 1e9
        i = int(np.argmin(np.abs(sess.t_ns - s["t"])))
        img = sess.read(i, False).color_bgr.copy()
        Tcw = np.linalg.inv(s["T_cam"])
        for oid, nm, Tw, st, nobs in s["lms"]:
            col = PAL[len(color) % len(PAL)] if oid not in color else color[oid]
            color.setdefault(oid, col)
            c = corners.get(nm)
            if c is None:
                continue
            q = (Tcw @ Tw)
            p = c @ q[:3, :3].T + q[:3, 3]
            if (p[:, 2] < 0.15).any():
                continue
            u = K[0, 0] * p[:, 0] / p[:, 2] + K[0, 2]; v = K[1, 1] * p[:, 1] / p[:, 2] + K[1, 2]
            if u.max() < 0 or u.min() > W or v.max() < 0 or v.min() > H:
                continue
            bgr = col[::-1]
            for e0, e1 in edges:
                cv2.line(img, (int(u[e0]), int(v[e0])), (int(u[e1]), int(v[e1])), bgr, 1 if st == "tentative" else 2, cv2.LINE_AA)
        rgb = Image.fromarray(img[:, :, ::-1]).resize((800, 600))
        dr = ImageDraw.Draw(rgb)
        sx, sy = 800 / W, 600 / H
        for oid, nm, Tw, st, nobs in s["lms"]:
            q = Tcw @ Tw
            if q[2, 3] < 0.2:
                continue
            u = K[0, 0] * q[0, 3] / q[2, 3] + K[0, 2]; v = K[1, 1] * q[1, 3] / q[2, 3] + K[1, 2]
            if 0 <= u < W and 0 <= v < H:
                txt = f"#{oid} {KO.get(nm, nm)}" + (" (후보)" if st == "tentative" else "")
                x, y = u * sx, v * sy - 60
                bb = dr.textbbox((x, y), txt, font=f_mid)
                dr.rectangle((bb[0] - 4, bb[1] - 2, bb[2] + 4, bb[3] + 2), fill=(255, 255, 255), outline=color.get(oid, (0, 0, 0)), width=2)
                dr.text((x, y), txt, font=f_mid, fill=color.get(oid, (0, 0, 0)))
        cur = [e for e in ev_all if 0 <= tt - e["t2"] < 2.5]
        if cur:
            e = cur[-1]
            msg = (f"다시 보임: {KO.get(e['obj'], e['obj'])} — {e['gap_s']:.0f}초 만에, 지도의 같은 물체로 연결" if e["ok"]
                   else f"다시 보임: {KO.get(e['obj'], e['obj'])} — 새 물체로 등록됨")
            dr.rectangle((0, 0, 800, 46), fill=(24, 51, 75) if e["ok"] else (184, 52, 42))
            dr.text((14, 6), msg, font=f_big, fill=(255, 255, 255))
        dr.text((10, 572), f"{tt:5.1f} s", font=f_small, fill=(255, 255, 255))
        # top view
        mp = Image.new("RGB", (MW, MH), (255, 255, 255)); dm = ImageDraw.Draw(mp)
        path = [to_px(sn["T_cam"][:3, 3]) for sn in snaps if sn["t"] <= s["t"]]
        if len(path) > 1:
            dm.line(path, fill=(170, 170, 170), width=3)
        cp = to_px(s["T_cam"][:3, 3]); fwd = s["T_cam"][:3, 2]
        tip = to_px(s["T_cam"][:3, 3] + fwd * 0.5)
        dm.line([cp, tip], fill=(24, 51, 75), width=3); dm.ellipse((cp[0] - 7, cp[1] - 7, cp[0] + 7, cp[1] + 7), fill=(24, 51, 75))
        for oid, nm, Tw, st, nobs in s["lms"]:
            p = to_px(Tw[:3, 3]); col = color.get(oid, (0, 0, 0))
            if st == "tentative":
                dm.ellipse((p[0] - 6, p[1] - 6, p[0] + 6, p[1] + 6), outline=col, width=2)
            else:
                dm.ellipse((p[0] - 7, p[1] - 7, p[0] + 7, p[1] + 7), fill=col, outline=(255, 255, 255), width=2)
        for j, (oid, nm, Tw, st, nobs) in enumerate(sorted(s["lms"], key=lambda l: l[0])):
            col = color.get(oid, (0, 0, 0)); y = 44 + j * 22
            dm.ellipse((MW - 150, y + 4, MW - 138, y + 16), fill=None if st == "tentative" else col, outline=col, width=2)
            dm.text((MW - 132, y), f"#{oid} {KO.get(nm, nm)}" + (" (후보)" if st == "tentative" else ""), font=f_small, fill=col)
        done = [e for e in ev_all if e["t2"] <= tt]
        keep = [l for l in s["lms"] if l[3] != "tentative"]
        n_ids = len({l[0] for l in keep}); names = [l[1] for l in keep]
        dm.rectangle((0, 0, MW, 34), fill=(245, 245, 245))
        dm.text((10, 6), "위에서 본 Object Pose Map", font=f_mid, fill=(40, 40, 40))
        dm.text((10, MH - 52), f"확정된 지도 물체 {n_ids}개 · 같은 종류 중복 {len(names) - len(set(names))}개", font=f_mid, fill=(40, 40, 40))
        dm.text((10, MH - 28), f"화면 밖에서 돌아온 물체 재연결 {sum(e['ok'] for e in done)} / {len(done)}", font=f_mid, fill=(24, 51, 75))
        fr = Image.new("RGB", (800 + MW, 600), (255, 255, 255)); fr.paste(rgb, (0, 0)); fr.paste(mp, (800, 0))
        fr.save(tmp / f"{k:05d}.jpg", quality=90)
        if cur and (not sheet_frames or tt - sheet_frames[-1][0] > 5):
            sheet_frames.append((tt, fr.copy()))
    out_fps = 10.0 * a.speed / step
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", f"{out_fps:.3f}", "-i", str(tmp / "%05d.jpg"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "24", "-vf", "scale=1320:-2", str(a.out)], check=True)
    Image.open(tmp / f"{len(sel) // 2:05d}.jpg").save(a.out.with_name(a.out.stem + "_poster.jpg"), quality=90)
    if a.sheet and sheet_frames:
        pick = sheet_frames[:4]
        w_, h_ = pick[0][1].size
        g = Image.new("RGB", (w_ * 2, h_ * ((len(pick) + 1) // 2)), (255, 255, 255))
        for j, (_, im) in enumerate(pick):
            g.paste(im, ((j % 2) * w_, (j // 2) * h_))
        g.save(a.sheet, quality=88)
    print("frames", len(sel), "fps", round(out_fps, 2), a.out)


if __name__ == "__main__":
    main()
