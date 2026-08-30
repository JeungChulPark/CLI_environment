#!/usr/bin/env python3
"""build_ism_gif.py — one GIF per dataset from a build_ism_inputs_imu output tree.

Each frame of the GIF = the RGB frame with EVERY accepted ISM detection composited
together (mask tint + bbox + "name appe"), plus a header (frame index, det count)
and a per-object legend. The per-frame `ism_<obj>.png` files only ever show ONE
object each, so they are not reused here.

  python3 tools/build_ism_gif.py --root output_slam/260714_sam --out-dir <same>
"""
import argparse
import glob
import json
import os
import re

import cv2
import numpy as np
from PIL import Image

# distinct BGR colours, stable per object name
_PALETTE = [
    (60, 76, 231), (52, 152, 219), (46, 204, 113), (241, 196, 15),
    (155, 89, 182), (26, 188, 156), (231, 76, 60), (149, 165, 166),
    (243, 156, 18), (39, 174, 96),
]


def colour_of(name, order):
    return _PALETTE[order.index(name) % len(_PALETTE)] if name in order else (200, 200, 200)


def rle_to_mask(seg):
    h, w = seg["size"]
    counts = seg["counts"]
    flat = np.zeros(h * w, np.uint8)
    i = 0
    v = 0
    for c in counts:
        flat[i:i + c] = v
        i += c
        v ^= 1
    return flat.reshape((h, w), order="F")


def load_dets(fdir):
    out = []
    for j in sorted(glob.glob(os.path.join(fdir, "detection_*.json"))):
        name = re.search(r"detection_(.+)\.json", os.path.basename(j)).group(1)
        a = json.load(open(j))
        a = a if isinstance(a, list) else [a]
        # every entry: --multi puts more than one box per object in this list
        for d in a:
            out.append(dict(name=name, score=float(d.get("score", 0.0)),
                            bbox=d.get("bbox"), seg=d.get("segmentation")))
    return out


def render_frame(fdir, fidx, order, brighten, width):
    img = cv2.imread(os.path.join(fdir, "rgb.png"))
    if img is None:
        return None
    if brighten != 1.0:
        img = cv2.convertScaleAbs(img, alpha=brighten, beta=8)
    dets = load_dets(fdir)
    overlay = img.copy()
    for d in dets:
        col = colour_of(d["name"], order)
        if d["seg"]:
            try:
                m = rle_to_mask(d["seg"]) > 0
                overlay[m] = (0.45 * np.array(col) + 0.55 * overlay[m]).astype(np.uint8)
            except Exception:
                pass
    img = overlay
    for d in dets:
        col = colour_of(d["name"], order)
        if not d["bbox"]:
            continue
        x, y, w, h = d["bbox"]
        cv2.rectangle(img, (x, y), (x + w, y + h), col, 2)
        lbl = f"{d['name']} {d['score']:.2f}"
        (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        ty = max(y - 4, th + 2)
        cv2.rectangle(img, (x, ty - th - 3), (x + tw + 4, ty + 2), col, -1)
        cv2.putText(img, lbl, (x + 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                    cv2.LINE_AA)

    # header
    hdr = np.zeros((26, img.shape[1], 3), np.uint8)
    txt = f"frame {fidx:05d}   detections: {len(dets)}"
    cv2.putText(hdr, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    if not dets:
        cv2.putText(hdr, "no detection", (img.shape[1] - 110, 18), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (120, 120, 120), 1, cv2.LINE_AA)
    out = np.vstack([hdr, img])

    # legend: which objects are live in THIS frame (wraps to as many rows as needed
    # so nothing is clipped -- 10 objects do not fit on one 560px row)
    per_row = max(1, out.shape[1] // 108)
    rows = [order[i:i + per_row] for i in range(0, len(order), per_row)] or [[]]
    leg = np.zeros((18 * len(rows) + 4, out.shape[1], 3), np.uint8)
    for r, chunk in enumerate(rows):
        y = 14 + r * 18
        for c, nm in enumerate(chunk):
            x = 6 + c * 108
            on = any(d["name"] == nm for d in dets)
            col = colour_of(nm, order) if on else (55, 55, 55)
            cv2.rectangle(leg, (x, y - 8), (x + 9, y + 1), col, -1)
            cv2.putText(leg, nm[:13], (x + 12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.32,
                        (255, 255, 255) if on else (105, 105, 105), 1, cv2.LINE_AA)
    out = np.vstack([out, leg])

    if width and out.shape[1] != width:
        sc = width / out.shape[1]
        out = cv2.resize(out, (width, int(round(out.shape[0] * sc))), interpolation=cv2.INTER_AREA)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="tree holding sam_*/frame_*/")
    ap.add_argument("--out-dir", default="", help="where to write the GIFs (default: --root)")
    ap.add_argument("--datasets", nargs="+", default=[])
    ap.add_argument("--stride", type=int, default=1, help="use every Nth frame")
    ap.add_argument("--width", type=int, default=560, help="output width px")
    ap.add_argument("--fps", type=float, default=8.0)
    ap.add_argument("--colors", type=int, default=96, help="GIF palette size (size/quality knob)")
    ap.add_argument("--png-dir", default="",
                    help="instead of a GIF, write one composited PNG per frame here "
                         "(<png-dir>/<dataset>/frame_XXXXXX.png) for frame-by-frame review")
    ap.add_argument("--brighten", type=float, default=1.7,
                    help="display-only gain; the scene is dim (does not affect any score)")
    args = ap.parse_args()
    out_dir = args.out_dir or args.root
    os.makedirs(out_dir, exist_ok=True)

    ds = args.datasets or sorted(os.path.basename(p) for p in glob.glob(f"{args.root}/sam_*")
                                 if os.path.isdir(p))
    for name in ds:
        droot = os.path.join(args.root, name)
        fdirs = sorted(glob.glob(f"{droot}/frame_*"),
                       key=lambda p: int(re.search(r"frame_(\d+)", p).group(1)))
        if not fdirs:
            print(f"[skip] {name}: no frames"); continue
        # stable object order = all objects seen in this dataset
        order = sorted({re.search(r"detection_(.+)\.json", os.path.basename(j)).group(1)
                        for j in glob.glob(f"{droot}/frame_*/detection_*.json")})
        sel = fdirs[::max(1, args.stride)]
        if args.png_dir:
            pd = os.path.join(args.png_dir, name)
            os.makedirs(pd, exist_ok=True)
            n = 0
            for fd in sel:
                fi = int(re.search(r"frame_(\d+)", fd).group(1))
                im = render_frame(fd, fi, order, args.brighten, args.width)
                if im is None:
                    continue
                cv2.imwrite(os.path.join(pd, f"frame_{fi:06d}.png"), im)
                n += 1
            print(f"[ok] {name}: {n}/{len(fdirs)} PNGs -> {pd}")
            continue
        frames = []
        for fd in sel:
            fi = int(re.search(r"frame_(\d+)", fd).group(1))
            im = render_frame(fd, fi, order, args.brighten, args.width)
            if im is not None:
                frames.append(Image.fromarray(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)))
        if not frames:
            print(f"[skip] {name}: nothing rendered"); continue
        # One shared palette for every frame: per-frame palettes make each frame a
        # full keyframe and blow the file up (152 frames -> 18.6MB at 560px).
        pal = frames[len(frames) // 2].quantize(colors=args.colors, method=Image.MEDIANCUT)
        frames = [f.quantize(palette=pal, dither=Image.FLOYDSTEINBERG) for f in frames]
        gif = os.path.join(out_dir, f"RESULT_ism_{name}.gif")
        frames[0].save(gif, save_all=True, append_images=frames[1:],
                       duration=int(1000 / args.fps), loop=0, optimize=True)
        mb = os.path.getsize(gif) / 1e6
        print(f"[ok] {name}: {len(frames)}/{len(fdirs)} frames, {len(order)} objects "
              f"-> {gif} ({mb:.1f} MB)")


if __name__ == "__main__":
    main()
