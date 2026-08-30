#!/usr/bin/env python3
"""reorg_imu_outputs.py — assemble the requested delivery layout for a bag.

Reads the staging dir produced by build_ism_inputs_imu.py (ISM + input bundle)
and run_pem_batch.py (PEM pose images), and writes:

  outputs/rgbd_imu_sdk_bag/<bag>/
    input/frame_XXXXXX/{rgb.png, depth.png, camera.json}     (every sampled frame)
    output/ism/frame_XXXXXX_<obj>.png                        (per-frame per-object seg)
    output/ism/frame_XXXXXX_<obj>.json                       (ISM detection: box + RLE mask)
    output/pem/frame_XXXXXX_<obj>.png                        (per-frame per-object pose)
    output/pem/frame_XXXXXX_<obj>.json                       (PEM detection_pem.json: R, t, score)

The .json files sit right next to their matching .png (same basename).

By default files are MOVED out of the staging dir (not copied) and the emptied
staging bag dir is removed, so each png/json ends up stored EXACTLY ONCE — no
duplicate copy left behind in outputs/pem_inputs. PEM must have finished reading
the bundle before this runs (it does: Stage B precedes Stage C). Pass
--keep-stage to copy instead and preserve the staging bundle (useful if you want
to re-run PEM without re-running ISM).

Pure stdlib (shutil) — runs in any env.
"""
import argparse
import os
import re
import shutil

FRAME_RE = re.compile(r"^frame_(\d+)$")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", required=True, help="staging dir: <output-root>/<bag>")
    ap.add_argument("--final", required=True, help="final dir: outputs/rgbd_imu_sdk_bag/<bag>")
    ap.add_argument("--keep-stage", action="store_true",
                    help="copy instead of move and keep the staging bundle "
                         "(default: move + delete staging so nothing is duplicated)")
    args = ap.parse_args()

    # move by default (single copy, no duplication); copy only when keeping staging
    xfer = shutil.copy2 if args.keep_stage else shutil.move

    in_root = os.path.join(args.final, "input")
    ism_root = os.path.join(args.final, "output", "ism")
    pem_root = os.path.join(args.final, "output", "pem")
    for d in (in_root, ism_root, pem_root):
        os.makedirs(d, exist_ok=True)

    frames = sorted(d for d in os.listdir(args.stage) if FRAME_RE.match(d))
    n_in = n_ism = n_pem = n_ism_json = n_pem_json = 0
    for fr in frames:
        fdir = os.path.join(args.stage, fr)
        # 1) input bundle
        dst_in = os.path.join(in_root, fr)
        os.makedirs(dst_in, exist_ok=True)
        for f in ("rgb.png", "depth.png", "camera.json"):
            src = os.path.join(fdir, f)
            if os.path.isfile(src):
                dst = os.path.join(dst_in, f)
                if os.path.exists(dst):
                    os.remove(dst)          # shutil.move won't overwrite a file
                xfer(src, dst)
        n_in += 1
        # 2) ISM per-object overlays  ism_<obj>.png -> frame_XXXXXX_<obj>.png
        #    + matching ISM detection json  detection_<obj>.json -> frame_XXXXXX_<obj>.json
        for f in os.listdir(fdir):
            if f.startswith("ism_") and f.endswith(".png"):
                obj = f[len("ism_"):-len(".png")]
                dst = os.path.join(ism_root, f"{fr}_{obj}.png")
                if os.path.exists(dst):
                    os.remove(dst)
                xfer(os.path.join(fdir, f), dst)
                n_ism += 1
                det = os.path.join(fdir, f"detection_{obj}.json")
                if os.path.isfile(det):
                    dst = os.path.join(ism_root, f"{fr}_{obj}.json")
                    if os.path.exists(dst):
                        os.remove(dst)
                    xfer(det, dst)
                    n_ism_json += 1
        # 3) PEM per-object pose  pem_<obj>/sam6d_results/{vis_pem.png, detection_pem.json}
        #    -> frame_XXXXXX_<obj>.{png, json}
        for d in os.listdir(fdir):
            if d.startswith("pem_") and os.path.isdir(os.path.join(fdir, d)):
                obj = d[len("pem_"):]
                res_dir = os.path.join(fdir, d, "sam6d_results")
                vis = os.path.join(res_dir, "vis_pem.png")
                if os.path.isfile(vis):
                    dst = os.path.join(pem_root, f"{fr}_{obj}.png")
                    if os.path.exists(dst):
                        os.remove(dst)
                    xfer(vis, dst)
                    n_pem += 1
                det = os.path.join(res_dir, "detection_pem.json")
                if os.path.isfile(det):
                    dst = os.path.join(pem_root, f"{fr}_{obj}.json")
                    if os.path.exists(dst):
                        os.remove(dst)
                    xfer(det, dst)
                    n_pem_json += 1

    # drop the now-emptied staging bundle unless asked to keep it
    if not args.keep_stage:
        shutil.rmtree(args.stage, ignore_errors=True)

    kept = " (staging kept)" if args.keep_stage else " (staging removed)"
    print(f"[reorg] {args.final}: input={n_in} frames, "
          f"ism={n_ism} imgs/{n_ism_json} json, pem={n_pem} imgs/{n_pem_json} json{kept}")


if __name__ == "__main__":
    main()
