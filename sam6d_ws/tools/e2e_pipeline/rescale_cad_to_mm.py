#!/usr/bin/env python3
"""rescale_cad_to_mm.py — convert meter-unit ascii PLYs to BOP/SAM-6D millimeters.

SAM-6D (and BOP) assume the CAD is in millimeters: every pose path does
`mesh.sample(...) / 1000.0`. The provided CAD PLYs (except milk, Febreze_low)
were exported from Blender in *meters*, so at inference the model point cloud is
1000x too small and ISM/PEM matching collapses -> zero detections.

This scales ONLY the x,y,z columns (first 3 floats per vertex line) by 1000,
preserving the exact normal/uv/color columns and face block verbatim. The
original file is backed up alongside as `<name>.orig_meter`.

Usage:
  python tools/e2e_pipeline/rescale_cad_to_mm.py            # process all meter-unit CAD
  python tools/e2e_pipeline/rescale_cad_to_mm.py --dry-run
"""
import argparse
import glob
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(__file__))
import pipeline_lib as L  # noqa: E402

SCALE = 1000.0
# already in mm — do not touch
SKIP = {"milk", "Febreze_low"}


def find_ply(folder):
    plys = sorted(glob.glob(os.path.join(folder, "*.ply")))
    return plys[0] if plys else None


def rescale_ascii_ply(src, dst):
    """Stream src -> dst, scaling first 3 vertex columns by SCALE."""
    n_vert = None
    with open(src, "r") as f, open(dst, "w") as o:
        # --- header ---
        in_header = True
        while in_header:
            line = f.readline()
            if line == "":
                raise ValueError("EOF in header")
            o.write(line)
            s = line.strip()
            if s.startswith("element vertex"):
                n_vert = int(s.split()[-1])
            elif s == "end_header":
                in_header = False
        if n_vert is None:
            raise ValueError("no 'element vertex' in header")
        # --- vertex block: scale first 3 tokens ---
        for _ in range(n_vert):
            line = f.readline()
            tok = line.split()
            tok[0] = "%.6f" % (float(tok[0]) * SCALE)
            tok[1] = "%.6f" % (float(tok[1]) * SCALE)
            tok[2] = "%.6f" % (float(tok[2]) * SCALE)
            o.write(" ".join(tok) + "\n")
        # --- faces / remainder: verbatim ---
        shutil.copyfileobj(f, o)
    return n_vert


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    folders = sorted(glob.glob(os.path.join(L.CAD_DIR, "*", "")))
    done = []
    for folder in folders:
        name = os.path.basename(folder.rstrip("/"))
        if name in SKIP:
            print(f"[skip] {name}: already mm")
            continue
        ply = find_ply(folder)
        if not ply:
            print(f"[skip] {name}: no .ply")
            continue
        backup = ply + ".orig_meter"
        if os.path.exists(backup):
            print(f"[skip] {name}: backup exists ({os.path.basename(backup)}) — already rescaled")
            continue
        with open(ply) as f:
            fmt = (f.readline(), f.readline())
        if "ascii" not in fmt[1]:
            print(f"[WARN] {name}: not ascii ({fmt[1].strip()}) — skipping, handle manually")
            continue
        if args.dry_run:
            print(f"[dry-run] {name}: would rescale {os.path.basename(ply)} x{SCALE:.0f} (backup -> .orig_meter)")
            continue
        tmp = ply + ".mm_tmp"
        nv = rescale_ascii_ply(ply, tmp)
        os.rename(ply, backup)      # preserve original (no longer matches *.ply)
        os.rename(tmp, ply)         # mm version takes the canonical name
        done.append((name, nv))
        print(f"[ok] {name}: {nv} verts scaled x{SCALE:.0f} -> {os.path.basename(ply)} (orig kept as .orig_meter)")

    print(f"\n[rescale] {len(done)} CAD files converted to mm")


if __name__ == "__main__":
    main()
