#!/usr/bin/env python3
"""probe_bop_prompts.py — pick each object's YOLO-World prompt by measuring detector recall.

The open-vocabulary detector is the whole pipeline's front door: if it proposes no box, the
ISM and PEM never see the object. Prompts written from a dataset's class names turn out to be
a poor guide to what the detector responds to — on LM-O, "yellow rubber duck" peaks at 0.03
confidence while "yellow toy duck" reaches 0.90 on the same frames, and the LineMOD object
called `can` is actually a watering can.

So the prompt is chosen by measurement, on the same footing for every dataset:

  * candidates come from the object's own name, the mean colour of its 42 renders, and any
    extra wordings listed as `prompt_candidates` in the table;
  * each candidate is scored on N evenly spaced frames of the scenes the object appears in,
    one object at a time so competing prompts do not suppress each other in NMS;
  * the winner is the one detected in the most frames above `--conf-report`, max confidence
    breaking ties.

Only the images are used — never the pose ground truth, and never the estimates' correctness.
This picks where to look, not whether an answer is right; the ISM and the verification stage
still decide what is accepted.

    python objpose/tools/probe_bop_prompts.py --table objpose/rt/bop/lmo_objects.json --apply
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
SAM6D = REPO / "sam6d_realtime"

# coarse colour vocabulary — only needs to produce a word the detector understands
PALETTE = {
    "black": (30, 30, 30), "white": (235, 235, 235), "grey": (140, 140, 140),
    "red": (180, 45, 45), "dark red": (120, 40, 40), "orange": (225, 130, 40),
    "yellow": (225, 200, 60), "green": (60, 140, 70), "dark green": (45, 80, 55),
    "blue": (55, 85, 175), "navy blue": (40, 50, 95), "purple": (120, 70, 160),
    "pink": (225, 150, 170), "brown": (120, 85, 60), "beige": (200, 185, 160),
}


def colour_word(tdir: Path) -> str:
    """dominant colour name over a few rendered views"""
    acc = []
    for v in (10, 20, 30, 40):
        rgb = cv2.imread(str(tdir / f"rgb_{v}.png"))
        m = cv2.imread(str(tdir / f"mask_{v}.png"), 0)
        if rgb is None or m is None:
            continue
        sel = m > 127
        if sel.sum() < 100:
            continue
        acc.append(rgb[sel][:, ::-1].mean(0))          # BGR -> RGB
    if not acc:
        return ""
    mean = np.mean(acc, axis=0)
    return min(PALETTE, key=lambda k: float(np.linalg.norm(np.asarray(PALETTE[k], float) - mean)))


def candidates(o: dict, colour: str) -> list[str]:
    """the object's own words, plus colour/《toy》/《plastic》 variants"""
    raw = o.get("ycb") or o.get("lm") or o["name"]
    words = [w for w in raw.replace("_", " ").split() if not w.isdigit()]
    noun = " ".join(words[-2:]) if len(words) > 1 else (words[0] if words else o["name"])
    head = words[-1] if words else o["name"]
    out = [o.get("prompt", ""), *o.get("prompt_candidates", []), " ".join(words), noun, head]
    if colour:
        out += [f"{colour} {head}", f"{colour} {noun}", f"{colour} plastic {head}", f"{colour} object"]
    out += [f"toy {head}", f"{noun} on a table"]
    seen, uniq = set(), []
    for p in out:
        p = " ".join(p.split())
        if p and p not in seen:
            seen.add(p)
            uniq.append(p)
    return uniq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--table", required=True)
    ap.add_argument("--frames", type=int, default=24, help="frames sampled per object")
    ap.add_argument("--conf", type=float, default=0.02, help="detector threshold used while probing")
    ap.add_argument("--conf-report", type=float, default=0.05, help="a frame counts as a hit above this")
    ap.add_argument("--apply", action="store_true", help="write the winners back into the table")
    ap.add_argument("--objects", type=int, nargs="*", default=[])
    a = ap.parse_args()

    tpath = Path(a.table).resolve()          # resolve before the chdir below
    sys.path.insert(0, str(SAM6D))
    os.chdir(SAM6D)
    import ultralytics.nn.text_model as _tm
    _tm.WEIGHTS_DIR = SAM6D / "assets"
    from ultralytics import YOLOWorld

    table = json.loads(tpath.read_text())
    scenes_dir = Path(table["scenes_dir"])
    rows = [o for o in table["objects"] if not a.objects or o["obj_id"] in set(a.objects)]

    # which scenes hold which object (scene_gt only — presence, not pose)
    where = {}
    for sc in sorted(p for p in scenes_dir.iterdir() if p.is_dir()):
        gt = json.loads((sc / "scene_gt.json").read_text())
        present = {r["obj_id"] for v in gt.values() for r in v}
        for oid in present:
            where.setdefault(oid, []).append(sc)

    weights = str(SAM6D / "yolov8m-worldv2.pt")
    results = {}
    for o in rows:
        scs = where.get(o["obj_id"], [])
        if not scs:
            print(f"{o['name']}: not present in any scene — kept '{o.get('prompt')}'")
            continue
        imgs = []
        per = max(1, a.frames // len(scs))
        for sc in scs:
            fs = sorted((sc / "rgb").glob("*.png")) or sorted((sc / "rgb").glob("*.jpg"))
            if fs:
                imgs += [fs[i] for i in np.linspace(0, len(fs) - 1, per).astype(int)]
        colour = colour_word(SAM6D / "template" / o["name"] / "templates")
        cands = candidates(o, colour)
        # a fresh model per object: ultralytics moves the CLIP text tower on set_classes and a
        # second call on the same instance raises a device mismatch
        model = YOLOWorld(weights)
        model.set_classes(cands)
        hits = np.zeros(len(cands), int)
        best = np.zeros(len(cands))
        for f in imgs:
            bgr = cv2.imread(str(f))
            r = model.predict(bgr, conf=a.conf, imgsz=640, verbose=False, device="cuda:0")
            if not len(r) or r[0].boxes is None or not len(r[0].boxes):
                continue
            b = r[0].boxes
            seen = set()
            for j in range(len(b)):
                k, c = int(b.cls[j]), float(b.conf[j])
                best[k] = max(best[k], c)
                if c >= a.conf_report and k not in seen:
                    hits[k] += 1
                    seen.add(k)
        order = sorted(range(len(cands)), key=lambda k: (hits[k], best[k]), reverse=True)
        win = cands[order[0]]
        results[o["obj_id"]] = win
        cur = o.get("prompt", "")
        mark = "=" if win == cur else "->"
        print(f"\n{o['name']}  ({colour or 'no colour'}, {len(imgs)} frames)")
        for k in order[:4]:
            flag = " *" if cands[k] == win else ("  (current)" if cands[k] == cur else "")
            print(f"   {hits[k]:3d}/{len(imgs)} frames, max {best[k]:.3f}   '{cands[k]}'{flag}")
        print(f"   {mark} {win!r}")
        del model

    if a.apply:
        for o in table["objects"]:
            if o["obj_id"] in results:
                o["prompt"] = results[o["obj_id"]]
        table["prompt_note"] = (f"Prompts chosen by objpose/tools/probe_bop_prompts.py: detector "
                                f"recall over {a.frames} evenly spaced frames per object, images only "
                                f"(no pose ground truth).")
        tpath.write_text(json.dumps(table, indent=1, ensure_ascii=False) + "\n")
        print(f"\n[apply] {len(results)} prompts written to {tpath}")


if __name__ == "__main__":
    main()
