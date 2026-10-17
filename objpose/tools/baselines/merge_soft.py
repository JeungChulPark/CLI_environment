#!/usr/bin/env python3
"""merge_soft.py — pose verification used as a score instead of a hard reject.

For every processed image and object: if the verified run (proposed_nogate: pose verification on)
answered, take its pose with score 1.0; otherwise fall back to the unverified geometric top-1 pose
of the same frame (proposed_noverify) with score 0.3. Same GT boxes, same images, same core, so this
is exactly "keep every answer, rank verified ones first". Writes <OUT>/proposedsoft_<ds>_bop19/.
"""
import json, os, sys
from pathlib import Path

OUT = Path(os.environ["CMP_OUT"])
OID = {}
REPO = Path(__file__).resolve().parents[3]
for ds, tab in (("ycbv", "ycbv_objects.json"), ("lmo", "lmo_objects.json")):
    oid = {o["name"]: o["obj_id"] for o in json.loads((REPO / "objpose/rt/bop" / tab).read_text())["objects"]}
    ver, geo, dst = OUT / f"proposednogate_{ds}_bop19", OUT / f"proposednoverify_{ds}_bop19", OUT / f"proposedsoft_{ds}_bop19"
    csv = ["scene_id,im_id,obj_id,score,R,t,time"]
    n_v = n_g = 0
    for sc in sorted(p.name for p in ver.iterdir() if p.is_dir()):
        V = {json.loads(l)["im_id"]: json.loads(l) for l in (ver / sc / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()}
        G = {json.loads(l)["im_id"]: json.loads(l) for l in (geo / sc / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()}
        (dst / sc).mkdir(parents=True, exist_ok=True)
        rows = []
        for im in sorted(G):
            v = {d["object"]: d for d in V.get(im, {"dets": []})["dets"]}
            g = {d["object"]: d for d in G[im]["dets"]}
            dets = []
            for nm in sorted(set(v) | set(g)):
                if nm in v:
                    d = dict(v[nm]); d["score"] = 1.0; d["verified"] = True; n_v += 1
                else:
                    d = dict(g[nm]); d["score"] = 0.3; d["verified"] = False; n_g += 1
                dets.append(d)
                R = " ".join(f"{x:.6f}" for r in d["R"] for x in (r if isinstance(r, list) else [r]))
                t = " ".join(f"{x:.4f}" for x in d["t_mm"])
                csv.append(f"{int(sc)},{im},{oid[nm]},{d['score']:.3f},{R},{t},{V.get(im, G[im])['ms']['total'] / 1000:.4f}")
            f = dict(G[im]); f["dets"] = dets
            rows.append(json.dumps(f))
        (dst / sc / "sam6d_frames.jsonl").write_text("\n".join(rows) + "\n")
    (dst / f"bop_proposed_soft_{ds}.csv").write_text("\n".join(csv) + "\n")
    print(ds, "verified", n_v, "fallback", n_g)
