#!/usr/bin/env python3
"""phase37 — attribute every 'object was there but ISM did not output it' to ONE stage.

presence_geom.csv (object projected into the SAM camera from the fused object map
+ ORB-SLAM3 trajectory + extrinsic)  x  stage_scores_190624.csv (every YOLO
candidate scored through all four gates with the gates bypassed)
   -> exclusive first-failure histogram, in the order the deployed pipeline applies:

   L1a  YOLO: no candidate for the prompt at all
   L1b  YOLO: candidates exist but none lands on the object   (localization)
   L1c  YOLO: on-object candidate below score_threshold 0.02
   L1d  YOLO: on-object candidate pushed out of the top_k=3 slot
   L2   semantic (DINOv2 cls) < 0.35
   L2s  semantic passes but another candidate wins the object's single slot (argmax sem)
   L4   masked appearance (DINOv2 patch, blocks [2,9]) < 0.605
   L3   HSV colour < 0.1214
   OK   would be emitted
"""
import argparse, collections, csv, math


def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--presence", default="presence_geom.csv")
    ap.add_argument("--scores", default="stage_scores_190624.csv")
    ap.add_argument("--out", default="attribution_190624.csv")
    ap.add_argument("--zmax", type=float, default=5.0)
    ap.add_argument("--zmin", type=float, default=0.25)
    ap.add_argument("--margin", type=int, default=16, help="px the box is grown by before the hit test")
    ap.add_argument("--edge", type=int, default=0, help="drop projections within this many px of the border")
    a = ap.parse_args()

    SIM, APPE, HSV, YTHR, TOPK = 0.35, 0.605, 0.1214, 0.02, 3

    rows = load(a.scores)
    frames = sorted({int(r["frame_idx"]) for r in rows})
    fset = set(frames)
    by_fo = collections.defaultdict(list)
    meta = {}
    for r in rows:
        fi = int(r["frame_idx"])
        meta[fi] = (float(r["frame_luma_mean"]), float(r["frame_lapvar"]))
        if r["cand_rank"] and int(r["cand_rank"]) > 0:
            by_fo[(fi, r["object"])].append(r)

    pres = [p for p in load(a.presence)
            if int(p["frame_idx"]) in fset and int(p["in_fov"]) == 1
            and a.zmin <= float(p["z_m"]) <= a.zmax
            and a.edge <= float(p["u"]) <= 640 - a.edge and a.edge <= float(p["v"]) <= 480 - a.edge]

    out = []
    for p in pres:
        fi, name = int(p["frame_idx"]), p["object"]
        u, v, z = float(p["u"]), float(p["v"]), float(p["z_m"])
        cands = by_fo.get((fi, name), [])
        hit = []
        for c in cands:
            x1, y1, x2, y2 = (int(c[k]) for k in ("x1", "y1", "x2", "y2"))
            if x1 - a.margin <= u <= x2 + a.margin and y1 - a.margin <= v <= y2 + a.margin:
                hit.append((( x2 - x1) * (y2 - y1), c))
        hit.sort(key=lambda t: t[0])                    # tightest box wins
        best = hit[0][1] if hit else None
        winner = next((c for c in cands if c["is_prod_winner"] == "1"), None)

        if not cands:
            stage = "L1a_no_candidate"
        elif best is None:
            stage = "L1b_localization"
        elif float(best["yolo_conf"]) < YTHR:
            stage = "L1c_yolo_threshold"
        elif int(best["cand_rank"]) > TOPK:
            stage = "L1d_topk_slot"
        elif float(best["sem"]) < SIM:
            stage = "L2_semantic"
        elif winner is not None and winner is not best:
            stage = "L2s_slot_taken"
        elif float(best["masked_appe"]) < APPE:
            stage = "L4_patch_appe"
        elif float(best["hsv"]) < HSV:
            stage = "L3_hsv"
        else:
            stage = "OK"
        lum, lap = meta.get(fi, (0.0, 0.0))
        out.append(dict(frame_idx=fi, object=name, obj_id=p["obj_id"], u=round(u, 1), v=round(v, 1),
                        z_m=z, stage=stage, n_cand=len(cands),
                        yolo_conf=best["yolo_conf"] if best else "", rank=best["cand_rank"] if best else "",
                        sem=best["sem"] if best else "", masked_appe=best["masked_appe"] if best else "",
                        hsv=best["hsv"] if best else "", roi_luma=best["roi_luma_mean"] if best else "",
                        frame_luma=lum, frame_lapvar=lap))

    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0].keys())); w.writeheader(); w.writerows(out)

    ORDER = ["L1a_no_candidate", "L1b_localization", "L1c_yolo_threshold", "L1d_topk_slot",
             "L2_semantic", "L2s_slot_taken", "L4_patch_appe", "L3_hsv", "OK"]
    tot = collections.Counter(r["stage"] for r in out)
    n = len(out)
    print(f"\n=== object-visible (frame,object) cells: {n}  over {len({r['frame_idx'] for r in out})} frames\n")
    print(f"{'stage':22s} {'n':>6s} {'%':>7s}   cumulative-loss")
    cum = 0
    for s in ORDER:
        c = tot.get(s, 0)
        if s != "OK":
            cum += c
        print(f"{s:22s} {c:6d} {100*c/n:6.1f}%   {100*cum/n:6.1f}%")

    print("\n=== by object")
    objs = sorted({r["object"] for r in out})
    hdr = "".join(f"{s.split('_')[0]:>7s}" for s in ORDER)
    print(f"{'object':22s}{'n':>6s}{hdr}{'recall':>8s}")
    for o in objs:
        sub = [r for r in out if r["object"] == o]
        cc = collections.Counter(r["stage"] for r in sub)
        line = "".join(f"{cc.get(s,0):7d}" for s in ORDER)
        print(f"{o:22s}{len(sub):6d}{line}{cc.get('OK',0)/len(sub):8.3f}")

    print("\n=== by frame brightness quartile (frame_luma_mean)")
    lums = sorted(r["frame_luma"] for r in out)
    qs = [lums[int(len(lums) * q)] for q in (0.25, 0.5, 0.75)]
    def qi(x):
        return 0 if x <= qs[0] else 1 if x <= qs[1] else 2 if x <= qs[2] else 3
    print(f"quartile cuts: {qs}")
    print(f"{'Q':>2s}{'n':>7s}{hdr}{'recall':>8s}")
    for q in range(4):
        sub = [r for r in out if qi(r["frame_luma"]) == q]
        if not sub:
            continue
        cc = collections.Counter(r["stage"] for r in sub)
        line = "".join(f"{cc.get(s,0):7d}" for s in ORDER)
        print(f"{q:2d}{len(sub):7d}{line}{cc.get('OK',0)/len(sub):8.3f}")
    print("\nwrote", a.out)


if __name__ == "__main__":
    main()
