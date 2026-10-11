"""Doc 27: the proposed method's object maps on this PC (RTX 4090, out_4090/) next to the laptop's (RTX 3080 Ti
Laptop, out/) and the laptop's baselines. Prints markdown tables; missing files are shown as '-'.
    python compare_4090.py [--a out --b out_4090]
"""
import argparse, json
from pathlib import Path

HERE = Path(__file__).resolve().parent
F = lambda v, n=3: "-" if v is None else (f"{v:.{n}f}" if isinstance(v, float) else str(v))


def load(p):
    return json.load(open(p)) if Path(p).exists() else None


def ycbv_row(out, method, match, clean):
    thr = "iou0.25" if match == "iou" else "center0.05"
    d = load(out / method / f"metrics_ycbv_vis_{thr}_vox0.01{'_clean' if clean else ''}.json")
    if not d:
        return None
    s = d["summary"]
    return [s["tp"], s["fp"], s["fn"], s["precision"], s["recall"], s["f1"], s["mean_iou"], s["mean_sd_mm"], s["frame_level"]["f1"]]


def hz(out, dataset):
    d = load(out / "ours_runs" / f"export_stats_{dataset}.json")
    if not d:
        return None
    flat = [y for x in d for y in (x if isinstance(x, list) else [x])]
    v = [x["sam6d_hz"] for x in flat if x.get("sam6d_hz")]
    return (min(v), sum(v) / len(v), max(v)) if v else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", default="out", help="laptop (RTX 3080 Ti) results")
    ap.add_argument("--b", default="out_4090", help="this PC (RTX 4090) results")
    ap.add_argument("--la", default="3080"); ap.add_argument("--lb", default="4090")
    a = ap.parse_args()
    A, B = HERE / a.a, HERE / a.b
    print(f"## YCB-V (55 objects, seen-surface GT)\n")
    print("| method | GPU | match | TP | FP | FN | P | R | F1 | mIoU | SD mm | frame F1 |\n|---|---|---|---|---|---|---|---|---|---|---|---|")
    rows = [("ours_cad", "iou", False, "제안 ours_cad"), ("ours_obs", "iou", True, "제안 ours_obs --clean"),
            ("ours_obs", "iou", False, "제안 ours_obs raw"), ("ours_cad", "center", False, "제안 ours_cad (중심 5 cm)"),
            ("ours_obs", "center", True, "제안 ours_obs --clean (중심 5 cm)")]
    for m, match, clean, name in rows:
        for out, g in ((A, a.la), (B, a.lb)):
            r = ycbv_row(out, m, match, clean)
            print(f"| {name} | {g} | {match} | " + (" | ".join(F(x) if i < 3 else F(x, 1 if i == 7 else 3) for i, x in enumerate(r)) if r else " | ".join(["-"] * 9)) + " |")
    for m, name in (("fmfusion", "FM-Fusion"), ("dualmap", "DualMap 공개"), ("dualmap_bgr", "DualMap BGR")):
        r = ycbv_row(A, m, "iou", False)
        if r:
            print(f"| {name} (노트북 실행) | {a.la} | iou | " + " | ".join(F(x) if i < 3 else F(x, 1 if i == 7 else 3) for i, x in enumerate(r)) + " |")
    for out, g in ((A, a.la), (B, a.lb)):
        h = hz(out, "ycbv")
        if h:
            print(f"\n- 인식 빈도 {g}: 영상별 {h[0]:.2f}~{h[2]:.2f} Hz, 평균 {h[1]:.2f} Hz")
    print(f"\n## ObChange-Kitchen (visits 2-6, 25 objects)\n")
    print("| method | GPU | mode | match | TP | FP | FN | P | R | F1 | mIoU | SD mm | 치운 물체 남음/직전지도/전체 | 새 물체 찾음/전체 |\n|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    sa, sb = load(A / "obchange_scores.json") or {}, load(B / "obchange_scores.json") or {}
    for m in ("ours_obs", "ours_cad", "fmfusion", "dualmap_bgr", "dualmap"):
        for mode, mn in (("obchange_kitchen", "방문별"), ("obchange_kitchen_all", "연속")):
            for tag in ("ctr5cm", "iou"):
                for S, g in ((sa, a.la), (sb, a.lb)):
                    s = S.get(f"{m}|{mode}|{tag}")
                    if not s:
                        continue
                    rv = s.get("revisit") or {}
                    rvs = f"{rv['removed_still_in_map']}/{rv['removed_mapped']}/{rv['removed']}" if rv else ""
                    ad = f"{rv['added_found']}/{rv['added']}" if rv else ""
                    print(f"| {m} | {g} | {mn} | {tag} | {s['tp']} | {s['fp']} | {s['fn']} | {F(s['precision'])} | {F(s['recall'])} | {F(s['f1'])} | {F(s['mean_iou'])} | {F(s['mean_sd_mm'], 1)} | {rvs} | {ad} |")
    for out, g in ((A, a.la), (B, a.lb)):
        for ds in ("obchange_kitchen", "obchange_kitchen_all"):
            h = hz(out, ds)
            if h:
                print(f"- 인식 빈도 {g} {ds}: {h[0]:.2f}~{h[2]:.2f} Hz")
    print("\n### ObChange per-visit (TP/FP/FN, centre 5 cm)")
    for m in ("ours_obs", "ours_cad"):
        for S, g in ((sa, a.la), (sb, a.lb)):
            s = S.get(f"{m}|obchange_kitchen|ctr5cm")
            if s:
                print(f"- {m} {g}: " + ", ".join(f"v{r['visit']} {r['tp']}/{r['fp']}/{r['fn']}" for r in s["per_visit"]))


if __name__ == "__main__":
    main()
