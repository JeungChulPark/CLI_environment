"""후보 단계 채점: pred_ladder_cand_*.json 의 cand_boxes 로 후보 재현율(IoU≥0.5, 정답 bbox_visib 기준), 장당 후보 수, 단계 시간을 낸다.
    python ladder/score_candidates.py [--iou 0.5] [--out results_candidates.json]
끝까지 간 정확도(찾은 비율 등)는 evaluate.py --methods ladder_cand_... 결과(results json)에서 합친다."""
import argparse, json, statistics as st, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import data as D, paths as P   # noqa: E402

def iou(a, b):   # [x,y,w,h] bop 형식(b) 대 [x1,y1,x2,y2](a)
    bx1, by1, bx2, by2 = b[0], b[1], b[0] + b[2], b[1] + b[3]
    ix = max(0, min(a[2], bx2) - max(a[0], bx1)); iy = max(0, min(a[3], by2) - max(a[1], by1))
    i = ix * iy; u = (a[2] - a[0]) * (a[3] - a[1]) + b[2] * b[3] - i
    return i / u if u > 0 else 0.0

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--results", default="", help="evaluate.py 결과 json (찾은 비율 등 합치기)")
    ap.add_argument("--out", default=str(P.OUT / "results_candidates.json")); a = ap.parse_args()
    ev = json.load(open(a.results))["methods"] if a.results else {}
    out = {}
    for f in sorted(P.OUT.glob("pred_ladder_cand_*.json")):
        d = json.load(open(f)); key = d["method"]; hit = tot = 0; ncand = []
        for im in d["images"]:
            cb = im.get("cand_boxes", []); ncand.append(len(cb))
            for g in D.gt_instances(im["scene_id"], im["im_id"]):
                if g["obj_id"] not in im["targets"]: continue
                tot += 1; hit += any(iou(c, g["bbox_visib"]) >= a.iou for c in cb)
        sm = d["stage_ms"]; e = ev.get(key, {})
        out[key] = {"candidates": d.get("candidates"), "images": len(d["images"]),
                    "cand_per_image_median": st.median(ncand), "cand_per_image_mean": round(st.fmean(ncand), 1),
                    "cand_recall_pct": round(100 * hit / tot, 1) if tot else None, "gt_targets": tot,
                    "proposals_ms": sm["proposals"]["median"], "descriptors_ms": sm["descriptors"]["median"],
                    "matching_ms": sm["matching"]["median"], "pem_ms": sm["pem"]["median"], "total_ms": d["time_ms"]["median"],
                    "found_pct": e.get("found_pct"), "answers_correct_pct": e.get("answers_correct_pct"),
                    "wrong_per_image": e.get("wrong_per_image"), "adds_auc": e.get("adds_auc"),
                    "bop_ar": (e.get("bop_ar") or {}).get("bop19_average_recall")}
        print(key, json.dumps(out[key], ensure_ascii=False))
    json.dump(out, open(a.out, "w"), indent=1, ensure_ascii=False); print("->", a.out)

if __name__ == "__main__":
    main()
