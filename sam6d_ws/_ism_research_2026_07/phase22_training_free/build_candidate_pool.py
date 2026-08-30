#!/usr/bin/env python3
"""build_candidate_pool.py — training-free 규칙 평가용 후보 전수 pool (READ-ONLY).

authoritative gate flag = phase21_common.build_cells (cur pairs + cur/{ds}_hsv.npz, Phase1C
622/86/313 정확 재현). 여기에 appe2/9 clstop1·sem 집계·42뷰 배열을 phase2_candidates.csv /
view_sims_phase2.npz 에서 uid 로 조인한다. 학습·GT-fit 없음.

산출: csv/candidate_pool.csv (routed 후보 1행/candidate) + metrics/pool_provenance.json
"""
import csv, json, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
P21 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "phase21_fn_audit")
sys.path.insert(0, P21)
import phase21_common as p21   # noqa: E402

REPO = p21.REPO
PH2 = os.path.join(REPO, "outputs", "phase2_appearance_semantic_yolo_bbox")
OUT = os.path.join(REPO, "outputs", "phase22_training_free_gate")


def main():
    gt = p21.load_gt(); PROTO = p21.build_protos(); cells = p21.build_cells(gt, PROTO)
    grid = p21.grid_from_cells(gt, cells)
    assert grid["TP"] == 622 and grid["FP"] == 86 and grid["FN"] == 313, f"Phase1C 재현 실패 {grid}"
    print("Phase1C 재현 OK:", grid)

    # 조인 소스: phase2_candidates (uid,object)-> appe2/9, sem 집계
    aug = {}
    for r in csv.DictReader(open(os.path.join(PH2, "csv", "phase2_candidates.csv"))):
        aug[(r["uid"], r["object"])] = r
    vs = np.load(os.path.join(PH2, "raw", "view_sims_phase2.npz"))

    rows = []
    for (ds, fr, obj), c in cells.items():
        if not c["cands"]:
            continue
        # conf 순위(선택 기준은 conf 내림차순 top3 후 best-by-sem). rank = conf desc
        cands = sorted(c["cands"], key=lambda x: -x["conf"])
        sel_uid = c["best"]["uid"] if c["best"] else None
        sems = [x["sem"] for x in cands]; appes = [x["appe"] for x in cands]; hsvs = [x["hsv"] for x in cands]
        for rank, x in enumerate(cands, start=1):
            a = aug.get((x["uid"], obj), {})
            key = f"{x['uid']}||{obj}"
            arr = vs[key] if key in vs.files else None
            row = {
                "dataset": ds, "frame_id": fr, "object": obj, "uid": x["uid"],
                "gt_visible": int(obj in gt.get((ds, fr), set())),
                "conf": round(x["conf"], 4), "conf_rank": rank, "n_cands": len(cands),
                "is_selected": int(x["uid"] == sel_uid),
                "sem_top5": round(x["sem"], 5), "appe11": round(x["appe"], 5), "hsv": round(x["hsv"], 5),
                "sim_thr": x["simthr"], "appe_gate": x["appegate"], "hsv_thr": p21.T,
                "passS": int(x["passS"]), "passA": int(x["passA"]), "passH": int(x["passH"]),
                "accept": int(x["accept"]),
                # 조인 feature(무료 aggregations)
                "sem_all_mean": float(a.get("sem_all_mean", x["sem"])),
                "sem_top1": float(a.get("sem_top1", x["sem"])),
                "sem_median": float(a.get("sem_median", x["sem"])),
                "appe2": float(a.get("appe2_clstop1", 0.0)),
                "appe9": float(a.get("appe9_clstop1", 0.0)),
                # cell 내부 상대 margin
                "sem_margin_vs_2nd": round(x["sem"] - (sorted(sems, reverse=True)[1] if len(sems) > 1 else x["sem"]), 5),
                "appe_margin_vs_2nd": round(x["appe"] - (sorted(appes, reverse=True)[1] if len(appes) > 1 else x["appe"]), 5),
                # 42뷰 통계(multi-template agreement)
                "view_pass_count": int((arr >= x["simthr"]).sum()) if arr is not None else -1,
                "view_median": round(float(np.median(arr)), 5) if arr is not None else -1,
                "view_std": round(float(np.std(arr)), 5) if arr is not None else -1,
                "view_top1_minus_median": round(float(arr.max() - np.median(arr)), 5) if arr is not None else -1,
            }
            rows.append(row)

    os.makedirs(os.path.join(OUT, "csv"), exist_ok=True)
    p = os.path.join(OUT, "csv", "candidate_pool.csv")
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    prov = {"phase1c_repro": grid, "n_candidate_rows": len(rows),
            "n_cells_with_candidate": len(set((r["dataset"], r["frame_id"], r["object"]) for r in rows)),
            "note": "authoritative gate flags = cur-hsv (Phase1C exact); features joined by uid"}
    json.dump(prov, open(os.path.join(OUT, "metrics", "pool_provenance.json"), "w"), indent=2, ensure_ascii=False)
    print(f"candidate_pool rows={len(rows)} -> {p}")
    print(json.dumps(prov, ensure_ascii=False))


if __name__ == "__main__":
    main()
