#!/usr/bin/env python3
"""analyze_top3.py — 검증 1: proposal group 단위 클래스 Top-3 관찰 (READ-ONLY).

라벨이 없어도 계산 가능한 것과 라벨이 필요한 것을 분리해 산출한다.
  라벨 불필요: 복수 클래스 고득점/동시수락 비율, Top1-Top2 gap 분포, 혼동 쌍,
              IoU 임계 민감도(0.3/0.5/0.7)
  라벨 필요  : 정답 클래스 Top-1/2/3 포함률 → labels/proposal_group_labels.csv 가 있을 때만
"""
import csv, glob, json, os, statistics
from collections import defaultdict, Counter

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FD = os.path.join(ROOT, "frame_dumps")
RES = os.path.join(ROOT, "results")
os.makedirs(RES, exist_ok=True)

pairs, boxes = [], {}
for f in sorted(glob.glob(os.path.join(FD, "*_pairs.csv"))):
    pairs += list(csv.DictReader(open(f)))
for f in sorted(glob.glob(os.path.join(FD, "*_boxes.csv"))):
    for r in csv.DictReader(open(f)):
        boxes[r["uid"]] = r
NUM = ("sem_top5 sem_top1 sem_mean appe11_clstop1 appe11_max42 appe9_max42 appe2_max42 "
       "hsv_render_masked hsv_render_bbox yolo_conf sim_thr appe_gate").split()
for r in pairs:
    for k in NUM:
        r[k] = float(r[k])
    r["is_candidate"] = int(r["is_candidate"])
print(f"pairs={len(pairs)} boxes={len(boxes)}")

# ---------------------------------------------- 운영 baseline 재현 (수락 판정)
cand = defaultdict(list)
for r in pairs:
    if r["is_candidate"]:
        cand[(r["dataset"], r["frame_id"], r["object"])].append(r)
accepted = {}          # (ds,frame,obj) -> row
winner = {}
for k, v in cand.items():
    b = max(v, key=lambda x: x["sem_top5"])
    winner[k] = b
    if b["sem_top5"] >= b["sim_thr"] and b["appe11_clstop1"] >= b["appe_gate"]:
        accepted[k] = b
print(f"후보 (frame,object) 그룹 {len(cand)} / semantic 승자 {len(winner)} / 최종 수락 {len(accepted)}")

# ---------------------------------------------- proposal group 구성 (IoU 민감도)
out_rows = []
summary = {}
for thr, key in ((0.3, "group_iou30"), (0.5, "group_iou50"), (0.7, "group_iou70")):
    g = defaultdict(list)          # (ds,frame,gid) -> [uid]
    for uid, b in boxes.items():
        g[(b["dataset"], b["frame_id"], int(b[key]))].append(uid)
    byuid = defaultdict(dict)
    for r in pairs:
        byuid[r["uid"]][r["object"]] = r
    n_multi_high = n_multi_acc = n_multi_acc3 = 0
    gaps, acc_counts = [], []
    conf_pairs = Counter()
    total = 0
    for (ds, fr, gid), uids in g.items():
        # 이 그룹에서 각 클래스의 최고 점수 (그룹 내 어떤 박스든)
        best = {}
        accs = set()
        for u in uids:
            for o, r in byuid[u].items():
                if not r["is_candidate"]:
                    continue
                if o not in best or r["appe11_clstop1"] > best[o]["appe11_clstop1"]:
                    best[o] = r
                k = (ds, fr, o)
                if k in accepted and accepted[k]["uid"] == u:
                    accs.add(o)
        if not best:
            continue
        total += 1
        ranked = sorted(best.items(), key=lambda kv: kv[1]["appe11_clstop1"], reverse=True)
        # "고득점" = 해당 객체의 운영 appe_gate 를 넘김
        high = [o for o, r in ranked if r["appe11_clstop1"] >= r["appe_gate"]
                and r["sem_top5"] >= r["sim_thr"]]
        if len(high) >= 2:
            n_multi_high += 1
        if len(accs) >= 2:
            n_multi_acc += 1
            for i, a in enumerate(sorted(accs)):
                for b2 in sorted(accs)[i+1:]:
                    conf_pairs[f"{a}+{b2}"] += 1
        if len(accs) >= 3:
            n_multi_acc3 += 1
        acc_counts.append(len(accs))
        if len(ranked) >= 2:
            gaps.append(ranked[0][1]["appe11_clstop1"] - ranked[1][1]["appe11_clstop1"])
        if thr == 0.5:
            t = [(o, r) for o, r in ranked[:3]]
            row = {"dataset": ds, "frame_id": fr, "proposal_group_id": gid,
                   "grouping_method": f"bbox IoU>{thr} union-find", "n_boxes": len(uids),
                   "n_classes_claiming": len(best),
                   "accepted_classes": ";".join(sorted(accs)), "accepted_class_count": len(accs),
                   "uid_list": ";".join(uids)}
            for i in range(3):
                if i < len(t):
                    o, r = t[i]
                    row[f"top{i+1}_class"] = o
                    row[f"top{i+1}_appe"] = r["appe11_clstop1"]
                    row[f"top{i+1}_sem"] = r["sem_top5"]
                    row[f"top{i+1}_yolo"] = r["yolo_conf"]
                    row[f"top{i+1}_hsv"] = r["hsv_render_masked"]
                else:
                    for s in ("class", "appe", "sem", "yolo", "hsv"):
                        row[f"top{i+1}_{s}"] = ""
            row["top1_top2_gap"] = round(gaps[-1], 4) if len(ranked) >= 2 else ""
            row["top2_top3_gap"] = (round(ranked[1][1]["appe11_clstop1"] - ranked[2][1]["appe11_clstop1"], 4)
                                    if len(ranked) >= 3 else "")
            # sem / yolo 기준 Top-3 도 별도 저장
            for skey, sname in (("sem_top5", "semtop"), ("yolo_conf", "yolotop")):
                rk = sorted(best.items(), key=lambda kv: kv[1][skey], reverse=True)[:3]
                row[f"{sname}_classes"] = ";".join(o for o, _ in rk)
                row[f"{sname}_scores"] = ";".join(f"{r[skey]:.4f}" for _, r in rk)
            out_rows.append(row)
    summary[f"iou_{thr}"] = {
        "proposal_groups": total,
        "multi_class_high_score": n_multi_high,
        "multi_class_high_rate": round(n_multi_high/max(1, total), 4),
        "multi_class_accepted": n_multi_acc,
        "multi_class_accept_rate": round(n_multi_acc/max(1, total), 4),
        "3plus_class_accepted": n_multi_acc3,
        "mean_accepted_per_group": round(statistics.fmean(acc_counts), 3) if acc_counts else 0,
        "top1_top2_gap_median": round(statistics.median(gaps), 4) if gaps else None,
        "top1_top2_gap_p10": round(sorted(gaps)[int(0.1*(len(gaps)-1))], 4) if gaps else None,
        "co_accepted_pairs": dict(conf_pairs.most_common(20)),
    }

with open(os.path.join(RES, "all_proposal_groups_top3.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys())); w.writeheader(); w.writerows(out_rows)

# 데이터셋별 동시수락
per_ds = defaultdict(lambda: [0, 0])
for r in out_rows:
    per_ds[r["dataset"]][0] += 1
    if r["accepted_class_count"] >= 2:
        per_ds[r["dataset"]][1] += 1
summary["per_dataset_iou0.5"] = {k: {"groups": v[0], "multi_accept": v[1],
                                     "rate": round(v[1]/max(1, v[0]), 4)} for k, v in sorted(per_ds.items())}
summary["baseline"] = {"candidate_groups": len(cand), "semantic_winners": len(winner),
                       "accepted_detections": len(accepted),
                       "accepted_per_object": dict(Counter(k[2] for k in accepted).most_common())}
json.dump(summary, open(os.path.join(RES, "class_top3_summary.json"), "w"), indent=2, ensure_ascii=False)
print(json.dumps(summary, indent=2, ensure_ascii=False)[:3000])
