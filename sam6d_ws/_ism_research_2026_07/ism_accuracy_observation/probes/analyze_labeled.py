#!/usr/bin/env python3
"""analyze_labeled.py — 라벨 기반 통합 분석 (READ-ONLY).

  (1) proposal group 의 정답 클래스 Top-1/2/3 포함률
  (2) HSV(렌더 prototype vs 실사 prototype)의 객체별 성능 + hard negative 집중
  (3) semantic / appearance / HSV 상관·오류 독립성
  (4) fusion 후보 오프라인 비교 (leave-one-dataset-out)
"""
import csv, glob, json, os
from collections import defaultdict, Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES = os.path.join(ROOT, "results")

labels = {r["uid"]: r["true_class"] for r in csv.DictReader(
    open(os.path.join(ROOT, "labels", "box_labels_merged.csv")))}
pairs = []
for f in sorted(glob.glob(os.path.join(ROOT, "frame_dumps", "*_pairs.csv"))):
    pairs += list(csv.DictReader(open(f)))
boxes = {}
for f in sorted(glob.glob(os.path.join(ROOT, "frame_dumps", "*_boxes.csv"))):
    for r in csv.DictReader(open(f)):
        boxes[r["uid"]] = r
NUM = ("sem_top5 sem_top1 sem_mean appe11_clstop1 appe11_max42 appe9_max42 appe2_max42 "
       "hsv_render_masked hsv_render_bbox yolo_conf sim_thr appe_gate").split()
for r in pairs:
    for k in NUM:
        r[k] = float(r[k])
    r["is_candidate"] = int(r["is_candidate"])

# ---- HSV 실사 prototype 점수 (LODO) 계산 ----
hist, uids = [], []
for f in sorted(glob.glob(os.path.join(ROOT, "hsv_features", "sam_*_hsv.npz"))):
    z = np.load(f, allow_pickle=True)
    uids += list(z["uid"]); hist.append(z["hist"])
hist = np.concatenate(hist, 0)
UI = {u: i for i, u in enumerate(uids)}
OBJS = sorted({r["object"] for r in pairs})


def hs32(v, region_off):
    a = v[..., region_off:region_off+64]          # H32|S32
    s = a.sum(-1, keepdims=True)
    return a / np.maximum(s, 1e-12)


def bhatt(A, b):
    return np.sqrt(np.maximum(A, 0) * np.maximum(b, 0)[None]).sum(1)


lab_uids = [u for u in uids if labels.get(u) not in (None, "unclear")]
ds_of = {u: u.split("|")[0] for u in uids}
DS = sorted({ds_of[u] for u in lab_uids})
Q = hs32(hist, 0)                                  # masked region

# LODO 실사 prototype 점수: 각 박스는 '자기 데이터셋을 제외한' 레퍼런스로 채점
real_score = {}                                    # (uid,obj) -> score
for held in DS:
    for o in OBJS:
        ref = [UI[u] for u in lab_uids if labels[u] == o and ds_of[u] != held]
        if len(ref) < 3:
            continue
        A = Q[ref]
        for u in [x for x in uids if ds_of[x] == held]:
            real_score[(u, o)] = float(bhatt(A, Q[UI[u]]).max())
for r in pairs:
    r["hsv_real"] = real_score.get((r["uid"], r["object"]), np.nan)


def auroc(pos, neg):
    if not pos or not neg:
        return None
    a = sorted([(v, 1) for v in pos] + [(v, 0) for v in neg])
    n = len(a); rr = [0.0]*n; i = 0
    while i < n:
        j = i
        while j+1 < n and a[j+1][0] == a[i][0]:
            j += 1
        avg = (i+j)/2.0 + 1
        for k in range(i, j+1):
            rr[k] = avg
        i = j+1
    rp = sum(rr[k] for k in range(n) if a[k][1] == 1)
    return (rp - len(pos)*(len(pos)+1)/2) / (len(pos)*len(neg))


def spearman(a, b):
    a, b = np.asarray(a), np.asarray(b)
    m = ~(np.isnan(a) | np.isnan(b))
    ra = np.argsort(np.argsort(a[m])); rb = np.argsort(np.argsort(b[m]))
    return float(np.corrcoef(ra, rb)[0, 1])


CAND = [r for r in pairs if r["is_candidate"] and labels.get(r["uid"]) not in (None, "unclear")]
print(f"labeled candidate pairs = {len(CAND)}")

# =============== (1) proposal group GT rank ===============
cand = defaultdict(list)
for r in pairs:
    if r["is_candidate"]:
        cand[(r["dataset"], r["frame_id"], r["object"])].append(r)
accepted = set()
for k, v in cand.items():
    b = max(v, key=lambda x: x["sem_top5"])
    if b["sem_top5"] >= b["sim_thr"] and b["appe11_clstop1"] >= b["appe_gate"]:
        accepted.add((k, b["uid"]))
acc_uid = defaultdict(set)
for (k, u) in accepted:
    acc_uid[u].add(k[2])

byuid = defaultdict(dict)
for r in pairs:
    byuid[r["uid"]][r["object"]] = r
g = defaultdict(list)
for uid, b in boxes.items():
    g[(b["dataset"], b["frame_id"], int(b["group_iou50"]))].append(uid)

rank_rows, stat = [], Counter()
gaps_correct, gaps_wrong = [], []
for key, us in g.items():
    lu = [u for u in us if labels.get(u) not in (None, "unclear")]
    if not lu:
        continue
    gt = Counter(labels[u] for u in lu).most_common(1)[0][0]
    if gt in ("carton", "other"):
        gt_known = False
    else:
        gt_known = True
    best = {}
    for u in us:
        for o, r in byuid[u].items():
            if r["is_candidate"] and (o not in best or r["appe11_clstop1"] > best[o]["appe11_clstop1"]):
                best[o] = r
    if len(best) < 1:
        continue
    for skey, sname in (("appe11_clstop1", "appe"), ("sem_top5", "sem"),
                        ("yolo_conf", "yolo"), ("hsv_real", "hsvreal")):
        rk = sorted(best.items(), key=lambda kv: (kv[1][skey] if not np.isnan(kv[1][skey]) else -9),
                    reverse=True)
        cls = [o for o, _ in rk]
        if gt_known:
            rankv = cls.index(gt)+1 if gt in cls else 99
            stat[f"{sname}_n"] += 1
            stat[f"{sname}_top1"] += (rankv == 1)
            stat[f"{sname}_top2"] += (rankv <= 2)
            stat[f"{sname}_top3"] += (rankv <= 3)
            stat[f"{sname}_out"] += (rankv > 3)
            if sname == "appe":
                if len(rk) >= 2:
                    d = rk[0][1][skey] - rk[1][1][skey]
                    (gaps_correct if rankv == 1 else gaps_wrong).append(d)
                if rankv == 2:
                    stat["appe_wrongtop1_gt_top2"] += 1
                if rankv == 3:
                    stat["appe_wrongtop1_gt_top3"] += 1
    accs = sorted({o for u in us for o in acc_uid.get(u, ())})
    rank_rows.append({"dataset": key[0], "frame_id": key[1], "group": key[2],
                      "gt_class": gt if gt_known else f"non-target({gt})",
                      "n_labeled_boxes": len(lu),
                      "appe_top3": ";".join(cls[:3]),
                      "accepted": ";".join(accs), "accepted_count": len(accs)})

with open(os.path.join(RES, "proposal_group_gt_rank.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rank_rows[0].keys())); w.writeheader(); w.writerows(rank_rows)
print("\n=== (1) 정답 클래스 순위 (라벨된 proposal group, IoU 0.5) ===")
for s in ("appe", "sem", "yolo", "hsvreal"):
    n = stat[f"{s}_n"]
    if n:
        print(f"  {s:8s} n={n:5d} top1={stat[f'{s}_top1']/n:.1%} top2={stat[f'{s}_top2']/n:.1%} "
              f"top3={stat[f'{s}_top3']/n:.1%} out-of-top3={stat[f'{s}_out']/n:.1%}")
print(f"  오답 Top-1 인데 정답이 Top-2: {stat['appe_wrongtop1_gt_top2']}건 / Top-3: {stat['appe_wrongtop1_gt_top3']}건")
if gaps_correct and gaps_wrong:
    print(f"  Top1-Top2 gap  정답Top1: median {np.median(gaps_correct):.4f} / "
          f"오답Top1: median {np.median(gaps_wrong):.4f}")

# =============== (2) 객체별 HSV 성능 ===============
rows = []
for o in OBJS:
    sub = [r for r in CAND if r["object"] == o and not np.isnan(r["hsv_real"])]
    pos = [r for r in sub if labels[r["uid"]] == o]
    neg = [r for r in sub if labels[r["uid"]] != o]
    hardn = [r for r in neg if labels[r["uid"]] in ("carton", "other")]
    if not pos or not neg:
        continue
    row = {"object": o, "n_pos": len(pos), "n_neg": len(neg), "n_hard_neg": len(hardn),
           "auroc_hsv_render": round(auroc([r["hsv_render_masked"] for r in pos],
                                           [r["hsv_render_masked"] for r in neg]) or 0, 3),
           "auroc_hsv_real": round(auroc([r["hsv_real"] for r in pos],
                                         [r["hsv_real"] for r in neg]) or 0, 3),
           "auroc_sem_top5": round(auroc([r["sem_top5"] for r in pos],
                                         [r["sem_top5"] for r in neg]) or 0, 3),
           "auroc_appe11": round(auroc([r["appe11_clstop1"] for r in pos],
                                       [r["appe11_clstop1"] for r in neg]) or 0, 3)}
    if hardn:
        row["auroc_hsv_real_vs_hardneg"] = round(auroc([r["hsv_real"] for r in pos],
                                                       [r["hsv_real"] for r in hardn]) or 0, 3)
        row["auroc_appe11_vs_hardneg"] = round(auroc([r["appe11_clstop1"] for r in pos],
                                                     [r["appe11_clstop1"] for r in hardn]) or 0, 3)
        row["auroc_sem_vs_hardneg"] = round(auroc([r["sem_top5"] for r in pos],
                                                  [r["sem_top5"] for r in hardn]) or 0, 3)
    row["spearman_hsv_sem"] = round(spearman([r["hsv_real"] for r in sub],
                                             [r["sem_top5"] for r in sub]), 3)
    row["spearman_hsv_appe"] = round(spearman([r["hsv_real"] for r in sub],
                                              [r["appe11_clstop1"] for r in sub]), 3)
    rows.append(row)
with open(os.path.join(RES, "hsv_object_summary.csv"), "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=sorted({k for r in rows for k in r},
                                            key=lambda x: (x != "object", x)))
    w.writeheader(); w.writerows(rows)
print("\n=== (2) 객체별 HSV / semantic / appearance AUROC ===")
print(f"{'object':22s} {'pos':>4s} {'neg':>5s} {'HSV렌더':>7s} {'HSV실사':>7s} {'sem':>6s} {'appe':>6s} "
      f"{'HSV실사vsHN':>10s} {'appevsHN':>9s} {'ρ(HSV,sem)':>10s} {'ρ(HSV,appe)':>11s}")
for r in rows:
    print(f"{r['object']:22s} {r['n_pos']:4d} {r['n_neg']:5d} {r['auroc_hsv_render']:7.3f} "
          f"{r['auroc_hsv_real']:7.3f} {r['auroc_sem_top5']:6.3f} {r['auroc_appe11']:6.3f} "
          f"{r.get('auroc_hsv_real_vs_hardneg','-'):>10} {r.get('auroc_appe11_vs_hardneg','-'):>9} "
          f"{r['spearman_hsv_sem']:10.3f} {r['spearman_hsv_appe']:11.3f}")

# =============== (3)(4) fusion 오프라인 비교 (LODO) ===============
def calib_th(rows_, sc):
    th = {}
    for o in OBJS:
        sub = [r for r in rows_ if r["object"] == o]
        if not sub:
            th[o] = -1e9; continue
        best, bt = -1, -1e9
        for t in sorted({round(sc(r), 4) for r in sub}):
            tp = sum(1 for r in sub if sc(r) >= t and labels[r["uid"]] == o)
            fp = sum(1 for r in sub if sc(r) >= t and labels[r["uid"]] != o)
            fn = sum(1 for r in sub if sc(r) < t and labels[r["uid"]] == o)
            p = tp/(tp+fp) if tp+fp else 0; rc = tp/(tp+fn) if tp+fn else 0
            f1 = 2*p*rc/(p+rc) if p+rc else 0
            if f1 > best:
                best, bt = f1, t
        th[o] = bt
    return th


# 결정 단위: semantic 승자 (운영과 동일)
W = []
for k, v in cand.items():
    b = max(v, key=lambda x: x["sem_top5"])
    if b["sem_top5"] >= b["sim_thr"] and labels.get(b["uid"]) not in (None, "unclear") \
            and not np.isnan(b["hsv_real"]):
        W.append(b)
print(f"\n라벨된 semantic 승자 결정 = {len(W)}")

FUS = {
    "S0_baseline_appe": lambda r: r["appe11_clstop1"],
    "S1_sem": lambda r: r["sem_top5"],
    "S2_hsv_real": lambda r: r["hsv_real"],
    "S3_hsv_render": lambda r: r["hsv_render_masked"],
    "S4_sem+appe": lambda r: 0.5*r["sem_top5"]+0.5*r["appe11_clstop1"],
    "S5_sem+hsv": lambda r: 0.5*r["sem_top5"]+0.5*r["hsv_real"],
    "S6_appe+hsv": lambda r: 0.5*r["appe11_clstop1"]+0.5*r["hsv_real"],
    "S7_sem+appe+hsv": lambda r: (r["sem_top5"]+r["appe11_clstop1"]+r["hsv_real"])/3,
}
res = {}
for name, sc in FUS.items():
    agg = Counter(); conf = Counter()
    for held in DS:
        tr = [r for r in W if r["dataset"] != held]
        te = [r for r in W if r["dataset"] == held]
        if not tr or not te:
            continue
        th = calib_th(tr, sc)
        for r in te:
            a = sc(r) >= th[r["object"]]
            pos = labels[r["uid"]] == r["object"]
            k = "TP" if (a and pos) else "FP" if a else "FN" if pos else "TN"
            agg[k] += 1
            if k == "FP":
                conf[f'{labels[r["uid"]]}->{r["object"]}'] += 1
    tp, fp, fn = agg["TP"], agg["FP"], agg["FN"]
    p = tp/(tp+fp) if tp+fp else 0; rc = tp/(tp+fn) if tp+fn else 0
    res[name] = {"TP": tp, "FP": fp, "FN": fn, "TN": agg["TN"],
                 "precision": round(p, 4), "recall": round(rc, 4),
                 "f1": round(2*p*rc/(p+rc), 4) if p+rc else 0,
                 "top_confusions": dict(conf.most_common(6))}
# 운영 고정 임계
agg = Counter(); conf = Counter()
for r in W:
    a = r["appe11_clstop1"] >= r["appe_gate"]
    pos = labels[r["uid"]] == r["object"]
    k = "TP" if (a and pos) else "FP" if a else "FN" if pos else "TN"
    agg[k] += 1
    if k == "FP":
        conf[f'{labels[r["uid"]]}->{r["object"]}'] += 1
tp, fp, fn = agg["TP"], agg["FP"], agg["FN"]
res["S0_operational_fixed_gate"] = {"TP": tp, "FP": fp, "FN": fn, "TN": agg["TN"],
    "precision": round(tp/(tp+fp), 4) if tp+fp else 0,
    "recall": round(tp/(tp+fn), 4) if tp+fn else 0,
    "top_confusions": dict(conf.most_common(6))}
json.dump({"gt_rank": dict(stat), "fusion_lodo": res},
          open(os.path.join(RES, "labeled_analysis.json"), "w"), indent=2, ensure_ascii=False)
print("\n=== (4) fusion 오프라인 비교 (leave-one-dataset-out, 재보정) ===")
for k, v in res.items():
    print(f"  {k:28s} P{v['precision']:.3f} R{v['recall']:.3f} FP{v['FP']:4d} FN{v['FN']:4d}  {list(v['top_confusions'])[:3]}")
