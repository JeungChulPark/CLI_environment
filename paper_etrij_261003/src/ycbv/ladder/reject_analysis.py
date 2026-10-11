"""개선안 2(검증 끔) 에는 있고 개선안 3(검증 켬) 에는 없는 답(= 검증이 버린 답)을 정답 여부와 탈락 사유로 나눈다."""
import json, sys, collections, numpy as np, trimesh
sys.path.insert(0, "/home/jucpark/DeepLearning/CLI_env_paper/paper_etrij_261003/src/ycbv")
import data as D, paths as P
from evaluate import adds_mm
OUT = "/home/jucpark/DeepLearning/server/CLI_environment/paper_etrij_261003/src/ycbv/out_4090/"
info = json.load(open(P.YCBV / "models_eval" / "models_info.json"))
models = {oid: np.asarray(trimesh.load(str(P.YCBV / "models_eval" / f"obj_{oid:06d}.ply"), process=False).vertices, float) for oid in range(1, 22)}
noverify = json.load(open(OUT + "pred_ladder_L1_th05.json"))
def best_per_img(pred):
    out = {}
    for x in pred["images"]:
        b = {}
        for p in x["preds"]:
            if p["obj_id"] not in b or p["score"] > b[p["obj_id"]]["score"]: b[p["obj_id"]] = p
        out[(x["scene_id"], x["im_id"])] = b
    return out
nv = best_per_img(noverify)
def correct(sid, iid, oid, p):
    gts = [g for g in D.gt_instances(sid, iid) if g["obj_id"] == oid]
    e = min(adds_mm(models[oid], g["R"], g["t_mm"], p["R"], p["t_mm"]) for g in gts)
    return e < 0.1 * info[str(oid)]["diameter"], e / info[str(oid)]["diameter"]
for fn, name in [("pred_hybS05_900.json", "개선안 3"), ("pred_hybS05_cf_900.json", "개선안 3 + 군집 먼저")]:
    ver = json.load(open(OUT + fn)); vb = best_per_img(ver)
    rej_rec = {}
    for x in ver["images"]:
        for r in (x.get("pose_rejected") or []):
            rej_rec[(x["scene_id"], x["im_id"], int(r["object"].split("_")[1]))] = r
    rows = []
    n_only_ver = 0
    for key, b2 in nv.items():
        b3 = vb.get(key, {})
        n_only_ver += len(set(b3) - set(b2))
        for oid, p in b2.items():
            if oid in b3: continue
            ok, rel = correct(key[0], key[1], oid, p)
            r = rej_rec.get((key[0], key[1], oid), {})
            rows.append(dict(sid=key[0], iid=key[1], oid=oid, ok=bool(ok), rel_err=rel, reason=r.get("rejection_reason", "(기록 없음)"),
                             mask_iou=r.get("mask_iou"), tex=r.get("texture_score"), occ=r.get("cluster_occupancy")))
    print(f"\n######## {name}: 버린 답 {len(rows)}개 (맞는 답 {sum(r['ok'] for r in rows)}, 오답 {sum(not r['ok'] for r in rows)}); 검증 켬에만 있는 답 {n_only_ver}")
    by = collections.defaultdict(lambda: [0, 0])
    for r in rows: by[r["reason"]][0 if r["ok"] else 1] += 1
    print(f"{'탈락 사유':34s} {'맞는 답':>7s} {'오답':>5s} {'맞는 비율':>8s}")
    for k, (c, w) in sorted(by.items(), key=lambda kv: -sum(kv[1])):
        print(f"{k:34s} {c:7d} {w:5d} {100*c/(c+w):7.0f}%")
    # 버려진 맞는 답의 오차 분포 (지름 대비)
    rel_ok = np.array([r["rel_err"] for r in rows if r["ok"]]); rel_w = np.array([r["rel_err"] for r in rows if not r["ok"]])
    print(f"버려진 맞는 답의 ADD-S/지름: 중앙값 {np.median(rel_ok):.3f}, 75% {np.percentile(rel_ok,75):.3f}, 90% {np.percentile(rel_ok,90):.3f}  (정답 기준 0.10)")
    print(f"버려진 오답의 ADD-S/지름: 중앙값 {np.median(rel_w):.3f}, 25% {np.percentile(rel_w,25):.3f}  (0.1~0.5 사이 = BOP AR 에서 부분 점수 받는 구간: {np.mean((rel_w>0.1)&(rel_w<0.5))*100:.0f}%)")
    # 문턱 완화 시뮬레이션: 기록된 점수 하나만 낮추면 통과했을 답 (다른 관문은 그대로라고 가정)
    def sweep(field, reason, ths, cur):
        sub = [r for r in rows if r["reason"] == reason and r[field] is not None]
        print(f"  [{reason}] 현재 문턱 {cur}: 대상 {len(sub)}개 (맞는 답 {sum(r['ok'] for r in sub)})")
        for t in ths:
            c = sum(1 for r in sub if r["ok"] and r[field] >= t); w = sum(1 for r in sub if (not r["ok"]) and r[field] >= t)
            print(f"     문턱 {t:.2f} 로 낮추면: 맞는 답 {c:3d}개 회복, 오답 {w:3d}개 유입")
    print("문턱 완화 시뮬레이션 (기록된 점수 기준, 근사):")
    sweep("occ", "convergence_insufficient", [0.25, 0.20, 0.15, 0.10, 0.05], 0.30)
    sweep("tex", "final_texture_below_threshold", [0.40, 0.35, 0.30, 0.25], 0.4496)
    sweep("mask_iou", "final_mask_below_threshold", [0.40, 0.35, 0.30, 0.25], 0.421)
    # 점수 분포: 맞는 답 vs 오답 (사유별 중앙값)
    print("사유별 기록 점수 중앙값 (맞는 답 | 오답): 윤곽IoU / 무늬 / 합의")
    for k in by:
        for lab, flag in (("맞음", True), ("오답", False)):
            sub = [r for r in rows if r["reason"] == k and r["ok"] == flag and r["occ"] is not None]
            if sub: print(f"  {k:34s} {lab}: n={len(sub):3d}  {np.median([r['mask_iou'] for r in sub]):.2f} / {np.median([r['tex'] for r in sub]):.2f} / {np.median([r['occ'] for r in sub]):.2f}")
    json.dump(rows, open(OUT + f"reject_analysis_{fn.replace('pred_','').replace('.json','')}.json", "w"), ensure_ascii=False, indent=0)
