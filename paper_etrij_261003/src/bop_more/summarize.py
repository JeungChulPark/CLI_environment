"""Summary table + rejection-reason counts for src/bop_more results (CPU only). usage: python3 summarize.py <out_root> ds1,ds2,..."""
import json, sys, collections, pathlib
root = pathlib.Path(sys.argv[1]); dss = sys.argv[2].split(",")
rows = []; rej_tab = {}; per_obj = {}
for ds in dss:
    d = root / ds
    rp = next(iter(sorted(d.glob("results_*.json"))), None)
    if rp is None: print("no results for", ds); continue
    R = json.load(open(rp))["methods"]; S = json.load(open(next(iter(d.glob("same_object_*.json")))))
    pair = next(iter(S["pairs"].values()))
    for key, name, so in (("ladder_L0", "SAM-6D L0 (원본)", "L0_원본"), ("ladder_hybS05cf", "개선안 3 + 군집 먼저", "개선안3_cf")):
        m = R[key]; ar = m.get("bop_ar", {})
        rows.append((ds, name, m["images"], m["visible_gt_targets"], m["answers"], m["found_pct"], m["answers_correct_pct"], m["wrong_per_image"],
                     m["adds_auc"], ar.get("bop19_average_recall"), ar.get("bop19_average_recall_vsd"), ar.get("bop19_average_recall_mssd"), ar.get("bop19_average_recall_mspd"),
                     m["time_ms"]["median"], pair[so]["adds_auc"], pair[so]["median_err_mm"], pair["common_answered"], m["gpu"]))
        per_obj[(ds, key)] = m["per_object"]
    pred = json.load(open(d / "pred_hybS05_cf.json"))
    c = collections.Counter(); ism_acc = 0; n_t = 0
    for im in pred["images"]:
        n_t += len(im["targets"]); ism_acc += len(im.get("ism_accepted", []))
        for r in im.get("pose_rejected", []): c[r.get("rejection_reason")] += 1
    rej_tab[ds] = {"targets": n_t, "ism_accepted(orig ISM score>0.5)": ism_acc, "answers": R["ladder_hybS05cf"]["answers"], "rejections": dict(c.most_common())}
print("| dataset | method | images | targets | answers | found % | answers correct % | wrong/img | ADD-S AUC | BOP AR | VSD | MSSD | MSPD | median ms/img | same-object ADD-S AUC | same-object median err mm | common answered |")
print("|" + "---|" * 17)
for r in rows: print("| " + " | ".join(str(x) for x in r[:17]) + " |")
print("\nGPU:", sorted({r[17] for r in rows}))
print("\nRejection reasons of our verifier (pred_hybS05_cf.json, pose_rejected per image):")
for ds, v in rej_tab.items(): print(ds, json.dumps(v, ensure_ascii=False))
print("\nPer-object found % (L0 / ours):")
for ds in dss:
    a = per_obj.get((ds, "ladder_L0")); b = per_obj.get((ds, "ladder_hybS05cf"))
    if a: print(ds, {k: (a[k]["found_pct"], b[k]["found_pct"], a[k]["targets"]) for k in a})
