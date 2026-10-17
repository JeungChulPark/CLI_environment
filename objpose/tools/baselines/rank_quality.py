#!/usr/bin/env python3
"""rank_quality.py — does the score separate right from wrong? For the original (PEM score) and the
proposed score mode (verification tiers + PEM score): take the top-N answers by score, N = number of
proposed high-score (>= 3) answers, and count the wrong ones (ADD-S > 0.1 d)."""
import json, os
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
OUT = Path(os.environ["CMP_OUT"])
DS = {"ycbv": ("objpose/rt/bop/ycbv_objects.json", "/home/jucpark/DeepLearning/Datasets/paper_benchmarks/YCB_Video"),
      "lmo": ("objpose/rt/bop/lmo_objects.json", "/home/jucpark/DeepLearning/Datasets/paper_benchmarks/bop/lmo")}
PTS = REPO / "sam6d_realtime/assets/model_points"
res = {}
lists_all = {}
for ds, (tab, root) in DS.items():
    name = {o["obj_id"]: o["name"] for o in json.loads((REPO / tab).read_text())["objects"]}
    info = json.loads((Path(root) / "models_eval/models_info.json").read_text())
    objs = {}
    for oid, nm in name.items():
        p = np.load(PTS / f"{nm}.npy").astype(float)
        if len(p) > 2000: p = p[np.random.default_rng(0).choice(len(p), 2000, replace=False)]
        objs[nm] = (p, cKDTree(p), 0.1 * info[str(oid)]["diameter"])
    lists = {}
    for m in ("orig", "proposedscore"):
        L = []
        for sc in sorted(p for p in (OUT / f"{m}_{ds}_bop19").iterdir() if p.is_dir()):
            scene = Path(root) / "test" / sc.name
            gt = json.loads((scene / "scene_gt.json").read_text()); gi = json.loads((scene / "scene_gt_info.json").read_text())
            for l in (sc / "sam6d_frames.jsonl").read_text().splitlines():
                if not l.strip(): continue
                f = json.loads(l); k = str(f["im_id"])
                G = {}
                for r, i in zip(gt[k], gi[k]):
                    if i.get("visib_fract", 1) >= 0.1 and r["obj_id"] in name:
                        T = np.eye(4); T[:3, :3] = np.reshape(r["cam_R_m2c"], (3, 3)); T[:3, 3] = r["cam_t_m2c"]; G[name[r["obj_id"]]] = T
                best = {}
                for d in f["dets"]:
                    if d["object"] not in best or d["score"] > best[d["object"]]["score"]: best[d["object"]] = d
                for nm, d in best.items():
                    ok = False
                    if nm in G:
                        p, tree, thr = objs[nm]
                        E = np.eye(4); E[:3, :3] = np.reshape(d["R"], (3, 3)); E[:3, 3] = d["t_mm"]
                        D = np.linalg.inv(G[nm]) @ E
                        ok = tree.query(p @ D[:3, :3].T + D[:3, 3])[0].mean() < thr
                    L.append((d["score"], ok))
        lists[m] = sorted(L, key=lambda t: -t[0])
    lists_all[ds] = lists
    N = sum(1 for s, _ in lists["proposedscore"] if s >= 3.0)
    for m, L in lists.items():
        top = L[:N]
        res[f"{m}_{ds}"] = {"N": N, "wrong_in_topN": int(sum(1 for _, ok in top if not ok)), "total": len(L),
                            "wrong_total": int(sum(1 for _, ok in L if not ok))}
        print(ds, m, res[f"{m}_{ds}"])
(OUT / "rank_quality.json").write_text(json.dumps(res, indent=1))

# curve: wrong answers among the top-N, N from 30% to 100% of all answers, plus the pose-verified-only ranking
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
fm.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
plt.rcParams.update({"font.family": "Noto Sans CJK JP", "axes.spines.top": False, "axes.spines.right": False, "font.size": 9})
curves = {}
fig, axs = plt.subplots(1, 2, figsize=(8.6, 3.0), dpi=220)
for ax, ds in zip(axs, ("ycbv", "lmo")):
    for m, col, lab in (("orig", "#d9822b", "원본 PEM 점수로 정렬"), ("proposedscore", "#18334b", "제안 방법 점수 (검증 등급 + PEM)")):
        L = lists_all[ds][m]
        w = np.cumsum([0 if ok else 1 for _, ok in L]); n = np.arange(1, len(L) + 1)
        ax.plot(100 * n / len(L), w, color=col, lw=2, label=lab)
        curves[f"{m}_{ds}"] = {int(p): int(w[int(len(L) * p / 100) - 1]) for p in (50, 70, 80, 90, 100)}
    ax.set_xlim(30, 100); ax.set_xlabel("점수 상위 몇 %의 답을 쓰나"); ax.set_ylabel("그 안의 틀린 답 수")
    ax.set_title("YCB-V" if ds == "ycbv" else "LM-O", fontsize=9.5); ax.grid(alpha=.25)
axs[0].legend(frameon=False, fontsize=8, loc="upper left")
fig.tight_layout(); fig.savefig(OUT / "rank_curve.png", facecolor="white")
print(json.dumps(curves))
(OUT / "rank_curve.json").write_text(json.dumps(curves, indent=1))
