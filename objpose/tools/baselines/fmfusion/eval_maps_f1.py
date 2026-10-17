#!/usr/bin/env python3
"""eval_maps_f1.py — 정적 장면(YCB-V, YCB-M)의 최종 객체 지도를 한눈에 읽히는 지표로 채점한다.

  예측   = 지도 객체 중 데이터셋 물체 이름이 붙은 것 (이름 없는 조각 · 배경 이름은 예측으로 세지 않고 따로 센다)
  맞음   = 같은 이름의 지도 객체 중 정답 중심에 가장 가까운 것, 10 cm 안일 때 (정답 하나에 하나만. 나머지 같은 이름 객체는 중복 = 틀림)
  정밀도 = 맞음 / 예측 수,  재현율 = 맞음 / 정답 물체 수,  F1 = 조화 평균
  표면 오차 [mm] = 맞은 객체의 지도 점 → 정답 CAD 표면 최근접 거리의 중앙값 (객체별 값의 중앙값 / 90%)
                   FM-Fusion · DualMap 은 복원한 점, Object Pose Map 은 추정 자세에 놓은 CAD 점
  방향 오차 [°]   = Object Pose Map 만 (점군 지도에는 방향이 없다)
좌표: 모든 지도는 같은 ORB-SLAM3 세계에 있고, build_memory_maps.run_memory 의 SE3 정렬(SLAM → 정답)을 한 번 적용한다.
BENCH_ROOT / BENCH_TABLE / BENCH_SLAM / BENCH_FPS 로 데이터셋을 바꾼다.

    <sam6d python> objpose/tools/baselines/fmfusion/eval_maps_f1.py --ours <dir> [--orig <dir>] [--fmf <dir>] [--dualmap <maps dir>] --out <dir>
"""
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d")); sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_memory_maps as BM   # noqa: E402
from eval_maps import read_ply_xyz, rot_err_deg   # noqa: E402

MATCH_M = 0.10


def score(items, gt_pts, gt_T, names):
    """items: [(name or None, points_world, R or None)]. 최종 지도만 보고 판정한다:
    같은 이름의 지도 객체 중 정답 중심에 가장 가까운 것이 10 cm 안이면 맞음, 나머지 같은 이름 객체는 중복. 가장 가까운 것도 10 cm 밖이면 모두 먼 위치."""
    trees = {nm: cKDTree(P) for nm, P in gt_pts.items()}
    cerr = [float(np.linalg.norm(P.mean(0) - gt_pts[nm].mean(0))) if nm in gt_pts else None for nm, P, R in items]   # eval_maps.py 와 같은 정의
    best = {}
    for i, (nm, P, R) in enumerate(items):
        if cerr[i] is not None and (nm not in best or cerr[i] < cerr[best[nm]]): best[nm] = i
    rows, used = [], set()
    for i, (nm, P, R) in enumerate(items):
        if nm not in names:
            rows.append({"kind": "unnamed"}); continue
        row = {"name": nm}
        if nm not in gt_pts:
            row["kind"] = "wrong_name"; rows.append(row); continue
        row.update(center_mm=cerr[i] * 1000, surface_mm=float(np.median(trees[nm].query(P)[0]) * 1000))
        if R is not None:
            row["rot_deg"] = rot_err_deg(R, gt_T[nm][:3, :3])
        ok = cerr[best[nm]] <= MATCH_M
        row["kind"] = ("match" if best[nm] == i else "duplicate") if ok else "far"
        if row["kind"] == "match": used.add(nm)
        rows.append(row)
    return rows, [nm for nm in gt_pts if nm not in used]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ours", type=Path, required=True); ap.add_argument("--orig", type=Path, default=None)
    ap.add_argument("--fmf", type=Path, default=None); ap.add_argument("--dualmap", type=Path, default=None)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--scenes", nargs="*", default=None); ap.add_argument("--dump", type=Path, default=None, help="장면별 정답 · 방법별 지도 객체(정답 좌표계)를 json 으로 남긴다")
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
    table = json.loads(BM.TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}; names = set(by_id.values())
    by_label = {o["ycb"].split("_", 1)[1].replace("_", " "): o["name"] for o in table}
    by_ycb = {o["ycb"]: o["name"] for o in table}
    model = {o["name"]: np.load(BM.PTS / f"{o['name']}.npy") / 1000.0 for o in table}
    per = {}
    scenes = sorted(p.name for p in a.ours.iterdir() if p.is_dir() and (p / "sam6d_frames.jsonl").exists())
    if a.scenes: scenes = [s for s in scenes if s in a.scenes]
    for s in scenes:
        BM.RUNS["rt"] = a.ours
        r = BM.run_memory(s, "rt", by_id); A = r["A"]
        gt_pts = {nm: model[nm] @ T[:3, :3].T + T[:3, 3] for nm, T in r["gobj"].items()}
        def opm(run):
            rr = r
            if run != a.ours:
                BM.RUNS["rt"] = run; rr = BM.run_memory(s, "rt", by_id)
            its = []
            for lm in sorted(rr["lms"], key=lambda m_: -m_["n_obs"]):
                T = A @ np.asarray(lm["T_w_obj"], float); its.append((lm["name"], model[lm["name"]] @ T[:3, :3].T + T[:3, 3], T[:3, :3]))
            return its
        cand = {"제안 방법": opm(a.ours)}
        if a.orig and (a.orig / s / "sam6d_frames.jsonl").exists():
            cand["SAM-6D + 같은 지도"] = opm(a.orig)
        if a.fmf and (a.fmf / s / "instance_info.txt").exists():
            it = []
            for line in (a.fmf / s / "instance_info.txt").read_text().splitlines():
                if line.startswith("#") or not line.strip(): continue
                f = line.split(";"); P = read_ply_xyz(a.fmf / s / f"{int(f[0]):04d}.ply")
                if len(P): it.append((int(f[2]), by_label.get(f[1].split("(")[0]), P @ A[:3, :3].T + A[:3, 3], None))
            cand["FM-Fusion"] = [x[1:] for x in sorted(it, key=lambda x: -x[0])]
        if a.dualmap and (a.dualmap / f"{s}.json").exists():
            it = [(o.get("n_obs", 0) or o["n_pts"], by_ycb.get(o["name"]), np.array(o["pts"]) @ A[:3, :3].T + A[:3, 3], None) for o in json.loads((a.dualmap / f"{s}.json").read_text()) if len(o["pts"]) >= 3]
            cand["DualMap"] = [x[1:] for x in sorted(it, key=lambda x: -x[0])]
        dump = {"scene": s, "gt": [{"name": nm, "center": gt_pts[nm].mean(0).tolist(), "T": np.asarray(r["gobj"][nm], float).tolist()} for nm in gt_pts], "methods": {}}
        for m, its in cand.items():
            rows, miss = score(its, gt_pts, r["gobj"], names)
            per.setdefault(m, []).append({"scene": s, "gt": len(gt_pts), "rows": rows, "missing": miss})
            dump["methods"][m] = [{"name": nm, "kind": row["kind"], "center": P.mean(0).tolist(), "R": None if R is None else np.asarray(R).tolist(), "n_pts": len(P),
                                   "pts": P[:: max(1, len(P) // 10000)].round(4).tolist(), **{k: row[k] for k in ("center_mm", "surface_mm", "rot_deg") if k in row}}
                                  for (nm, P, R), row in zip(its, rows)]
        if a.dump:
            a.dump.mkdir(parents=True, exist_ok=True); (a.dump / f"{s}.json").write_text(json.dumps(dump, ensure_ascii=False))
    summary = {}
    for m, sc in per.items():
        rows = [x for s_ in sc for x in s_["rows"]]; pred = [x for x in rows if x["kind"] != "unnamed"]; mt = [x for x in pred if x["kind"] == "match"]
        gt = sum(s_["gt"] for s_ in sc); cnt = lambda k: sum(x["kind"] == k for x in rows)
        P_ = len(mt) / max(len(pred), 1); R_ = len(mt) / max(gt, 1); q = lambda v, p: float(np.percentile(v, p)) if len(v) else None
        summary[m] = {"scenes": len(sc), "gt_objects": gt, "predicted": len(pred), "matched": len(mt), "duplicate": cnt("duplicate"), "wrong_name": cnt("wrong_name"), "far": cnt("far"),
                      "missing": gt - len(mt), "unnamed_objects": cnt("unnamed"),
                      "precision": round(100 * P_, 1), "recall": round(100 * R_, 1), "f1": round(100 * 2 * P_ * R_ / max(P_ + R_, 1e-9), 1),
                      "surface_mm_p50": q([x["surface_mm"] for x in mt], 50), "surface_mm_p90": q([x["surface_mm"] for x in mt], 90),
                      "center_mm_p50": q([x["center_mm"] for x in mt], 50), "rot_deg_p50": q([x["rot_deg"] for x in mt if "rot_deg" in x], 50)}
        v = summary[m]
        print(f"{m:22s} 정답 {gt:3d} 예측 {len(pred):3d} | 맞음 {len(mt):3d} 중복 {v['duplicate']} 이름 오류 {v['wrong_name']} 먼 위치 {v['far']} | 정밀도 {v['precision']} 재현율 {v['recall']} F1 {v['f1']} | "
              f"표면 {v['surface_mm_p50'] and round(v['surface_mm_p50'], 1)}/{v['surface_mm_p90'] and round(v['surface_mm_p90'], 1)} mm 방향 {v['rot_deg_p50'] and round(v['rot_deg_p50'], 1)} | 이름 없는 객체 {v['unnamed_objects']}")
    (a.out / "summary_f1.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False)); (a.out / "per_scene_f1.json").write_text(json.dumps(per, ensure_ascii=False))


if __name__ == "__main__":
    main()
