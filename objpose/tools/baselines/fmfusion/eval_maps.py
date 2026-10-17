#!/usr/bin/env python3
"""eval_maps.py — FM-Fusion 물체 지도와 Object Pose Map 을 YCB-V 정답 물체에 같은 기준으로 채점한다.

두 지도 모두 같은 ORB-SLAM3 월드에 있으므로, build_memory_maps.run_memory 와 같은 SE3 정렬(A: SLAM → GT, 자세 포함)을
한 번 적용한 뒤 비교한다.
  FM-Fusion   물체 = 점군(<scene>/NNNN.ply, 보인 표면만) + 이름(instance_info.txt)
  Object Pose Map  물체 = 6D 자세 T_w_obj + CAD 모델 점(sam6d_realtime/assets/model_points)
  정답        물체 = 장면 GT 자세(프레임 중앙값, build_memory_maps.gt_scene) + CAD 모델 점

짝짓기: 같은 이름의 정답 물체와 중심 거리 < MATCH_M 이면 '맞음'. 같은 정답에 두 번째로 붙는 것은 '중복',
정답에 없는 이름은 '다른 물체', 이름은 맞지만 멀면 '먼 위치'.
  중심 오차 [mm]   FM-Fusion 은 점군 중심(보인 면만 있어 카메라 쪽으로 치우침), Object Pose Map 은 모델 중심
  표면 오차 [mm]   지도 점 → 정답 모델 표면 최근접 거리의 중앙값 (FM-Fusion: 복원 점, Object Pose Map: 추정 자세의 모델 점)
  3D IoU           축 정렬 상자(GT 월드) — FM-Fusion 논문 계열 지표(AP25/AP50)의 기준
  방향 오차 [°]    Object Pose Map 만 (FM-Fusion 은 방향이 없음)

    /home/jucpark/anaconda3/envs/sam6d/bin/python objpose/tools/baselines/fmfusion/eval_maps.py \
        --fmf objpose/output/fmfusion_ycbv_gt --ours objpose/output/cmp_260925_fair/proposed_ycbv_realtime \
        --orig objpose/output/cmp_260925_fair/orig_ycbv_realtime --out objpose/output/fmfusion_ycbv_gt/eval
"""
from __future__ import annotations

import argparse, json, sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM  # noqa: E402

MATCH_M = 0.10


def read_ply_xyz(p: Path) -> np.ndarray:
    raw = p.read_bytes()
    end = raw.index(b"end_header\n") + len(b"end_header\n")
    head = raw[:end].decode()
    n = int(next(l.split()[-1] for l in head.splitlines() if l.startswith("element vertex")))
    props = [l.split() for l in head.splitlines() if l.startswith("property")]
    dt = np.dtype([(pr[2], {"double": "<f8", "float": "<f4", "uchar": "u1"}[pr[1]]) for pr in props])
    v = np.frombuffer(raw[end:end + n * dt.itemsize], dt)
    return np.stack([v["x"], v["y"], v["z"]], 1).astype(float)


def align(r):
    return r["A"]


def aabb_iou(P, Q):
    lo = np.maximum(P.min(0), Q.min(0)); hi = np.minimum(P.max(0), Q.max(0))
    inter = np.prod(np.clip(hi - lo, 0, None))
    vp = np.prod(P.max(0) - P.min(0)); vq = np.prod(Q.max(0) - Q.min(0))
    return float(inter / (vp + vq - inter))


def rot_err_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1, 1))))


def score(items, gt_pts, gt_T):
    """items: [(name, points_world(N,3), R or None)] 관측 많은 순. gt_pts: name → 정답 모델 점(월드)."""
    trees = {nm: cKDTree(P) for nm, P in gt_pts.items()}
    rows, used = [], set()
    for nm, P, R in items:
        row = {"name": nm, "n_pts": len(P)}
        if nm not in gt_pts:
            row["kind"] = "wrong_class"; rows.append(row); continue
        c_err = float(np.linalg.norm(P.mean(0) - gt_pts[nm].mean(0)))
        row.update(center_mm=c_err * 1000, surface_mm=float(np.median(trees[nm].query(P)[0]) * 1000),
                   iou=aabb_iou(P, gt_pts[nm]))
        if R is not None:
            row["rot_deg"] = rot_err_deg(R, gt_T[nm][:3, :3])
        if nm in used:
            row["kind"] = "duplicate"
        elif c_err > MATCH_M:
            row["kind"] = "far"
        else:
            row["kind"] = "match"; used.add(nm)
        rows.append(row)
    return rows, [nm for nm in gt_pts if nm not in used]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fmf", type=Path, required=True)
    ap.add_argument("--ours", type=Path, required=True)
    ap.add_argument("--orig", type=Path, default=None)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    table = json.loads(BM.TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    by_label = {o["ycb"].split("_", 1)[1].replace("_", " "): o["name"] for o in table}
    model = {o["name"]: np.load(BM.PTS / f"{o['name']}.npy") / 1000.0 for o in table}
    methods = {"FM-Fusion": None, "Object Pose Map (제안)": a.ours}
    if a.orig:
        methods["Object Pose Map (원본 SAM-6D)"] = a.orig
    per = {m: [] for m in methods}
    scenes = sorted(p.name for p in a.fmf.iterdir() if p.is_dir() and (p / "instance_info.txt").exists())
    for s in scenes:
        BM.RUNS["rt"] = a.ours
        r = BM.run_memory(s, "rt", by_id)
        A = r["A"]
        gt_pts = {nm: model[nm] @ T[:3, :3].T + T[:3, 3] for nm, T in r["gobj"].items()}
        # FM-Fusion
        items = []
        for line in (a.fmf / s / "instance_info.txt").read_text().splitlines():
            if line.startswith("#") or not line.strip():
                continue
            f = line.split(";")
            lab = f[1].split("(")[0]
            P = read_ply_xyz(a.fmf / s / f"{int(f[0]):04d}.ply")
            items.append((int(f[2]), by_label.get(lab, lab), P @ A[:3, :3].T + A[:3, 3], None))
        items.sort(key=lambda x: -x[0])
        rows, miss = score([x[1:] for x in items], gt_pts, r["gobj"])
        per["FM-Fusion"].append({"scene": s, "gt": len(gt_pts), "rows": rows, "missing": miss})
        # Object Pose Map (같은 채점, 모델 점을 추정 자세로 둔다)
        for m, run_dir in methods.items():
            if run_dir is None:
                continue
            if run_dir != a.ours:
                BM.RUNS["rt"] = run_dir
                r = BM.run_memory(s, "rt", by_id)
            its = []
            for lm in sorted(r["lms"], key=lambda m_: -m_["n_obs"]):
                T = A @ np.asarray(lm["T_w_obj"], float)
                its.append((lm["name"], model[lm["name"]] @ T[:3, :3].T + T[:3, 3], T[:3, :3]))
            rows, miss = score(its, gt_pts, r["gobj"])
            per[m].append({"scene": s, "gt": len(gt_pts), "rows": rows, "missing": miss})

    summary = {}
    for m, sc in per.items():
        rows = [x for s in sc for x in s["rows"]]
        mt = [x for x in rows if x["kind"] == "match"]
        cnt = lambda k: sum(x["kind"] == k for x in rows)
        q = lambda v, p: float(np.percentile(v, p)) if len(v) else None
        summary[m] = {
            "gt_objects": sum(s["gt"] for s in sc), "map_objects": len(rows), "matched": len(mt),
            "duplicate": cnt("duplicate"), "far": cnt("far"), "wrong_class": cnt("wrong_class"),
            "missing": sum(len(s["missing"]) for s in sc),
            "center_mm_p50": q([x["center_mm"] for x in mt], 50), "center_mm_p90": q([x["center_mm"] for x in mt], 90),
            "surface_mm_p50": q([x["surface_mm"] for x in mt], 50), "surface_mm_p90": q([x["surface_mm"] for x in mt], 90),
            "iou_p50": q([x["iou"] for x in mt], 50),
            "iou25": sum(x.get("iou", 0) >= 0.25 for x in rows if x["kind"] == "match"),
            "iou50": sum(x.get("iou", 0) >= 0.50 for x in rows if x["kind"] == "match"),
            "rot_deg_p50": q([x["rot_deg"] for x in mt if "rot_deg" in x], 50),
        }
    (a.out / "per_scene.json").write_text(json.dumps(per, indent=1, ensure_ascii=False))
    (a.out / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
    for m, v in summary.items():
        print(m, json.dumps(v, ensure_ascii=False))


if __name__ == "__main__":
    main()
