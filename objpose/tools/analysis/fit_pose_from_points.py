#!/usr/bin/env python3
"""fit_pose_from_points.py — 점군 지도(FM-Fusion, DualMap)의 객체 점에 CAD 모델을 맞춰 6D 자세를 구하고, 제안 방법이 지도에 저장한 자세와 같은 지표로 비교한다.

대상: 정답 물체 중 그 방법이 TP 로 올린 것(같은 이름, 관측 점 겹침 비율 0.25 이상). 정답마다 같은 이름의 가장 가까운 지도 객체를 쓴다. 이름은 맞았다고 본다.
CAD 맞춤: 객체 점(부분) → 모델 점(전체) 방향의 점-점 ICP. 시작 자세는 정육면체 회전 24개 + 무작위 회전 40개, 위치는 중심 맞춤. 거리 한계 2 cm → 1 cm 두 단계.
          객체 점에서 모델까지 거리의 중앙값이 가장 작은 결과를 고른다(정답을 쓰지 않는다).
지표:  Rotation error (대칭 고려 전 / BOP models_info 의 대칭을 고려한 최소값),  ADD, ADD-S, ADD(-S) (대칭 물체는 ADD-S),  ADD(-S) < 0.1 × 지름
    ~/anaconda3/envs/dualmap/bin/python objpose/tools/analysis/fit_pose_from_points.py --dump <dump dir> --inst <inst_iou json> --table <objects.json> --out <json>
"""
import argparse, itertools, json
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rsc

REPO = Path(__file__).resolve().parents[3]; PTS = REPO / "sam6d_realtime/assets/model_points"
ap = argparse.ArgumentParser()
ap.add_argument("--dump", type=Path, required=True); ap.add_argument("--inst", type=Path, required=True); ap.add_argument("--table", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--scenes", nargs="*", default=None); a = ap.parse_args()
tab = json.loads(a.table.read_text()); info = json.loads((Path(tab["models_dir"]) / "models_info.json").read_text())
oid = {o["name"]: str(o["obj_id"]) for o in tab["objects"]}; model = {o["name"]: np.load(PTS / f"{o['name']}.npy") / 1000.0 for o in tab["objects"]}


def sym_rots(name):
    i = info[oid[name]]; S = [np.eye(3)]
    for m in i.get("symmetries_discrete", []): S.append(np.array(m, float).reshape(4, 4)[:3, :3])
    for c in i.get("symmetries_continuous", []):
        ax = np.array(c["axis"], float); S += [Rsc.from_rotvec(ax / np.linalg.norm(ax) * t).as_matrix() for t in np.linspace(0, 2 * np.pi, 72, endpoint=False)[1:]]
    return S


SYM = {n: sym_rots(n) for n in oid}; is_sym = {n: len(SYM[n]) > 1 for n in oid}
ang = lambda A, B: float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))
R24 = []
for p in itertools.permutations(range(3)):
    for s in itertools.product([1, -1], repeat=3):
        R = np.zeros((3, 3))
        for i in range(3): R[i, p[i]] = s[i]
        if np.linalg.det(R) > 0: R24.append(R)
INIT = R24 + list(Rsc.random(40, random_state=0).as_matrix())
crit = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40); est = o3d.pipelines.registration.TransformationEstimationPointToPoint()


def fit(M, P):
    """모델 점 M 을 객체 점 P 에 맞춘 4x4 (모델 → 세계)."""
    Ms = M[:: max(1, len(M) // 2000)]; Ps = P[:: max(1, len(P) // 1500)]; pp = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Ps)); best = None; cand = []
    for R0 in INIT:
        T0 = np.eye(4); T0[:3, :3] = R0; T0[:3, 3] = Ps.mean(0) - R0 @ Ms.mean(0)
        pm = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Ms @ R0.T + T0[:3, 3])); X = np.eye(4)
        for th in (0.02, 0.01): X = o3d.pipelines.registration.registration_icp(pp, pm, th, X, est, crit).transformation
        T = np.linalg.inv(X) @ T0; d = float(np.median(cKDTree(Ms @ T[:3, :3].T + T[:3, 3]).query(Ps)[0]))
        cand.append(T)
        if best is None or d < best[0]: best = (d, T)
    return best[1], best[0] * 1000, cand


def metrics(name, T, Tg):
    M = model[name]; E = M @ T[:3, :3].T + T[:3, 3]; G = M @ Tg[:3, :3].T + Tg[:3, 3]
    add = float(np.linalg.norm(E - G, axis=1).mean() * 1000); adds = float(cKDTree(G).query(E)[0].mean() * 1000); e = adds if is_sym[name] else add
    return {"rot_deg": ang(T[:3, :3], Tg[:3, :3]), "rot_sym_deg": min(ang(T[:3, :3] @ S, Tg[:3, :3]) for S in SYM[name]), "trans_mm": float(np.linalg.norm(T[:3, 3] - Tg[:3, 3]) * 1000),
            "add_mm": add, "adds_mm": adds, "add_s_mm": e, "pass": bool(e < 0.1 * info[oid[name]]["diameter"]), "symmetric": is_sym[name]}


inst = json.loads(a.inst.read_text()); tp = {(r["method"], r["scene"], r["object"]) for r in inst if r["object"] and r["iou_name"] >= 0.25}
n_gt = sum(1 for r in inst if r["object"] and r["method"] == "제안 방법"); rows = []
for f in sorted(a.dump.glob("*.json")):
    if a.scenes and f.stem not in a.scenes: continue
    D = json.loads(f.read_text())
    for g in D["gt"]:
        Tg = np.asarray(g["T"], float); n = g["name"]
        for m, objs in D["methods"].items():
            if (m, D["scene"], n) not in tp: continue
            c = [o for o in objs if o["name"] == n]
            if not c: continue
            o = min(c, key=lambda o_: np.linalg.norm(np.asarray(o_["center"]) - np.asarray(g["center"]))); P = np.asarray(o["pts"], float)
            if o["R"] is not None:               # 자세를 저장하는 방법: 저장된 자세 그대로
                R = np.asarray(o["R"], float); T = np.eye(4); T[:3, :3] = R; T[:3, 3] = P.mean(0) - R @ model[n][:: max(1, len(model[n]) // 10000)].mean(0); res = None; src = "저장된 자세"
            else:
                if len(P) < 30: continue
                T, res, cand = fit(model[n], P); src = "점군에 CAD 맞춤"
            row = {"scene": D["scene"], "object": n, "method": m, "source": src, "n_pts": len(P), "fit_residual_mm": res, "T": np.round(T, 5).tolist(), **metrics(n, T, Tg)}
            if o["R"] is None:                   # 참고: 시작 자세 64개의 결과 중 정답에 가장 가까운 것을 골랐을 때(정답을 알고 고른 상한)
                mb = min((metrics(n, Tc, Tg) for Tc in cand), key=lambda x: x["add_s_mm"]); row.update(oracle_rot_sym_deg=mb["rot_sym_deg"], oracle_add_s_mm=mb["add_s_mm"], oracle_pass=mb["pass"])
            rows.append(row)
    print(f.stem, len(rows), flush=True)
a.out.write_text(json.dumps({"n_gt": n_gt, "rows": rows}, ensure_ascii=False))
for m in dict.fromkeys(r["method"] for r in rows):
    R = [r for r in rows if r["method"] == m]; q = lambda k: float(np.median([r[k] for r in R]))
    print(f"{m:18s} TP {len(R):3d} | 방향 오차 중앙 {q('rot_deg'):6.1f}° (대칭 고려 {q('rot_sym_deg'):5.1f}°) | 대칭 고려 30° 넘게 틀림 {100 * np.mean([r['rot_sym_deg'] > 30 for r in R]):5.1f}% | "
          f"ADD(-S) 중앙 {q('add_s_mm'):5.1f} mm | ADD(-S)<0.1d {100 * np.mean([r['pass'] for r in R]):5.1f}% (정답 전체 대비 {100 * sum(r['pass'] for r in R) / n_gt:4.1f}%) | 위치 {q('trans_mm'):5.1f} mm"
          + (f" | 정답을 알고 고르면 ADD(-S)<0.1d {100 * np.mean([r['oracle_pass'] for r in R]):5.1f}%" if "oracle_pass" in R[0] else ""))
