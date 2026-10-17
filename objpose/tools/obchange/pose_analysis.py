#!/usr/bin/env python3
"""pose_analysis.py — ObChange 의 카메라 자세 출처 비교: 데이터셋 tf(로봇 위치 추정) vs ORB-SLAM3 RGB-D.

ObChange 에는 정답 카메라 궤적이 없다. 그래서
  (1) 두 궤적의 차이: ORB-SLAM3 궤적을 tf 궤적에 SE3 로 한 번 맞춘 뒤 위치 RMSE·회전 차, 그리고 프레임 사이 이동량의 튐
  (2) 정답 물체 기준: 제안 방법의 프레임별 추정(T_cam_obj)을 각 자세로 지도 좌표에 옮겼을 때 정답 중심과의 거리,
      같은 물체 관측들의 흩어짐(중앙 위치에서의 거리)
을 본다. (2)의 ORB-SLAM3 는 (1)의 정렬로 tf 의 map 좌표에 옮긴 것이라, 전체 치우침은 tf 와 같아지고 프레임 사이 일관성만 비교된다.
    python3 objpose/tools/obchange/pose_analysis.py --run <out>/proposed --orb <out>/slam_orb --out <out>/eval/pose_analysis.json
"""
import argparse, json
from pathlib import Path
import numpy as np

OB = Path("/home/jucpark/DeepLearning/Datasets/ObChange/bop")
FPS = 10.0


def q2R(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def load(p):
    out = {}
    for r in np.loadtxt(p, ndmin=2):
        T = np.eye(4); T[:3, :3] = q2R(r[4:8]); T[:3, 3] = r[1:4]; out[int(round(r[0] * FPS))] = T
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True); ap.add_argument("--orb", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True); ap.add_argument("--room", default="kitchen")
    a = ap.parse_args()
    table = {o["name"]: o["ycb"] for o in json.loads((Path(__file__).resolve().parents[3] / "objpose/rt/bop/obchange_objects.json").read_text())["objects"]}
    res = {}; allA = {}
    for g in sorted((OB / "gt").glob(f"{a.room}_s*.json")):
        s = g.stem
        tf, orb = load(OB / "poses_tf" / f"{s}_frames.txt"), load(a.orb / f"{s}_frames.txt")
        ks = sorted(set(tf) & set(orb))
        M = sum(tf[k][:3, :3] @ orb[k][:3, :3].T for k in ks); U, _, Vt = np.linalg.svd(M)
        R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
        t = np.mean([tf[k][:3, 3] - R @ orb[k][:3, 3] for k in ks], 0)
        A = np.eye(4); A[:3, :3] = R; A[:3, 3] = t; allA[s] = A
        orbA = {k: A @ orb[k] for k in ks}
        dp = np.array([np.linalg.norm(tf[k][:3, 3] - orbA[k][:3, 3]) for k in ks])
        dr = np.array([np.degrees(np.arccos(np.clip((np.trace(tf[k][:3, :3].T @ orbA[k][:3, :3]) - 1) / 2, -1, 1))) for k in ks])
        step = lambda P: np.array([np.linalg.norm(P[b][:3, 3] - P[a_][:3, 3]) for a_, b in zip(ks[:-1], ks[1:])])
        st_tf, st_orb = step(tf), step(orbA)
        rstep = lambda P: np.array([np.degrees(np.arccos(np.clip((np.trace(P[a_][:3, :3].T @ P[b][:3, :3]) - 1) / 2, -1, 1))) for a_, b in zip(ks[:-1], ks[1:])])
        row = {"frames": len(ks), "path_tf_m": float(st_tf.sum()), "path_orb_m": float(st_orb.sum()),
               "traj_diff_rmse_cm": float(np.sqrt((dp ** 2).mean()) * 100), "traj_diff_max_cm": float(dp.max() * 100),
               "rot_diff_med_deg": float(np.median(dr)), "rot_diff_max_deg": float(dr.max()),
               "step_max_tf_cm": float(st_tf.max() * 100), "step_max_orb_cm": float(st_orb.max() * 100),
               "step_p99_tf_cm": float(np.percentile(st_tf, 99) * 100), "step_p99_orb_cm": float(np.percentile(st_orb, 99) * 100),
               "rot_step_max_tf_deg": float(rstep(tf).max()), "rot_step_max_orb_deg": float(rstep(orbA).max())}
        gt = {o["ycb"]: np.array(o["center"]) for o in json.loads(g.read_text())["objects"] if o.get("center")}
        obs = {"tf": {}, "orb": {}}
        fp = a.run / s / "sam6d_frames.jsonl"
        for l in (fp.read_text().splitlines() if fp.exists() else []):
            f = json.loads(l)
            for d in f["dets"]:
                y = table[d["object"]]
                if y not in gt or f["idx"] not in tf or f["idx"] not in orbA: continue
                pc = np.append(np.array(d["t_mm"], float).reshape(3) / 1000.0, 1)
                for nm, P in (("tf", tf), ("orb", orbA)):
                    pw = (P[f["idx"]] @ pc)[:3]
                    if np.linalg.norm(pw - gt[y]) < 0.30:
                        obs[nm].setdefault(y, []).append(pw)
        for nm in ("tf", "orb"):
            err = []; sc = []
            for y, P in obs[nm].items():
                P = np.array(P); err += list(np.linalg.norm(P - gt[y], axis=1) * 100); sc += list(np.linalg.norm(P - np.median(P, 0), axis=1) * 100)
            row[f"obj_err_med_cm_{nm}"] = float(np.median(err)) if err else None
            row[f"obj_err_p90_cm_{nm}"] = float(np.percentile(err, 90)) if err else None
            row[f"obj_scatter_med_cm_{nm}"] = float(np.median(sc)) if sc else None
            row[f"obj_scatter_p90_cm_{nm}"] = float(np.percentile(sc, 90)) if sc else None
            row[f"n_obs_{nm}"] = len(err)
        res[s] = row
        print(s, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()})
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps({"scenes": res, "align_orb_to_tf": {s: A.tolist() for s, A in allA.items()}}, indent=1))
    keys = [k for k in next(iter(res.values())) if k != "frames"]
    print("MEAN", {k: round(float(np.mean([r[k] for r in res.values() if r[k] is not None])), 2) for k in keys})


if __name__ == "__main__":
    main()
