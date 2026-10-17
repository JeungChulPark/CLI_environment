#!/usr/bin/env python3
"""inst_iou_obchange.py — ObChange 방문별 지도의 관측 점 기준 3D 인스턴스 IoU (objpose/tools/analysis/inst_iou.py 와 같은 규칙).

  관측 점    데이터셋이 주는 재구성 점군(anno/<Room>/scene<N>/planes/*/merged_plane_clouds_ds002.pcd, 물체가 놓인 평면 주변)
  정답 라벨  _GT.anno 의 점 번호(물체마다). 채점 대상은 템플릿이 있는 11종, 나머지 라벨 물체와 배경은 그대로 둔다
  지도 배정  지도 객체의 점에서 TAU 안인 관측 점 → 가장 가까운 객체 하나 (제안: 저장된 자세의 CAD 점, FM-Fusion: 복원 점 전부, DualMap: 저장된 점 전부)
open3d 가 필요해 dualmap 환경으로 돌린다.
    ~/anaconda3/envs/dualmap/bin/python objpose/tools/obchange/inst_iou_obchange.py --scenes kitchen_s2 [--tau 0.01] [--out x.json]
"""
import argparse, json, os, re, sys
from pathlib import Path
import numpy as np, open3d as o3d
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
OB = Path("/home/jucpark/DeepLearning/Datasets/ObChange"); OUT = REPO / "objpose/output/obchange_261005"
PTS = REPO / "sam6d_realtime/assets/model_points"
CATALOG = ["002_master_chef_can", "003_cracker_box", "004_sugar_box", "005_tomato_soup_can", "008_pudding_box", "009_gelatin_box",
           "019_pitcher_base", "024_bowl", "025_mug", "040_large_marker", "061_foam_brick"]
ROOM = {"kitchen": "Kitchen", "office": "Office", "livingarea": "LivingArea", "smallroom": "SmallRoom", "bigroom": "BigRoom"}
TAB = json.loads((REPO / "objpose/rt/bop/ycbv_objects.json").read_text())["objects"]
BY_LABEL = {o["ycb"].split("_", 1)[1].replace("_", " "): o["ycb"] for o in TAB}; YCBV = {o["ycb"]: o["name"] for o in TAB}


def read_ply_xyz(p):
    raw = p.read_bytes(); end = raw.index(b"end_header\n") + len(b"end_header\n"); head = raw[:end].decode()
    n = int(next(l.split()[-1] for l in head.splitlines() if l.startswith("element vertex")))
    props = [l.split() for l in head.splitlines() if l.startswith("property")]
    dt = np.dtype([(pr[2], {"double": "<f8", "float": "<f4", "uchar": "u1"}[pr[1]]) for pr in props])
    v = np.frombuffer(raw[end:end + n * dt.itemsize], dt)
    return np.stack([v["x"], v["y"], v["z"]], 1).astype(float)


def scene_gt(scene):
    """관측 점 P, 점마다 라벨 번호(-1 = 배경), 라벨 이름 목록."""
    room, n = scene.rsplit("_s", 1); P, lab, names = [], [], []
    for pl in sorted((OB / "anno" / ROOM[room] / f"scene{n}" / "planes").iterdir()):
        ga = pl / "merged_plane_clouds_ds002_GT.anno"
        if not ga.exists(): continue
        X = np.asarray(o3d.io.read_point_cloud(str(pl / "merged_plane_clouds_ds002.pcd")).points); L = np.full(len(X), -1)
        for line in ga.read_text().splitlines():
            f = line.split()
            if len(f) < 3: continue
            idx = np.array([int(x) for x in f[1:] if x.lstrip("-").isdigit()]); idx = idx[(idx >= 0) & (idx < len(X))]
            L[idx] = len(names); names.append(f[0])
        P.append(X); lab.append(L)
    return np.concatenate(P), np.concatenate(lab), names


def maps(scene):
    """방법 → [(이름 or None, 점)]"""
    out = {}
    its = []
    for o in json.loads((Path(os.environ.get("PROPOSED_MAPS", OUT / "maps/proposed")) / f"{scene}.json").read_text()):
        M = np.load(PTS / f"{YCBV[o['name']]}.npy") / 1000.0; its.append((o["name"], M @ np.asarray(o["R"]).T + np.asarray(o["center"])))
    out["제안 방법"] = its
    sd = Path(os.environ.get("FMF_DIR", OUT / "fmfusion")) / scene; its = []
    ren = json.loads(Path(os.environ["FMF_NAMES"]).read_text())[scene] if os.environ.get("FMF_NAMES") else None   # 객체 번호 → 다시 붙인 이름(cad_relabel.py)
    for line in (sd / "instance_info.txt").read_text().splitlines():
        if line.startswith("#") or not line.strip(): continue
        f = line.split(";"); Q = read_ply_xyz(sd / f"{int(f[0]):04d}.ply")
        if len(Q): its.append((BY_LABEL.get(f[1].split("(")[0]) if ren is None else ren.get(str(int(f[0]))), Q))
    out["FM-Fusion"] = its
    out["DualMap"] = [(o["name"], np.asarray(o["pts"], float)) for o in json.loads((Path(os.environ.get("DUALMAP_MAPS", OUT / "maps/dualmap")) / f"{scene}.json").read_text()) if len(o["pts"])]
    return out


def assign(P, clouds, tau):
    best = np.full(len(P), np.inf); lab = np.full(len(P), -1)
    for i, C in enumerate(clouds):
        sel = np.where(np.all((P > C.min(0) - tau) & (P < C.max(0) + tau), axis=1))[0]
        if not len(sel): continue
        d = cKDTree(C).query(P[sel], distance_upper_bound=tau)[0]
        ok = d < best[sel]; best[sel[ok]] = d[ok]; lab[sel[ok]] = i
    return lab


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", nargs="+", required=True); ap.add_argument("--tau", type=float, default=0.010); ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(); out = []
    for sc in a.scenes:
        P, glab, gn = scene_gt(sc); gsz = np.bincount(glab[glab >= 0], minlength=len(gn))
        for m, its in maps(sc).items():
            plab = assign(P, [c for _, c in its], a.tau); psz = np.bincount(plab[plab >= 0], minlength=len(its))
            pair = np.zeros((len(gn), len(its)), int); both = (glab >= 0) & (plab >= 0); np.add.at(pair, (glab[both], plab[both]), 1)
            iou = pair / np.maximum(gsz[:, None] + psz[None, :] - pair, 1)
            for i, n in enumerate(gn):
                if n not in CATALOG: continue
                same = [j for j, (nm, _) in enumerate(its) if nm == n]
                jb = same[int(iou[i, same].argmax())] if same else None
                dist = float(np.median(cKDTree(its[jb][1]).query(P[glab == i])[0]) * 1000) if jb is not None and iou[i, jb] > 0 else None   # 관측된 정답 점 → 같은 이름 객체까지 거리 중앙값 [mm]
                out.append({"scene": sc, "method": m, "object": n, "gt_points": int(gsz[i]), "dist_mm": dist, "iou_name": float(iou[i, same].max()) if same else 0.0, "iou_any": float(iou[i].max()) if its else 0.0, "any_name": its[int(iou[i].argmax())][0] if its else None})
            named = [j for j, (nm, _) in enumerate(its) if nm in CATALOG]
            out.append({"scene": sc, "method": m, "object": None, "named": len(named), "named_with_points": int(sum(psz[j] > 0 for j in named)), "objects": len(its), "observed_points": len(P)})
        print(sc, "관측 점", len(P), "정답 라벨 물체", len(gn), flush=True)
    if a.out: a.out.write_text(json.dumps(out, ensure_ascii=False))
    rows = [r for r in out if r["object"]]
    for m in dict.fromkeys(r["method"] for r in rows):
        R = [r for r in rows if r["method"] == m]; info = [r for r in out if r["object"] is None and r["method"] == m]
        named = sum(r["named"] for r in info); s = f"{m:12s} 정답 {len(R):3d} 이름 붙은 객체 {named:3d} (관측 영역 안 {sum(r['named_with_points'] for r in info)}) |"
        for thr in (0.25, 0.5):
            tp = sum(r["iou_name"] >= thr for r in R); p = tp / max(named, 1); rc = tp / len(R)
            s += f" IoU≥{thr}: 맞음 {tp} 정밀도 {100 * p:.1f} 재현율 {100 * rc:.1f} F1 {100 * 2 * p * rc / max(p + rc, 1e-9):.1f} |"
        print(s + f" 평균 IoU 이름 기준 {np.mean([r['iou_name'] for r in R]):.3f} / 이름 무시 {np.mean([r['iou_any'] for r in R]):.3f}")
    if len(a.scenes) == 1:
        for r in rows: print(f"  {r['method']:12s} {r['object']:22s} 관측 점 {r['gt_points']:6d}  IoU 이름 기준 {r['iou_name']:.2f}  이름 무시 {r['iou_any']:.2f}")


if __name__ == "__main__":
    main()
