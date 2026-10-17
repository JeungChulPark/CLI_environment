#!/usr/bin/env python3
"""inst_iou.py — 관측된 점 기준 3D 인스턴스 IoU. 카메라가 실제로 본 표면만 평가하므로 점군 지도와 CAD 자세 지도를 같은 조건으로 비교한다.

  관측 점    정답 카메라 자세로 깊이 영상을 쌓은 점(3 mm 복셀로 솎음)
  정답 라벨  정답 자세에 놓은 CAD 표면에서 TAU 안인 관측 점 → 그 물체(가장 가까운 물체 하나), 나머지는 배경
  지도 배정  지도 객체의 점(제안: 추정 자세의 CAD 점, 나머지: 복원 점)에서 TAU 안인 관측 점 → 그 객체(가장 가까운 객체 하나). 이름 없는 객체도 포함
  IoU        (같은 객체에 배정되고 같은 정답 라벨인 점 수) / (둘 중 하나에 해당하는 점 수)
  정답 물체마다  같은 이름 객체 중 최고 IoU(이름 기준)와 모든 객체 중 최고 IoU(이름 무시)를 낸다.
지도 객체는 eval_maps_f1.py --dump 의 장면 json 을 읽는다. BENCH_ROOT / BENCH_TABLE 로 데이터셋을 바꾼다.
    <sam6d python> objpose/tools/analysis/inst_iou.py --dump <dump dir> [--scenes 000054] [--out per_object.json]
"""
import argparse, json, sys
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM   # noqa: E402

TAU = 0.010


def observed(sc, ids, Twc, cam, stride, pix=4):
    P = []
    for k in range(0, len(ids), stride):
        f = sc / "depth" / f"{int(ids[k]):06d}.png"
        raw = cv2.imread(str(f), cv2.IMREAD_UNCHANGED)
        if raw is None: continue
        c = cam[ids[k]]; K = np.array(c["cam_K"]).reshape(3, 3)
        d = raw[::pix, ::pix].astype(np.float32) * float(c.get("depth_scale", 1.0)) / 1000.0
        v, u = np.mgrid[0:raw.shape[0]:pix, 0:raw.shape[1]:pix]
        m = (d > 0.2) & (d < 2.0); z = d[m]
        X = np.c_[(u[m] - K[0, 2]) * z / K[0, 0], (v[m] - K[1, 2]) * z / K[1, 1], z]
        P.append(X @ Twc[k][:3, :3].T + Twc[k][:3, 3])
    P = np.concatenate(P); _, idx = np.unique(np.floor(P / 0.003).astype(np.int64), axis=0, return_index=True)
    return P[idx]


def assign(P, clouds, tau=TAU):
    """관측 점마다 가장 가까운 점군의 번호(-1 = 어느 것도 tau 안에 없음)."""
    best = np.full(len(P), np.inf); lab = np.full(len(P), -1)
    for i, C in enumerate(clouds):
        if len(C) == 0: continue
        sel = np.where(np.all((P > C.min(0) - tau) & (P < C.max(0) + tau), axis=1))[0]
        if not len(sel): continue
        d = cKDTree(C).query(P[sel], distance_upper_bound=tau)[0]
        ok = d < best[sel]; best[sel[ok]] = d[ok]; lab[sel[ok]] = i
    return lab


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", type=Path, required=True); ap.add_argument("--scenes", nargs="*", default=None)
    ap.add_argument("--frames", type=int, default=120, help="장면마다 쓸 깊이 영상 수"); ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--tau", type=float, default=TAU, help="관측 점을 객체에 배정하는 거리 [m]")
    a = ap.parse_args()
    table = json.loads(BM.TABLE.read_text())["objects"]; by_id = {o["obj_id"]: o["name"] for o in table}
    model = {o["name"]: np.load(BM.PTS / f"{o['name']}.npy") / 1000.0 for o in table}
    out = []
    for f in sorted(p for p in a.dump.glob("*.json") if p.stem != "stream_stats"):
        if a.scenes and f.stem not in a.scenes: continue
        D = json.loads(f.read_text()); sc = BM.YCBV / f.stem
        ids, Twc, gobj, K = BM.gt_scene(sc, by_id); cam = json.loads((sc / "scene_camera.json").read_text())
        P = observed(sc, ids, Twc, cam, max(1, len(ids) // a.frames))
        gn = [g["name"] for g in D["gt"]]
        glab = assign(P, [model[n] @ np.asarray(g["T"])[:3, :3].T + np.asarray(g["T"])[:3, 3] for n, g in zip(gn, D["gt"])], a.tau)
        gsz = [int((glab == i).sum()) for i in range(len(gn))]
        for m, objs in D["methods"].items():
            plab = assign(P, [np.asarray(o["pts"], float) for o in objs], a.tau); psz = np.bincount(plab[plab >= 0], minlength=len(objs))
            pair = np.zeros((len(gn), len(objs)), int)
            both = (glab >= 0) & (plab >= 0); np.add.at(pair, (glab[both], plab[both]), 1)
            iou = pair / np.maximum(np.array(gsz)[:, None] + psz[None, :] - pair, 1)
            for i, n in enumerate(gn):
                same = [j for j, o in enumerate(objs) if o["name"] == n]
                jb = same[int(iou[i, same].argmax())] if same else None
                dist = float(np.median(cKDTree(np.asarray(objs[jb]["pts"], float)).query(P[glab == i])[0]) * 1000) if jb is not None and iou[i, jb] > 0 and gsz[i] else None
                out.append({"scene": f.stem, "method": m, "object": n, "gt_points": gsz[i], "dist_mm": dist, "iou_name": float(iou[i, same].max()) if same else 0.0,
                            "iou_any": float(iou[i].max()) if len(objs) else 0.0, "any_name": objs[int(iou[i].argmax())]["name"] if len(objs) else None})
            out.append({"scene": f.stem, "method": m, "object": None, "named": sum(o["kind"] != "unnamed" for o in objs), "observed_points": len(P)})
        print(f.stem, len(P), flush=True)
    if a.out: a.out.write_text(json.dumps(out, ensure_ascii=False))
    rows = [r for r in out if r["object"]]
    for m in dict.fromkeys(r["method"] for r in rows):
        R = [r for r in rows if r["method"] == m]; named = sum(r["named"] for r in out if r["object"] is None and r["method"] == m); s = f"{m:20s} 정답 {len(R):3d} 이름 붙은 객체 {named:3d} |"
        for thr in (0.25, 0.5):
            tp = sum(r["iou_name"] >= thr for r in R); p = tp / max(named, 1); rc = tp / len(R)
            s += f" IoU≥{thr}: 맞음 {tp} 정밀도 {100 * p:.1f} 재현율 {100 * rc:.1f} F1 {100 * 2 * p * rc / max(p + rc, 1e-9):.1f} |"
        s += f" 평균 IoU 이름 기준 {np.mean([r['iou_name'] for r in R]):.3f} / 이름 무시 {np.mean([r['iou_any'] for r in R]):.3f} | 이름 무시 IoU≥0.5 {sum(r['iou_any'] >= 0.5 for r in R)}"
        print(s)
    if a.scenes and len(a.scenes) == 1:
        for r in rows: print(f"  {r['method']:20s} {r['object'].replace('ycbv_', ''):18s} 관측 점 {r['gt_points']:6d}  IoU 이름 기준 {r['iou_name']:.2f}  이름 무시 {r['iou_any']:.2f}")


if __name__ == "__main__":
    main()
