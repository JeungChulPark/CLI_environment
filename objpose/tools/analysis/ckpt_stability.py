#!/usr/bin/env python3
"""ckpt_stability.py — 영상의 10 %, 20 %, …, 100 % 시점에서 세 방법의 지도를 비교해, 지도가 만들어지는 동안 객체가 얼마나 일관되게 유지되는지 잰다 (YCB-Video).

  제안 방법   저장된 인식 결과를 순서대로 다시 넣고 각 시점의 지도 상태를 본다(map_jitter.replay)
  FM-Fusion · DualMap  영상의 앞 N % 만 넣어 만든 지도(run_checkpoints.sh). 100 % 는 기존 결과
정답 물체마다 각 시점에서 정답 중심 10 cm 안의 지도 객체를 찾는다.
  상태        맞는 이름 / 다른 이름 · 이름 없음 / 없음
  위치        맞는 이름 객체의 중심(제안: 저장된 자세의 CAD 중심, 나머지: 점의 평균)
    <sam6d python> objpose/tools/analysis/ckpt_stability.py --out <json>
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
REPO = Path(__file__).resolve().parents[3]; sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d")); sys.path.insert(0, str(Path(__file__).parent)); sys.path.insert(0, str(REPO / "objpose/tools/baselines/fmfusion"))
import build_memory_maps as BM   # noqa: E402
from map_jitter import replay    # noqa: E402
from eval_maps import read_ply_xyz   # noqa: E402

O = REPO / "objpose/output"; V = O / "fmf_cmp_260928"; C = O / "checkpoints_261006"; PL = O / "dualmap_plain_261005"
PPS = list(range(10, 101, 10)); M3 = ["제안 방법", "FM-Fusion", "DualMap"]


def fmf_objs(d, by_label, A):
    out = []
    if not (d / "instance_info.txt").exists(): return out
    for line in (d / "instance_info.txt").read_text().splitlines():
        if line.startswith("#") or not line.strip(): continue
        f = line.split(";"); P = read_ply_xyz(d / f"{int(f[0]):04d}.ply")
        if len(P): out.append((by_label.get(f[1].split("(")[0]), (P @ A[:3, :3].T + A[:3, 3]).mean(0)))
    return out


def dm_objs(f, by_ycb, A):
    if not f.exists(): return []
    return [(by_ycb.get(o["name"]), (np.asarray(o["pts"], float) @ A[:3, :3].T + A[:3, 3]).mean(0)) for o in json.loads(f.read_text()) if len(o["pts"]) >= 3]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, default=C / "stability.json"); a = ap.parse_args()
    tab = json.loads(BM.TABLE.read_text())["objects"]; by_id = {o["obj_id"]: o["name"] for o in tab}
    by_label = {o["ycb"].split("_", 1)[1].replace("_", " "): o["name"] for o in tab}; by_ycb = {o["ycb"]: o["name"] for o in tab}
    model_c = {o["name"]: (np.load(BM.PTS / f"{o['name']}.npy") / 1000.0).mean(0) for o in tab}
    scenes = sorted(p.stem for p in (V / "eval_own/dump_final").glob("*.json")); rows = []
    for s in scenes:
        raw, lm, gobj, dur, A = replay(s, V / "proposed_own", by_id)
        gc = {n: T[:3, :3] @ model_c[n] + T[:3, 3] for n, T in gobj.items()}          # 정답 CAD 중심(점의 평균)
        for pp in PPS:
            t = dur * pp / 100; objs = {}
            objs["제안 방법"] = [(nm, tr_[-1][2] @ model_c[nm] + tr_[-1][1]) for (oid, nm), tr in lm.items() for tr_ in [[x for x in tr if x[0] <= t]] if tr_ and tr_[-1][0] >= t - 3.0 or (tr_ and pp == 100)]
            objs["FM-Fusion"] = fmf_objs(V / "fmfusion_own" / s if pp == 100 else C / "fmfusion" / f"{s}_f{pp}", by_label, A)
            objs["DualMap"] = dm_objs(PL / "ycbv/maps/plain_c05" / f"{s}.json" if pp == 100 else C / "dualmap/maps" / f"plain_c05_f{pp}" / f"{s}.json", by_ycb, A)
            for m in M3:
                for n, c in gc.items():
                    near = sorted(((float(np.linalg.norm(p - c)), nm, p) for nm, p in objs[m] if np.linalg.norm(p - c) < 0.10), key=lambda x: x[0])
                    same = [x for x in near if x[1] == n]
                    rows.append({"scene": s, "pp": pp, "method": m, "object": n, "state": "ok" if same else ("other" if near else "none"), "pos": same[0][2].tolist() if same else None, "gt_err_mm": same[0][0] * 1000 if same else None})
        print(s, flush=True)
    a.out.write_text(json.dumps(rows, ensure_ascii=False))
    for m in M3:
        R = [r for r in rows if r["method"] == m]; keys = sorted({(r["scene"], r["object"]) for r in R}); step, dev, lost, ever, flips = [], [], 0, 0, 0
        for k in keys:
            seq = sorted((r for r in R if (r["scene"], r["object"]) == k), key=lambda r: r["pp"]); st = [r["state"] for r in seq]
            if "ok" in st:
                ever += 1; first = st.index("ok"); lost += st[-1] != "ok"; flips += sum(1 for x, y in zip(st[first:], st[first + 1:]) if (x == "ok") != (y == "ok"))
            P = [(r["pp"], np.array(r["pos"])) for r in seq if r["pos"] is not None]
            step += [np.linalg.norm(q - p) * 1000 for (a_, p), (b_, q) in zip(P, P[1:]) if b_ - a_ == 10]
            if seq[-1]["pos"] is not None: dev += [np.linalg.norm(p - np.array(seq[-1]["pos"])) * 1000 for _, p in P[:-1]]
        print(f"{m:10s} 정답 {len(keys)} | 한 번이라도 맞는 이름으로 있던 물체 {ever} → 마지막에 없거나 이름이 바뀐 것 {lost} | 맞음↔아님 전환 {flips}회 | "
              f"시점 사이 위치 이동 중앙 {np.median(step):.1f} mm (95% {np.percentile(step, 95):.1f}) | 마지막 위치와의 차 중앙 {np.median(dev):.1f} mm (95% {np.percentile(dev, 95):.1f})")


if __name__ == "__main__":
    main()
