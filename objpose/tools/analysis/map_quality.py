#!/usr/bin/env python3
"""map_quality.py — 최종 객체 지도의 객체별 기하 품질. eval_maps_f1.py --dump 의 장면 json 을 읽는다(새로 돌리는 것 없음).

  지도→정답 [mm]  지도 객체의 점에서 정답 자세에 놓은 CAD 표면까지 최근접 거리의 평균 (ADD-S 와 같은 계산. 제안 방법은 추정 자세의 CAD 점)
  정답→지도 [mm]  정답 CAD 점에서 지도 객체의 점까지 최근접 거리의 평균 (복원이 덜 된 만큼 커진다)
  ADD [mm]        같은 CAD 점끼리의 거리 평균. 자세가 있는 제안 방법만. 대칭 물체에서는 뒤집힌 자세도 크게 나온다.
  상자 IoU        정답 물체 좌표계에서 지도 객체 점의 축 정렬 상자와 CAD 상자의 겹침 비율
    <sam6d python> objpose/tools/analysis/map_quality.py --dump <dump dir> [--scenes 000054] [--csv out.csv]
"""
import argparse, csv, json, sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "objpose/tools/ycbv_sam6d"))
import build_memory_maps as BM   # noqa: E402

KOR = {"match": "맞음", "duplicate": "중복", "wrong_name": "이름 틀림", "far": "먼 위치"}


def box_iou(a_min, a_max, b_min, b_max):
    inter = np.prod(np.clip(np.minimum(a_max, b_max) - np.maximum(a_min, b_min), 0, None))
    return float(inter / (np.prod(a_max - a_min) + np.prod(b_max - b_min) - inter + 1e-12))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", type=Path, required=True); ap.add_argument("--scenes", nargs="*", default=None); ap.add_argument("--csv", type=Path, default=None)
    a = ap.parse_args()
    table = json.loads(BM.TABLE.read_text())["objects"]
    model = {o["name"]: np.load(BM.PTS / f"{o['name']}.npy") / 1000.0 for o in table}
    rows = []
    for f in sorted(a.dump.glob("*.json")):
        if a.scenes and f.stem not in a.scenes: continue
        D = json.loads(f.read_text()); gT = {g["name"]: np.asarray(g["T"], float) for g in D["gt"]}
        for m, objs in D["methods"].items():
            for o in objs:
                nm = o["name"]
                if o["kind"] in ("unnamed", "wrong_name") or nm not in gT: continue
                T = gT[nm]; Mo = model[nm]; G = Mo @ T[:3, :3].T + T[:3, 3]; P = np.asarray(o["pts"], float)
                acc = float(cKDTree(G).query(P)[0].mean() * 1000); comp = float(cKDTree(P).query(G)[0].mean() * 1000)
                add = None
                if o["R"] is not None:                       # dump 의 점은 CAD 점을 같은 간격으로 솎은 것이라 같은 번호끼리 대응된다
                    Gs = G[:: max(1, len(Mo) // 10000)]
                    if len(Gs) == len(P): add = float(np.linalg.norm(P - Gs, axis=1).mean() * 1000)
                L = (P - T[:3, 3]) @ T[:3, :3]               # 정답 물체 좌표계
                iou = box_iou(L.min(0), L.max(0), Mo.min(0), Mo.max(0))
                rows.append([D["scene"], m, nm.replace("ycbv_", ""), KOR[o["kind"]], round(o["center_mm"], 1), round(acc, 1), round(comp, 1), "" if add is None else round(add, 1),
                             round(iou, 2), "" if "rot_deg" not in o else round(o["rot_deg"], 1), round(float(np.ptp(Mo, axis=0).max()) * 1000)])
    head = ["장면", "방법", "물체", "판정", "중심 오차(mm)", "지도→정답(mm)", "정답→지도(mm)", "ADD(mm)", "상자 IoU", "방향 오차(°)", "물체 크기(mm)"]
    if a.csv:
        with open(a.csv, "w", newline="", encoding="utf-8-sig") as fh:
            w = csv.writer(fh); w.writerow(head); w.writerows(rows)
    print(" | ".join(head))
    for r in rows: print(" | ".join(str(x) for x in r))


if __name__ == "__main__":
    main()
