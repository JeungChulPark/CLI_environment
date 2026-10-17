#!/usr/bin/env python3
"""map_eval.py — ObChange 방문별 객체 지도 채점(변화가 아니라 지도 자체).

정답 물체(템플릿이 있는 11종)마다
  맞음        같은 이름의 지도 객체가 정답 중심 0.10 m 안
  이름 틀림   0.10 m 안에 객체는 있으나 이름이 다르거나 없음
  놓침        0.10 m 안에 객체가 없음
지도 쪽에서는(11종 이름이 붙은 객체만)
  중복        이미 맞은 정답에 또 붙은 같은 이름 객체
  없는 물체   어느 정답과도 맞지 않는 객체 (--bg 면 물체 없는 방문 <room>_s1 에도 있는 객체는 배경으로 미리 뺀다)
중심 오차 = 맞은 객체의 중심과 정답 중심(정답 점의 상자 중심) 사이 거리.
    python3 objpose/tools/obchange/map_eval.py --maps <out>/maps --methods proposed fmfusion dualmap --room kitchen office --bg
"""
import argparse, json
from pathlib import Path
import numpy as np
from change_eval import OB, CATALOG, MATCH, dist, load_gt, drop_bg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps", type=Path, required=True); ap.add_argument("--methods", nargs="+", required=True)
    ap.add_argument("--room", nargs="+", default=["kitchen"]); ap.add_argument("--bg", action="store_true")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    summary = {}
    for m in a.methods:
        md = a.maps / m
        keys = ("gt", "match", "wrong_name", "missing", "duplicate", "false_obj", "map_objects")
        tot = {k: 0 for k in keys}; errs = []; rooms = {}
        for room in a.room:
            R = {k: 0 for k in keys}; e_room = []
            for g in sorted((OB / "gt").glob(f"{room}_s*.json")):
                G = load_gt(g.stem, CATALOG)
                if not G or not (md / f"{g.stem}.json").exists():
                    continue
                M = json.loads((md / f"{g.stem}.json").read_text())
                bg = json.loads((md / f"{room}_s1.json").read_text()) if a.bg and (md / f"{room}_s1.json").exists() else []
                M = drop_bg(M, bg)
                used = set()
                for gt in G:
                    R["gt"] += 1
                    c = sorted((dist(gt["center"], o["center"]), i) for i, o in enumerate(M) if o["name"] == gt["name"] and dist(gt["center"], o["center"]) < MATCH)
                    if c:
                        R["match"] += 1; e_room.append(c[0][0] * 100); used.add(c[0][1])
                        for _, i in c[1:]:
                            R["duplicate"] += 1; used.add(i)
                    elif any(dist(gt["center"], o["center"]) < MATCH for o in M):
                        R["wrong_name"] += 1
                    else:
                        R["missing"] += 1
                cat = [i for i, o in enumerate(M) if o["name"] in CATALOG]
                R["map_objects"] += len(cat); R["false_obj"] += sum(1 for i in cat if i not in used)
            if R["gt"]:
                R["center_cm_p50"] = float(np.median(e_room)) if e_room else None
                rooms[room] = R; errs += e_room
                for k in keys: tot[k] += R[k]
        if not rooms: continue
        tot.update(center_cm_p50=float(np.median(errs)) if errs else None, center_cm_p90=float(np.percentile(errs, 90)) if errs else None,
                   recall_pct=round(100 * tot["match"] / tot["gt"], 1), precision_pct=round(100 * tot["match"] / max(tot["map_objects"], 1), 1), rooms=rooms)
        summary[m] = tot
        print(f"{m:10s} 정답 {tot['gt']:3d} | 맞음 {tot['match']:3d} ({tot['recall_pct']}%) 이름 틀림 {tot['wrong_name']:3d} 놓침 {tot['missing']:3d} | 지도 객체 {tot['map_objects']:3d} 중 중복 {tot['duplicate']} 없는 물체 {tot['false_obj']} (정밀도 {tot['precision_pct']}%) | "
              f"중심 오차 중앙 {tot['center_cm_p50'] and round(tot['center_cm_p50'], 1)} cm, 90% {tot['center_cm_p90'] and round(tot['center_cm_p90'], 1)} cm")
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps({"rooms": a.room, "bg": a.bg, "methods": summary}, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
