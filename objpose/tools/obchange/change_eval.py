#!/usr/bin/env python3
"""change_eval.py — ObChange 두 방문 사이의 물체 변화(그대로·옮겨짐·사라짐·새로 생김) 채점기.

기준은 데이터셋 논문(Langer 외, Frontiers in Robotics and AI 2022)을 따른다.
  * 그대로/옮겨짐 경계 d = 0.20 m, 검출 인정 거리 0.10 m(정답 중심 ↔ 지도 객체 중심)
  * 채점 단위 = 방문 쌍(A<B)마다 A 의 정답 물체 각각 + B 의 정답 물체 각각 (Kitchen 전체 물체 기준 120개)
      A 쪽 물체의 범주: 그대로 / 옮겨짐 / 사라짐,   B 쪽 물체의 범주: 그대로 / 옮겨짐 / 새로 생김
  * 정답 중심 = 정답 점 라벨의 축 정렬 상자 중심 (gt_centers.py)

방법의 지도 형식
  방문별 지도   <maps>/<method>/<scene>.json = [{name, center, ...}]  → 같은 이름끼리 가까운 순으로 짝지어 범주를 정한다(비교 규칙).
  이어 넣기     <maps>/<method>/<A>__<B>.json = {A:[{id,name,center}], B:[…]} → 같은 id 가 남아 있으면 그대로/옮겨짐,
                A 에만 있으면 사라짐(같은 이름의 새 id 가 B 에 있으면 옮겨짐으로 짝), B 에만 있으면 새로 생김.
name 은 YCB 번호 이름('025_mug'). 이름이 없는(None) 객체는 이름 기준 채점에서 빠지고 위치 기준 채점에만 쓰인다.

  --classes  채점할 정답 물체(기본: 템플릿이 있는 11종),  --bg  물체 없는 기준 방문(<room>_s1)에도 있는 객체(0.10 m 안)는 배경으로 뺀다.
  --room 은 여러 개를 줄 수 있고, 방별 값과 합계를 낸다.
    python3 objpose/tools/obchange/change_eval.py --maps <out>/maps --methods proposed proposed_online dualmap fmfusion --room kitchen
"""
import argparse, itertools, json
from pathlib import Path
import numpy as np

OB = Path("/home/jucpark/DeepLearning/Datasets/ObChange/bop")
D_STATIC, MATCH = 0.20, 0.10
CATALOG = ["002_master_chef_can", "003_cracker_box", "004_sugar_box", "005_tomato_soup_can", "008_pudding_box", "009_gelatin_box",
           "019_pitcher_base", "024_bowl", "025_mug", "040_large_marker", "061_foam_brick"]
dist = lambda a, b: float(np.linalg.norm(np.asarray(a) - np.asarray(b)))


def diff_by_name(A, B, d=D_STATIC):
    """방문별 지도 두 개 → 객체마다 범주. 같은 이름끼리 가까운 순으로 1:1."""
    ca, cb = ["removed"] * len(A), ["novel"] * len(B)
    pairs = sorted((dist(a["center"], b["center"]), i, j) for i, a in enumerate(A) for j, b in enumerate(B)
                   if a["name"] is not None and a["name"] == b["name"])
    ua, ub = set(), set()
    for dd, i, j in pairs:
        if i in ua or j in ub: continue
        ua.add(i); ub.add(j); ca[i] = cb[j] = "static" if dd < d else "moved"
    return ca, cb


def diff_by_id(A, B, d=D_STATIC):
    ca, cb = ["removed"] * len(A), ["novel"] * len(B)
    ib = {b["id"]: j for j, b in enumerate(B)}
    ua, ub = set(), set()
    for i, a in enumerate(A):
        if a["id"] in ib:
            j = ib[a["id"]]; ua.add(i); ub.add(j)
            ca[i] = cb[j] = "static" if dist(a["center"], B[j]["center"]) < d else "moved"
    pairs = sorted((dist(a["center"], b["center"]), i, j) for i, a in enumerate(A) for j, b in enumerate(B)
                   if i not in ua and j not in ub and a["name"] == b["name"])
    for dd, i, j in pairs:                       # 지워지고 같은 이름이 새로 생김 = 옮겨짐
        if i in ua or j in ub: continue
        ua.add(i); ub.add(j); ca[i] = cb[j] = "static" if dd < d else "moved"
    return ca, cb


def diff_by_pos(A, B, d=D_STATIC):
    """이름을 쓰지 않는 비교: 가까운 자리에 객체가 그대로 있으면 static, 아니면 changed."""
    ca, cb = ["changed"] * len(A), ["changed"] * len(B)
    pairs = sorted((dist(a["center"], b["center"]), i, j) for i, a in enumerate(A) for j, b in enumerate(B))
    ua, ub = set(), set()
    for dd, i, j in pairs:
        if dd >= d: break
        if i in ua or j in ub: continue
        ua.add(i); ub.add(j); ca[i] = cb[j] = "static"
    return ca, cb


def load_gt(scene, classes):
    out = []
    for o in json.loads((OB / "gt" / f"{scene}.json").read_text())["objects"]:
        if o["ycb"] in classes and o.get("center") is not None:
            out.append({"name": o["ycb"], "center": o["center"]})
    return out


def drop_bg(objs, bg):
    return [o for o in objs if not any(dist(o["center"], b["center"]) < MATCH and (o["name"] == b["name"] or o["name"] is None or b["name"] is None) for b in bg)]


def eval_room(md, room, classes, bg_on):
    scenes = sorted(p.stem for p in (OB / "gt").glob(f"{room}_s*.json") if load_gt(p.stem, classes))
    bg_name = f"{room}_s1" if bg_on else None
    online = any("__" in p.stem for p in md.glob(f"{room}_s*.json"))
    cats = ["static", "moved", "removed", "novel"]
    S = {"gt": 0, "detected": 0, "correct": 0, "fp_changed": 0, "fp_static": 0, "pos_detected": 0, "pos_correct": 0,
         "per": {c: {"gt": 0, "detected": 0, "correct": 0} for c in cats}, "confusion": {}}
    for A, B in itertools.combinations(scenes, 2):
        gA, gB = load_gt(A, classes), load_gt(B, classes)
        gca, gcb = diff_by_name(gA, gB)
        if online:
            snap = json.loads((md / f"{A}__{B}.json").read_text()); mA, mB = snap["A"], snap["B"]
            bg = json.loads((md / f"{bg_name}__{A}.json").read_text())["A"] if bg_name and (md / f"{bg_name}__{A}.json").exists() else []
        else:
            mA, mB = json.loads((md / f"{A}.json").read_text()), json.loads((md / f"{B}.json").read_text())
            bg = json.loads((md / f"{bg_name}.json").read_text()) if bg_name and (md / f"{bg_name}.json").exists() else []
        mA, mB = drop_bg(mA, bg), drop_bg(mB, bg)
        ca, cb = (diff_by_id if online else diff_by_name)(mA, mB)
        pa, pb = diff_by_pos(mA, mB)
        for G, gc, M, mc, pc in ((gA, gca, mA, ca, pa), (gB, gcb, mB, cb, pb)):
            hit = set()
            for g, k in zip(G, gc):
                S["gt"] += 1; S["per"][k]["gt"] += 1
                c = [(dist(g["center"], o["center"]), i) for i, o in enumerate(M) if o["name"] == g["name"]]
                c = [x for x in c if x[0] < MATCH]
                if c:
                    i = min(c)[1]; hit.add(i)
                    S["detected"] += 1; S["per"][k]["detected"] += 1
                    S["confusion"][f"{k}>{mc[i]}"] = S["confusion"].get(f"{k}>{mc[i]}", 0) + 1
                    if mc[i] == k:
                        S["correct"] += 1; S["per"][k]["correct"] += 1
                c2 = [(dist(g["center"], o["center"]), i) for i, o in enumerate(M)]
                c2 = [x for x in c2 if x[0] < MATCH]
                if c2:
                    S["pos_detected"] += 1
                    if pc[min(c2)[1]] == ("static" if k == "static" else "changed"):
                        S["pos_correct"] += 1
            for i, o in enumerate(M):
                if i not in hit and o["name"] in classes:
                    S["fp_static" if mc[i] == "static" else "fp_changed"] += 1
    return S


def add(T, S):
    for k, v in S.items():
        if isinstance(v, (int, float)):
            T[k] = T.get(k, 0) + v
        elif k == "per":
            for c, d in v.items():
                for kk, vv in d.items():
                    T.setdefault("per", {}).setdefault(c, {}).setdefault(kk, 0); T["per"][c][kk] += vv
        elif k == "confusion":
            for kk, vv in v.items():
                T.setdefault("confusion", {}); T["confusion"][kk] = T["confusion"].get(kk, 0) + vv
    return T


def finish(S):
    pct = lambda x, y: round(100 * x / y, 1) if y else None
    S.update(detected_pct=pct(S["detected"], S["gt"]), correct_pct=pct(S["correct"], S["gt"]),
             correct_of_detected_pct=pct(S["correct"], S["detected"]),
             pos_detected_pct=pct(S["pos_detected"], S["gt"]), pos_correct_pct=pct(S["pos_correct"], S["gt"]))
    return S


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps", type=Path, required=True); ap.add_argument("--methods", nargs="+", required=True)
    ap.add_argument("--room", nargs="+", default=["kitchen"]); ap.add_argument("--classes", nargs="*", default=CATALOG)
    ap.add_argument("--bg", action="store_true", help="물체 없는 기준 방문(<room>_s1)에도 있는 객체를 배경으로 뺀다")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    summary = {}
    for m in a.methods:
        md = a.maps / m; tot = {}; rooms = {}
        for room in a.room:
            if not list(md.glob(f"{room}_s*.json")):
                continue
            S = eval_room(md, room, a.classes, a.bg); add(tot, S); rooms[room] = finish(S)
        if not rooms:
            continue
        S = finish(tot); S["rooms"] = rooms; summary[m] = S
        print(f"{m:18s} 정답 {S['gt']:4d} | 이름 기준: 검출 {S['detected']:4d} ({S['detected_pct']}%) 범주 맞음 {S['correct']:4d} ({S['correct_pct']}%, 검출 중 {S['correct_of_detected_pct']}%) "
              f"잘못된 변화 {S['fp_changed']} 잘못된 그대로 {S['fp_static']} | 위치 기준: 검출 {S['pos_detected_pct']}% 맞음 {S['pos_correct_pct']}%")
        print("   범주별(정답/검출/맞음): " + "  ".join(f"{c} {v['gt']}/{v['detected']}/{v['correct']}" for c, v in S["per"].items()))
        for r, R in rooms.items():
            print(f"   [{r}] 정답 {R['gt']} 검출 {R['detected']} 맞음 {R['correct']} 잘못된 변화 {R['fp_changed']}")
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps({"rooms": a.room, "classes": a.classes, "bg": a.bg, "d_static": D_STATIC, "match": MATCH, "methods": summary}, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
