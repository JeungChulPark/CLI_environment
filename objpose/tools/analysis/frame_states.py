#!/usr/bin/env python3
"""frame_states.py — YCB-Video 에서 프레임마다 각 방법의 인식 결과가 맞았는지 보고, 그것이 최종 지도 등록으로 이어졌는지 센다.

화면에 보이는 정답 물체(visib_fract >= 0.1)마다, 그 방법이 처리한 프레임 하나하나의 상태:
  맞음        제안: 같은 이름의 추정이 있고 위치 오차 30 mm 미만 / FM-Fusion · DualMap: 정답 상자와 IoU 0.5 이상인 검출 상자에 같은 이름
  틀린 자세   (제안만) 같은 이름의 추정이 있지만 위치 오차 30 mm 이상
  다른 이름   그 자리에 다른 물체 이름의 결과
  이름 없음   (FM-Fusion · DualMap) 상자는 있으나 이름이 없음
  없음        그 물체에 대한 결과가 없음
최종 지도 등록 여부 = inst_iou json 의 같은 이름 겹침 비율 0.25 이상(TP).
    python3 objpose/tools/analysis/frame_states.py [--scenes 000054]
"""
import argparse, collections, json
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[3]; O = REPO / "objpose/output"
BOP = Path.home() / "DeepLearning/Datasets/paper_benchmarks/YCB_Video/test"; V = O / "fmf_cmp_260928"
FMF = Path.home() / "DeepLearning/Datasets/fmfusion_ycbv_260928/scans"; DML = O / "dualmap_plain_261005/ycbv_detlog/plain_c05_log_in"
TAB = json.loads((REPO / "objpose/rt/bop/ycbv_objects.json").read_text())["objects"]
BY_ID = {o["obj_id"]: o["name"] for o in TAB}; BY_LABEL = {o["ycb"].split("_", 1)[1].replace("_", " "): o["name"] for o in TAB}
PLAIN = json.loads((O / "dualmap_plain_261005/plain_prompts_ycb21.json").read_text()); BY_PLAIN = {PLAIN[o["ycb"]]: o["name"] for o in TAB}
ST = ["맞음", "틀린 자세", "다른 이름", "이름 없음", "없음"]


def iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3]); inter = max(x2 - x1, 0) * max(y2 - y1, 0)
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter + 1e-9)


def box_state(gt_name, gbox, dets):
    """dets: [(이름 or None, xyxy)]"""
    c = sorted(((iou(gbox, b), n) for n, b in dets), key=lambda x: -x[0])
    c = [x for x in c if x[0] >= 0.5]
    if not c: return "없음"
    if any(n == gt_name for _, n in c): return "맞음"
    return "다른 이름" if c[0][1] else "이름 없음"


def scene_states(sc):
    gt = json.loads((BOP / sc / "scene_gt.json").read_text()); info = json.loads((BOP / sc / "scene_gt_info.json").read_text())
    def gts(im):                                   # 그 프레임에 보이는 정답: 이름 → (카메라 좌표 위치 mm, 상자 xyxy)
        out = {}
        for r, i in zip(gt.get(str(im), []), info.get(str(im), [])):
            if i["visib_fract"] >= 0.1 and i["bbox_visib"][2] > 0:
                x, y, w, h = i["bbox_visib"]; out[BY_ID[r["obj_id"]]] = (np.array(r["cam_t_m2c"], float), (x, y, x + w, y + h))
        return out
    S = {m: collections.defaultdict(list) for m in ("제안 방법", "FM-Fusion", "DualMap")}      # 방법 → 물체 → [(프레임, 상태)]
    for l in (V / "proposed_own" / sc / "sam6d_frames.jsonl").read_text().splitlines():
        if not l.strip(): continue
        f = json.loads(l); G = gts(f["idx"] + 1); D = [(d["object"], np.array(d["t_mm"], float)) for d in f["dets"]]
        for n, (t, _) in G.items():
            same = [np.linalg.norm(p - t) for o, p in D if o == n]
            st = ("맞음" if min(same) < 30 else "틀린 자세") if same else ("다른 이름" if any(o != n and np.linalg.norm(p - t) < 30 for o, p in D) else "없음")
            S["제안 방법"][n].append((f["idx"], st))
    for p in sorted((FMF / sc / "prediction_gsam").glob("frame-*_label.json")):
        k = int(p.stem.split("-")[1].split("_")[0]); G = gts(k + 1); J = json.loads(p.read_text())
        D = [((BY_LABEL.get(max(m["labels"], key=m["labels"].get)) if m.get("labels") else None), m["box"]) for m in J["mask"] if "box" in m]
        for n, (_, b) in G.items(): S["FM-Fusion"][n].append((k, box_state(n, b, D)))
    f = DML / f"{sc}_det.jsonl"
    if f.exists():
        idxs = sorted({int(round(r[0] * 30.0)) for r in np.loadtxt(V / "slam" / f"{sc}_frames.txt", ndmin=2)}); lines = [json.loads(l) for l in f.read_text().splitlines() if l.strip()]
        for j, J in enumerate(lines):
            if j * 10 >= len(idxs): break
            k = idxs[j * 10]; G = gts(k + 1); D = [(BY_PLAIN.get(n), b) for n, b in zip(J["labels"], J["xyxy"])]
            for n, (_, b) in G.items(): S["DualMap"][n].append((k, box_state(n, b, D)))
    return S


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--scenes", nargs="*", default=None); ap.add_argument("--out", type=Path, default=V / "eval_own/frame_states.json"); a = ap.parse_args()
    scenes = a.scenes or sorted(p.stem for p in (V / "eval_own/dump_final").glob("*.json"))
    inst = json.loads((V / "eval_own/inst_iou_final.json").read_text()); tp = {(r["method"], r["scene"], r["object"]) for r in inst if r["object"] and r["iou_name"] >= 0.25}
    rows = []
    for sc in scenes:
        for m, objs in scene_states(sc).items():
            for n, seq in objs.items():
                st = [s for _, s in seq]; c = collections.Counter(st)
                drop = sum(1 for x, y in zip(st, st[1:]) if x == "맞음" and y != "맞음"); pairs = sum(1 for x in st[:-1] if x == "맞음")
                rows.append({"scene": sc, "method": m, "object": n, "frames": len(st), **{k: c[k] for k in ST}, "drops": drop, "pairs_from_ok": pairs, "in_map": (m, sc, n) in tp})
    a.out.write_text(json.dumps(rows, ensure_ascii=False))
    for m in ("제안 방법", "FM-Fusion", "DualMap"):
        R = [r for r in rows if r["method"] == m]
        if not R: print(m, "프레임별 기록 없음"); continue
        F = sum(r["frames"] for r in R); tot = {k: sum(r[k] for r in R) for k in ST}; drops = sum(r["drops"] for r in R); pairs = sum(r["pairs_from_ok"] for r in R)
        seen = [r for r in R if r["맞음"] > 0]; never = [r for r in R if r["맞음"] == 0]
        print(f"\n== {m}: 정답 물체 {len(R)}개, 물체 × 프레임 {F}칸")
        print("   프레임 상태 비율: " + ", ".join(f"{k} {100 * v / F:.1f}%" for k, v in tot.items() if v))
        print(f"   맞은 프레임 바로 다음 프레임에서 맞지 않은 비율: {100 * drops / max(pairs, 1):.1f}%  ({drops}/{pairs})")
        print(f"   한 번이라도 맞게 인식한 물체 {len(seen)}개 → 최종 지도에 맞게 등록 {sum(r['in_map'] for r in seen)}개, 등록 안 됨 {sum(not r['in_map'] for r in seen)}개")
        print(f"   한 번도 맞게 인식하지 못한 물체 {len(never)}개 → 최종 지도에 맞게 등록 {sum(r['in_map'] for r in never)}개")
        lost = sorted((r for r in seen if not r["in_map"]), key=lambda r: -r["맞음"])
        for r in lost[:8]: print(f"      등록 안 됨: {r['scene']} {r['object'][5:]:18s} 맞은 프레임 {r['맞음']}/{r['frames']}  (틀린 자세 {r['틀린 자세']}, 다른 이름 {r['다른 이름']}, 이름 없음 {r['이름 없음']}, 없음 {r['없음']})")


if __name__ == "__main__":
    main()
