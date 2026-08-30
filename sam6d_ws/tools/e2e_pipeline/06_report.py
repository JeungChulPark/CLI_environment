#!/usr/bin/env python3
"""06_report.py — assemble pose_quality_report.md from stats + heuristic + VLM labels.

Merges (if present) outputs_e2e/evaluation/vlm_labels.csv — Claude Vision labels for
the sampled frames — over the heuristic summary. Reports GT-less limitations honestly.

Run:
  python tools/e2e_pipeline/06_report.py
"""
import csv
import json
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(__file__))
import pipeline_lib as L  # noqa: E402

EVAL_DIR = os.path.join(L.OUT, "evaluation")
LOG_DIR = os.path.join(L.OUT, "_logs")


def load_stats():
    p = os.path.join(EVAL_DIR, "detection_stats.json")
    return json.load(open(p)) if os.path.isfile(p) else {}


def load_vlm():
    p = os.path.join(EVAL_DIR, "vlm_labels.csv")
    out = {}
    if os.path.isfile(p):
        for r in csv.DictReader(open(p)):
            out[(r["bag_name"], r["object_id"], r["frame_idx"])] = r
    return out


def main():
    stats = load_stats()
    vlm = load_vlm()
    by_obj, by_bag = {}, {}
    for key, s in stats.items():
        by_obj.setdefault(s["object"], []).append(s)
        by_bag.setdefault(s["bag"], []).append(s)

    def agg_rate(items):
        rates = [i["detection_rate"] for i in items]
        return round(st.mean(rates), 3) if rates else 0.0

    lines = []
    A = lines.append
    A("# SAM-6D 9-bag × 16-object 포즈 추정 — GT-less 품질 평가 리포트\n")
    A(f"- 생성: 06_report.py  |  결과 루트: `{os.path.relpath(L.OUT, L.ROOT)}/`")
    A(f"- (bag,object) 쌍: {len(stats)}  |  VLM 라벨링된 프레임: {len(vlm)}\n")

    A("## 전체 요약\n")
    tot_frames = sum(s["frames"] for s in stats.values())
    tot_ok = sum(s["pose_ok"] for s in stats.values())
    A(f"- 평가 (bag,object) 쌍: **{len(stats)}**")
    A(f"- 총 프레임-객체 추론: **{tot_frames}**  |  PEM 포즈 생성(ISM 게이트 통과): **{tot_ok}** "
      f"({round(100*tot_ok/tot_frames,1) if tot_frames else 0}%)")
    A(f"- ISM 게이트(det_score≥0.2)로 부재/저신뢰 객체-프레임의 무의미 포즈를 억제함\n")

    A("## Bag별 요약\n")
    A("| bag | (object)수 | 평균 검출률 | 총 PEM포즈 |")
    A("|---|---|---|---|")
    for bag in sorted(by_bag):
        items = by_bag[bag]
        A(f"| {bag} | {len(items)} | {agg_rate(items):.2f} | {sum(i['pose_ok'] for i in items)} |")
    A("")

    A("## Object별 요약 (전 bag 집계)\n")
    A("| object | 평균 검출률 | 평균 score | 최고 score |")
    A("|---|---|---|---|")
    obj_rank = []
    for obj in sorted(by_obj):
        items = by_obj[obj]
        means = [i["score_mean"] for i in items if i["score_mean"] is not None]
        maxes = [i["score_max"] for i in items if i["score_max"] is not None]
        mscore = round(st.mean(means), 3) if means else None
        obj_rank.append((obj, agg_rate(items), mscore, max(maxes) if maxes else None))
    for obj, rate, mscore, mx in sorted(obj_rank, key=lambda x: -(x[1])):
        A(f"| {obj} | {rate:.2f} | {mscore if mscore is not None else '-'} | {mx if mx is not None else '-'} |")
    A("")

    ranked = sorted([o for o in obj_rank], key=lambda x: -x[1])
    A("## 가장 잘 검출되는 / 가장 실패하는 객체\n")
    A("- **최고 검출**: " + ", ".join(f"{o}({r:.2f})" for o, r, *_ in ranked[:3]))
    A("- **최저 검출**: " + ", ".join(f"{o}({r:.2f})" for o, r, *_ in ranked[-3:]) + "\n")

    # Best / worst individual cases by score (from summary)
    sumcsv = os.path.join(EVAL_DIR, "pose_quality_summary.csv")
    cases = []
    if os.path.isfile(sumcsv):
        for r in csv.DictReader(open(sumcsv)):
            sc = r["notes"].split("score=")[-1].split()[0] if "score=" in r["notes"] else None
            if sc:
                try:
                    cases.append((float(sc), r))
                except ValueError:
                    pass
    cases.sort(key=lambda x: -x[0])
    A("## 대표 성공 사례 (최고 score)\n")
    for sc, r in cases[:5]:
        A(f"- {r['bag_name']}/{r['object_id']} `{r['frame_idx']}` score={sc:.3f} → `{r['result_image']}`")
    A("\n## 대표 실패/저품질 사례 (최저 score, PEM 통과분)\n")
    for sc, r in cases[-5:]:
        A(f"- {r['bag_name']}/{r['object_id']} `{r['frame_idx']}` score={sc:.3f} → `{r['result_image']}`")
    A("")

    if vlm:
        A("## VLM(Claude Vision) 정성 평가 — 표본\n")
        A("| bag | object | frame | detection | pose | 실패유형 | note |")
        A("|---|---|---|---|---|---|---|")
        for (b, o, fr), r in sorted(vlm.items()):
            A(f"| {b} | {o} | {fr} | {r.get('detection_label','')} | {r.get('pose_label','')} "
              f"| {r.get('failure_mode','')} | {r.get('notes','')[:60]} |")
        A("")

    A("## GT 부재 한계 (정직성)\n")
    A("GT 6D 포즈가 없으므로 아래 지표는 **계산 불가**이며 본 리포트에서 산출하지 않음:")
    A("- ADD / ADD-S (모델점 평균 거리)")
    A("- Rotation Error / Translation Error")
    A("- 엄밀한 TP / TN / FP / FN, Precision / Recall\n")
    A("대신 **계산 가능한 프록시**만 사용: PEM/ISM score 분포, depth 평면 타당성(tz), 검출률, "
      "VLM 정성 라벨. 이들은 상대 비교·이상 탐지에는 유효하나 절대 정확도 측정은 아님.\n")

    A("## 향후 GT 구축 제안\n")
    A("1. 객체별 6D GT: ArUco/AprilTag 보드 위 배치 또는 로봇 EEF 기준 알려진 변환으로 GT 포즈 수집")
    A("2. BOP 포맷 `scene_gt.json` + `scene_camera.json` 작성 → ADD(-S) 자동 산출 파이프라인 연결")
    A("3. 장면당 존재 객체 목록(presence GT)만이라도 라벨 → 진짜 TP/FP/FN, Precision/Recall 가능")
    A("4. 멀티객체 동시 추론 + presence GT로 ISM forced-output(부재 객체 오검출) 정량화\n")

    out = os.path.join(EVAL_DIR, "pose_quality_report.md")
    with open(out, "w") as f:
        f.write("\n".join(lines))
    print(f"[report] wrote {out}  ({len(stats)} pairs, {len(vlm)} VLM-labeled)")


if __name__ == "__main__":
    main()
