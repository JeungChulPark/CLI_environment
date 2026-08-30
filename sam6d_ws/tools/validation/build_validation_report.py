#!/usr/bin/env python3
"""SAM-6D 신뢰성 검증 리포트 빌더 (구현 코드 비변경 — 로그 소비 전용).

입력
  --debug-dir : 노드가 남긴 sam6d_debug.csv 가 있는 디렉터리 (output_dir/_run/debug)
  --pem-root  : 프레임별 <frame_id>/detection_pem.json (rotation fallback; 생략 시 debug 부모)
  --labels    : 수동 가시성 라벨 CSV (frame_id, manually_labeled_milk_visible)
  --bag       : 데이터셋 이름
  --out       : 출력 디렉터리

출력
  <out>/frame_results.csv  : 프레임당 1행 (raw + stabilized pose, decision_type, Δ)
  <out>/metrics.json       : 정량 지표 + stabilization ON/OFF variance 비교 (FR-10)

FR-10 (stabilization 정량 비교):
  노드가 sam6d_debug.csv 에 raw(t_x_mm..) 와 stabilized(t_x_stab_mm.., q*_stab) 를 함께
  기록하므로, 단일 run 에서 raw vs stabilized 의 variance 와 reduction rate 를 계산한다.

numpy 의존성 (degradation 규칙):
  - numpy 있음  → rotation spread(평균 quaternion eigenvector 법)까지 계산.
  - numpy 없음  → rotation_spread_deg 만 생략(null). 아래는 numpy 없이도 항상 동작:
      · translation stability(spread/Δt, 순수 파이썬 std·math.dist)
      · frame count / false positive(FP) / true negative(TN) / recall
      · frame_to_frame delta_rotation(quaternion 내적, 순수 파이썬)
      · frame_results.csv 저장
  - 시각화(make_visualizations.py)는 cv2+numpy 필요(별도 스크립트).
"""
import argparse
import csv
import json
import math
import os

try:
    import numpy as np
    NUMPY_OK = True
except ImportError:
    np = None
    NUMPY_OK = False


def _f(v):
    try:
        if v is None or v == "" or str(v).lower() in ("none", "nan"):
            return None
        return float(v)
    except (TypeError, ValueError):
        return None


def _truthy(v):
    return str(v).strip().lower() in ("1", "true", "yes", "on", "t")


def _norm_fid(v):
    s = str(v).strip()
    return f"{int(s):06d}" if s.isdigit() else s


def _quat(row, suffix):
    q = [_f(row.get(f"qx_{suffix}")), _f(row.get(f"qy_{suffix}")),
         _f(row.get(f"qz_{suffix}")), _f(row.get(f"qw_{suffix}"))]
    return q if all(x is not None for x in q) else None


def load_debug_csv(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    frames, order = {}, []
    for r in rows:
        key = r.get("frame_counter") or r.get("frame_id")
        if key not in frames:
            frames[key] = []
            order.append(key)
        frames[key].append(r)

    out = []
    for key in order:
        group = frames[key]
        head = group[0]
        decision = head.get("final_decision", "")
        sel = head.get("selected_candidate", "")
        frame_id = _norm_fid(head.get("frame_id", key))

        best = None
        for r in group:
            if _truthy(r.get("is_best")):
                best = r
                break
        if best is None:
            cand = [r for r in group if str(r.get("candidate")) == str(sel)]
            best = cand[0] if cand else None
        if best is None:
            pool = [r for r in group if str(r.get("candidate_decision")) == "PASS"] \
                or [r for r in group if str(r.get("candidate")) != "-1"]
            if pool:
                best = max(pool, key=lambda r: _f(r.get("final_score")) or -1.0)

        rec = {"frame_counter": _f(head.get("frame_counter")), "frame_id": frame_id,
               "final_decision": decision, "selected_candidate": sel,
               "pose_output_exists": decision in ("PUBLISH", "PUBLISH_LEGACY")}
        if best is not None:
            rec.update({
                "similarity_score": _f(best.get("similarity_score")),
                "mask_score": _f(best.get("appearance_score")),
                "bbox_score": _f(best.get("geometric_score")),
                "ism_score": _f(best.get("ism_score")),
                "pose_score": _f(best.get("pose_score")),
                "final_score": _f(best.get("final_score")),
                "object_id": best.get("object_id"),
                "t_raw": [_f(best.get("t_x_mm")), _f(best.get("t_y_mm")), _f(best.get("t_z_mm"))],
                "t_stab": [_f(best.get("t_x_stab_mm")), _f(best.get("t_y_stab_mm")), _f(best.get("t_z_stab_mm"))],
                "q_raw": _quat(best, "raw"),
                "q_stab": _quat(best, "stab"),
            })
        else:
            rec.update({k: None for k in ("similarity_score", "mask_score", "bbox_score",
                       "ism_score", "pose_score", "final_score", "object_id",
                       "q_raw", "q_stab")})
            rec["t_raw"] = rec["t_stab"] = [None, None, None]
        # stabilized 결측 시 raw 로 대체(stabilize OFF run 호환)
        if rec["t_stab"][0] is None:
            rec["t_stab"] = rec["t_raw"]
        if rec["q_stab"] is None:
            rec["q_stab"] = rec["q_raw"]
        # ── Depth/Pose gate 컬럼 캡쳐(없으면 None) ──
        g = best if best is not None else head
        def _tb(v):
            return _truthy(v) if v not in (None, "", "None") else None
        rec["gate_mode"] = head.get("depth_gate_mode")
        rec["before_gate"] = head.get("final_publish_decision_before_gate") or decision
        rec["after_gate"] = head.get("final_publish_decision_after_gate") or decision
        rec["depth_gate_pass"] = _tb(g.get("depth_gate_pass"))
        rec["workspace_gate_pass"] = _tb(g.get("workspace_gate_pass"))
        rec["depth_gate_reason"] = g.get("depth_gate_reason")
        rec["workspace_gate_reason"] = g.get("workspace_gate_reason")
        rec["depth_valid_ratio"] = _f(g.get("depth_valid_ratio"))
        rec["depth_error_median"] = _f(g.get("depth_error_median"))
        rec["depth_agreement_ratio"] = _f(g.get("depth_agreement_ratio"))
        # gate_pass_frame: depth·workspace 모두 통과(미계산=None→통과 취급)
        rec["gate_pass_frame"] = (rec["depth_gate_pass"] is not False) and \
                                 (rec["workspace_gate_pass"] is not False)
        out.append(rec)
    return out


def load_labels(path):
    labels = {}
    if not path or not os.path.isfile(path):
        return labels
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            # GT 컬럼명 허용: milk_visible(현행 표준) / manually_labeled_milk_visible / gt_milk_visible / visible
            vis = (r.get("milk_visible")
                   if r.get("milk_visible") is not None else
                   r.get("manually_labeled_milk_visible",
                         r.get("gt_milk_visible", r.get("visible"))))
            labels[_norm_fid(r.get("frame_id"))] = _truthy(vis)
    return labels


def load_pem_R_quat(pem_root, frame_id, sel_idx):
    """fallback: detection_pem.json R → quat (CSV 에 q* 없을 때)."""
    p = os.path.join(pem_root, frame_id, "detection_pem.json")
    if not os.path.isfile(p):
        return None
    try:
        dets = json.load(open(p))
    except Exception:
        return None
    if not isinstance(dets, list) or not dets:
        return None
    try:
        i = int(sel_idx)
        if not (0 <= i < len(dets)):
            raise ValueError
    except (TypeError, ValueError):
        i = max(range(len(dets)), key=lambda k: dets[k].get("score", -1.0))
    R = dets[i].get("R")
    if R is None:
        return None
    flat = []
    for r in R:
        flat.extend(r) if isinstance(r, (list, tuple)) else flat.append(r)
    return mat_to_quat(flat) if len(flat) == 9 else None


def mat_to_quat(flat):
    m = [[float(flat[0]), float(flat[1]), float(flat[2])],
         [float(flat[3]), float(flat[4]), float(flat[5])],
         [float(flat[6]), float(flat[7]), float(flat[8])]]
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return [(m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s, (m[1][0]-m[0][1])/s, 0.25*s]
    return None  # fallback 단순화: 비대각 우세 케이스는 CSV quat 사용 권장


def quat_geodesic_deg(q1, q2):
    if q1 is None or q2 is None:
        return None
    d = abs(sum(a*b for a, b in zip(q1, q2)))
    d = max(-1.0, min(1.0, d))
    return math.degrees(2.0 * math.acos(d))


def decision_type(visible, pose):
    if visible is None:
        return "UNKNOWN"
    return ("TP" if pose else "FN") if visible else ("FP" if pose else "TN")


def percentile(vals, q):
    v = sorted(x for x in vals if x is not None)
    if not v:
        return None
    if len(v) == 1:
        return v[0]
    k = (len(v)-1)*(q/100.0)
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return v[lo] if lo == hi else v[lo] + (v[hi]-v[lo])*(k-lo)


def _std(vals):
    v = [x for x in vals if x is not None]
    if len(v) < 2:
        return 0.0 if v else None
    mu = sum(v)/len(v)
    return math.sqrt(sum((x-mu)**2 for x in v)/len(v))


def mean_quat(quats):
    """평균 quaternion (eigenvector 법, numpy 필요). 실패 시 None."""
    qs = [q for q in quats if q is not None]
    if not qs or np is None:
        return None
    A = np.zeros((4, 4))
    for q in qs:
        qv = np.array(q, dtype=float)
        A += np.outer(qv, qv)
    A /= len(qs)
    w, V = np.linalg.eigh(A)
    return list(V[:, int(np.argmax(w))])


def rot_spread_deg(quats):
    """평균 회전 대비 각도 RMS (deg). 회전 산포(variance) 척도."""
    qm = mean_quat(quats)
    if qm is None:
        return None
    angs = [quat_geodesic_deg(q, qm) for q in quats if q is not None]
    angs = [a for a in angs if a is not None]
    if not angs:
        return None
    return math.sqrt(sum(a*a for a in angs)/len(angs))


def trans_spread_mm(ts):
    """평균 위치 대비 표준편차 결합 (mm). sqrt(varx+vary+varz)."""
    valid = [t for t in ts if t and t[0] is not None]
    if len(valid) < 2:
        return None
    sx = _std([t[0] for t in valid])
    sy = _std([t[1] for t in valid])
    sz = _std([t[2] for t in valid])
    return math.sqrt(sx*sx + sy*sy + sz*sz)


def reduction(raw, stab):
    if raw in (None, 0) or stab is None:
        return None
    return round(1.0 - stab/raw, 4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--debug-dir", required=True)
    ap.add_argument("--pem-root", default=None)
    ap.add_argument("--labels", default=None)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    print(f"[numpy] {'사용 가능 (rotation spread 계산)' if NUMPY_OK else '없음 → rotation spread 생략 (나머지 정상 동작)'}")
    csv_path = os.path.join(args.debug_dir, "sam6d_debug.csv")
    if not os.path.isfile(csv_path):
        raise SystemExit(
            f"[입력 없음] sam6d_debug.csv 없음: {csv_path}\n"
            f"  → 먼저 ROS 노드를 해당 데이터셋 config 로 실행해 디버그 로그를 생성하세요:\n"
            f"     ros2 launch sam6d_ros sam6d_inference.launch.py "
            f"config:=src/sam6d_ros/config/no_cli_milk_nomilk.yaml")
    pem_root = args.pem_root or os.path.dirname(os.path.abspath(args.debug_dir.rstrip("/")))
    os.makedirs(args.out, exist_ok=True)

    frames = load_debug_csv(csv_path)
    labels = load_labels(args.labels)
    have_labels = bool(labels)

    for fr in frames:
        fr["visible"] = labels.get(fr["frame_id"], None if have_labels else True)
        # quat 결측 시 pem json fallback
        if fr["pose_output_exists"] and fr["q_raw"] is None:
            qr = load_pem_R_quat(pem_root, fr["frame_id"], fr["selected_candidate"])
            fr["q_raw"] = fr["q_raw"] or qr
            fr["q_stab"] = fr["q_stab"] or qr
        fr["decision_type"] = decision_type(fr["visible"], fr["pose_output_exists"])

    # 연속 delta (raw / stab 각각)
    prev = {}
    for fr in frames:
        dtr = dts = drr = drs = None
        if fr["pose_output_exists"] and fr["t_raw"][0] is not None:
            oid = fr.get("object_id")
            p = prev.get(oid)
            if p is not None:
                dtr = math.dist(fr["t_raw"], p["t_raw"]) if None not in fr["t_raw"]+p["t_raw"] else None
                dts = math.dist(fr["t_stab"], p["t_stab"]) if None not in fr["t_stab"]+p["t_stab"] else None
                drr = quat_geodesic_deg(fr["q_raw"], p["q_raw"])
                drs = quat_geodesic_deg(fr["q_stab"], p["q_stab"])
            prev[oid] = fr
        fr["d_trans_raw"] = round(dtr, 3) if dtr is not None else None
        fr["d_trans_stab"] = round(dts, 3) if dts is not None else None
        fr["d_rot_raw"] = round(drr, 3) if drr is not None else None
        fr["d_rot_stab"] = round(drs, 3) if drs is not None else None

    # ── frame_results.csv ──
    cols = ["bag_name", "frame_id", "timestamp", "manually_labeled_milk_visible",
            "predicted_milk_visible", "pose_output_exists", "bbox_score", "mask_score",
            "similarity_score", "pose_score", "final_decision", "decision_type",
            "translation_raw_xyz", "translation_stab_xyz", "rotation_raw_quat",
            "rotation_stab_quat", "delta_translation_raw", "delta_translation_stab",
            "delta_rotation_raw_deg", "delta_rotation_stab_deg",
            "depth_gate_reason", "workspace_gate_reason", "depth_valid_ratio",
            "depth_error_median", "depth_agreement_ratio", "gate_pass_frame",
            "final_publish_decision_before_gate", "final_publish_decision_after_gate",
            "visualization_image_path"]
    with open(os.path.join(args.out, "frame_results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for fr in frames:
            jt = lambda v: json.dumps(v) if v and v[0] is not None else ""
            w.writerow({
                "bag_name": args.bag, "frame_id": fr["frame_id"],
                "timestamp": fr.get("frame_counter"),
                "manually_labeled_milk_visible": fr["visible"],
                "predicted_milk_visible": fr["pose_output_exists"],
                "pose_output_exists": fr["pose_output_exists"],
                "bbox_score": fr.get("bbox_score"), "mask_score": fr.get("mask_score"),
                "similarity_score": fr.get("similarity_score"), "pose_score": fr.get("pose_score"),
                "final_decision": fr["final_decision"], "decision_type": fr["decision_type"],
                "translation_raw_xyz": jt(fr["t_raw"]), "translation_stab_xyz": jt(fr["t_stab"]),
                "rotation_raw_quat": json.dumps(fr["q_raw"]) if fr["q_raw"] else "",
                "rotation_stab_quat": json.dumps(fr["q_stab"]) if fr["q_stab"] else "",
                "delta_translation_raw": fr["d_trans_raw"], "delta_translation_stab": fr["d_trans_stab"],
                "delta_rotation_raw_deg": fr["d_rot_raw"], "delta_rotation_stab_deg": fr["d_rot_stab"],
                "depth_gate_reason": fr.get("depth_gate_reason"),
                "workspace_gate_reason": fr.get("workspace_gate_reason"),
                "depth_valid_ratio": fr.get("depth_valid_ratio"),
                "depth_error_median": fr.get("depth_error_median"),
                "depth_agreement_ratio": fr.get("depth_agreement_ratio"),
                "gate_pass_frame": fr.get("gate_pass_frame"),
                "final_publish_decision_before_gate": fr.get("before_gate"),
                "final_publish_decision_after_gate": fr.get("after_gate"),
                "visualization_image_path": "",
            })

    # ── 분류 지표 (Ground Truth = visibility_labels.csv. proxy 미사용) ──
    #   positive = "milk 있음". TP=milk有&pose, FP=milk無&pose, FN=milk有&pose없음, TN=milk無&pose없음.
    dts_all = [fr["decision_type"] for fr in frames]
    tp, tn, fp, fn = (dts_all.count(x) for x in ("TP", "TN", "FP", "FN"))
    no_milk = tn + fp
    milk = tp + fn
    precision = round(tp / (tp + fp), 4) if (tp + fp) else None   # TP/(TP+FP)
    recall    = round(tp / (tp + fn), 4) if (tp + fn) else None   # TP/(TP+FN)
    f1 = (round(2 * precision * recall / (precision + recall), 4)
          if (precision and recall and (precision + recall) > 0) else None)
    accuracy = round((tp + tn) / len(frames), 4) if frames else None
    confusion_matrix = {
        "TP": tp, "FP": fp, "FN": fn, "TN": tn,
        "rows": "GT(milk_visible)", "cols": "prediction(pose_output)",
        "matrix": {"GT_milk":    {"pred_pose": tp, "pred_no_pose": fn},
                   "GT_no_milk": {"pred_pose": fp, "pred_no_pose": tn}},
    }

    # ── Depth/Pose Gate metrics (baseline / shadow_as_hard / hard) ──
    def _cm(pred_pose_fn):
        a = b = c = d = 0  # tp,fp,fn,tn
        for fr in frames:
            if fr["visible"] is None:
                continue
            pp = pred_pose_fn(fr)
            if fr["visible"]:
                a += pp; c += (not pp)
            else:
                b += pp; d += (not pp)
        P = round(a / (a + b), 4) if (a + b) else None
        R = round(a / (a + c), 4) if (a + c) else None
        F = (round(2 * P * R / (P + R), 4) if (P and R) else None)
        A = round((a + d) / (a + b + c + d), 4) if (a + b + c + d) else None
        return {"TP": a, "FP": b, "FN": c, "TN": d, "precision": P, "recall": R, "f1": F, "accuracy": A}

    base_cm = _cm(lambda fr: fr["pose_output_exists"])                       # 현재 실제 발행
    # shadow_as_hard: score-PUBLISH 이면서 gate 통과한 프레임만 발행했다고 가정
    hard_pred = lambda fr: bool(fr["pose_output_exists"] and fr.get("gate_pass_frame", True))
    shadow_cm = _cm(hard_pred)
    gate_mode = next((fr.get("gate_mode") for fr in frames if fr.get("gate_mode")), None)
    # mode==hard 이면 실제 발행이 곧 hard; 아니면 시뮬과 동일
    hard_cm = base_cm if gate_mode == "hard" else shadow_cm
    # gate pass/fail × GT (score-PUBLISH 프레임 대상)
    gpf = {"GT_milk": {"gate_pass": 0, "gate_reject": 0},
           "GT_no_milk": {"gate_pass": 0, "gate_reject": 0}}
    for fr in frames:
        if not fr["pose_output_exists"] or fr["visible"] is None:
            continue
        key = "GT_milk" if fr["visible"] else "GT_no_milk"
        gpf[key]["gate_pass" if fr.get("gate_pass_frame", True) else "gate_reject"] += 1
    recall_drop = (round(base_cm["recall"] - shadow_cm["recall"], 4)
                   if base_cm["recall"] is not None and shadow_cm["recall"] is not None else None)
    precision_gain = (round((shadow_cm["precision"] or 0) - (base_cm["precision"] or 0), 4)
                      if base_cm["precision"] is not None else None)
    hard_recall = shadow_cm["recall"]
    gate_metrics = {
        "gate_mode": gate_mode,
        "baseline": base_cm,
        "shadow_as_hard": shadow_cm,
        "hard": hard_cm,
        "fp_reduction": base_cm["FP"] - shadow_cm["FP"],
        "fn_increase": shadow_cm["FN"] - base_cm["FN"],
        "recall_drop": recall_drop,
        "precision_gain": precision_gain,
        "gate_pass_fail_matrix": gpf,
        # AC-5/6 판정
        "hard_recall_ok(>=0.80)": (hard_recall is not None and hard_recall >= 0.80),
        "hard_precision_improved(>baseline)": ((shadow_cm["precision"] or 0) > (base_cm["precision"] or 0)),
        "hard_apply_verdict": (
            "SUCCESS_CANDIDATE" if (hard_recall is not None and hard_recall >= 0.80
                                    and (shadow_cm["precision"] or 0) > (base_cm["precision"] or 0))
            else "FAIL_RECALL<0.80" if (hard_recall is not None and hard_recall < 0.80)
            else "NO_PRECISION_GAIN"),
    }

    # ── FR-10: stabilization variance 비교 ──
    # 가장 긴 "연속 TP(=객체 가시+발행)" 구간에서만 산출 — FP/위치점프 프레임이
    # spread 를 오염시키지 않도록 한다. (only_Milk 는 전 구간이 TP → 전체 시퀀스)
    def longest_run(frs):
        best, cur = [], []
        for fr in frs:
            ok = fr["pose_output_exists"] and fr["t_raw"][0] is not None and \
                (fr["decision_type"] in ("TP", "UNKNOWN") or not have_labels)
            if ok:
                cur.append(fr)
                if len(cur) > len(best):
                    best = cur[:]
            else:
                cur = []
        return best

    seq = longest_run(frames)
    t_raw_spread = trans_spread_mm([fr["t_raw"] for fr in seq])
    t_stab_spread = trans_spread_mm([fr["t_stab"] for fr in seq])
    r_raw_spread = rot_spread_deg([fr["q_raw"] for fr in seq])
    r_stab_spread = rot_spread_deg([fr["q_stab"] for fr in seq])

    # 연속 프레임 delta 도 동일 구간 내에서만 (FP→TP 점프 제외)
    dtr_raw = [math.dist(seq[i]["t_raw"], seq[i-1]["t_raw"]) for i in range(1, len(seq))]
    dtr_stab = [math.dist(seq[i]["t_stab"], seq[i-1]["t_stab"]) for i in range(1, len(seq))]
    drr_raw = [d for i in range(1, len(seq))
               if (d := quat_geodesic_deg(seq[i]["q_raw"], seq[i-1]["q_raw"])) is not None]
    drr_stab = [d for i in range(1, len(seq))
                if (d := quat_geodesic_deg(seq[i]["q_stab"], seq[i-1]["q_stab"])) is not None]
    pub = seq

    def mstats(v):
        return {"mean": round(sum(v)/len(v), 3), "p95": round(percentile(v, 95), 3),
                "max": round(max(v), 3)} if v else {"mean": None, "p95": None, "max": None}

    stabilization_comparison = {
        "n_published_frames": len(pub),
        "numpy_available": NUMPY_OK,
        "rotation_spread_computed": NUMPY_OK,
        "translation_spread_mm": {  # 평균 위치 대비 결합 표준편차 (positional jitter)
            "raw": round(t_raw_spread, 4) if t_raw_spread is not None else None,
            "stabilized": round(t_stab_spread, 4) if t_stab_spread is not None else None,
            "reduction_rate": reduction(t_raw_spread, t_stab_spread)},
        "rotation_spread_deg": {    # 평균 회전 대비 각도 RMS (rotational jitter)
            "raw": round(r_raw_spread, 4) if r_raw_spread is not None else None,
            "stabilized": round(r_stab_spread, 4) if r_stab_spread is not None else None,
            "reduction_rate": reduction(r_raw_spread, r_stab_spread)},
        "frame_to_frame_delta_translation_mm": {"raw": mstats(dtr_raw), "stabilized": mstats(dtr_stab),
            "reduction_rate_mean": reduction(
                sum(dtr_raw)/len(dtr_raw) if dtr_raw else None,
                sum(dtr_stab)/len(dtr_stab) if dtr_stab else None)},
        "frame_to_frame_delta_rotation_deg": {"raw": mstats(drr_raw), "stabilized": mstats(drr_stab),
            "reduction_rate_mean": reduction(
                sum(drr_raw)/len(drr_raw) if drr_raw else None,
                sum(drr_stab)/len(drr_stab) if drr_stab else None)},
        "note": "reduction_rate = 1 - stabilized/raw. spread=평균대비 표준편차, delta=연속프레임 변화량.",
    }

    metrics = {
        "bag_name": args.bag, "total_frames": len(frames), "labels_provided": have_labels,
        "total_no_milk_frames": no_milk, "true_negative_no_pose_frames": tn,
        "false_positive_pose_frames": fp,
        "false_positive_rate": round(fp/no_milk, 4) if no_milk else None,
        "true_negative_rate": round(tn/no_milk, 4) if no_milk else None,
        "total_milk_frames": milk, "tp_frames": tp, "fn_frames": fn,
        "ground_truth": "visibility_labels.csv" if have_labels else "MISSING (정확 분류 불가)",
        "confusion_matrix": confusion_matrix,
        "precision": precision, "recall": recall, "f1_score": f1, "accuracy": accuracy,
        "decision_counts": {"TP": tp, "TN": tn, "FP": fp, "FN": fn, "UNKNOWN": dts_all.count("UNKNOWN")},
        "gate_metrics": gate_metrics,
        "stabilization_comparison": stabilization_comparison,
    }
    with open(os.path.join(args.out, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2, ensure_ascii=False)

    print(f"[OK] {args.bag}: {len(frames)} frames → {args.out}/frame_results.csv")
    if not have_labels:
        print("  [경고] visibility_labels.csv 미제공 → GT 없음. Precision/Recall/F1 부정확(전부 visible 가정).")
    print(f"  Confusion Matrix (GT × prediction):")
    print(f"                 pred:pose   pred:no_pose")
    print(f"    GT milk有  :   TP={tp:<5}    FN={fn}")
    print(f"    GT milk無  :   FP={fp:<5}    TN={tn}")
    print(f"  Precision={precision}  Recall={recall}  F1={f1}  Accuracy={accuracy}")
    print(f"  stabilization: t_reduce={stabilization_comparison['translation_spread_mm']['reduction_rate']} "
          f"r_reduce={stabilization_comparison['rotation_spread_deg']['reduction_rate']}")
    if gate_metrics["gate_mode"]:
        b = gate_metrics["baseline"]; s = gate_metrics["shadow_as_hard"]
        print(f"  [depth_gate mode={gate_metrics['gate_mode']}] "
              f"baseline FP={b['FP']} R={b['recall']} P={b['precision']} → "
              f"shadow_as_hard FP={s['FP']} FN={s['FN']} R={s['recall']} P={s['precision']} "
              f"(ΔFP={-gate_metrics['fp_reduction']:+d} ΔFN={gate_metrics['fn_increase']:+d}) "
              f"verdict={gate_metrics['hard_apply_verdict']}")


if __name__ == "__main__":
    main()
