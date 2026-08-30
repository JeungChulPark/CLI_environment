#!/usr/bin/env python3
"""rules.py — training-free gate 결정 규칙 (사전 고정, 학습·GT-fit 없음).

각 함수: cands(한 cell의 후보 dict 리스트, pool 컬럼) -> (accept:bool, chosen_uid|None, reason).
파라미터 출처 = {운영 threshold, 고정 비율, 후보 내부 상대 margin, 42뷰 통계}. GT 미사용.
후보 dict 키: sem_top5,appe11,hsv,appe2,appe9,sem_all_mean,passS,passA,passH,accept,
  sim_thr,appe_gate,hsv_thr,conf_rank,is_selected,view_pass_count,view_median,view_top1_minus_median.
"""

# ---- 사전 고정 상수 (GT 미사용) ----
BORDERLINE_RATIO = 0.90     # score >= ratio*thr 이면 'borderline fail'(severe 아님)
SEVERE_RATIO = 0.50         # score < severe*thr 이면 severe fail (rescue 금지)
STRONG_FRAC = 0.10          # 통과 gate가 thr + frac*(max-thr) 이상이면 'strong'
VIEW_STABLE_N = 5           # 42뷰 중 sim_thr 통과 개수 >= N 이면 semantic 안정
SCORE_MAX = 1.0


def _sel(cands):
    for c in cands:
        if c["is_selected"]:
            return c
    return max(cands, key=lambda c: c["sem_top5"]) if cands else None


def phase1c(cands):
    """P0 운영: best-by-sem 이 3 gate 모두 통과."""
    s = _sel(cands)
    if s is None:
        return False, None, "no_candidate"
    return bool(s["accept"]), (s["uid"] if s["accept"] else None), ("accept" if s["accept"] else "phase1c_reject")


def rerank_topk(cands, K):
    """A. conf-top-K 후보 중 3 gate 모두 통과하는 최상위(conf) 후보 선택. threshold 불변."""
    ck = sorted(cands, key=lambda c: c["conf_rank"])[:K]
    for c in ck:
        if c["accept"]:
            return True, c["uid"], f"rerank_top{K}_pass_rank{c['conf_rank']}"
    return False, None, f"rerank_top{K}_none"


def _strong(score, thr):
    return score >= thr + STRONG_FRAC * (SCORE_MAX - thr)


def one_borderline_rescue(cands):
    """B. 선택 후보가 정확히 1개 gate만 borderline-fail, 나머지 2개 strong-pass 이면 rescue.
    HSV severe-fail 은 절대 rescue 금지(FP+248 방지)."""
    s = _sel(cands)
    if s is None:
        return False, None, "no_candidate"
    if s["accept"]:
        return True, s["uid"], "accept"
    fails = [g for g in ("S", "A", "H") if not s[{"S": "passS", "A": "passA", "H": "passH"}[g]]]
    if len(fails) != 1:
        return False, None, "rescue_no(multi_fail)"
    g = fails[0]
    sc = {"S": s["sem_top5"], "A": s["appe11"], "H": s["hsv"]}[g]
    thr = {"S": s["sim_thr"], "A": s["appe_gate"], "H": s["hsv_thr"]}[g]
    if sc < SEVERE_RATIO * thr:
        return False, None, f"rescue_no(severe_{g})"
    if sc < BORDERLINE_RATIO * thr:
        return False, None, f"rescue_no(not_borderline_{g})"
    # 나머지 2 gate strong?
    others = {"S": (s["sem_top5"], s["sim_thr"]), "A": (s["appe11"], s["appe_gate"]),
              "H": (s["hsv"], s["hsv_thr"])}
    ok = all(_strong(*others[x]) for x in ("S", "A", "H") if x != g)
    if ok:
        return True, s["uid"], f"rescue_borderline_{g}"
    return False, None, f"rescue_no(others_not_strong_{g})"


def rank_consensus(cands):
    """C. 선택 후보가 sem·appe 모두 cell 내 rank-1(consensus) 이면서 S·H 통과 → appe 절대게이트 대체.
    (appe11 절대임계 대신 상대 순위 사용, threshold 학습 없음)."""
    s = _sel(cands)
    if s is None:
        return False, None, "no_candidate"
    if s["accept"]:
        return True, s["uid"], "accept"
    # appe11 만 실패한 경우, appe rank-1 이고 sem rank-1 이면 consensus 로 통과
    top_sem = max(cands, key=lambda c: c["sem_top5"])["uid"]
    top_appe = max(cands, key=lambda c: c["appe11"])["uid"]
    if s["passS"] and s["passH"] and not s["passA"] and s["uid"] == top_sem == top_appe:
        return True, s["uid"], "rank_consensus_appe"
    return False, None, "consensus_no"


def multiblock_consensus(cands):
    """D. appe11 절대게이트 대신 block2/9/11 rank 합의(선택후보가 >=2 block 에서 cell rank-1).
    per-block 절대임계 없음(rank-only)."""
    s = _sel(cands)
    if s is None:
        return False, None, "no_candidate"
    if s["passS"] and s["passH"]:
        votes = 0
        for col in ("appe2", "appe9", "appe11"):
            if s["uid"] == max(cands, key=lambda c: c[col])["uid"]:
                votes += 1
        if votes >= 2:
            return True, s["uid"], f"multiblock_{votes}of3"
    if s["accept"]:
        return True, s["uid"], "accept"
    return False, None, "multiblock_no"


def view_stability(cands):
    """E. semantic borderline-fail 이지만 42뷰 중 N개 이상 통과(일관) & appe·hsv 통과 → rescue."""
    s = _sel(cands)
    if s is None:
        return False, None, "no_candidate"
    if s["accept"]:
        return True, s["uid"], "accept"
    if (not s["passS"]) and s["passA"] and s["passH"] and s["view_pass_count"] >= VIEW_STABLE_N \
            and s["sem_top5"] >= BORDERLINE_RATIO * s["sim_thr"]:
        return True, s["uid"], f"view_stable_{s['view_pass_count']}"
    return False, None, "view_no"


def pareto_select(cands):
    """G. (sem,appe11,hsv) Pareto 비지배 후보 중 3 gate 통과 최상위 선택. severe HSV 제외."""
    if not cands:
        return False, None, "no_candidate"
    pool = [c for c in cands if c["hsv"] >= SEVERE_RATIO * c["hsv_thr"]]
    if not pool:
        pool = cands
    def dominated(a):
        for b in pool:
            if b is a:
                continue
            if (b["sem_top5"] >= a["sem_top5"] and b["appe11"] >= a["appe11"] and b["hsv"] >= a["hsv"]
                    and (b["sem_top5"] > a["sem_top5"] or b["appe11"] > a["appe11"] or b["hsv"] > a["hsv"])):
                return True
        return False
    front = [c for c in pool if not dominated(c)]
    passing = [c for c in front if c["accept"]]
    if passing:
        best = max(passing, key=lambda c: c["sem_top5"])
        return True, best["uid"], "pareto_pass"
    return False, None, "pareto_no"


METHODS = {
    "P0_phase1c": phase1c,
    "P1_rerank_top2": lambda c: rerank_topk(c, 2),
    "P1_rerank_top3": lambda c: rerank_topk(c, 3),
    "P7_one_borderline_rescue": one_borderline_rescue,
    "P2_rank_consensus": rank_consensus,
    "P3_multiblock_consensus": multiblock_consensus,
    "P4_view_stability": view_stability,
    "P6_pareto_select": pareto_select,
}
