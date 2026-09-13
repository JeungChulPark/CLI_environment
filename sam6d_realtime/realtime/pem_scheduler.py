#!/usr/bin/env python3
"""pem_scheduler.py — PEM 을 프레임마다 전 객체에 돌리지 않고 나눠 돌린다.

왜
--
이 기계에서 잰 PEM 비용(RTX 3080 Ti Laptop, BF16 + fine_npoint=1024):

    검출 1개  48.0 ms      2개  60.1 ms      4개  87.2 ms      8개  149.3 ms

검출 1개에도 48 ms 를 내고, 4개면 87 ms 다. **검출당 증가분보다 고정비가 크다**
(1개 48 ms, 추가 1개당 +13 ms). 그러니 "프레임당 검출 수" 를 줄이는 것이 가장 값싼
카드다. 4개를 매 프레임 푸는 대신 한 프레임에 1~2개만 풀고 나머지는 직전 포즈를
그대로 내보내면, PEM 이 87 ms -> 48 ms 로 떨어진다.

전제: 30 FPS 영상에서 물체는 프레임 사이에 거의 움직이지 않는다. 4개 객체를 번갈아
풀면 각 객체는 4 프레임(=133 ms)마다 갱신된다. 그 사이의 포즈는 직전 값이다.

무엇을 하나
----------
매 프레임 검출 목록을 받아 **이번에 풀 것**과 **직전 포즈를 물려줄 것**으로 가른다.
우선순위는 이렇게 잡는다(앞이 높다):

  1. 한 번도 못 푼 객체            — 포즈가 아예 없으므로 반드시 푼다
  2. 오래 안 푼 객체               — 마지막 갱신이 오래됐을수록 먼저
  3. 직전 점수가 낮았던 객체       — 불확실한 것을 먼저 다시 본다

`max_age` 프레임을 넘겨 갱신되지 못한 객체는 **강제로 이번에 푼다**. 예산(budget)이
모자라도 마찬가지다 — 조용히 낡은 포즈가 계속 나가는 것을 막는다.

한계
----
* 물려준 포즈는 **그 시점의 포즈**다. 물체나 카메라가 움직이면 그만큼 낡는다.
  `max_age` 가 그 상한을 강제한다.
* 객체가 빠르게 움직이는 장면에서는 budget 을 키우거나 꺼야 한다.
* 포즈를 물려줄 때 `pose_source` 를 `"carried"` 로 바꾸고 `carried_age` 를 실어
  보내므로, 소비자가 "이건 이번 프레임에 푼 게 아니다" 를 구분할 수 있다.
"""
from __future__ import annotations

import copy


class PemScheduler:
    """프레임당 PEM 에 넣을 객체를 고르고, 나머지는 직전 결과를 물려준다."""

    def __init__(self, enabled=True, budget=1, max_age=4, log=None):
        self.enabled = bool(enabled)
        self.budget = max(1, int(budget))     # 프레임당 실제로 풀 객체 수
        self.max_age = max(1, int(max_age))   # 이 프레임 수를 넘기면 예산 무시하고 푼다
        self.log = log
        self._last = {}                       # name -> {"row":..., "frame":..., "score":...}
        self._frame = 0
        self.stats = dict(frames=0, seen=0, solved=0, carried=0, forced=0)

    def reset(self):
        self._last.clear()

    # ------------------------------------------------------------------ 선택
    def select(self, names):
        """이번 프레임에 풀 이름 집합을 고른다. enabled=False 면 전부."""
        self._frame += 1
        self.stats["frames"] += 1
        self.stats["seen"] += len(names)
        if not self.enabled or len(names) <= self.budget:
            self.stats["solved"] += len(names)
            return set(names)

        forced, rest = [], []
        for n in names:
            e = self._last.get(n)
            if e is None:
                forced.append(n)                        # 처음 보는 객체
            elif self._frame - e["frame"] >= self.max_age:
                forced.append(n)                        # 너무 오래됨
            else:
                rest.append(n)

        pick = list(forced)
        self.stats["forced"] += len(forced)
        if len(pick) < self.budget:
            # 오래된 것 우선, 같으면 직전 점수가 낮은 것 우선
            def key(n):
                e = self._last[n]
                return (-(self._frame - e["frame"]), e.get("score") or 0.0)
            rest.sort(key=key)
            pick += rest[: self.budget - len(pick)]

        pick = set(pick)
        self.stats["solved"] += len(pick)
        self.stats["carried"] += len(names) - len(pick)
        return pick

    # ------------------------------------------------------------------ 기록
    def remember(self, rows):
        """이번 프레임에 실제로 푼 결과를 기록한다."""
        for r in rows:
            nm = r.get("object")
            if nm is None:
                continue
            self._last[nm] = dict(row=copy.deepcopy(r), frame=self._frame,
                                  score=r.get("score"))

    def carried_rows(self, names):
        """이번에 풀지 않은 객체의 직전 결과를 나이 표시와 함께 돌려준다."""
        out = []
        for n in names:
            e = self._last.get(n)
            if e is None:
                continue
            age = self._frame - e["frame"]
            if age > self.max_age:
                continue                       # 너무 낡았으면 아예 내보내지 않는다
            r = copy.deepcopy(e["row"])
            r["pose_source"] = "carried"
            r["carried_age"] = int(age)
            out.append(r)
        return out

    def summary(self):
        s = self.stats
        n = max(1, s["seen"])
        return (f"pem_scheduler: frames={s['frames']} dets={s['seen']} "
                f"solved={s['solved']} ({100.0*s['solved']/n:.1f}%) "
                f"carried={s['carried']} forced={s['forced']} "
                f"budget={self.budget} max_age={self.max_age}")


def from_config(cfg, log=print):
    cfg = dict(cfg or {})
    if not cfg.get("enabled"):
        return None
    s = PemScheduler(enabled=True,
                     budget=int(cfg.get("budget", 1)),
                     max_age=int(cfg.get("max_age", 4)),
                     log=log)
    log(f"[pem] scheduler on — budget={s.budget}/frame max_age={s.max_age}")
    return s
