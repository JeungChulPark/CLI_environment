#!/usr/bin/env python3
"""mask_cache.py — MobileSAM 마스크를 프레임 사이에 재사용한다.

왜
--
이 기계에서 잰 MobileSAM 비용(RTX 3080 Ti Laptop):

    박스 1개  35.3 ms      박스 4개  38.8 ms      박스 8개  46.5 ms

박스가 8배로 늘어도 11 ms 밖에 안 는다 — 거의 전부가 **박스 수와 무관한 고정비**
(이미지 인코더)다. 그래서 "박스를 줄이는" 최적화는 듣지 않고, **"프레임을 건너뛰는"**
최적화만 듣는다. ISM 신경망 합계 50.5 ms 중 MobileSAM 이 38.8 ms(77%)이므로
10 프레임에 한 번만 돌리면 그 자리가 ~4 ms 가 된다.

무엇을 하나
----------
직전에 계산한 마스크를 박스와 함께 들고 있다가, 새 프레임의 박스가 **충분히 겹치면**
(IoU >= iou_thresh) 그 마스크를 박스 이동량만큼 평행이동해서 돌려준다. 겹침이 부족하거나
너무 오래됐거나 주기가 됐으면 그 박스만 MobileSAM 에 다시 넣는다.

즉 프레임 단위가 아니라 **박스 단위**로 재사용을 판단한다. 새로 들어온 물체 하나 때문에
프레임 전체를 다시 계산하지 않는다.

안전장치
--------
* `iou_thresh`  겹침이 이보다 낮으면 재계산. 물체가 빨리 움직이면 자동으로 재계산된다.
* `max_age`     이 프레임 수를 넘긴 마스크는 버린다. 조용히 낡는 것을 막는다.
* `interval`    이 주기마다 무조건 전부 재계산. 누적 드리프트의 상한을 강제한다.
* 어느 하나라도 걸리면 **그 박스는 원래대로 MobileSAM 을 탄다.** 재사용은 언제나 선택적이다.

`yolo_ism.segment_boxes()` 와 **같은 계약**을 지킨다: `boxes` 순서에 정렬된 전체 이미지
bool 마스크 목록, 만들지 못한 자리는 `None`. 그래서 호출부는 한 줄도 바뀌지 않는다.
`enabled=False` 면 원본 함수로 그대로 위임한다(§26 원본 동작 보존).
"""
from __future__ import annotations

import numpy as np


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


def _shift(mask, dx, dy):
    """마스크를 (dx, dy) 만큼 평행이동. 밀려난 자리는 False."""
    if dx == 0 and dy == 0:
        return mask
    out = np.zeros_like(mask)
    h, w = mask.shape
    sy0, sy1 = max(0, -dy), min(h, h - dy)
    dy0, dy1 = max(0, dy), min(h, h + dy)
    sx0, sx1 = max(0, -dx), min(w, w - dx)
    dx0, dx1 = max(0, dx), min(w, w + dx)
    if sy1 > sy0 and sx1 > sx0:
        out[dy0:dy1, dx0:dx1] = mask[sy0:sy1, sx0:sx1]
    return out


class MaskCache:
    """프레임 사이에 MobileSAM 마스크를 재사용한다. Sam6DCore 인스턴스당 하나."""

    def __init__(self, enabled=True, iou_thresh=0.90, max_age=5, interval=0,
                 shift=True, log=None):
        self.enabled = bool(enabled)
        self.iou_thresh = float(iou_thresh)
        self.max_age = int(max_age)
        self.interval = int(interval)      # 0 = 주기 강제 없음
        self.shift = bool(shift)
        self.log = log
        self._entries = []                 # [{box, mask, frame}]
        self._frame = 0
        self.stats = dict(frames=0, boxes=0, reused=0, recomputed=0,
                          forced=0, sam_calls=0)

    def reset(self):
        self._entries.clear()

    # ------------------------------------------------------------------ 조회
    def _assign(self, boxes, shape):
        """박스 <-> 캐시 항목을 **일대일**로 맺는다.

        IoU 만으로 각 박스가 독립적으로 최적 항목을 고르게 두면, 물체가 직전 프레임에
        **다른 물체가 있던 자리**로 이동했을 때 그 물체의 마스크를 받아 간다. 합성
        테스트에서 이 버그가 잡혔다(속도 30px/frame 에서 재사용률이 속도 12 보다 높게
        나왔다 — 물체들이 서로의 옛 자리로 이동한 것). 그래서 항목 하나는 최대 한
        박스만 뒷받침하고, 겹침이 큰 쌍부터 확정한다.
        """
        live = [e for e in self._entries
                if e["mask"].shape == shape and self._frame - e["frame"] <= self.max_age]
        pairs = []
        for bi, b in enumerate(boxes):
            for ei, e in enumerate(live):
                v = _iou(e["box"], b)
                if v >= self.iou_thresh:
                    pairs.append((v, bi, ei))
        pairs.sort(reverse=True)
        taken_b, taken_e, out = set(), set(), {}
        for v, bi, ei in pairs:
            if bi in taken_b or ei in taken_e:
                continue
            taken_b.add(bi); taken_e.add(ei)
            e, cb = live[ei], boxes[bi]
            if not self.shift:
                out[bi] = e["mask"]
            else:
                pb = e["box"]
                dx = int(round(((cb[0] + cb[2]) - (pb[0] + pb[2])) / 2.0))
                dy = int(round(((cb[1] + cb[3]) - (pb[1] + pb[3])) / 2.0))
                out[bi] = _shift(e["mask"], dx, dy)
        return out

    # ------------------------------------------------------------------ 본체
    def segment_boxes(self, segment_fn, seg, bgr, boxes, device):
        """`yolo_ism.segment_boxes` 와 같은 계약. segment_fn 이 원본 함수."""
        boxes = [list(map(int, b)) for b in boxes]
        if not self.enabled:
            if boxes:
                self.stats["sam_calls"] += 1
            return segment_fn(seg, bgr, boxes, device)

        self._frame += 1
        self.stats["frames"] += 1
        self.stats["boxes"] += len(boxes)
        if not boxes:
            return []

        shape = bgr.shape[:2]
        forced = bool(self.interval) and (self._frame % self.interval == 1)
        if forced:
            self.stats["forced"] += 1

        out = [None] * len(boxes)
        hits = {} if forced else self._assign(boxes, shape)
        todo = []
        for i, b in enumerate(boxes):
            m = hits.get(i)
            if m is None:
                todo.append(i)
            else:
                out[i] = m
                self.stats["reused"] += 1

        if todo:
            self.stats["sam_calls"] += 1
            self.stats["recomputed"] += len(todo)
            fresh = segment_fn(seg, bgr, [boxes[i] for i in todo], device)
            for k, i in enumerate(todo):
                out[i] = fresh[k] if k < len(fresh) else None

        # 이번 프레임에서 실제로 계산한 것만 캐시에 넣는다. 재사용한 것을 다시 넣으면
        # 평행이동 오차가 누적되어 원본에서 얼마나 멀어졌는지 알 수 없게 된다.
        if todo:
            keep = [e for e in self._entries
                    if self._frame - e["frame"] <= self.max_age]
            for i in todo:
                if out[i] is not None:
                    keep.append(dict(box=boxes[i], mask=out[i], frame=self._frame))
            self._entries = keep[-32:]
        return out

    def summary(self):
        s = self.stats
        n = max(1, s["boxes"])
        return (f"mask_cache: frames={s['frames']} boxes={s['boxes']} "
                f"reused={s['reused']} ({100.0*s['reused']/n:.1f}%) "
                f"recomputed={s['recomputed']} sam_calls={s['sam_calls']} "
                f"forced_refresh={s['forced']}")


def install(core, cfg, log=print):
    """Sam6DCore 에 캐시를 끼운다. yolo_ism.py 는 건드리지 않는다.

    `sam6d_realtime/README.md` 가 yolo_ism*.py 를 "원본 그대로 복사(수정 금지)" 로
    못박아 두었으므로, 원본 함수를 감싼 것을 모듈 속성으로 바꿔 끼우는 방식을 쓴다.
    끄면 원본이 그대로 복원된다.
    """
    import yolo_ism as yi

    cfg = dict(cfg or {})
    if not cfg.get("enabled"):
        return None
    cache = MaskCache(
        enabled=True,
        iou_thresh=float(cfg.get("iou_thresh", 0.90)),
        max_age=int(cfg.get("max_age", 5)),
        interval=int(cfg.get("interval", 0)),
        shift=bool(cfg.get("shift", True)),
        log=log,
    )
    if getattr(yi, "_orig_segment_boxes", None) is None:
        yi._orig_segment_boxes = yi.segment_boxes

    def wrapped(seg, bgr, boxes, device):
        return cache.segment_boxes(yi._orig_segment_boxes, seg, bgr, boxes, device)

    yi.segment_boxes = wrapped
    core.mask_cache = cache
    log(f"[ism] mask_cache on — iou>={cache.iou_thresh} max_age={cache.max_age} "
        f"interval={cache.interval or 'off'} shift={cache.shift}")
    return cache


def uninstall():
    import yolo_ism as yi
    if getattr(yi, "_orig_segment_boxes", None) is not None:
        yi.segment_boxes = yi._orig_segment_boxes
        yi._orig_segment_boxes = None
