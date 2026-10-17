"""consensus_pose.py — 화면에 보이는 지도 객체의 포즈를 관측 사이의 합의로 정한다(실시간 화면의 기본 동작).

객체의 등록 · 삭제는 ObjectMemory 가 그대로 하고, 등록된 객체의 포즈만 바꾼다. 계산은 map_designs.consensus(설계 "1n")와 같다:
관측을 같은 이름의 가장 가까운 객체(30 cm 안)에 배정하고, 위치 2.5 cm · 방향 20° 안에서 가장 많은 관측이 동의하는 관측의 방향과 그 무리의 가중 평균 위치를 쓴다.
  add()    인식 결과 한 프레임: 통과한 자세(인식 카메라 좌표)와 검증 점수
  poses()  → {object_id: T_w_obj(4x4)}. 새 관측이 오거나 키프레임이 고쳐졌을 때만 다시 계산한다
OBJPOSE_MAP_POSE=mean 으로 hub 를 실행하면 쓰지 않는다(예전 동작: 누적 평균).
"""
from __future__ import annotations

import threading
import time

import numpy as np

from online_designs import MD, KEEP


class ConsensusPose:
    def __init__(self, memory):
        self.mem = memory; self.obs = []; self._lock = threading.Lock(); self._seen = None; self._out = {}; self.ms = []

    def add(self, t_ns, dets):
        rows = []
        for d in dets:
            T = np.eye(4); T[:3, :3] = np.asarray(d["R"], float).reshape(3, 3); T[:3, 3] = np.asarray(d["t_mm"], float) / 1000.0; dist = float(np.linalg.norm(T[:3, 3]))
            rows.append((int(t_ns), d["object"], T, float(d.get("mask_iou") or 0.5) * float(d.get("cluster_occupancy") or 0.5) * (1.0 if dist <= 1.5 else 0.5)))
        with self._lock: self.obs += rows

    def poses(self):
        mem = self.mem
        with self._lock:
            n = len(self.obs); lms = [m for m in mem.landmarks() if m["status"] in KEEP]
            state = (n, mem.fusion.poses.kfs.updates, mem._kf_version_seen, tuple((m["object_id"], m["n_obs"]) for m in lms))
            if state == self._seen: return self._out
            t0 = time.perf_counter(); obs = self.obs[:n]
            with mem._lock: frames = list(mem.frames)[:mem._fed]
            cams = {f.t_ns: mem._camera(f) for f in frames}; C = [np.asarray(m["T_w_obj"], float)[:3, 3] for m in lms]; assign = {i: [] for i in range(len(lms))}
            for st, name, T, w in obs:
                Tw = cams.get(st)
                if Tw is None: continue
                Two = Tw @ T; c = [(np.linalg.norm(Two[:3, 3] - C[i]), i) for i, m in enumerate(lms) if m["name"] == name]
                if c and min(c)[0] < 0.30: assign[min(c)[1]].append({"T": Two, "w": w})
            out = {}
            for i, m in enumerate(lms):
                r = MD.consensus(m["name"], assign[i])
                if r is not None: out[m["object_id"]] = r[0]
            self._seen = state; self._out = out; self.ms.append((time.perf_counter() - t0) * 1e3)
            if len(self.ms) > 5000: del self.ms[:2500]
            return out

    def stats(self):
        v = np.asarray(self.ms, float)
        return None if not len(v) else {"updates": int(len(v)), "median_ms": round(float(np.median(v)), 1), "max_ms": round(float(v.max()), 1)}
