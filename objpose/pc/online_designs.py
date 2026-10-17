"""online_designs.py — 지도 설계(합의로 합치기 · 깊이로 확인하고 다듬기 · 약한 증거 누적)를 실행 중에 계산한다.

인식 결과가 한 프레임 도착할 때마다 그때까지 들어온 자료만으로 설계 조합 11가지의 지도를 다시 만든다(objpose/tools/lab/map_designs.py 의 run 을 그대로 쓴다).
지금 방식의 지도(ObjectMemory)는 손대지 않고, 그 지도의 객체 · 카메라 자세(키프레임 보정 반영)를 매번 그대로 읽는다. 설계는 인식에 영향을 주지 않으므로 한 번의 실행에서 모든 조합을 함께 낸다.
  add()    인식 결과 한 프레임: 통과한 자세(검증 점수 포함), 떨어진 후보(이름 + 상자), 그 프레임의 깊이 영상(전체 해상도)
  run()    별도 스레드. 새 프레임이 있거나 키프레임이 고쳐졌으면 한 번 갱신한다. 갱신이 인식보다 느리면 밀린 프레임을 다음 갱신에서 한꺼번에 반영한다(언제나 최신 상태만 계산)
  final()  마지막 프레임까지 반영한 지도 + 갱신 시간 통계
갱신 시간: pass_ms = 11가지를 모두 계산한 한 번의 갱신, alone_ms[조합] = 그 조합 하나만 돌렸을 때의 시간, lag_ms = 인식 결과가 도착한 뒤 그 프레임이 지도에 반영되기까지.
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "objpose/tools/lab"))
_argv = sys.argv; sys.argv = [_argv[0]]
import map_designs as MD      # noqa: E402
sys.argv = _argv

KEEP = MD.KEEP


class _LiveScene(MD.Scene):
    """map_designs.Scene 과 같은 계산, 입력만 실행 중인 상태(지금까지의 프레임)에서 가져온다."""

    def __init__(self, o: "OnlineDesigns", lm0, cams):
        self.o = o; self.K = o.K; self.ses = SimpleNamespace(W=o.W, H=o.H); self.model = o.model; self.tau = 0; self.lm0 = lm0; self.Tws = cams; self.saved_s = 0.0; self._f32 = {}
        ok = [(st, T) for st, T in cams.items() if T is not None]; self._st = [st for st, _ in ok]
        self._Tcw = np.linalg.inv(np.stack([T for _, T in ok])) if ok else np.zeros((0, 4, 4))

    def depth(self, stamp):
        if stamp not in self._f32:
            d = self.o.depths.get(stamp); self._f32[stamp] = None if d is None else d.astype(np.float32) / 1000.0
            if len(self._f32) > 160: self._f32.pop(next(iter(self._f32)))
        return self._f32[stamp]

    def observations(self):
        out = []
        for st, name, T, w, dist in self.o.obs:
            Tw = self.Tws.get(st)
            if Tw is not None: out.append({"name": name, "T": Tw @ T, "w": w, "stamp": st, "dist": dist})
        return out

    def weak(self):
        out = []
        for st, name, p in self.o.weak:
            Tw = self.Tws.get(st)
            if Tw is not None: out.append({"name": name, "p": (Tw @ p)[:3], "stamp": st})
        return out

    def views(self, c, n=30):
        if not len(self._Tcw): return []
        x = self._Tcw[:, :3, :3] @ c + self._Tcw[:, :3, 3]; z = np.where(np.abs(x[:, 2]) < 1e-9, 1e-9, x[:, 2]); u = self.K[0, 0] * x[:, 0] / z + self.K[0, 2]; v = self.K[1, 1] * x[:, 1] / z + self.K[1, 2]
        idx = np.flatnonzero((x[:, 2] > 0.3) & (x[:, 2] < 2.0) & (u > 30) & (u < self.ses.W - 30) & (v > 30) & (v < self.ses.H - 30))
        if len(idx) > n: idx = idx[np.linspace(0, len(idx) - 1, n).astype(int)]
        return [(self._st[i], self._Tcw[i], float(x[i, 2])) for i in idx]

    def _memo(self, kind, fn, name, T, views):
        """같은 객체 · 같은 자세 · 같은 프레임 묶음이면 앞서 계산한 값을 다시 쓴다(조합끼리, 그리고 그 객체가 화면 밖에 있는 동안)."""
        key = (kind, name, np.asarray(T).round(6).tobytes(), tuple(st for st, _, _ in views)); hit = self.o.memo.get(key)
        if hit is not None: self.saved_s += hit[1]; return hit[0]
        t = time.perf_counter(); val = fn(name, T, views); self.o.memo[key] = (val, time.perf_counter() - t); return val

    def support(self, name, T, views): return self._memo("s", super().support, name, T, views)

    def refine(self, name, T, views): return self._memo("r", super().refine, name, T, views)


class OnlineDesigns:
    def __init__(self, memory, K, size):
        self.mem = memory; self.K = np.asarray(K, float).reshape(3, 3); self.W, self.H = int(size[0]), int(size[1]); self.model = {}; self.memo = {}; self._memo_ver = None
        self.obs = []; self.weak = []; self.depths = {}; self.arrive = {}; self._lock = threading.Lock(); self._run = threading.Lock(); self._seen = (-1, -1); self.last = None; self.timeline = []; self._n_prev = 0

    # ── 입력 ──
    def add(self, t_ns, dets, weak, depth):
        """dets: [{object, R, t_mm, mask_iou, cluster_occupancy}] (인식 카메라 좌표), weak: [{object, bbox}] 검증에서 떨어진 후보, depth: 그 프레임의 깊이(uint16 mm, 전체 해상도) or None"""
        t_ns = int(t_ns); rows = []; pts = []; D = None if depth is None else np.asarray(depth)
        for d in dets:
            T = np.eye(4); T[:3, :3] = np.asarray(d["R"], float).reshape(3, 3); T[:3, 3] = np.asarray(d["t_mm"], float) / 1000.0; dist = float(np.linalg.norm(T[:3, 3]))
            mi = d.get("mask_iou"); co = d.get("cluster_occupancy")
            rows.append((t_ns, d["object"], T, float(mi or 0.5) * float(co or 0.5) * (1.0 if dist <= 1.5 else 0.5), dist))
        acc = {d["object"] for d in dets}
        for c in (weak or []):                       # 떨어진 후보의 3D 위치(상자 가운데 40 % 영역의 깊이 중앙값) — 도착했을 때 한 번만 구한다
            if D is None or c["object"] in acc or not c.get("bbox"): continue
            x1, y1, x2, y2 = c["bbox"]; cx, cy, w, h = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) * 0.2, (y2 - y1) * 0.2
            patch = D[int(max(cy - h, 0)):int(cy + h) + 1, int(max(cx - w, 0)):int(cx + w) + 1].astype(np.float32) / 1000.0; v = patch[(patch > 0.25) & (patch < 4.0)]
            if len(v) < 5: continue
            z = float(np.median(v)); pts.append((t_ns, c["object"], np.array([(cx - self.K[0, 2]) * z / self.K[0, 0], (cy - self.K[1, 2]) * z / self.K[1, 1], z, 1.0])))
        with self._lock:
            self.obs += rows; self.weak += pts; self.arrive[t_ns] = time.perf_counter()
            if D is not None: self.depths[t_ns] = np.ascontiguousarray(D).astype(np.uint16, copy=False)

    # ── 갱신 ──
    def step(self, force=False):
        with self._run:
            mem = self.mem; ver = mem.fusion.poses.kfs.updates
            with mem._lock: frames = list(mem.frames)[:mem._fed]; built = mem._kf_version_seen
            with self._lock: have = set(self.arrive)
            frames = [f for f in frames if f.t_ns in have]; state = (len(frames), built)
            if not frames or (state == self._seen and not force): return None
            t0 = time.perf_counter()
            if self._memo_ver != ver or len(self.memo) > 20000: self.memo.clear(); self._memo_ver = ver          # 키프레임이 고쳐지면 카메라 자세가 바뀌므로 다시 계산
            lm0 = [{"name": m["name"], "T_w_obj": np.asarray(m["T_w_obj"], float), "n_obs": m["n_obs"], "status": m["status"]} for m in mem.landmarks() if m["status"] in KEEP]
            cams = {f.t_ns: mem._camera(f) for f in frames}; timing = {}
            R = MD.run(_LiveScene(self, lm0, cams), timing); now = time.perf_counter(); newest = max(self.arrive[f.t_ns] for f in frames[self._n_prev:] or frames[-1:])
            self.timeline.append({"frames": len(frames), "new_frames": len(frames) - self._n_prev, "pass_ms": round((now - t0) * 1e3, 1), "lag_ms": round((now - newest) * 1e3, 1),
                                  "alone_ms": {k: round(v * 1e3, 1) for k, v in timing.items() if k != "total"}, "registered": {k: len(R[k]) for k in ["V0", "M0"] + MD.COMBOS}})
            self._seen = state; self._n_prev = len(frames); self.last = R
            return R

    def run(self, stop: threading.Event, idle_s=0.02):
        while not stop.is_set():
            try:
                if self.step() is None: time.sleep(idle_s)
            except Exception as e:                   # noqa: BLE001  (지도 설계 계산이 실행을 멈추게 하지 않는다)
                print(f"[online_designs] {type(e).__name__}: {e}", flush=True); time.sleep(0.2)

    def final(self):
        """마지막 프레임까지 반영한 지도(map_designs.run 과 같은 형식) + "online": 갱신 시간 통계. 마지막 갱신 뒤에 새 프레임이 없었으면 다시 계산하지 않는다."""
        self.step(); R = dict(self.last or {}); tl = self.timeline

        def st(v): v = np.asarray(v, float); return None if not len(v) else {"median": round(float(np.median(v)), 1), "p95": round(float(np.percentile(v, 95)), 1), "max": round(float(v.max()), 1)}
        R["online"] = {"passes": len(tl), "frames": tl[-1]["frames"] if tl else 0, "pass_ms": st([x["pass_ms"] for x in tl]), "lag_ms": st([x["lag_ms"] for x in tl]),
                       "new_frames_per_pass": st([x["new_frames"] for x in tl]), "alone_ms": {k: st([x["alone_ms"][k] for x in tl]) for k in ["V0", "M0"] + MD.COMBOS} if tl else {}}
        return R
