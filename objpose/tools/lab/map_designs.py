#!/usr/bin/env python3
"""map_designs.py — 실제 환경 실행 한 건(hub 실행 폴더)의 기록으로 객체 지도를 네 가지 방식으로 만든다. 인식기는 다시 돌리지 않는다.

  V0  지금 방식            실행 중에 만들어진 지도 그대로(summary.json 의 memory_landmarks). 같은 이름 30 cm 묶기, 위치 평균 · 방향 다수결, 존재 확률, 깊이 검사(빈 자리 삭제)
  V1  설계 1: 합의로 합치기  객체마다 묶인 관측 가운데 서로 가까운(25 mm, 20°, 대칭 고려) 자세의 가장 큰 무리만 쓴다. 검증 점수(윤곽 겹침 × 합의 비율)와 거리로 가중.
                           무리가 관측 3개 미만이거나 가중치의 30 % 미만이면 그 객체는 뺀다(자세가 모이지 않음).
  V2  설계 2: 깊이로 확인하고 다듬기  V1 의 객체마다 CAD 를 지도 자세로 여러 프레임에 투영해 깊이 영상과 비교한다.
                           지지율 = 맞는 점(2 cm 안) / (맞는 점 + 뚫린 점(5 cm 넘게 멀리 보임)). 앞이 가려진 점은 세지 않는다.
                           주변 깊이 점에 CAD 를 다시 맞춰(정합, 6 cm · 20° 안에서) 지지율이 오르면 그 자세를 쓴다. 판단할 프레임이 3장 이상인데 지지율이 0.2 미만이면(거의 전부 뚫려 보임) 뺀다.
  V3  설계 3: 약한 증거 누적  검증에서 떨어진 후보(이름 + 상자)도 깊이로 3D 위치를 구해 "여기 이 물체로 보이는 것이 있다"는 증거로 쌓는다.
                           (가) 지도에 못 올라간 관측 무리라도 통과한 자세가 1개 이상이고 같은 자리의 증거가 6개 이상이면 등록 후보로 올리고, 설계 2 의 확인(지지율 0.5 이상, 프레임 3장 이상)을 통과하면 등록.
                           (나) 10 cm 안에 이름이 다른 객체가 둘이면 증거가 많은 쪽만 남긴다.
  조합: 설계를 하나씩만 쓴 것("1", "2", "3"), 둘씩("1+2", "1+3", "2+3"), 셋 모두("1+2+3"), 그리고 설계 1 에서 객체를 빼는 규칙을 없앤 "1n" 과 그 조합. 언제나 1 → 2 → 3 순서로 적용한다.
  모든 방식의 객체에 설계 2 의 지지율을 함께 잰다(정답이 없는 영상에서 객체가 실제 표면 위에 놓였는지를 보는 값).
    <sam6d python> objpose/tools/lab/map_designs.py <run> --session <세션 이름> --out <json>
"""
import argparse, json, sys, time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree

REPO = Path(__file__).resolve().parents[3]
for p in ("objpose/pc", "objpose/tools/obj_anchor", "objectmemory_ws/object_memory/src"): sys.path.insert(0, str(REPO / p))
from conv_session import ConvSession                    # noqa: E402
from fit_extrinsic import final_poses                   # noqa: E402
from core.fusion import symmetry_group                  # noqa: E402

DATA = Path.home() / "DeepLearning/Datasets/data_slam_converted"; RT = REPO / "objpose/rt"; PTS = REPO / "sam6d_realtime/assets/model_points"
KEEP = ("active", "lost", "remembered"); DROP = 0.2     # 설계 2 가 객체를 빼는 지지율 기준. 작은 물체는 깊이 잡음으로 지지율이 낮게 나오므로 "거의 전부 뚫려 보이는" 경우만 뺀다
_=0; jl = lambda p: [json.loads(l) for l in open(p) if l.strip()]


def session_setup(name):
    """→ (인식 카메라 녹화 폴더, 두 카메라 사이 변환 4x4)"""
    if name == "260901_cbnu_eightcircle": return DATA / name / "SAM", RT / "260901_cbnu/X_slam_sam_260901_cbnu_eight.json"
    if name.startswith("260901_cbnu_"): return DATA / name / "SAM", RT / "260901_cbnu/X_slam_sam_260901_cbnu.json"
    if name.endswith("Object"): return DATA / name / "SLAM", RT / "single_cam/X_identity.json"
    return DATA / name / "SAM", RT / "X_slam_sam.json"


def rot_angle(A, B): return float(np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1))))


def as_list(M): return tuple(tuple(float(v) for v in r) for r in np.asarray(M))


class Scene:
    def __init__(self, run: Path, name: str):
        self.run = run; S = json.loads((run / "summary.json").read_text()); lane = (S.get("lanes") or [S])[0]; bag, xf = session_setup(name)
        self.X = np.asarray(json.loads(xf.read_text())["T_slam_sam"], float).reshape(4, 4); self.tau = int(round(float(lane.get("sam_tau_s") or 0.0) * 1e9))
        self.pb = final_poses(run); self.ses = ConvSession(bag, offset_ns=0, cache_dir=REPO / "objpose/output/.clock_cache"); self.K = np.asarray(self.ses.K, float); self._depth = {}
        self.lm0 = [m for m in S.get("memory_landmarks", []) if m["status"] in KEEP]
        self.frames = jl(run / "sam6d/frames.jsonl"); self.dets = jl(run / "sam6d/detections.jsonl"); self.model = {}
        self.Tws = {}
        for f in self.frames:
            T, _ = self.pb.at(int(f["stamp_ns"]) + self.tau); self.Tws[int(f["stamp_ns"])] = None if T is None else np.asarray(T, float) @ self.X
        self.summary = S

    def pts(self, name):
        if name not in self.model:
            P = np.load(PTS / f"{name}.npy") / 1000.0; self.model[name] = P[np.random.default_rng(0).choice(len(P), min(len(P), 1200), replace=False)]
        return self.model[name]

    def depth(self, stamp):
        if stamp not in self._depth:
            i = int(np.argmin(np.abs(self.ses.t_ns - stamp)))
            self._depth[stamp] = self.ses.read(i, True).depth_raw.astype(np.float32) / 1000.0 if abs(int(self.ses.t_ns[i]) - stamp) <= 5_000_000 else None
            if len(self._depth) > 400: self._depth.pop(next(iter(self._depth)))
        return self._depth[stamp]

    # ---- 관측 ----
    def observations(self):
        out = []
        for d in self.dets:
            Tw = self.Tws.get(int(d["stamp_ns"]))
            if Tw is None: continue
            T = np.eye(4); T[:3, :3] = np.asarray(d["R"], float).reshape(3, 3); T[:3, 3] = np.asarray(d["t_mm"], float) / 1000.0; dist = float(np.linalg.norm(T[:3, 3]))
            w = float(d.get("mask_iou") or 0.5) * float(d.get("cluster_occupancy") or 0.5) * (1.0 if dist <= 1.5 else 0.5)
            out.append({"name": d["object"], "T": Tw @ T, "w": w, "stamp": int(d["stamp_ns"]), "dist": dist})
        return out

    def weak(self):
        """검증에서 떨어진 후보의 3D 위치(상자 가운데 40 % 영역의 깊이 중앙값)"""
        out = []
        for f in self.frames:
            st = int(f["stamp_ns"]); Tw = self.Tws.get(st); acc = set(f.get("objects", []))
            if Tw is None: continue
            for c in f["diagnostics"].get("pem_candidates", []):
                if c["object"] in acc or not c.get("bbox"): continue
                D = self.depth(st)
                if D is None: continue
                x1, y1, x2, y2 = c["bbox"]; cx, cy, w, h = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) * 0.2, (y2 - y1) * 0.2
                patch = D[int(max(cy - h, 0)):int(cy + h) + 1, int(max(cx - w, 0)):int(cx + w) + 1]; v = patch[(patch > 0.25) & (patch < 4.0)]
                if len(v) < 5: continue
                z = float(np.median(v)); p = np.array([(cx - self.K[0, 2]) * z / self.K[0, 0], (cy - self.K[1, 2]) * z / self.K[1, 1], z, 1.0])
                out.append({"name": c["object"], "p": (Tw @ p)[:3], "stamp": st, "why": c.get("rejection_reason") or c.get("input_rejection")})
        return out

    # ---- 설계 2: 깊이 지지율 ----
    def views(self, c, n=30):
        cand = []
        for st, Tw in self.Tws.items():
            if Tw is None: continue
            Tcw = np.linalg.inv(Tw); x = Tcw[:3, :3] @ c + Tcw[:3, 3]
            if not (0.3 < x[2] < 2.0): continue
            u = self.K[0, 0] * x[0] / x[2] + self.K[0, 2]; v = self.K[1, 1] * x[1] / x[2] + self.K[1, 2]
            if 30 < u < self.ses.W - 30 and 30 < v < self.ses.H - 30: cand.append((st, Tcw, float(x[2])))
        if len(cand) > n: cand = [cand[i] for i in np.linspace(0, len(cand) - 1, n).astype(int)]
        return cand

    def support(self, name, T, views):
        """→ (지지율 중앙값 or None, 판단한 프레임 수)"""
        P = self.pts(name) @ T[:3, :3].T + T[:3, 3]; rat = []
        for st, Tcw, _ in views:
            D = self.depth(st)
            if D is None: continue
            X = P @ Tcw[:3, :3].T + Tcw[:3, 3]; z = X[:, 2]; u = np.rint(self.K[0, 0] * X[:, 0] / np.maximum(z, 1e-6) + self.K[0, 2]).astype(int); v = np.rint(self.K[1, 1] * X[:, 1] / np.maximum(z, 1e-6) + self.K[1, 2]).astype(int)
            ok = (z > 0.2) & (u >= 0) & (u < D.shape[1]) & (v >= 0) & (v < D.shape[0])
            if ok.sum() < 0.5 * len(P): continue
            d = D[v[ok], u[ok]]; zz = z[ok]; val = d > 0.2; sup = val & (np.abs(d - zz) < 0.02); free = val & (d > zz + 0.05)
            if sup.sum() + free.sum() < 0.15 * ok.sum(): continue          # 대부분 가려졌거나 깊이가 없음
            rat.append(sup.sum() / float(sup.sum() + free.sum()))
        return (float(np.median(rat)) if rat else None), len(rat)

    def refine(self, name, T, views):
        """주변 깊이 점에 CAD 를 다시 맞춘다. → 새 T (제한: 6 cm, 20°)"""
        M = self.pts(name); c = T[:3, 3]; rad = float(np.linalg.norm(M - M.mean(0), axis=1).max()) + 0.03; obs = []
        for st, Tcw, zc in sorted(views, key=lambda x: x[2])[:12]:
            D = self.depth(st)
            if D is None: continue
            x = Tcw[:3, :3] @ c + Tcw[:3, 3]; u0 = self.K[0, 0] * x[0] / x[2] + self.K[0, 2]; v0 = self.K[1, 1] * x[1] / x[2] + self.K[1, 2]; r = self.K[0, 0] * rad / x[2] + 6
            ys = slice(int(max(v0 - r, 0)), int(min(v0 + r, D.shape[0]))); xs = slice(int(max(u0 - r, 0)), int(min(u0 + r, D.shape[1]))); d = D[ys, xs]; vv, uu = np.mgrid[ys, xs]; m = (d > 0.2) & (np.abs(d - x[2]) < rad)
            if m.sum() < 20: continue
            z = d[m]; Xc = np.c_[(uu[m] - self.K[0, 2]) * z / self.K[0, 0], (vv[m] - self.K[1, 2]) * z / self.K[1, 1], z]; Twc = np.linalg.inv(Tcw); obs.append(Xc @ Twc[:3, :3].T + Twc[:3, 3])
        if not obs: return T
        O = np.concatenate(obs); O = O[np.linalg.norm(O - c, axis=1) < rad]
        if len(O) < 80: return T
        if len(O) > 6000: O = O[np.random.default_rng(0).choice(len(O), 6000, replace=False)]
        Tn = T.copy()
        for _ in range(15):
            Pm = M @ Tn[:3, :3].T + Tn[:3, 3]; dist, idx = cKDTree(Pm).query(O); keep = dist < min(0.03, np.percentile(dist, 70))
            if keep.sum() < 40: break
            A = Pm[idx[keep]]; B = O[keep]; ca, cb = A.mean(0), B.mean(0); U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb)); Rd = Vt.T @ np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T
            Td = np.eye(4); Td[:3, :3] = Rd; Td[:3, 3] = cb - Rd @ ca; Tn = Td @ Tn
            if np.linalg.norm(Tn[:3, 3] - T[:3, 3]) > 0.06 or rot_angle(T[:3, :3], Tn[:3, :3]) > 20: return T
        return Tn


def consensus(name, obs):
    """→ (T, 무리에 든 관측 수, 가중치 비율) or None. 관측 쌍을 한꺼번에 계산한다(관측 300개에 수십 ms) — 실행 중 지도 갱신에서도 쓴다."""
    if not obs: return None
    Ts = np.stack([o["T"] for o in obs]); w = np.array([o["w"] for o in obs]); R = Ts[:, :3, :3]; t = Ts[:, :3, 3]
    G = np.stack([np.eye(3)] + [np.asarray(S, float) for S in symmetry_group(name)])                     # 대칭 물체는 같은 자세로 치는 회전들
    tr = np.einsum("ijbc,gcb->ijg", np.einsum("iab,jac->ijbc", R, R), G).max(2); ang = np.degrees(np.arccos(np.clip((tr - 1) / 2, -1, 1)))
    inl = (np.linalg.norm(t[:, None] - t[None], axis=2) <= 0.025) & (ang <= 20); s = inl @ w; i = int(np.argmax(s)); idx = np.flatnonzero(inl[i])
    T = Ts[i].copy(); T[:3, 3] = np.average(t[idx], axis=0, weights=w[idx])
    return T, len(idx), float(s[i] / max(w.sum(), 1e-9))


COMBOS = ["1", "2", "3", "1+2", "1+3", "2+3", "1+2+3", "1n", "1n+2", "1n+3", "1n+2+3"]     # 1n = 설계 1 에서 "무리가 작으면 객체를 뺀다"는 규칙을 없앤 것


def run(scene: Scene, timing=None):
    """→ {"V0": 지금 방식, "<조합>": …}. 조합은 1 → 2 → 3 순서로 적용한다. (V1 = "1", V2 = "1+2", V3 = "1+2+3" 도 함께 넣는다)"""
    tick = time.perf_counter; t_all = tick(); saved = lambda: getattr(scene, "saved_s", 0.0)       # timing: 조합마다 그 조합만 돌렸을 때 드는 시간(초). 다른 조합과 겹쳐 재사용한 계산도 더한다
    obs = scene.observations(); weak = scene.weak()
    V0 = [{"name": m["name"], "T": np.asarray(m["T_w_obj"], float).reshape(4, 4), "n_obs": int(m["n_obs"])} for m in scene.lm0]
    assign = {i: [] for i in range(len(V0))}; loose = []           # 관측을 V0 객체에 배정(같은 이름, 30 cm 안에서 가장 가까운 것)
    for o in obs:
        c = [(np.linalg.norm(o["T"][:3, 3] - m["T"][:3, 3]), i) for i, m in enumerate(V0) if m["name"] == o["name"]]
        if c and min(c)[0] < 0.30: assign[min(c)[1]].append(o)
        else: loose.append(o)
    groups = []                                                    # 지도에 못 올라간 관측의 무리(같은 이름, 30 cm)
    for o in loose:
        g = next((g for g in groups if g[0]["name"] == o["name"] and np.linalg.norm(g[0]["T"][:3, 3] - o["T"][:3, 3]) < 0.30), None)
        (g.append(o) if g else groups.append([o]))

    def simple():                                                  # M0 단순 누적: 같은 이름의 관측을 도착 순서대로 30 cm 안에서 묶기만 한다. 위치는 평균, 방향은 마지막 관측. 걸러내기 · 삭제 없음
        G = []
        for o in sorted(obs, key=lambda o: o["stamp"]):
            c = o["T"][:3, 3]; g = next((g for g in G if g["name"] == o["name"] and np.linalg.norm(g["sum"] / g["n"] - c) < 0.30), None)
            if g is None: g = {"name": o["name"], "sum": np.zeros(3), "n": 0}; G.append(g)
            g["sum"] = g["sum"] + c; g["n"] += 1; g["R"] = o["T"][:3, :3]
        out = []
        for g in G:
            T = np.eye(4); T[:3, :3] = g["R"]; T[:3, 3] = g["sum"] / g["n"]; out.append({"name": g["name"], "T": T, "n_obs": g["n"]})
        return out

    def checked(lms, refine):
        res = []
        for m in lms:
            vw = scene.views(m["T"][:3, 3]); s0, k0 = scene.support(m["name"], m["T"], vw); T = m["T"]; s, k = s0, k0; moved = 0.0
            if refine and vw:
                Tn = scene.refine(m["name"], T, vw); s1, k1 = scene.support(m["name"], Tn, vw)
                if s1 is not None and (s0 is None or s1 > s0 + 0.02): moved = float(np.linalg.norm(Tn[:3, 3] - T[:3, 3]) * 1000); T, s, k = Tn, s1, k1
            res.append(dict(m, T=T, support=s, support_frames=k, refined_mm=round(moved, 1)))
        return res

    def design1(drop_rule):
        out = []
        for i, m in enumerate(V0):
            r = consensus(m["name"], assign[i])
            if r is None: out.append(dict(m, note="관측 기록 없음(그대로)")); continue
            T, n, share = r
            if drop_rule and (n < 3 or share < 0.30): continue
            out.append({"name": m["name"], "T": T, "n_obs": m["n_obs"], "consensus": n, "share": round(share, 3)})
        return out

    def design2(lms): return [m for m in checked(lms, True) if not (m["support_frames"] >= 3 and m["support"] is not None and m["support"] < DROP)]

    ev = {}
    for x in weak: ev.setdefault(x["name"], []).append(x["p"])
    for o in obs: ev.setdefault(o["name"], []).append(o["T"][:3, 3])
    ev = {k: np.asarray(v, float) for k, v in ev.items()}

    def evidence(name, c, r): return int((np.linalg.norm(ev[name] - c, axis=1) < r).sum()) if name in ev else 0

    def design3(lms, refine):
        L = [dict(m) for m in lms]; added = 0
        for g in groups:
            name = g[0]["name"]; r = consensus(name, g); T = r[0]; c = T[:3, 3]
            if any(m["name"] == name and np.linalg.norm(m["T"][:3, 3] - c) < 0.30 for m in L): continue
            ev = evidence(name, c, 0.15)
            if ev < 6: continue
            m = checked([{"name": name, "T": T, "n_obs": len(g), "consensus": r[1], "share": round(r[2], 3)}], refine)[0]
            if m["support"] is not None and m["support_frames"] >= 3 and m["support"] >= 0.5: m["note"] = f"약한 증거로 등록(증거 {ev})"; L.append(m); added += 1
        drop = set()
        for i in range(len(L)):
            for j in range(i):
                if i in drop or j in drop or L[i]["name"] == L[j]["name"] or np.linalg.norm(L[i]["T"][:3, 3] - L[j]["T"][:3, 3]) > 0.10: continue
                ei = evidence(L[i]["name"], L[i]["T"][:3, 3], 0.12); ej = evidence(L[j]["name"], L[j]["T"][:3, 3], 0.12); drop.add(i if ei < ej else j)
        return [m for k, m in enumerate(L) if k not in drop], added, len(drop)

    def dump(lms):
        lms = [m if "support_frames" in m else checked([m], False)[0] for m in lms]
        return [{"name": m["name"], "T_w_obj": np.asarray(m["T"]).round(5).tolist(), "n_obs": m.get("n_obs"), "support": None if m.get("support") is None else round(m["support"], 3), "support_frames": m.get("support_frames"),
                 "consensus": m.get("consensus"), "share": m.get("share"), "refined_mm": m.get("refined_mm"), "note": m.get("note")} for m in lms]
    t_pre = tick() - t_all; t = tick(); s0 = saved(); out = {"V0": dump(V0)}; extra = {}; t_v0 = tick() - t + saved() - s0
    for combo in COMBOS:
        parts = combo.split("+"); L = V0; add = drp = 0; t = tick(); s0 = saved()
        if "1" in parts: L = design1(True)
        if "1n" in parts: L = design1(False)
        if "2" in parts: L = design2(L)
        if "3" in parts: L, add, drp = design3(L, "2" in parts)
        out[combo] = dump(L); extra[combo] = {"added_by_design3": add, "dropped_name_conflict": drp}
        if timing is not None: timing[combo] = t_pre + tick() - t + saved() - s0
    out["V1"], out["V2"], out["V3"] = out["1"], out["1+2"], out["1+2+3"]
    t = tick(); s0 = saved(); out["M0"] = dump(simple())
    if timing is not None: timing["M0"] = t_pre + tick() - t + saved() - s0
    out["counts"] = {"observations": len(obs), "unassigned_observations": len(loose), "weak_evidence": len(weak), "added_by_design3": extra["1+2+3"]["added_by_design3"], "dropped_name_conflict": extra["1+2+3"]["dropped_name_conflict"]}; out["combo_counts"] = extra
    if timing is not None: timing["V0"] = t_pre + t_v0; timing["total"] = tick() - t_all
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("run", type=Path); ap.add_argument("--session", required=True); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args()
    sc = Scene(a.run.resolve(), a.session); R = run(sc); R["session"] = a.session; R["run"] = str(a.run); R["frames"] = len(sc.frames); R["sam_tau_s"] = sc.tau / 1e9
    a.out.parent.mkdir(parents=True, exist_ok=True); a.out.write_text(json.dumps(R, ensure_ascii=False))
    for v in ["V0"] + COMBOS:
        L = R[v]; names = [m["name"] for m in L]; sup = [m["support"] for m in L if m["support"] is not None]
        print(f"{v}: 등록 {len(L)} 종류 {len(set(names))} 초과 {len(names) - len(set(names))} | 지지율 평균 {np.mean(sup) if sup else float('nan'):.2f} (0.5 이상 {sum(s >= 0.5 for s in sup)}/{len(sup)})", flush=True)
    print(R["counts"], flush=True)
