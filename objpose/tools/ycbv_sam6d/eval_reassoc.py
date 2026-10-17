#!/usr/bin/env python3
"""eval_reassoc.py — 다시 보인 물체를 지도의 같은 물체에 연결하는지 사건 단위로 잰다.

build_memory_maps.py 와 같은 입력(실시간 SAM-6D 결과 + ORB-SLAM3 현재 위치)과 같은 객체 메모리 설정으로
메모리를 새로 돌리면서, 매 검출이 어느 지도 물체 번호(object_id)에 붙었는지 기록한다.

정답 짝: 검출을 GT 카메라 위치로 월드에 옮겨 같은 이름 GT 물체 중심과 MATCH_M 이내면 "그 물체의 올바른 검출".
(메모리가 쓰는 SLAM 위치와 무관하게 정한다 → SLAM 없는 기준선에도 같은 라벨.)

재등장 사건(주 지표, vis*): GT 상 물체가 보이지 않던(보이는 비율 < 0.1 또는 중심이 화면 밖) 구간이 GAP_S 초 이상
  이어진 뒤 다시 보인 사건. 사라지기 전 마지막 올바른 검출의 지도 번호와, 다시 보인 뒤 첫 올바른 검출의 번호를 비교한다.
검출 틈 사건(보조, gap*): 같은 GT 물체의 올바른 검출 사이가 GAP_S 초 이상 벌어진 뒤 다시 나온 검출.
  연결 성공 = 직전 검출과 같은 지도 물체 번호에 붙음
  새로 등록 = 새 번호를 만듦 (직전 번호가 지도에 남아 있으면 중복, 지워졌으면 잊고 다시 만듦)
  다른 번호 = 이미 있던 다른 번호에 붙음
틈의 원인: 틈 동안 GT 물체 중심이 화면 밖으로 나간 프레임이 있으면 "시야 밖", 아니면 "시야 안(가림·놓침)".

    BENCH_* 환경변수는 build_memory_maps.py 와 같다.
    python eval_reassoc.py --run-dir <run> --name <label> --out <json> [--no-slam]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_memory_maps as bm                            # noqa: E402
from fusion import Fusion, PoseBuffer                     # noqa: E402
from memory import ObjectMemory, DepthCheckConfig         # noqa: E402

MATCH_M = 0.10


class RecMemory(ObjectMemory):
    """ObjectMemory that keeps every step's association decisions (per memory build)."""

    def __init__(self, *a, **k):
        self.log = {}
        super().__init__(*a, **k)

    def _step(self, mem, f, idx):
        fr = super()._step(mem, f, idx)
        status = {lm.object_id: getattr(lm.status, "value", str(lm.status)) for lm in mem.store.all_landmarks()}
        self.log.setdefault(id(mem), []).append((f.t_ns, f.dets, fr, status))
        return fr


def run(scene, run_dir, by_id, no_slam, extents=None):
    sc = bm.YCBV / scene
    ids, Twc_gt, gobj, K = bm.gt_scene(sc, by_id)
    n = len(ids)
    slam = bm.load_slam(scene, n)
    poses = PoseBuffer(max_gap_s=0.25)
    for i in sorted(slam):
        poses.add(bm.T0_NS + i * bm.STEP_NS, "OK", np.eye(4) if no_slam else slam[i], map_id=0)
    mem = RecMemory(Fusion(np.eye(4), poses), K, (640, 480),
                    depth_check=DepthCheckConfig(free_margin_m=0.15, confirm_frames=3, max_speed_mps=0.6, max_rot_dps=45.0),
                    assoc_trans_gate_m=0.30, assoc_rot_gate_deg=None,
                    **({"occlusion_extents": extents} if extents else {}))
    frames = [json.loads(l) for l in (run_dir / scene / "sam6d_frames.jsonl").read_text().splitlines() if l.strip()]
    frames.sort(key=lambda f: f.get("done_s", f["idx"]))
    for f in frames:
        raw = cv2.imread(str(sc / "depth" / f"{f['idx'] + 1:06d}.png"), cv2.IMREAD_UNCHANGED)
        depth = np.rint(raw.astype(np.float32) * 0.1).astype(np.uint16)[::8, ::8]
        mem.add_frame(int(f["t_ns"]), [(d["object"], d["score"], d["R"], d["t_mm"]) for d in f["dets"]], depth=depth)
        mem.update()
    log = mem.log[id(mem._mem)]
    final = {m["object_id"]: m for m in mem.landmarks()}

    # GT 물체 중심의 화면 안 여부 (프레임별)
    cen = {nm: G[:3, 3] for nm, G in gobj.items()}
    inview = {}
    for nm, c in cen.items():
        Tcw = np.linalg.inv(Twc_gt)                                  # (n,4,4)
        pc = Tcw[:, :3, :3] @ c + Tcw[:, :3, 3]
        z = pc[:, 2]
        u = K[0, 0] * pc[:, 0] / np.maximum(z, 1e-6) + K[0, 2]
        v = K[1, 1] * pc[:, 1] / np.maximum(z, 1e-6) + K[1, 2]
        inview[nm] = (z > 0.05) & (u >= 0) & (u < 640) & (v >= 0) & (v < 480)

    obs = {nm: [] for nm in gobj}          # nm -> [(k, object_id, decision, status_snapshot)]
    wrong = 0
    for t_ns, dets, fr, status in log:
        k = int(round((t_ns - bm.T0_NS) / bm.STEP_NS))
        for dec in fr.decisions:
            if dec.object_id is None:
                continue
            j = dec.detection_id % 100
            name, _, T = dets[j]
            Tw = Twc_gt[k] @ T
            g = cen.get(name)
            if g is None or np.linalg.norm(Tw[:3, 3] - g) > MATCH_M:
                wrong += 1
                continue
            obs[name].append((k, dec.object_id, getattr(dec.decision_type, "value", str(dec.decision_type)), status))
    # GT 가시성 (프레임별 보이는 비율): scene_gt 순서 = scene_gt_info 순서
    gt = json.loads((sc / "scene_gt.json").read_text())
    info = json.loads((sc / "scene_gt_info.json").read_text())
    vis = {nm: np.zeros(n) for nm in gobj}
    for i, f in enumerate(ids):
        for r_, g_ in zip(gt[f], info[f]):
            vis[by_id[r_["obj_id"]]][i] = g_["visib_fract"]
    return dict(obs=obs, inview=inview, vis=vis, final=final, wrong=wrong, n=n, gobj=list(gobj))


def vis_events(r, absent_s, vis_min=0.1):
    """GT 기준 재등장: 보이는 비율 < vis_min(또는 중심이 화면 밖)인 구간이 absent_s 초 이상 이어진 뒤 다시 보임."""
    out = []
    for nm in r["gobj"]:
        present = (r["vis"][nm] >= vis_min) & r["inview"][nm]
        seq = sorted(r["obs"][nm], key=lambda x: x[0])
        k = 0
        n = len(present)
        while k < n:
            if present[k]:
                k += 1
                continue
            a = k
            while k < n and not present[k]:
                k += 1
            b = k                                                   # absent [a, b)
            if a == 0 or b >= n or (b - a) / bm.FPS < absent_s:
                continue
            cause = "out" if not r["inview"][nm][a:b].all() else "occl"
            before = [x for x in seq if x[0] < a]
            after = [x for x in seq if x[0] >= b]
            ev = {"obj": nm, "a": a, "b": b, "absent_s": round((b - a) / bm.FPS, 2), "cause": cause}
            if not before:
                ev["result"] = "not_mapped_before"
            elif not after:
                ev["result"] = "not_redetected"
            else:
                id1 = before[-1][1]; k2, id2, dec2, st2 = after[0]
                ev["delay_s"] = round((k2 - b) / bm.FPS, 2)
                st_at_return = st2.get(id1)
                ev["kept"] = st_at_return not in (None, "deleted", "merged")
                if id2 == id1:
                    ev["result"] = "same"
                elif dec2 == "new_tentative":
                    ev["result"] = "new_dup" if ev["kept"] else "new_forgot"
                else:
                    ev["result"] = "other"
            out.append(ev)
    return out


def summarize_vis(evs):
    s = {}
    for c in ("all", "out", "occl"):
        e = [x for x in evs if c == "all" or x["cause"] == c]
        d = {"n": len(e)}
        for k in ("not_mapped_before", "not_redetected", "same", "new_dup", "new_forgot", "other"):
            d[k] = sum(x["result"] == k for x in e)
        linked = [x for x in e if "kept" in x]
        d["relinked"] = len(linked)
        d["success"] = round(d["same"] / len(linked), 4) if linked else None
        d["kept_during_absence"] = sum(x["kept"] for x in linked)
        d["delay_median_s"] = float(np.median([x["delay_s"] for x in linked])) if linked else None
        s[c] = d
    return s


def events(r, gap_s):
    out = []
    frag = {}
    for nm, seq in r["obs"].items():
        seq.sort(key=lambda x: x[0])
        frag[nm] = len({s[1] for s in seq})
        for (k1, id1, _, _), (k2, id2, dec2, st2) in zip(seq, seq[1:]):
            if (k2 - k1) / bm.FPS < gap_s:
                continue
            cause = "out" if not r["inview"][nm][k1 + 1:k2].all() else "in"
            if id2 == id1:
                res = "same"
            elif dec2 == "new_tentative":
                res = "new_dup" if st2.get(id1) not in (None, "deleted", "merged") else "new_forgot"
            else:
                res = "other"
            out.append({"obj": nm, "k1": k1, "k2": k2, "gap_s": round((k2 - k1) / bm.FPS, 2), "cause": cause, "result": res})
    return out, frag


def summarize(evs):
    s = {"events": len(evs)}
    for c in ("all", "out", "in"):
        e = [x for x in evs if c == "all" or x["cause"] == c]
        s[c] = {"n": len(e), **{k: sum(x["result"] == k for x in e) for k in ("same", "new_dup", "new_forgot", "other")}}
        s[c]["success"] = round(s[c]["same"] / len(e), 4) if e else None
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", required=True, type=Path)
    ap.add_argument("--name", required=True)
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--no-slam", action="store_true", help="baseline: memory gets identity camera poses (camera-frame positions)")
    ap.add_argument("--occl", action="store_true", help="map-based occlusion: no miss penalty behind mapped objects")
    ap.add_argument("--gaps", nargs="*", type=float, default=[1.0, 3.0])
    ap.add_argument("--scenes", nargs="*", default=None)
    a = ap.parse_args()
    table = json.loads(bm.TABLE.read_text())["objects"]
    by_id = {o["obj_id"]: o["name"] for o in table}
    extents = None
    if a.occl:
        from core.map_occlusion import box_corners
        extents = {o["name"]: box_corners(np.load(bm.PTS / f"{o['name']}.npy") / 1000.0) for o in table}
    scenes = a.scenes or sorted(p.name for p in bm.YCBV.iterdir())
    res = {"name": a.name, "run_dir": str(a.run_dir), "no_slam": a.no_slam, "occl": a.occl, "match_m": MATCH_M, "fps": bm.FPS, "scenes": {}}
    allev = {g: [] for g in a.gaps}
    frag_all, wrong, gt_objs, seen_objs = [], 0, 0, 0
    vev = {g: [] for g in a.gaps}
    for s in scenes:
        r = run(s, a.run_dir, by_id, a.no_slam, extents)
        wrong += r["wrong"]
        gt_objs += len(r["gobj"])
        sc = {"wrong_dets": r["wrong"]}
        for g in a.gaps:
            ev, frag = events(r, g)
            allev[g] += ev
            sc[f"gap{g:g}"] = summarize(ev)
            if g == a.gaps[0]:
                sc["events"] = ev
                sc["ids_per_object"] = frag
                frag_all += [v for v in frag.values() if v > 0]
                seen_objs += sum(1 for v in frag.values() if v > 0)
        for g in a.gaps:
            ve = vis_events(r, g)
            vev[g] += ve
            sc[f"vis{g:g}"] = {"summary": summarize_vis(ve), "events": ve}
        res["scenes"][s] = sc
        print(s, json.dumps({f"gap{g:g}": sc[f"gap{g:g}"]["all"] for g in a.gaps}), flush=True)
    res["total"] = {f"gap{g:g}": summarize(allev[g]) for g in a.gaps}
    for g in a.gaps:
        res["total"][f"vis{g:g}"] = summarize_vis(vev[g])
    res["total"]["objects_gt"] = gt_objs
    res["total"]["objects_detected"] = seen_objs
    res["total"]["ids_per_object_mean"] = round(float(np.mean(frag_all)), 3) if frag_all else None
    res["total"]["objects_single_id"] = sum(1 for v in frag_all if v == 1)
    res["total"]["wrong_dets"] = wrong
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(res, indent=1, ensure_ascii=False))
    print("TOTAL", json.dumps(res["total"], ensure_ascii=False))


if __name__ == "__main__":
    main()
