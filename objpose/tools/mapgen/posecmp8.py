#!/usr/bin/env python3
"""posecmp8.py — 프레임별 6D 포즈 비교: [후보 생성 2] × [후보 판정 2] × [포즈 검증 2] = 8 구성을 같은 프레임 · 같은 후보 · 같은 마스크로 돌린다.

  후보 생성  F  FastSAM-x 화면 전체 분할(원본 SAM-6D 의 FastSAM 설정: iou 0.9, conf 0.25, max_det 200, 640) — 마스크도 FastSAM-x 의 것
             Y  YOLO-World 문구 상자(문구마다 점수 0.02 이상 상위 3개, 실시간 인식기와 같은 다중 라벨 NMS) + 상자 안 MobileSAM 마스크
  후보 판정  T  총점: 원본 SAM-6D ISM 점수 (의미 + 외형 + 기하)/(2 + 가시도), 물체마다 NMS 0.25 후 1위, 기준 0.5 (live_six 구성 ① · ⑤ 와 같다)
             S  단계별 기준: 이 저장소의 인식기(yolo_ism_object_n, YCB 설정) — 의미 ≥ 0.35 → 마스크 외형 ≥ 0.605 → 물체 간 NMS. F 는 상자 하나를 모든 물체에 후보로 준다(top_k 200, 구성 ② 와 같은 방식)
  포즈 검증  0  끔: PEM 의 기하 점수 1위 가설(원본 SAM-6D 의 선택)
             1  켬: 마스크 IoU → 텍스처 → 가설 수렴(실시간 인식기와 같은 기준). 통과 못 하면 출력하지 않는다(탈락 사유는 기록)
같은 후보 생성 방식의 두 판정은 프레임마다 한 번 만든 후보 · 마스크를 함께 쓴다(판정 S 안의 분할 호출을 저장해 둔 마스크로 바꿔 끼운다).
PEM 은 (물체, 마스크) 하나씩 따로 돌리고 난수 시드를 (장면, 프레임, 물체, 마스크)로 정한다 → 같은 물체 · 같은 마스크면 어느 구성에서든 같은 입력이다. 검증 끔 · 켬도 같은 시드.
SLAM, 지도 누적, 관측 합의는 쓰지 않는다. 찾을 물체 = 물체표의 21종 전부.
    <sam6d python> objpose/tools/mapgen/posecmp8.py --table objpose/rt/bop/ycbv_objects.json --frames <장면별 im_id.json> --out <dir> [--scenes …] [--limit N]
출력 <out>/<장면>.jsonl (프레임마다 후보 수, 판정별 물체 결정, 8 구성의 출력과 탈락 사유, 단계별 ms), <out>/<장면>.done, <out>/run_config.json
"""
import argparse, hashlib, json, os, random, sys, time, zlib
from pathlib import Path
REPO = Path(__file__).resolve().parents[3]
ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--table", type=Path, required=True); ap.add_argument("--frames", type=Path, required=True)
ap.add_argument("--ism-config", default="configs/yolo_ism_ycbv.yaml"); ap.add_argument("--scenes", nargs="*", default=None); ap.add_argument("--limit", type=int, default=0)
ap.add_argument("--variants", action="store_true", help="YOLO 후보만: YT1 · YS1 + 변형 YA1(상자를 모든 물체의 후보로) · YR1(기준 완화) · YB1(둘 다), 검증 켬만")
ap.add_argument("--stride", type=int, default=1)
a = ap.parse_args(); a.out = a.out.resolve(); a.out.mkdir(parents=True, exist_ok=True)
TAB = json.loads(a.table.read_text()); ROOT = Path(TAB["scenes_dir"]); NAMES = [o["name"] for o in TAB["objects"]]; FR = json.loads(a.frames.read_text())
os.environ["LIVE_SIX"] = "2"                       # 원본 ISM(FastSAM-x, ViT-S, 기준 0.5) + 이 저장소의 Sam6DCore(PEM 점 1024, 검증) 를 함께 띄운다
sys.path.insert(0, str(REPO / "objpose/tools/lab"))
import cv2, numpy as np, torch     # noqa: E402
import live_six as LS              # noqa: E402
import yolo_ism as yi              # noqa: E402
import yolo_ism_object_n as o_n    # noqa: E402
STAGED = o_n.recognize_frame_auto                  # hybrid 가 이 이름을 원본 판정으로 바꾸기 전에 보관
VERIFY = {"enabled": True, "mask_iou_min": 0.420998, "texture_min_score": 0.449562, "cluster_rotation_deg": 20, "cluster_translation_mm": 25, "cluster_min_occupancy": 0.5}     # rt_bop.py · still_bop.py 와 같다
hc = LS.LiveCore({"ism": {"config": a.ism_config, "objects": NAMES}, "runtime": {"device": "cuda:0", "det_score_thresh": 0.2, "verify": dict(VERIFY)}})
core, ism, RO = hc.core, hc.ism, LS.LC.RO; DEV = core.device; THR = hc.thresh
assert sorted(o["name"] for o in core.objs) == sorted(NAMES) and RO.OIDS == [o["name"] for o in core.objs]
VER_CORE = core.verify; VER_PEM = core.pem.cfg.appe_rerank["verify"]
FS = ism.segmentor_model; FS_ARGS = dict(FS.args)
YOLO_CLS = type(core.yolo)
TOPK0 = {o["name"]: o.get("top_k", 3) for o in core.objs}


def seed_all(s):
    random.seed(s); np.random.seed(s); torch.manual_seed(s); torch.cuda.manual_seed_all(s)


def sync():
    torch.cuda.synchronize()


def clamp_box(xy, w, h):
    x1 = max(0, min(int(xy[0]), w - 1)); y1 = max(0, min(int(xy[1]), h - 1))
    return [x1, y1, max(x1 + 1, min(int(xy[2]), w)), max(y1 + 1, min(int(xy[3]), h))]


# ---------------------------------------------------------------- 후보 생성
def gen_fastsam(rgb):
    """FastSAMUpstream.generate_masks 와 같은 호출 + 원본의 작은 후보 제거. 점수(conf)도 남긴다(판정 S 의 상자 순서)."""
    det = FS.model.predict(rgb, **FS_ARGS)[0]
    if det.masks is None: return []
    m = torch.nn.functional.interpolate(det.masks.data.to(DEV).unsqueeze(1).float(), size=rgb.shape[:2], mode="bilinear", align_corners=False)[:, 0]
    d = RO.Detections({"masks": m, "boxes": det.boxes.data[:, :4].to(DEV)}); d.add_attribute("conf", det.boxes.data[:, 4].to(DEV))
    d.remove_very_small_detections(config=ism.post_processing_config.mask_post_processing)
    h, w = rgb.shape[:2]; out = []
    for i in range(len(d)):
        mk = (d.masks[i] > 0.5).cpu().numpy()
        if mk.any(): out.append({"box": clamp_box(d.boxes[i].tolist(), w, h), "conf": float(d.conf[i]), "mask": mk})
    return out


def gen_yolo(bgr):
    """Sam6DCore.process 의 YOLO-World 호출 그대로(다중 라벨 NMS, conf 0.02, 640) → 문구마다 상위 3개 → 상자 안 MobileSAM(segment_box, 판정 S 가 부르는 것과 같은 함수)."""
    h, w = bgr.shape[:2]
    with o_n.multi_label_nms(core.multi_label):
        res = YOLO_CLS.predict(core.yolo, bgr, conf=core.min_score, imgsz=core.imgsz, verbose=False, device=DEV)
    pb = {i: [] for i in range(len(core.unique_prompts))}
    if len(res) and res[0].boxes is not None:
        b = res[0].boxes
        for j in range(len(b)):
            ci = int(b.cls[j])
            if ci in pb: pb[ci].append((clamp_box(b.xyxy[j].tolist(), w, h), float(b.conf[j])))
    out, seen = [], {}
    for ci in pb:
        pb[ci] = sorted(pb[ci], key=lambda t: t[1], reverse=True)[:3]
        for box, cf in pb[ci]:
            k = tuple(box)
            if k in seen: seen[k]["conf"] = max(seen[k]["conf"], cf); continue
            mk = yi.segment_box(core.segmentor, bgr, box, DEV)
            if mk is None or not mk.any(): continue
            seen[k] = {"box": box, "conf": cf, "mask": mk.astype(bool)}; out.append(seen[k])
    return out, pb


# ---------------------------------------------------------------- 판정 T (총점)
def judge_total(cands, rgb):
    """live_cores.orig_recognize 와 같은 계산을, 이미 만든 후보 마스크에 적용한다."""
    dec = {n: {"decision": "no-candidate"} for n in NAMES}
    if not cands: return {}, dec
    props = {"masks": torch.from_numpy(np.stack([c["mask"] for c in cands])).float().to(DEV), "boxes": torch.tensor([c["box"] for c in cands], dtype=torch.float32, device=DEV)}
    dets = RO.Detections(props); dets.add_attribute("cid", torch.arange(len(cands), device=DEV))
    dets.remove_very_small_detections(config=ism.post_processing_config.mask_post_processing)
    if len(dets) == 0: return {}, dec
    qd, qa = ism.descriptor_model(rgb, dets); sel, pio, sem, best = ism.compute_semantic_score(qd)
    dets.filter(sel); qa = qa[sel, :]
    if len(dets) == 0:
        return {}, {n: {"decision": "below-semantic-conf(0.2)"} for n in NAMES}
    appe, ref_aux = ism.compute_appearance_score(best, pio, qa)
    batch = {"depth": torch.from_numpy(hc.cur["raw"].astype(np.int32)).unsqueeze(0).to(DEV), "cam_intrinsic": torch.from_numpy(hc.cur["K"]).unsqueeze(0).to(DEV), "depth_scale": torch.from_numpy(np.array(1.0)).unsqueeze(0).to(DEV)}
    uv = ism.project_template_to_image(best, pio, batch, dets.masks); geo, vis = ism.compute_geometric_score(uv, dets, qa, ref_aux, visible_thred=ism.visible_thred)
    final = (sem + appe + geo * vis) / (1 + 1 + vis)
    dets.add_attribute("scores", final); dets.add_attribute("object_ids", pio); dets.apply_nms_per_object_id(nms_thresh=ism.post_processing_config.nms_thresh)
    hits = {}
    for n in NAMES: dec[n] = {"decision": "not-best-for-any-candidate"}
    for kk in torch.unique(dets.object_ids).tolist():
        idx = torch.nonzero(dets.object_ids == kk)[:, 0]; j = int(idx[torch.argmax(dets.scores[idx])]); sc = float(dets.scores[j]); nm = RO.OIDS[int(kk)]
        ci = int(dets.cid[j]); dec[nm] = {"decision": "accepted" if sc > THR else f"total<={THR}", "score": round(sc, 4), "box": cands[ci]["box"]}
        if sc > THR: hits[nm] = {"mask": cands[ci]["mask"], "box": cands[ci]["box"], "score": sc}
    return hits, dec


# ---------------------------------------------------------------- 판정 S (단계별 기준)
_MASKS, _CROP, _FWD = {}, {}, {}
_seg0, _crop0, _fwd0 = yi.segment_box, yi.crop_resize_pad, o_n.dinov2_blocks_forward


def _seg(seg, bgr, box, device):                      # 판정 S 안의 분할 → 이 프레임에 만들어 둔 후보 마스크
    return _MASKS[tuple(int(v) for v in box)]


def _crop(img, box, *aa, **kk):                       # 같은 상자의 같은 자르기는 프레임 안에서 한 번만(결과 동일, 속도만)
    k = (id(img), tuple(int(v) for v in box), aa, tuple(sorted(kk.items())))
    if k not in _CROP: _CROP[k] = _crop0(img, box, *aa, **kk)
    return _CROP[k]


def _fwd(model, crops, device, blocks):
    if len(crops) != 1 or crops[0] is None: return _fwd0(model, crops, device, blocks)
    k = (id(crops[0]), tuple(sorted(blocks)))
    if k not in _FWD: _FWD[k] = _fwd0(model, crops, device, blocks)
    return _FWD[k]


RELAX = {"similarity_threshold": 0.30, "appe_v2_gate": 0.55}       # --variants 의 기준 완화(원래 0.35 · 0.605)


def judge_staged(cands, pb, bgr, rgb, every_prompt, relax=False):
    _MASKS.clear(); _CROP.clear(); _FWD.clear()
    keep = {o["name"]: {k: o[k] for k in RELAX} for o in core.objs}
    if relax:
        for o in core.objs: o.update(RELAX)
    for c in sorted(cands, key=lambda c: -c["conf"]): _MASKS.setdefault(tuple(c["box"]), c["mask"])     # 정수 상자가 같은 후보가 둘이면 점수가 높은 쪽(판정 S 는 상자로만 후보를 구분한다)
    if every_prompt:                                   # 구성 ② 방식: FastSAM 상자 하나하나를 모든 문구의 후보로, 물체마다 200개까지
        lst = sorted(((c["box"], c["conf"]) for c in cands), key=lambda t: t[1], reverse=True); pb = {i: list(lst) for i in range(len(core.unique_prompts))}
        for o in core.objs: o["top_k"] = 200
    yi.segment_box, yi.crop_resize_pad, o_n.dinov2_blocks_forward = _seg, _crop, _fwd
    try:
        res = STAGED(core.groups, pb, bgr, rgb, yi.normalize_rgb(rgb), core.model, DEV, core.segmentor, core.pool, core.tsim)
    finally:
        yi.segment_box, yi.crop_resize_pad, o_n.dinov2_blocks_forward = _seg0, _crop0, _fwd0
        for o in core.objs: o["top_k"] = TOPK0[o["name"]]; o.update(keep[o["name"]])
    hits, dec = {}, {}
    for n in NAMES:
        r = res.get(n, {}); d = {"decision": r.get("decision"), "best_sem": round(float(r.get("best_sem") or 0), 4), "masked_appe": round(float(r.get("masked_appe") or 0), 4)}
        if r.get("box") is not None: d["box"] = [int(v) for v in r["box"]]
        dec[n] = d
        if r.get("accepted") and r.get("mask") is not None: hits[n] = {"mask": r["mask"].astype(bool), "box": [int(v) for v in r["box"]], "score": float(r.get("masked_appe") or 0)}
    return hits, dec


# ---------------------------------------------------------------- PEM (물체 하나씩, 검증 끔 / 켬)
_PEM = {}


def pem(name, hit, bgr, depth, K, verify_on, key):
    mh = hashlib.md5(np.packbits(hit["mask"]).tobytes()).hexdigest()[:16]; ck = (name, mh, verify_on)
    if ck in _PEM: return _PEM[ck]
    VER_CORE["enabled"] = VER_PEM["enabled"] = verify_on; VER_CORE["as_score"] = VER_PEM["as_score"] = True
    o_n.recognize_frame_auto = lambda *aa, **kk: {name: {"accepted": True, "mask": hit["mask"], "box": hit["box"], "decision": "detected"}}
    seed_all(zlib.crc32(f"{key}/{name}/{mh}".encode()))
    sync(); t0 = time.perf_counter(); rows, ms, _, _ = core.process(bgr, depth, K, want_mask=False); sync(); dt = (time.perf_counter() - t0) * 1e3
    diag = core.last_frame_diag; inp = next((c.get("input_rejection") for c in diag.get("pem_candidates", []) if c["object"] == name), None)
    if not rows:
        out = {"object": name, "output": False, "reason": f"pem-input:{inp}" if inp else "pem-no-output", "ms": round(dt, 1)}
    else:
        r = rows[0]; v = r.get("verify") or {}; acc = bool(v.get("accepted", False)) if verify_on else True
        out = {"object": name, "output": acc, "R": r["R"], "t_mm": r["t_mm"], "score": r["score"], "ms": round(dt, 1), "mask_hash": mh,
               **({"verify_accepted": acc, "reason": None if acc else v.get("rejection_reason", "verification-failed"), "mask_iou": r.get("mask_iou"), "texture": r.get("texture_score"), "occupancy": r.get("cluster_occupancy")} if verify_on else {})}
    _PEM[ck] = out; return out


# ---------------------------------------------------------------- 실행
(a.out / "run_config.json").write_text(json.dumps({"argv": sys.argv, "table": str(a.table), "frames": str(a.frames), "ism_config": a.ism_config, "objects": NAMES, "verify": VERIFY,
    "total_threshold": THR, "orig_desc": hc.desc, "fastsam_args": {k: str(v) for k, v in FS_ARGS.items()}, "yolo": {"conf": core.min_score, "imgsz": core.imgsz, "multi_label": core.multi_label, "top_per_prompt": 3},
    "pem_fine_npoint": core.pcfg.model.fine_npoint, "time": time.strftime("%F %T")}, indent=1, ensure_ascii=False))
for s in (a.scenes or sorted(FR)):
    if (a.out / f"{s}.done").exists(): continue
    d = ROOT / s; cam = json.loads((d / "scene_camera.json").read_text()); ids = [int(i) for i in FR[s]][:: a.stride][: (a.limit or None)]; fo = open(a.out / f"{s}.jsonl", "w"); t_s = time.perf_counter()
    for im in ids:
        c = cam[str(im)]; bgr = cv2.imread(str(d / "rgb" / f"{im:06d}.png")); raw = cv2.imread(str(d / "depth" / f"{im:06d}.png"), cv2.IMREAD_UNCHANGED)
        depth = np.rint(raw.astype(np.float32) * float(c["depth_scale"])).astype(np.uint16); K = np.asarray(c["cam_K"], np.float64).reshape(3, 3); rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        hc.cur.update(raw=np.asarray(depth).astype(np.int32), K=K); key = f"{s}/{im}"; _PEM.clear(); ms = {}; rec = {"scene": s, "im_id": im, "cand": {}, "judge": {}, "out": {}}
        seed_all(zlib.crc32(key.encode()))
        if a.variants:
            with torch.inference_mode():
                sync(); t0 = time.perf_counter(); cY, pbY = gen_yolo(bgr); sync(); ms["gen_Y"] = (time.perf_counter() - t0) * 1e3
            rec["cand"] = {"Y": len(cY)}
            for j, ev, rx in (("T", None, None), ("S", False, False), ("A", True, False), ("R", False, True), ("B", True, True)):
                seed_all(zlib.crc32(f"{key}/Y{j}".encode())); sync(); t0 = time.perf_counter()
                with torch.inference_mode():
                    hits, dec = judge_total(cY, rgb) if j == "T" else judge_staged(cY, pbY, bgr, rgb, every_prompt=ev, relax=rx)
                sync(); ms[f"judge_Y{j}"] = (time.perf_counter() - t0) * 1e3; rec["judge"]["Y" + j] = dec
                rec["out"][f"Y{j}1"] = [pem(n, hits[n], bgr, depth, K, True, key) for n in sorted(hits)]
            rec["ms"] = {k: round(v, 1) for k, v in ms.items()}; fo.write(json.dumps(rec) + "\n"); fo.flush(); continue
        with torch.inference_mode():
            sync(); t0 = time.perf_counter(); cF = gen_fastsam(rgb); sync(); ms["gen_F"] = (time.perf_counter() - t0) * 1e3
            t0 = time.perf_counter(); cY, pbY = gen_yolo(bgr); sync(); ms["gen_Y"] = (time.perf_counter() - t0) * 1e3
        rec["cand"] = {"F": len(cF), "Y": len(cY)}
        for g, cands in (("F", cF), ("Y", cY)):
            for j in ("T", "S"):
                seed_all(zlib.crc32(f"{key}/{g}{j}".encode())); sync(); t0 = time.perf_counter()
                with torch.inference_mode():
                    hits, dec = judge_total(cands, rgb) if j == "T" else judge_staged(cands, pbY if g == "Y" else None, bgr, rgb, every_prompt=(g == "F"))
                sync(); ms[f"judge_{g}{j}"] = (time.perf_counter() - t0) * 1e3; rec["judge"][g + j] = dec
                for v in (0, 1):
                    rec["out"][f"{g}{j}{v}"] = [pem(n, hits[n], bgr, depth, K, bool(v), key) for n in sorted(hits)]
        rec["ms"] = {k: round(v, 1) for k, v in ms.items()}; fo.write(json.dumps(rec) + "\n"); fo.flush()
    fo.close(); (a.out / f"{s}.done").write_text(f"{len(ids)} frames {time.perf_counter() - t_s:.0f}s\n"); print(f"[posecmp8] {s}: {len(ids)} frames {time.perf_counter() - t_s:.0f}s", flush=True)
print("[posecmp8] done", flush=True)
