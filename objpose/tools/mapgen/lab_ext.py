#!/usr/bin/env python3
"""lab_ext.py — 실제 환경 실행 한 건(hub 실행 폴더, OBJPOSE_SAVE_MASKS=1)의 인식 결과를 FM-Fusion · DualMap 의 지도 생성 프로그램이 읽는 폴더로 꺼낸다.

  프레임   인식이 처리했고 카메라 자세가 있는 프레임만, 처리 순서대로 1번부터. 카메라 자세는 그 실행의 SLAM 결과(실행이 끝났을 때의 키프레임 보정 반영, map_designs.Scene 과 같음)
  <out>/bop/test/<세션>/{rgb, depth(0.1 mm), scene_camera.json(cam_K, depth_scale, cam_R_w2c, cam_t_w2c)}      DualMap 입력(BOP 형식)
  <out>/scans/<세션>/{color, depth, pose}                                                                    FM-Fusion 입력
  <out>/run/<세션>/{sam6d_frames.jsonl, pred/frame-<번호>_mask.png · _label.json}                             인식 결과(프레임 번호 = 0부터)
  <out>/table.json, <out>/fmf_<세션>.yaml (카메라 값만 세션에 맞춘 FM-Fusion 설정)
    <sam6d python> objpose/tools/mapgen/lab_ext.py <run> --session <세션> --out <run>/ext_in
"""
import argparse, json, os, shutil, sys
from pathlib import Path
import cv2
import numpy as np
REPO = Path(__file__).resolve().parents[3]; sys.path.insert(0, str(REPO / "objpose/tools/lab"))
ap = argparse.ArgumentParser(); ap.add_argument("run", type=Path); ap.add_argument("--session", required=True); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args(); _argv = sys.argv; sys.argv = sys.argv[:1]
import map_designs as MD   # noqa: E402
run = a.run.resolve(); s = a.session; sc = MD.Scene(run, s); out = a.out.resolve(); shutil.rmtree(out, ignore_errors=True)
TAB = json.loads((REPO / "objpose/rt/bop/lab8_objects.json").read_text()); WORDS = {o["name"]: o["ycb"].split("_", 1)[1].replace("_", " ") for o in TAB["objects"]}; TAGS = ". ".join(WORDS.values())
B = out / "bop/test" / s; F = out / "scans" / s; R = out / "run" / s
for d in (B / "rgb", B / "depth", F / "color", F / "depth", F / "pose", R / "pred"): d.mkdir(parents=True)
dets = {}
for d in sc.dets: dets.setdefault(int(d["stamp_ns"]), []).append(d)
cam = {}; rows = []; n = 0; K = np.asarray(sc.K, float)
for f in sc.frames:
    st = int(f["stamp_ns"]); Tw = sc.Tws.get(st)
    if Tw is None: continue
    i = int(np.argmin(np.abs(sc.ses.t_ns - st)))
    if abs(int(sc.ses.t_ns[i]) - st) > 5_000_000: continue
    fr = sc.ses.read(i, True); dep = np.clip(fr.depth_raw.astype(np.float32) * 10.0, 0, 65535).astype(np.uint16)
    cv2.imwrite(str(B / "rgb" / f"{n + 1:06d}.png"), fr.color_bgr); cv2.imwrite(str(B / "depth" / f"{n + 1:06d}.png"), dep)
    os.symlink(B / "rgb" / f"{n + 1:06d}.png", F / "color" / f"frame-{n:06d}.png"); os.symlink(B / "depth" / f"{n + 1:06d}.png", F / "depth" / f"frame-{n:06d}.png"); np.savetxt(F / "pose" / f"frame-{n:06d}.txt", Tw, fmt="%.9f")
    Tcw = np.linalg.inv(Tw); cam[str(n + 1)] = {"cam_K": K.reshape(-1).tolist(), "depth_scale": 0.1, "cam_R_w2c": Tcw[:3, :3].reshape(-1).tolist(), "cam_t_w2c": (Tcw[:3, 3] * 1000.0).tolist()}
    ds = dets.get(st, []); m = run / "sam6d/masks" / f"{st}.png"
    if ds and m.exists():
        os.symlink(m, R / "pred" / f"frame-{n:06d}_mask.png")
        (R / "pred" / f"frame-{n:06d}_label.json").write_text(json.dumps({"raw_tags": TAGS, "tags": TAGS, "mask": [{"value": 0, "label": "background"}] + [{"value": j + 1, "labels": {WORDS[x["object"]]: float(x["score"])}, "box": [float(v) for v in x["bbox"]], "object": x["object"]} for j, x in enumerate(ds)]}))
    rows.append({"idx": n, "stamp_ns": st, "dets": len(ds)}); n += 1
(B / "scene_camera.json").write_text(json.dumps(cam)); (B / "scene_gt.json").write_text("{}"); (R / "sam6d_frames.jsonl").write_text("".join(json.dumps(x) + "\n" for x in rows))
(out / "table.json").write_text(json.dumps(dict(TAB, scenes_dir=str(out / "bop/test"))))
rep = {"camera_fx": K[0, 0], "camera_fy": K[1, 1], "camera_cx": K[0, 2], "camera_cy": K[1, 2]}; t = (REPO / "objpose/tools/obchange/fmf_obchange_small.yaml").read_text().splitlines()
(out / f"fmf_{s}.yaml").write_text("\n".join(f"{l.split(':')[0]}: {rep[l.split(':')[0]]}" if l.split(":")[0] in rep else l for l in t) + "\n")
print(f"{s}: 프레임 {n}, 검출 있는 프레임 {len(list((R / 'pred').glob('*_label.json')))}", flush=True)
