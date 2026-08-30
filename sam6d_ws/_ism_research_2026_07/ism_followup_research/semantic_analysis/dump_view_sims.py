#!/usr/bin/env python3
"""dump_view_sims.py — 라벨된 박스에 대해 42-view CLS 유사도를 전부 저장한다 (READ-ONLY).

이전 6데이터 관찰(ism_accuracy_observation)은 sem_top5 / sem_top1 / sem_mean 만 저장했고
42개 개별 유사도는 남기지 않았다. sem_top5 와 sem_mean 의 객체별 우열 원인을 규명하려면
"어떤 view 가 점수를 만들었는가" 를 봐야 하므로 여기서 곡선 전체를 복원한다.

운영 모듈은 import 만 하고 수정하지 않는다. 판정 로직도 재현하지 않는다 —
필요한 것은 crop → DINOv2 CLS → 42 템플릿 CLS 코사인 뿐이므로
YOLO 와 MobileSAM 은 실행하지 않는다 (박스 좌표는 이전 덤프의 uid 에서 가져온다).

산출:
  semantic_analysis/view_sims.npz          uid|object -> float32[42]
  results/semantic_view_similarity.csv     view 단위 롱포맷 (라벨된 pair 전용)
"""
import csv, os, sys
from pathlib import Path

import cv2
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                               # ism_followup_research
RSRCH = os.path.dirname(ROOT)                              # _ism_research_2026_07
REPO = os.path.dirname(RSRCH)                              # sam6d_ws
sys.path.insert(0, REPO)
import yolo_ism as yi                    # noqa: E402
import yolo_ism_object_n as o_n          # noqa: E402

OBS = os.path.join(RSRCH, "ism_accuracy_observation")
CONV = os.path.expanduser("~/temp_ws/CLI_environment/data_slam/260714_frame_data/converted")
COLOR_TOPIC = "/camera/camera/color/image_raw"
RES = os.path.join(ROOT, "results")
os.makedirs(RES, exist_ok=True)


def bag_frames(ds, wanted):
    """wanted 에 든 frame index 만 디코딩해서 내보낸다."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import get_typestore, Stores
    ts = get_typestore(Stores.ROS2_HUMBLE)
    hi = max(wanted)
    with AnyReader([Path(os.path.join(CONV, ds))], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic == COLOR_TOPIC]
        i = -1
        for conn, t, raw in reader.messages(connections=conns):
            i += 1
            if i > hi:
                break
            if i not in wanted:
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            img = cv2.cvtColor(buf, cv2.COLOR_RGB2BGR) if msg.encoding.lower() == "rgb8" else buf.copy()
            yield i, img


def main():
    # ---------------------------------------------------------------- 라벨/박스 적재
    labels = {}
    for r in csv.DictReader(open(os.path.join(OBS, "labels", "box_labels_merged.csv"))):
        labels[r["uid"]] = r["true_class"]

    # 어떤 객체의 후보였는지 (candidate_for) — pair 구성을 운영과 동일하게 맞춘다
    cand_for, coords = {}, {}
    for ds in ("sam_105018", "sam_105314", "sam_105652", "sam_110104", "sam_110532", "sam_110633"):
        p = os.path.join(OBS, "frame_dumps", f"{ds}_boxes.csv")
        for r in csv.DictReader(open(p)):
            if r["uid"] not in labels:
                continue
            cand_for[r["uid"]] = [c for c in r["candidate_for"].split(";") if c]
            coords[r["uid"]] = (ds, int(r["frame_id"]),
                                [int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"])])
    print(f"라벨 {len(labels)} / 박스좌표 확보 {len(coords)}")

    # ---------------------------------------------------------------- 모델
    defaults, objs = o_n.load_config(o_n.DEFAULT_CONFIG)
    device = defaults.get("device", "cuda:0") if torch.cuda.is_available() else "cpu"
    model = yi.build_dinov2(defaults.get("dinov2_checkpoint") or yi.DEFAULT_DINOV2_CKPT, device)
    objs = o_n.prepare_objects(objs, model, device, False)
    for o in objs:
        o["_tcls"] = o["tcls"].to(device)          # [42, 384] (L2 정규화 가정 — 운영과 동일 경로)
    print("객체:", [o["name"] for o in objs], "| 템플릿 view 수:",
          {o["name"]: int(o["tcls"].shape[0]) for o in objs})

    # ---------------------------------------------------------------- 프레임별 CLS
    by_ds = {}
    for uid, (ds, fid, bb) in coords.items():
        by_ds.setdefault(ds, {}).setdefault(fid, []).append(uid)

    sims_out, rows = {}, []
    for ds in sorted(by_ds):
        want = set(by_ds[ds])
        n = 0
        for fi, bgr in bag_frames(ds, want):
            uids = by_ds[ds][fi]
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            norm = yi.normalize_rgb(rgb)
            crops, keep = [], []
            for u in uids:
                c = yi.crop_resize_pad(norm, coords[u][2])
                if c is not None:
                    crops.append(c); keep.append(u)
            if not crops:
                continue
            cls_all, _ = o_n.dinov2_blocks_forward(model, crops, device, [11])
            for bi, u in enumerate(keep):
                q = cls_all[bi].to(device)
                for o in objs:
                    if o["name"] not in cand_for.get(u, []):
                        continue                       # 운영에서 후보가 아니었던 조합은 제외
                    s = (o["_tcls"] @ q).cpu().numpy().astype(np.float32)
                    sims_out[f"{u}|{o['name']}"] = s
                    ss = np.sort(s)[::-1]
                    rows.append({
                        "uid": u, "dataset": ds, "frame_id": fi, "object": o["name"],
                        "true_class": labels[u],
                        "is_positive": int(labels[u] == o["name"]),
                        "n_view": len(s),
                        "sem_top1": round(float(ss[0]), 5),
                        "sem_top5": round(float(ss[:5].mean()), 5),
                        "sem_mean": round(float(s.mean()), 5),
                        "sem_median": round(float(np.median(s)), 5),
                        "sem_min": round(float(ss[-1]), 5),
                        "sem_std": round(float(s.std()), 5),
                        "top1_view": int(np.argmax(s)),
                        "sims": ";".join(f"{v:.5f}" for v in s),
                    })
            n += len(keep)
        print(f"  {ds}: 박스 {n}")

    np.savez_compressed(os.path.join(HERE, "view_sims.npz"), **sims_out)
    out = os.path.join(RES, "semantic_view_similarity.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print(f"\n-> {out}  ({len(rows)} pair)")
    print(f"-> {os.path.join(HERE, 'view_sims.npz')}")


if __name__ == "__main__":
    main()
