#!/usr/bin/env python3
"""반자동 Ground Truth(milk 가시성) 라벨 생성 도구.

기존 proxy(NO_OBJECT=milk 부재) 방식을 폐기하고, 사람이 검수하는 GT 체계로 전환한다.

출력 CSV (정확히 2열):
    frame_id,milk_visible
  - milk_visible 는 0/1 만 사용 (1=milk 보임, 0=milk 안 보임).
  - 초안(draft)으로 모델 예측(pose 발행 여부)을 미리 채워 사용자의 수정량을 줄인다.
    ※ 이 초안은 GT 가 아니다. 사용자가 contact sheet 를 보고 0/1 을 직접 교정해야 GT 가 된다.

검수 보조:
  --frames-dir 가 주어지면 모든 프레임 썸네일 격자(contact sheet)를 만들어,
  각 칸에 frame_id 와 현재 draft 값을 표시한다. 사용자는 이를 보고 CSV 의 0/1 만 고친다.

사용:
  python3 make_gt_labels.py \
      --debug-dir  outputs/validation/SLAM_with_milk_nomilk/_run/debug \
      --frames-dir outputs/validation/SLAM_with_milk_nomilk/frames \
      --out        outputs/validation/SLAM_with_milk_nomilk/visibility_labels.csv \
      --contact-sheet outputs/validation/SLAM_with_milk_nomilk/gt_contact_sheet.png
"""
import argparse
import csv
import math
import os


def load_predictions(debug_csv):
    """frame_id -> prediction(1=pose 발행=milk 추정 있음, 0=NO_OBJECT)."""
    from collections import defaultdict
    byf = defaultdict(list)
    with open(debug_csv, newline="") as f:
        for r in csv.DictReader(f):
            byf[r.get("frame_counter") or r.get("frame_id")].append(r)
    preds = {}
    order = []
    for key, g in byf.items():
        fid = str(g[0].get("frame_id", "")).strip()
        if fid.isdigit():
            fid = f"{int(fid):06d}"
        dec = g[0].get("final_decision", "")
        preds[fid] = 1 if dec in ("PUBLISH", "PUBLISH_LEGACY") else 0
        order.append(fid)
    # frame_id 정렬
    order = sorted(set(order), key=lambda s: int(s) if s.isdigit() else s)
    return order, preds


def contact_sheet(order, preds, frames_dir, out_path, cols=10, thumb=(192, 144)):
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("[contact-sheet] cv2/numpy 없음 — 격자 생략")
        return
    tiles = []
    tw, th = thumb
    for fid in order:
        p = os.path.join(frames_dir, f"{fid}.png")
        img = cv2.imread(p, cv2.IMREAD_COLOR) if os.path.isfile(p) else None
        if img is None:
            img = np.full((th, tw, 3), 40, np.uint8)
        else:
            img = cv2.resize(img, (tw, th))
        draft = preds.get(fid, 1)
        color = (0, 200, 0) if draft == 1 else (0, 0, 255)
        cv2.rectangle(img, (0, 0), (tw - 1, th - 1), color, 3)
        txt = f"{fid} d={draft}"
        cv2.putText(img, txt, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, txt, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(img)
    rows_n = math.ceil(len(tiles) / cols)
    blank = np.full((th, tw, 3), 40, np.uint8)
    while len(tiles) < rows_n * cols:
        tiles.append(blank)
    grid = np.vstack([np.hstack(tiles[r * cols:(r + 1) * cols]) for r in range(rows_n)])
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    cv2.imwrite(out_path, grid)
    print(f"[contact-sheet] {len(order)} frames → {out_path} (녹색테두리=draft 1, 빨강=0)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--debug-dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames-dir", default=None)
    ap.add_argument("--contact-sheet", default=None)
    ap.add_argument("--cols", type=int, default=10)
    ap.add_argument("--no-draft", action="store_true",
                    help="draft 를 예측으로 채우지 않고 모두 1 로 둠(편향 방지용)")
    args = ap.parse_args()

    debug_csv = os.path.join(args.debug_dir, "sam6d_debug.csv")
    if not os.path.isfile(debug_csv):
        raise SystemExit(f"sam6d_debug.csv 없음: {debug_csv}")
    order, preds = load_predictions(debug_csv)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame_id", "milk_visible"])
        for fid in order:
            draft = 1 if args.no_draft else preds.get(fid, 1)
            w.writerow([fid, draft])
    n1 = sum(1 for fid in order if (1 if args.no_draft else preds.get(fid, 1)) == 1)
    print(f"[GT draft] {len(order)} frames → {args.out}  (draft milk_visible=1: {n1}, =0: {len(order)-n1})")

    if args.frames_dir and args.contact_sheet:
        contact_sheet(order, preds, args.frames_dir, args.contact_sheet, cols=args.cols)

    print("→ 다음: contact sheet 를 보고 visibility_labels.csv 의 milk_visible(0/1)만 교정하세요.")
    print("  (draft 는 모델 예측일 뿐 GT 가 아닙니다. 교정 후 build_validation_report.py 에 --labels 로 전달)")


if __name__ == "__main__":
    main()
