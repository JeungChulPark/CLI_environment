#!/usr/bin/env python3
"""sam6d_debug.csv 의 frame_id 들로 가시성 라벨 스켈레톤 생성.

GT pose 라벨이 없으므로 milk 가시성은 사람이 라벨링한다. 이 스크립트는
전 프레임을 visible=1 로 채운 visibility_labels.csv 를 만들어준다.
사용자는 milk 가 안 보이는 프레임의 값을 0 으로 바꾸면 된다.

  python3 make_visibility_template.py \
      --debug-dir outputs/validation/SLAM_with_milk_nomilk/_run/debug \
      --out       outputs/validation/SLAM_with_milk_nomilk/visibility_labels.csv
"""
import argparse
import csv
import os


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--debug-dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--default-visible", type=int, default=1,
                    help="기본 가시성 값(1=보임). only_Milk 는 1 유지, nomilk 는 생성 후 편집.")
    args = ap.parse_args()

    csv_path = os.path.join(args.debug_dir, "sam6d_debug.csv")
    if not os.path.isfile(csv_path):
        raise SystemExit(f"sam6d_debug.csv 없음: {csv_path}")

    seen = []
    seen_set = set()
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            fid = str(r.get("frame_id", "")).strip()
            if fid.isdigit():
                fid = f"{int(fid):06d}"
            if fid and fid not in seen_set:
                seen_set.add(fid)
                seen.append(fid)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame_id", "manually_labeled_milk_visible"])
        for fid in seen:
            w.writerow([fid, args.default_visible])

    print(f"[OK] {len(seen)} frames → {args.out}")
    print("→ milk 가 안 보이는 프레임의 값을 0 으로 편집하세요 (nomilk 데이터셋).")


if __name__ == "__main__":
    main()
