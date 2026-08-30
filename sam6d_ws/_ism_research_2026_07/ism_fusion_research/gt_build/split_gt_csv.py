#!/usr/bin/env python3
"""split_gt_csv.py — GT CSV 를 '사용자가 적을 칸' 과 '분석용 메타' 로 분리한다.

이유
  1) 사용자가 채울 칸은 visible_objects / user_reviewed / notes 셋뿐인데,
     기존 CSV 는 10열이라 어디를 적어야 하는지 한눈에 들어오지 않는다.
  2) `_current_ism_accepted` (현재 모델 판정) 가 같은 파일에 있으면 먼저 눈에 들어와
     확증 편향이 생긴다. 균등층은 통계의 분모라 이 편향이 치명적이다.
     → 아예 파일에서 분리한다.

결과
  user_visibility_gt.csv   ← 사용자가 여는 파일 (7열, 채울 칸 3개)
      priority        어디까지 할지 판단용 (읽기 전용)
      dataset_name    읽기 전용
      frame_id        읽기 전용
      image_path      읽기 전용 — 이 경로의 PNG 를 열어서 보면 된다
      visible_objects ★ 채울 칸
      user_reviewed   ★ 채울 칸
      notes           ★ 선택

  frame_meta.csv           ← 분석 스크립트용. 사용자는 볼 필요 없다.
      dataset_name, frame_id, priority, strata,
      current_ism_accepted, n_yolo_candidate_objects
      (dataset_name + frame_id 로 다시 결합한다)

이미 입력한 내용이 있으면 그대로 보존한다. 되돌리려면 user_visibility_gt.csv.bak 을 쓰면 된다.
"""
import csv, os, shutil

HERE = os.path.dirname(os.path.abspath(__file__))
GT = os.path.join(os.path.dirname(os.path.dirname(HERE)), "gt_input")
MAIN = os.path.join(GT, "user_visibility_gt.csv")
META = os.path.join(GT, "frame_meta.csv")

rows = list(csv.DictReader(open(MAIN, newline="", encoding="utf-8")))
if "priority" in rows[0]:
    print("이미 분리된 형식입니다. 변경하지 않습니다.")
    raise SystemExit(0)

pre = os.path.join(GT, "user_visibility_gt.csv.pre_split")
if not os.path.exists(pre):
    shutil.copy2(MAIN, pre)
    print(f"분리 전 원본 보관: {os.path.basename(pre)}")

USER_COLS = ["priority", "dataset_name", "frame_id", "image_path",
             "visible_objects", "user_reviewed", "notes"]
META_COLS = ["dataset_name", "frame_id", "priority", "strata",
             "current_ism_accepted", "n_yolo_candidate_objects"]

user_rows, meta_rows = [], []
for r in rows:
    user_rows.append({
        "priority": r["_priority"], "dataset_name": r["dataset_name"],
        "frame_id": r["frame_id"], "image_path": r["image_path"],
        "visible_objects": r["visible_objects"],
        "user_reviewed": r["user_reviewed"], "notes": r["notes"]})
    meta_rows.append({
        "dataset_name": r["dataset_name"], "frame_id": r["frame_id"],
        "priority": r["_priority"], "strata": r["_strata"],
        "current_ism_accepted": r["_current_ism_accepted"],
        "n_yolo_candidate_objects": r["_n_yolo_candidate_objects"]})

tmp = MAIN + ".tmp"
with open(tmp, "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=USER_COLS); w.writeheader(); w.writerows(user_rows)
os.replace(tmp, MAIN)
with open(META, "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=META_COLS); w.writeheader(); w.writerows(meta_rows)

done = sum(1 for r in user_rows if r["user_reviewed"] == "yes")
print(f"user_visibility_gt.csv : {len(user_rows)}행 × {len(USER_COLS)}열 "
      f"(채울 칸 3개, 완료 {done})")
print(f"frame_meta.csv         : {len(meta_rows)}행 × {len(META_COLS)}열 (분석용)")
print(f"우선순위 분포: " + ", ".join(
    f"P{p}={sum(1 for r in user_rows if r['priority']==p)}" for p in ("1", "2", "3")))
