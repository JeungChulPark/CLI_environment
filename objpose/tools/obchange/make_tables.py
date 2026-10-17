#!/usr/bin/env python3
"""make_tables.py — eval/*.json 을 읽어 논문·발표에 넣을 표(마크다운)를 만든다.
    python3 objpose/tools/obchange/make_tables.py <out_dir> <md_path>"""
import json, sys, time
from pathlib import Path
O, md = Path(sys.argv[1]), Path(sys.argv[2])
NAME = {"proposed": "제안 방법", "fmfusion": "FM-Fusion", "dualmap": "DualMap", "proposed_online": "제안 방법 (한 지도에 이어 넣기)"}
ROOM = {"kitchen": "Kitchen", "office": "Office", "livingarea": "LivingArea", "smallroom": "SmallRoom", "bigroom": "BigRoom"}
L = []
ch = {k: json.loads((O / "eval" / f"change_{k}.json").read_text()) for k in ("nobg", "bg") if (O / "eval" / f"change_{k}.json").exists()}
mp = {k: json.loads((O / "eval" / f"map_{k}.json").read_text()) for k in ("nobg", "bg") if (O / "eval" / f"map_{k}.json").exists()}
rooms = [r for r in ROOM if any(r in v.get("rooms", {}) for v in ch["nobg"]["methods"].values())]
L += [f"# ObChange 객체 지도 비교 — 자동 생성 표", "", f"생성: {time.strftime('%Y-%m-%d %H:%M')} · 포함된 방: {', '.join(ROOM[r] for r in rooms)}",
      "", "채점 대상은 CAD 템플릿이 있는 YCB 물체 11종. 검출 인정 거리 10 cm, 그대로/옮겨짐 경계 20 cm (Langer 외 2022 와 같은 기준).", ""]
for key, title in (("bg", "배경 제외(물체 없는 방문에도 있는 객체를 뺀 뒤 채점)"), ("nobg", "배경 제외 없음")):
    if key not in mp: continue
    L += [f"## 표 A. 방문별 객체 지도 — {title}", "", "| 방법 | 정답 물체 | 맞음 (재현율) | 이름 틀림 | 놓침 | 없는 물체 | 정밀도 | 중심 오차 중앙값 / 90% (cm) | 방향 |", "|---|---|---|---|---|---|---|---|---|"]
    for m in ("proposed", "fmfusion", "dualmap"):
        v = mp[key]["methods"].get(m)
        if not v: continue
        L.append(f"| {NAME[m]} | {v['gt']} | {v['match']} ({v['recall_pct']}%) | {v['wrong_name']} | {v['missing']} | {v['false_obj']} | {v['precision_pct']}% | "
                 f"{v['center_cm_p50']:.1f} / {v['center_cm_p90']:.1f} | {'있음 (6D 포즈)' if m == 'proposed' else '없음'} |")
    L.append("")
    L += [f"## 표 B. 두 방문 사이의 변화 판정 — {title}", "", "| 방법 | 정답 | 검출 | 범주 맞음 | 검출 중 범주 맞음 | 잘못 보고한 변화 | 그대로 | 옮겨짐 | 사라짐 | 새로 생김 |", "|---|---|---|---|---|---|---|---|---|---|"]
    for m in ("proposed", "fmfusion", "dualmap", "proposed_online"):
        v = ch[key]["methods"].get(m)
        if not v: continue
        per = " | ".join(f"{v['per'][c]['correct']}/{v['per'][c]['gt']}" for c in ("static", "moved", "removed", "novel"))
        L.append(f"| {NAME[m]} | {v['gt']} | {v['detected']} ({v['detected_pct']}%) | {v['correct']} ({v['correct_pct']}%) | {v['correct_of_detected_pct']}% | {v['fp_changed']} | {per} |")
    L += ["", "범주 열은 맞음/정답 수.", ""]
    L += [f"### 방별 범주 맞음 (맞음/정답) — {title}", "", "| 방법 | " + " | ".join(ROOM[r] for r in rooms) + " |", "|---|" + "---|" * len(rooms)]
    for m in ("proposed", "fmfusion", "dualmap", "proposed_online"):
        v = ch[key]["methods"].get(m)
        if not v: continue
        L.append(f"| {NAME[m]} | " + " | ".join(f"{v['rooms'][r]['correct']}/{v['rooms'][r]['gt']}" if r in v["rooms"] else "-" for r in rooms) + " |")
    L.append("")
ps = sorted((O / "eval").glob("pose_*.json"))
if ps:
    L += ["## 표 C. 카메라 자세 — 데이터셋 자세(로봇 위치 추정) 대 ORB-SLAM3", "",
          "| 방 | 방문 수 | 궤적 차 RMSE (cm) | 회전 차 중앙값 (°) | 정답 중심과의 거리 중앙값 (cm): 데이터셋 / ORB | 관측 흩어짐 중앙값 (cm): 데이터셋 / ORB |", "|---|---|---|---|---|---|"]
    import numpy as np
    for p in ps:
        d = json.loads(p.read_text())["scenes"]; r = p.stem.split("_", 1)[1]
        f = lambda k: float(np.mean([s[k] for s in d.values() if s.get(k) is not None])) if any(s.get(k) is not None for s in d.values()) else float("nan")
        L.append(f"| {ROOM.get(r, r)} | {len(d)} | {f('traj_diff_rmse_cm'):.1f} | {f('rot_diff_med_deg'):.1f} | {f('obj_err_med_cm_tf'):.1f} / {f('obj_err_med_cm_orb'):.1f} | {f('obj_scatter_med_cm_tf'):.2f} / {f('obj_scatter_med_cm_orb'):.2f} |")
    L.append("")
figs = sorted((O / "figs").glob("change_*.png"))
if figs:
    L += ["## 그림", ""] + [f"- `{f}`" for f in figs] + [""]
md.write_text("\n".join(L)); print(md)
