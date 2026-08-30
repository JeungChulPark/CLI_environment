# SAM-6D 신뢰성 검증 도구 (tools/validation)

구현 코드를 변경하지 않고, 노드가 남기는 디버그 로그(`sam6d_debug.csv` + 프레임별
`detection_pem.json`)와 수동 가시성 라벨을 결합해 검증 지표·시각화를 산출한다.

데이터셋(실제 경로):
- `SLAM_with_milk_nomilk` = `data/milk_nomilk_bag/` — 문제 A(no-object/False Positive)
- `only_Milk` = `data/only_milk/` — 문제 B(pose stability)

상세 기준/판정: `_bmad-output/implementation-artifacts/validation-sam6d-reliability-fix.md`

## 스크립트

| 스크립트 | 역할 | 실행 환경 |
|---|---|---|
| `extract_bag_frames.py` | 노드 stem 인덱싱 재현 → `<frame_id>.png` RGB 추출(시각화 소스) | ROS2 + cv_bridge |
| `make_visibility_template.py` | 디버그 CSV frame_id 로 `visibility_labels.csv` 스켈레톤 생성 | python3(표준) |
| `build_validation_report.py` | 디버그 CSV + pem R + 라벨 → `frame_results.csv` + `metrics.json` | python3(표준, numpy 불요) |
| `make_visualizations.py` | TN/FP/stability 3종 이미지 렌더 | python3 + cv2 + numpy |

## 절차 (예: SLAM_with_milk_nomilk)

```bash
cd ~/temp_ws/CLI_environment/sam6d_ws
colcon build --packages-select sam6d_ros && source install/setup.bash

# 1) 추론(디버그 로그) — config:= 주의, bag 자동 재생
ros2 launch sam6d_ros sam6d_inference.launch.py config:=src/sam6d_ros/config/no_cli_milk_nomilk.yaml

# 1b) RGB 추출(터미널 2개)
python3 tools/validation/extract_bag_frames.py --out outputs/validation/SLAM_with_milk_nomilk/frames &
ros2 bag play data/milk_nomilk_bag

# 2) 가시성 라벨(생성 후 milk 없는 frame_id 를 0 으로 편집)
python3 tools/validation/make_visibility_template.py \
    --debug-dir outputs/validation/SLAM_with_milk_nomilk/_run/debug \
    --out outputs/validation/SLAM_with_milk_nomilk/visibility_labels.csv

# 3) 지표
python3 tools/validation/build_validation_report.py --bag SLAM_with_milk_nomilk \
    --debug-dir outputs/validation/SLAM_with_milk_nomilk/_run/debug \
    --pem-root  outputs/validation/SLAM_with_milk_nomilk/_run \
    --labels    outputs/validation/SLAM_with_milk_nomilk/visibility_labels.csv \
    --out       outputs/validation/SLAM_with_milk_nomilk

# 4) 시각화
python3 tools/validation/make_visualizations.py \
    --frame-results outputs/validation/SLAM_with_milk_nomilk/frame_results.csv \
    --frames-dir    outputs/validation/SLAM_with_milk_nomilk/frames \
    --pem-root      outputs/validation/SLAM_with_milk_nomilk/_run \
    --camera-json   src/sam6d_ros/config/d455f_camera.json \
    --out           outputs/validation/SLAM_with_milk_nomilk/visualizations
```

`only_Milk` 는 config 를 `no_cli_only_milk.yaml` 로 바꿔 동일 반복. 상시 가시이므로
`build_validation_report.py` 에서 `--labels` 생략 시 전부 visible 로 간주되고,
시각화는 `--mode stability` 로 pose_stability_sequence grid 만 생성하면 된다.

## FR-10 — Stabilization ON/OFF 정량 비교

- 노드 `_write_debug_log` 가 **raw**(`t_x/y/z_mm`, `q*_raw`)와 **stabilized**(`t_*_stab_mm`, `q*_stab`)를
  같은 CSV에 함께 기록한다. 따라서 **단일 ON run**에서 안정화 효과를 비교할 수 있다(추가 OFF run 불요).
- `build_validation_report.py` 는 **연속 TP 구간**에서:
  - `translation_spread_mm` / `rotation_spread_deg` 의 raw·stabilized·**reduction_rate**(=1−stab/raw)
  - `frame_to_frame_delta_*` (연속 변화량 mean/p95/max)
  를 `metrics.json.stabilization_comparison` 으로 산출한다.
- `make_visualizations.py --mode stabcmp`(또는 all) 가 raw(빨강) vs stabilized(파랑) 시계열 비교
  플롯(`stabilization_compare/stabilization_compare.png`)을 생성한다.
- 교차검증이 필요하면 `stabilize.enabled:false` config로 OFF run을 1회 더 돌려 두 run의
  spread를 비교한다(선택).

## 기타 주의

- `detection_pem.json` 의 R/t 는 백엔드 raw 값(필터/안정화 전)이며, 시각화 axis 는 cam_K(mm 가정) best-effort.
- spread/reduction 은 FP·위치점프 프레임을 제외한 **연속 TP 구간**에서만 계산한다.
