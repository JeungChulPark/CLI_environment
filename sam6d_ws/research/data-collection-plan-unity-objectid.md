# 데이터 수집 계획서 — SAM-6D × SLAM × Unity Object-ID/Memory 검증

> **작성일**: 2026-06-25
> **목적**: `research/technical-unity-object-id-first-sam6d-slam-design.md`의 MVP 실증 및 평가 지표(§9, §8.5 removal 포함)를 검증하기 위해 수집해야 하는 ros2 bag 시나리오·녹화 사양·GT 전략을 정의한다.
> **대상 평가**: duplicate count, id switch, false new/merge, re-ID accuracy, pose jitter, runtime, SLAM drift sensitivity, **ghost object / removal detection(신규)**.
> **전제**: RGB-D 카메라 또는 ros2 bag 입력, ORB-SLAM3(우선)/RTAB-Map, 정적 씬·소수 객체(≤5).

---

## 1. 수집 공통 제약 (모든 bag)

| 항목 | 요건 | 이유 |
|---|---|---|
| 카메라 모션 | 느리고 부드럽게, 급회전·모션블러 금지 | ORB-SLAM3 tracking lost 방지 |
| 씬 텍스처 | 벽/바닥 특징점 충분, 민무늬 벽 회피 | SLAM 초기화·추적 안정 |
| 객체 배치 | 안정 평면, **위치 줄자 측정·기록** | identity·pose GT |
| 조명 | 일정 유지 | depth·appearance 일관성 |
| 초기화 | 시작 시 특징 풍부한 구간을 천천히 비추어 SLAM init | 초기 스케일/추적 |
| 단위 | CAD=mm, 측정 GT=m (변환 기록) | 검출0 사고 방지 |

---

## 2. 녹화 topic 체크리스트

```
필수:
  /camera/color/image_raw                 (sensor_msgs/Image)        # PEM + SLAM + ISM
  /camera/aligned_depth_to_color/image_raw(sensor_msgs/Image)        # PEM depth (정렬 필수)
  /camera/color/camera_info               (sensor_msgs/CameraInfo)   # intrinsic
  /camera/depth/camera_info               (sensor_msgs/CameraInfo)
권장:
  /tf, /tf_static                                                    # 외부 GT/마커 프레임
  /camera/imu (있으면)                                              # ORB-SLAM3 VI 모드
주의:
  - aligned depth + camera_info는 사후 복구 불가 → 반드시 동시 녹화.
  - rosbag2 압축(zstd) 사용, 토픽별 rate 확인(컬러·depth 동일 stamp 정렬).
  - 같은 bag으로 SLAM 재현 가능하도록 원본 RGB-D 보존(파생물만 저장 금지).
```

녹화 예시:
```bash
ros2 bag record -o <scenario_id>_<take> \
  /camera/color/image_raw \
  /camera/aligned_depth_to_color/image_raw \
  /camera/color/camera_info /camera/depth/camera_info \
  /tf /tf_static
```

---

## 3. Ground Truth 전략

| 수준 | 방법 | 산출 GT |
|---|---|---|
| **Identity GT** (필수, 공짜) | 배치만으로 객체 수·종류 결정, layout 사진 + 좌표 메모 | 객체별 정답 id, 등장/제거 타임라인 |
| **6D pose GT** (강력 권장) | 바닥에 **AprilTag/ArUco** 1개로 map 원점 고정 + 객체 옆 소형 마커 | 공통 좌표계의 객체 6D GT |
| **pose GT 대체** (마커 없을 때) | 줄자 위치 측정 + 멀티뷰 reprojection consistency | 위치 GT(자세는 proxy) |
| **Removal GT** (S7/S8 필수) | 제거/이동 **시각(타임스탬프) 수기 기록** 또는 동작 로그 | removal latency·precision 산정 기준 |

> 마커는 객체를 **가리지 않게** 옆/아래에 부착. SLAM 특징과 충돌하지 않도록 크기·위치 조정.

---

## 4. 시나리오 정의

### 4.1 필수셋 (Phase A MVP — 3종)

| ID | 시나리오 | 동작 | 객체수 | 길이 | 검증 지표 |
|---|---|---|---|---|---|
| **S0** | 정적 jitter baseline | 카메라 **고정**, 객체 1개 응시 | 1 | ~30s | pose jitter(static std/velocity) *fusion 전 기준선* |
| **S1** | 단일객체 orbit + 재관측 ★ | 객체 한 바퀴 → **FOV 이탈 → 복귀 재관측** | 1 | 1~2min | duplicate=0, id switch=0, re-ID, 멀티뷰 pose 일관성 |
| **S3** | 소수 다중클래스 traverse | 서로 다른 class 2~3개 지나며 관측, 일부 재방문 | 2~3 | 1~2min | class-gated association, multi-object map, false new/merge |

### 4.2 권장 추가 (Phase A 강화)

| ID | 시나리오 | 동작 | 객체수 | 길이 | 검증 지표 |
|---|---|---|---|---|---|
| **S2** | Loop closure + drift | 방 한 바퀴 크게 돌아 **출발점 복귀**(drift 후 loop close) | 1~2 | 2~4min | **SLAM drift sensitivity** → id switch/false new 곡선 |
| **S4** | 동일 class 다중 인스턴스 | 같은 종류 2개 **가깝게** 배치 후 통과 | 2(동일class) | 1min | distance gate 한계, **false merge**, appearance 필요성 |
| **S5** | occlusion / 일시 소실 | 객체를 가렸다 → 재노출 | 1~2 | 1min | LOST(UNKNOWN)→재획득, tracking continuity |
| **S6** | clutter / negative | CAD에 없는 distractor 혼재 | 2 + distractor | 1min | false new 억제, precision |

### 4.3 ★신규 — Removal / Negative-Observation (Phase B, §8.5 검증)

| ID | 시나리오 | 동작 | 객체수 | 길이 | 검증 지표 |
|---|---|---|---|---|---|
| **S7** | **Object removal** | 객체 관측 → 카메라 이동 → **객체를 치움** → **같은 자리 재방문**(빈 곳을 충분히 응시) | 1~2 | 2~3min | **ghost object=0**, removal precision/recall, **removal latency** |
| **S7b** | **Removal vs 검출실패 구분** | 객체는 그대로 두되, **재방문 시 일부러 안 보이는 각도/원거리**로 스침 | 1 | 1~2min | 검출실패를 **제거로 오판(FP) 안 함** 검증 (recall 0.62 caveat) |
| **S8** | **Object moved(재배치)** | 객체를 **다른 위치로 옮김** → 재방문 | 1~2 | 2~3min | 옛 위치 삭제 + 새 위치 일관 id 유지 vs 새 id 생성(정책 평가) |

> **S7 수집 핵심**: 객체를 치운 뒤 **그 빈 위치를 frustum 안에, 가림 없이, 충분한 시간** 동안 봐야 negative evidence가 N회 쌓여 ABSENT 판정이 작동한다. 그냥 지나치면 removal을 평가할 수 없다.
> **S7b의 의도**: "안 보임 = 제거"로 성급히 판정하지 않는지(가시성 게이트가 negative를 거르는지) 검증하는 **음성 대조군**.

---

## 5. 시나리오 ↔ 평가지표 매핑

| 지표 (보고서 §9) | 검증 시나리오 |
|---|---|
| Unity duplicate count | S1, S2, S3 |
| Object id switch | S1, S2, S3, S5 |
| False new / False merge | S3, S4, S6 |
| Re-identification accuracy | S1, S2 |
| Pose jitter (before/after fusion) | S0 + S1 |
| Runtime (객체 수별) | 전 시나리오 |
| SLAM drift sensitivity | S2 (필수), S1 |
| **Ghost object count** | **S7** |
| **Removal precision/recall** | **S7, S7b** |
| **Removal latency** | **S7** |
| **Moved-object handling** | **S8** |

---

## 6. 수집 분량 가이드

- 각 시나리오 **2~3회 반복**(궤적·속도 변형) → 통계 안정성.
- 객체 수는 **1 → 2~3 → 5** 단계로 늘려 runtime/정확도 trade-off 곡선 확보.
- **최소 1차 수집(Phase A 착수)**: S0×2, S1×3, S3×2, **S7×2**.
  - S7을 최소셋에 포함하는 이유: removal을 안 찍으면 "객체 항상 존재" 가정을 깰 수 없고, 1차 데이터로 §8.5의 필요성조차 실증 불가.
- **2차 수집(Phase A 강화/Phase B)**: S2, S4, S5, S6, S7b, S8 각 2회.

---

## 7. 수집 시 자주 빠뜨리는 것 (체크리스트)

1. ☐ aligned depth + camera_info **동시 녹화** (PEM 입력, 사후 복구 불가).
2. ☐ **재방문(revisit) 구간 포함** — orbit만 하고 안 돌아오면 re-ID 평가 불가.
3. ☐ S7/S8: **제거·이동 시각을 수기 기록**(removal latency 산정 기준).
4. ☐ S7: 치운 뒤 **빈 위치를 가림 없이 충분히 응시**.
5. ☐ layout 사진 + 객체 좌표 메모(identity/pose GT).
6. ☐ AprilTag/ArUco map 원점 고정(가능하면).
7. ☐ 동일 씬을 **raw→Unity 직결(no memory) ablation**용으로도 1회 재생/처리할 수 있게 원본 보존.
8. ☐ 카메라 모션 느리게(ORB-SLAM3 tracking lost 시 해당 take 폐기·재촬영).

---

## 8. 산출 디렉터리 구조 (제안)

```
data/rgbd_bag/unity_objectid/
  S0_static_jitter/        {take1,take2}/   <bag> + layout.json + notes.md
  S1_orbit_revisit/        ...
  S2_loop_drift/           ...
  S3_multiclass_traverse/  ...
  S4_sameclass_close/      ...
  S5_occlusion/            ...
  S6_clutter_negative/     ...
  S7_removal/              {take}/ <bag> + layout.json + removal_timeline.json
  S7b_removal_vs_missdet/  ...
  S8_moved/                ... + move_timeline.json
```
`layout.json` 예: `{objects:[{gt_id, class, position_m:[x,y,z], marker_id}], map_origin_marker:0}`
`removal_timeline.json` 예: `{events:[{gt_id, action:"remove", t_sec: 84.2}]}`

---

## 9. 다음 액션 제안

1. **1차 수집 즉시 착수** — S0×2, S1×3, S3×2, S7×2 (Phase A + removal 실증 최소셋).
2. **`/bmad-architect`로 진행** — 본 계획의 GT/topic 사양을 `object_memory_node` 평가 하네스 설계에 반영.
3. **계획 보강** — AprilTag GT 셋업 가이드(마커 크기·검출 노드·좌표 등록 절차)나 S2 drift 주입 실험 설계를 더 구체화.

**질문**: 1차 수집 최소셋을 **S0/S1/S3 + S7(removal)** 로 확정할까요? removal을 1차에 포함하면 "객체 항상 존재" 가정을 처음부터 깨고 §8.5를 실증할 수 있습니다. 카메라/로봇 하드웨어(예: RealSense 모델, 이동 방식)를 알려주시면 topic 이름·모션 가이드를 그에 맞게 조정하겠습니다.
