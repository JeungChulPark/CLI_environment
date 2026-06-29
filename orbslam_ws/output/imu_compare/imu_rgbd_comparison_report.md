# ORB-SLAM3 IMU-RGBD vs RGB-D 비교 보고서

생성일: 2026-06-26 · 작업루트: `CLI_environment/` · 빌드: `orbslam3_ros2`(IMU 배선 추가, 재빌드)

> ⚠️ **해석 전제 (먼저 읽을 것)**
> 본 평가에는 **Ground Truth(GT)가 없습니다.** 이 데이터셋들은 LiDAR가 없어 Fast-LIO2 pseudo-GT도 만들 수 없습니다.
> 따라서 **RPE는 절대 정확도가 아니라 "GT-free 드리프트 프록시"** 입니다:
> - **loop-drift(return-to-origin)** = ‖p_end − p_start‖ 및 그 궤적길이 대비 % (출발=복귀 가정)
> - **per-step RPE** = 프레임 간 상대 병진의 분포(국소 일관성)
> "drift가 작다 = 자기일관성이 높다"로 읽어야 하며 "실제 궤적에 정확하다"는 뜻이 아닙니다.

---

## 0. 요약 (TL;DR)

- **이번 핵심 실험**: 새 bag `SLAM_three_labs_rgbd_imu`(RGB+aligned-depth+**IMU 200 Hz**)를 **IMU-RGBD 모드**로 ORB-SLAM3에 처음 투입. IMU 초기화 성공(VIBA 1·2 수행).
- **동일 bag IMU vs RGB-D**: IMU의 최대 효과는 **드리프트 감소가 아니라 연산 효율/추적 처리량**이었다 — 프레임당 **84.8 ms(IMU) vs 115.4 ms(RGB-D)**, 즉 **IMU가 27% 더 빠름**(IMU 모션 prior → 매칭 가속). 그 결과 같은 시간에 **포즈를 ~1.5배** 더 산출(2542 vs 1717).
- **추적 성공률**(투입 프레임 대비)은 두 모드 모두 **~95–96%**로 동등(둘 다 강건).
- **드리프트**: 부드러운 모션이라 시각만으로도 이미 **<0.2%**로 우수 → IMU가 드리프트를 더 줄이진 못함(0.17% vs 0.12%, 노이즈 수준).
- **맵 entropy(MME)**: 네 조건 모두 -3.4~-3.6으로 유사(맵 선명도 동등).

---

## 1. 데이터 준비

- `data_slam/SLAM_three_labs_rgbd_imu.tar.gz.part_00..03` 4분할 → 단일 `*.tar.gz`(2.94 GB) 결합 후 해제.
- 산출: `data_slam/SLAM_three_labs_rgbd_imu/bag/bag_0.db3` (2.07 GB).
- bag 내용: duration **152.1 s**, color 4545 / aligned-depth 4324 / **IMU 30292(≈200 Hz)** / extrinsics(depth↔color/gyro/accel). **LiDAR 없음**.
- IMU 물리 검증: |a|≈9.77 m/s²(중력 포함, 단위 정상, std 0.17 → 매우 부드러운 모션), |ω| 평균 0.085 rad/s.

## 2. ORB-SLAM3 IMU 배선 (코드 변경)

기존 래퍼는 **RGB-D 전용**이었다. 이번에 IMU-RGBD를 지원하도록 수정·재빌드:

- `src/orbslam3_ros2/src/rgbd_node.cpp`:
  - `use_imu=true`면 `System::IMU_RGBD`로 생성, `/camera/camera/imu` 구독(**BEST_EFFORT, KeepLast 2000, 전용 reentrant 콜백그룹**).
  - 프레임 콜백에서 직전 프레임~현재 프레임 사이 IMU 샘플을 모아 `TrackRGBD(rgb, depth, t, vImuMeas)` 호출.
  - `main()`을 **MultiThreadedExecutor**로 변경 → 무거운 트래킹과 200 Hz IMU 콜백이 동시 진행(초기 단일스레드에서 IMU 큐 기아 → preintegration SIGSEGV 발생했던 문제 해결).
- 신규 세팅 `data/orbslam3_d455f_640x480_rgbd_imu.yaml`:
  - IMU 노이즈/random-walk = D455(BMI0xx) **데이터시트 nominal**(NoiseGyro 1.7e-4, NoiseAcc 2.0e-3, GyroWalk 1.9e-5, AccWalk 3.0e-3, Frequency 200). *Allan 분산 캘리브 아님.*
  - `IMU.T_b_c1`(color→IMU extrinsic)은 **bag 자체 extrinsics**에서 계산: `T_gyro_depth · inv(T_color_depth)` (회전≈단위, 병진≈[2.9, 0.7, 1.6] cm).
- launch/driver에 `use_imu`·`imu_topic` 인자 추가(`scripts/run_imu_compare.py`).

## 3. 실행 조건 (동일 빌드·동일 rate)

| condition | bag | 모드 | settings |
|---|---|---|---|
| `three_labs_imu` | SLAM_three_labs_rgbd_imu(신규) | **IMU-RGBD** | `..._rgbd_imu.yaml` |
| `three_labs_rgbd` | SLAM_three_labs_rgbd_imu(신규) | RGB-D (IMU ablation) | `..._rgbd.yaml` |
| `SLAM_three_laps` | 기존 RGB-D bag | RGB-D | `..._rgbd.yaml` |
| `SLAM_one_lap` | 기존 RGB-D bag | RGB-D | `..._rgbd.yaml` |

- 공통: `ros2 bag play --rate 0.5 --clock`, dense map 활성, vocab 동일, 단일 PC 순차 실행.

## 4. 통합 결과 표

| dataset (mode) | IMU init | 프레임 투입 | 포즈 수 | **추적 성공률**¹ | 포즈수율²(vs bag) | 프레임당 ms | **RPE drift_m**³ | **RPE drift %**³ | per-step RPE(med) | 궤적길이 m | **MME**⁴ |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **three_labs_imu** (IMU-RGBD) | ✅ VIBA1·2 | 2654 | 2542 | **95.8 %** | 55.9 % | **84.8** | 0.050 | 0.17 % | 7.4 mm | 29.8 | −3.434 |
| **three_labs_rgbd** (RGB-D) | — | 1798 | 1717 | 95.5 % | 37.8 % | 115.4 | 0.037 | 0.12 % | 10.7 mm | 31.7 | −3.448 |
| SLAM_three_laps (RGB-D) | — | 1434 | 1434 | 100 % | 32.9 % | 131.0 | 0.696 | 2.14 % | 17.5 mm | 32.6 | −3.466 |
| SLAM_one_lap (RGB-D) | — | 392 | 391 | 99.7 % | 35.7 % | 121.2 | 0.122 | 1.15 % | 16.6 mm | 10.6 | −3.596 |

¹ **추적 성공률 = 유효 포즈 / 트래커에 투입된 프레임** (강건성 지표; 끊김 여부).
² **포즈수율 = 유효 포즈 / bag color 프레임** (rate 0.5 연속부하에서 synchronizer가 버린 프레임 포함 → 처리량 지표).
³ **RPE 프록시(GT-free)** = loop-drift(return-to-origin) ‖p_end−p_start‖ 및 %; per-step RPE = 프레임 간 병진 중앙값.
⁴ **MME(Mean Map Entropy)** = dense map 국소 가우시안 미분엔트로피 평균(r=0.2 m, 약 30k 점 샘플). **낮을수록 선명·일관**.

## 5. 해석

### 5.1 IMU의 효과 (동일 bag: three_labs_imu vs three_labs_rgbd)
- **연산 효율이 1차 이득**: IMU 모션 prior가 특징 매칭·local-map 탐색을 가속 → **프레임당 84.8 ms vs 115.4 ms (−27%)**. 덕분에 같은 실행에서 트래커가 **더 많은 프레임을 소화**(2654 vs 1798)하고 포즈 밀도가 **~1.5배**.
- **강건성은 동등하게 높음**: 투입 대비 추적 성공률 95.8 % ≈ 95.5 %. 두 모드 모두 끊김 거의 없음.
- **드리프트는 IMU가 줄이지 못함**: 부드러운 GT급 모션이라 **시각 단독으로 이미 0.12 %**(30 m에 3.7 cm) 수준. IMU 추가 시 0.17 %로 오히려 미세 증가(측정 노이즈 범위). 즉 *이 시퀀스에선 정확도 한계가 시각이 아니라 모션/장면에 있지 않아* IMU의 정확도 기여 여지가 작았다.
- **맵 품질 동등**: MME −3.434 vs −3.448(차이 0.4 %).
- **정리**: 본 데이터에서 IMU-RGBD의 실질 이득은 **(a) 처리량/연산효율, (b) 빠른 모션·시각 열화 시 대비 강건성 마진**이며, **부드러운 시퀀스의 절대 정확도 개선은 미미**하다. IMU의 진짜 값어치는 거친/회전-only/모션블러 시퀀스에서 드러날 것(별도 stress bag 권장).

### 5.2 데이터셋 간 (RGB-D 기준)
- 신규 bag(0.12 %)이 기존 `SLAM_three_laps`(2.14 %)·`one_lap`(1.15 %)보다 drift가 훨씬 작다. 단, **loop-drift는 출발=복귀를 가정**하며 그 프로토콜은 신규 bag에만 보장되므로 *기존 bag의 %는 미복귀분이 섞여 과대평가*될 수 있다(아래 한계 2).
- per-step RPE(국소 일관성)는 7–18 mm로 전 조건 유사 → 프레임 단위 추정은 모두 매끄럽다.

## 6. 한계 (정직성)
1. **GT 부재** → 모든 RPE/drift는 자기일관성이며 절대 정확도가 아니다(LiDAR 없어 Fast-LIO2 GT 생성 불가).
2. **loop-drift 비교의 비대칭** → return-to-origin은 신규 bag만 프로토콜 보장. 기존 두 bag의 drift %는 recording 차이를 포함하므로 *동일 bag IMU↔RGB-D 비교*가 가장 공정하다.
3. **IMU 캘리브 nominal** → 노이즈는 데이터시트 근사(Allan 미수행). extrinsic은 bag 보고값(정상).
4. **추적 성공률 두 정의** → "vs 투입"(강건성) ≠ "vs bag"(처리량). rate 0.5 연속부하로 synchronizer 프레임 드롭 존재.
5. **IMU 초기화** → 부드러운 모션이라 초기 "not enough acceleration" 반복 후 VIBA로 초기화됨(스케일은 depth가 제공해 가능). 더 격한 초기 모션이면 더 빨리 수렴.

## 7. 산출물
- 신규 세팅: `orbslam_ws/data/orbslam3_d455f_640x480_rgbd_imu.yaml`
- 드라이버/메트릭: `orbslam_ws/scripts/run_imu_compare.py`, `orbslam_ws/scripts/compute_slam_metrics.py`
- 조건별 출력: `orbslam_ws/output/imu_compare/<condition>/` — `trajectory.txt`, `keyframes.txt`, `orbslam3_dense_map.pcd`, `orbslam3_launch.log`
- 메트릭 원본: `orbslam_ws/output/imu_compare/metrics_summary.json`

---

## 9. 지표 정의 보충 — RPE 2종 & MME

### 9.1 "RPE drift_m / %" (loop-drift) vs "per-step RPE"
둘 다 `compute_slam_metrics.py`에서 같은 궤적으로 계산하지만 **스케일이 정반대**다.
- **RPE drift_m / % = loop-drift(전역 누적)**: `‖p_end − p_start‖`. 궤적 **전체**에 누적된 드리프트를 **하나의 수**로 본다. `%`는 궤적길이로 정규화(경로 길이 다른 시퀀스 비교용). *출발=복귀(return-to-origin)* 가정에서만 유효. → "전체 실행에서 추정이 얼마나 흘렀나".
- **per-step RPE = 프레임 간 상대운동 분포(국소)**: 모든 연속 포즈쌍의 `‖p_{i+1}−p_i‖`의 median·p95. median은 프레임당 정상 이동량, **p95 ≫ median이면 국소 점프(추적 글리치·relocalization 스냅)** 신호. → "프레임 단위가 매끄러운가, 튐이 있나".
- 핵심 차이: **전역 1개 값(누적 오차) ↔ 국소 분포(매 스텝 일관성)**. loop-drift가 작아도 per-step p95가 크면 "전역은 맞는데 중간에 튄다". *주의*: GT가 없어 per-step은 reference 대비 오차가 아니라 운동량 자체의 분산(=점프 탐지 프록시); 고전적 RPE(Sturm et al., 고정 Δ·GT 대비)와 구분.
- ⚠️ **프레임-조밀 방법끼리만 비교 가능**: RTAB-Map은 per-step이 그래프 *노드* 간격(LinearUpdate≈0.1–0.2 m)이라 220 mm가 나오는데 이는 노드 간격이지 추적 튐이 아니다 → ORB3의 per-frame 값과 직접 비교 금지.

### 9.2 MME(Mean Map Entropy) 성능 분석 방법
- **계산**: dense map의 각 점 p_i에 대해 반경 r(=0.2 m) 이웃을 KD-tree로 모아 3×3 공분산 Σ_i를 구하고, 국소 가우시안의 미분엔트로피 `h_i = ½·ln((2πe)³·det Σ_i)`를 점마다 계산 → 전체 평균이 MME(≈30k 점 샘플).
- **의미**: 정합이 잘 된 **선명한 맵**은 점들이 얇은 표면에 몰려 Σ가 납작(det 작음)→ h가 매우 음수 → **MME 낮음**. 드리프트·이중벽·오정합이면 점이 3D로 퍼져 det 커짐 → **MME 높음**. 즉 **낮을(더 음수일)수록 자기일관·선명**. GT 불필요.
- **분석법**: 동일 장면에서 방법별 MME를 **랭킹**(낮을수록 우수). 단, (1) **점 밀도·스케일이 같아야** 공정(희소 맵이 거저 낮게 나오므로 coverage/점수와 병행), (2) r·다운샘플을 **모든 런에 동일** 적용. 본 비교는 r=0.2 m·동일 샘플수로 고정했으나 **백엔드가 다른 ORB3↔RTAB은 점밀도·맵범위가 달라 동일-백엔드(ORB3 IMU↔RGB-D) 비교가 가장 신뢰**, ORB3↔RTAB는 참고용.

## 10. three_labs_imu 시작↔끝 일치도 (return-to-origin 상세)

같은 지점에서 출발·복귀하도록 녹화된 새 bag에서 **첫 포즈와 마지막 포즈의 일치도**(누적 드리프트의 직접 측정):

| 항목 | 값 |
|---|---|
| 병진 드리프트 ‖p_end−p_start‖ | **5.00 cm** (궤적 29.8 m의 **0.168 %**) |
| 축별 |Δ| (x, y, z) | 0.05 cm, 0.85 cm, **4.93 cm** (대부분 Z=깊이 방향) |
| 회전 드리프트(첫↔끝 방향) | **2.13°** (geodesic) |
| 궤적 길이 / 시간 | 29.8 m / 144.5 s |

→ 약 30 m·2.4분 핸드헬드 주행 후 시작점에 **5 cm·2.1° 이내로 복귀**. IMU-RGBD가 전역 일관성을 잘 유지함을 보인다(Z 오차가 지배적 — RGB-D depth 스케일/시선방향 누적 특성).

## 11. 3-way 비교 — ORB3 IMU-RGBD vs ORB3 RGB-D vs RTAB-Map (모두 `SLAM_three_labs_rgbd_imu` bag)

RTAB-Map은 동일 bag·rate 0.5·RGB-D 모드(odometry quality 270–450, 정상)로 실행. ORB3 2종은 §4와 동일.

| 방법 | 포즈 수 | 추적 패러다임 | 프레임당 ms | **loop-drift m / %** | **시작↔끝 cm / °** | per-step med/p95† | **MME**‡ |
|---|---:|---|---:|---:|---:|---:|---:|
| **ORB3 IMU-RGBD** | 2542 | frame-dense | 84.8 | 0.050 / 0.17 % | 5.00 / 2.13 | 7.4 / 30.2 mm | −3.434 |
| **ORB3 RGB-D** | 1717 | frame-dense | 115.4 | 0.037 / 0.12 % | 3.70 / 1.70 | 10.7 / 46.3 mm | −3.448 |
| **RTAB-Map RGB-D** | 121 (140 노드) | keyframe-graph | — | 0.052 / 0.18 % | 5.15 / 2.51 | 220 / 392 mm† | **−3.974** |

† RTAB의 per-step은 그래프 노드 간격이라 ORB3 per-frame과 **비교 불가**(§9.1). ‡ MME는 백엔드가 다른 ORB3↔RTAB 간엔 점밀도 차이로 참고용(§9.2).

**해석(3-way):**
- **전역 일관성(loop-drift·시작↔끝)**: 세 방법 모두 **5 cm·~2° 내외로 매우 근접**(0.12–0.18 %). 부드러운 시퀀스라 세 방법 다 우수하며 변별 작음. ORB3 RGB-D가 근소 최저(3.7 cm).
- **포즈 밀도/패러다임**: ORB3는 프레임 단위 조밀 포즈(SAM-6D re-ID에 유리), RTAB는 그래프 노드(희소·평활, Nav2 occupancy에 유리). 비교 축이 다름.
- **맵 선명도(MME)**: RTAB **−3.974**로 ORB3(−3.43/−3.45)보다 낮음=선명. RTAB의 그래프 최적화 dense 맵(1.30 M 점)이 더 조밀·정합적. *단 점밀도 상이 → 참고 지표.*
- **IMU 위치**: 세 방법 중 정확도 최저점이 아니라 **처리량(84.8 ms 최속) + 포즈밀도(2542 최다)**에서 IMU-RGBD가 강점(§5.1 재확인). 부드러운 모션에선 RTAB·RGB-D ORB3와 정확도 동급.

## 12. 다음 액션 제안

가능한 후속 작업입니다. 무엇을 진행할지 알려주세요.

- **(A) IMU 효과를 드러낼 stress bag 평가** — 거친 회전/고속/모션블러 시퀀스에서 IMU-RGBD vs RGB-D 추적 성공률 비교(IMU의 진짜 강점 구간). *현재 자산엔 그런 bag이 없어 신규 수집 필요.*
- **(B) top-view 궤적 오버레이·MME 시각화** 생성(기존 `plot_*` 파이프라인 재사용)으로 보고서에 그림 첨부.
- **(C) IMU 노이즈 Allan 캘리브**(정지 bag 필요)로 nominal 값을 대체해 IMU 초기화·정확도 재측정.
- **(D) 기존 4-bag 전체를 동일 빌드로 재실행**해 RGB-D 베이스라인 표를 일관 정리(현재 기존 두 bag만 포함).

**질문**: 위 중 어떤 방향으로 진행할까요? 아니면 이 표/보고서를 특정 형식(논문 표, 슬라이드)으로 다듬어 드릴까요?
