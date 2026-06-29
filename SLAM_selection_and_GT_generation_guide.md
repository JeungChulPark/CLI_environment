# SLAM 선택 근거(성능 지표) & GT 생성 가이드

> **문서 목적**: ORB-SLAM3 / RTAB-Map / HDL-Graph-SLAM 중 하나를 선택하는 **정량 근거(5대 성능 지표)**와, 그 지표 산출에 필요한 **Ground Truth(GT) 궤적을 LI-Init + Fast-LIO2로 생성**하기 위한 **입력 데이터 수집 방법**을 한 문서로 정리한 자립형 레퍼런스. 다른 PC/작업자에게 그대로 전달해 작업을 재현하는 것이 목적이다.
>
> **작성일**: 2026-06-22 · **언어**: 한국어 · **출처 검증**: 본문 말미 참고문헌

---

## 0. 요약 (TL;DR)

- **비교 대상**: ORB-SLAM3, RTAB-Map, HDL-Graph-SLAM
- **선택 지표 5개**: ① ATE RMSE, ② RPE, ③ Tracking 성공률, ④ 연산비용(시간·CPU·RAM), ⑤ 맵 형태(정성)
- **GT 방법**: VLP-16 pointcloud + D455f IMU → **Fast-LIO2(오프라인, rosbag 재생)**로 기준 궤적 생성
- **선행 캘리브**: **LI-Init**으로 IMU↔LiDAR extrinsic + 시간오프셋 추정 (Fast-LIO2 정확도의 전제)
- **2번의 녹화**: (A) 캘리브용 = 격하게 흔들기, (B) GT/평가용 = 부드러운 다회 폐궤적
- **반드시 지킬 것**: 두 센서 **강체 고정**, HDL은 Fast-LIO2 GT로 채점 금지(편향), GT는 폐궤적 drift로 신뢰성 검증

---

## 1. 시스템 배경 & 제약

### 1.1 상위 설계도(맥락)
1. SLAM 추정 카메라 포즈 → SAM-6D 객체 포즈의 프레임 간 동일 ID 인식에 활용
2. 로봇 위치 + 객체 포즈 → Unity 3DGS 3D Map에 표현
3. Unity 사용자 상호작용으로 목표 지점 설정 → ROS Navigation 자율주행

→ SLAM 포즈가 **(A) 객체 re-ID, (B) Unity 전역 정합, (C) 내비게이션** 3곳에 쓰이므로, 지표는 이 용도들에 매핑되어야 한다.

### 1.2 센서
| 센서 | 모델 | 용도 |
|---|---|---|
| RGB-D | Intel RealSense **D455f** (RGB + Depth + IMU) | ORB-SLAM3·RTAB-Map 입력, Fast-LIO2의 IMU 공급원 |
| LiDAR | Velodyne **VLP-16** (16ch, 360°) | HDL-Graph-SLAM 입력, Fast-LIO2의 pointcloud 공급원 |

### 1.3 제약 조건 (중요)
- **환경**: 실내 → GNSS/RTK-GPS GT 불가
- **GT 기준 장비 전무**: 모션캡처(Vicon/OptiTrack) ❌ / Total Station ❌ / 고정밀 추가 LiDAR ❌
- **공간 크기**: 약 **가로 4 m × 세로 1.4 m**(가로 6걸음 × 세로 2걸음 수준)의 소규모 실내
- **수집 방식**: 센서를 부착한 **삼각대(강체 모듈)를 손으로 들고** 이동
- **단계**: ① Husky 무탑재(handheld) → ② Husky 탑재(추후)

---

## 2. SLAM 선택을 위한 5대 성능 지표

| # | 지표 | 정의 | 왜 중요(다운스트림) | GT 필요? | 측정 도구 |
|---|---|---|---|---|---|
| 1 | **ATE RMSE** | 정렬 후 전체 궤적의 절대 위치 오차(RMSE) | 전역 일관성 → **Unity 3DGS 정합** | ✅ 필요 | EVO `evo_ape` |
| 2 | **RPE** | 고정 구간의 상대 운동 오차(드리프트) | 국소 정확도 → **SAM-6D 객체 re-ID의 핵심 전제** | ✅ 필요(또는 상대 일관성) | EVO `evo_rpe` |
| 3 | **Tracking 성공률** | 추적 유지 비율 / loss 횟수 | 강건성 → 내비 중 위치 끊김 방지 | ❌ (로그 기반) | SLAM 로그 |
| 4 | **연산비용** | 프레임당 처리시간(ms), CPU%, 메모리(RSS) | 실시간성·온보드 적합성(Husky) | ❌ (프로파일링) | `htop`/`psutil`, 내부 타이머 |
| 5 | **맵 형태** | 맵 종류·품질(정성) + (권장)무참조 정량 | 내비(occupancy)·시각화 적합성 | ❌ | 시각 검토 + (권장)MME |

### 2.1 지표별 보충
- **ATE vs RPE**: ATE는 전역(맵 정확도), RPE는 국소(드리프트·loop closure 정확도). 본 과제의 객체 re-ID는 프레임 간 상대 포즈 일관성이 결정적이므로 **RPE가 1차 지표**, ATE는 전역 정합 보조.
- **지표 4 — "GPU"는 빼라**: ORB-SLAM3·RTAB-Map·HDL-Graph-SLAM은 **모두 CPU 기반**이라 GPU 사용량은 변별력이 없다(GPU 부담은 SAM-6D 쪽). **프레임당 시간 + CPU% + 메모리**로 측정하고, 반드시 **동일 하드웨어·동일 bag**으로 3종을 비교한다.
- **지표 5 — 정성만은 약함**: 가능하면 무참조(No-Reference) 정량지표 **MME(Mean Map Entropy)** 를 추가하면(맵 선명도=일관성, GT 불필요) 방어력이 올라간다.
- **(선택) 권장 보조지표 — Task 기반**: 본 시스템의 진짜 목적인 **객체 re-ID 일관성**(프레임 간 동일 ID 유지율)이나 **내비 목표 도달 성공률**은 GT 없이도 "왜 *우리 시스템*에 이 SLAM이 최선인가"를 직접 보여주는 강력한 근거다.

### 2.2 평가 표준 포맷·도구
- 궤적 포맷: **TUM**(`timestamp tx ty tz qx qy qz qw`)을 표준으로 사용(실내 RGB-D 관례). SLAM 출력 → TUM 변환 → EVO 투입.
- 도구: **EVO** (`evo_ape`, `evo_rpe`, `evo_traj`) — ATE/RPE 계산, 궤적 정렬(Umeyama), 타임스탬프 보간 동기화, 시각화 지원.

### 2.3 평가 모드 원칙 — online(선택) / localization(배포검증) 2단계
SLAM은 **online(맵을 만들며 동시에 추적)** 과 **localization(완성된 고정 맵에 위치만 정합)** 두 동작 모드가 있고, **평가는 실제 배포 모드를 따라가야** 충실하다. 본 시스템은 둘 다의 성격을 가진다(SLAM 포즈를 *실시간*으로 SAM-6D에 공급 = online / 3DGS 맵은 *미리 제작한 고정 맵* = localization). 따라서 **두 모드를 동시 비교하지 말고 단계로 분리**한다.

| | **1단계: online 3-way 비교** | **2단계: localization 1개 검증** |
|---|---|---|
| 목적 | "어떤 SLAM을 고를까" (핵심 질문) | "고른 SLAM이 고정맵에 안정 정합하나" (배포 타당성) |
| 시점 | 지금(handheld, SLAM 선택) | 추후(Husky + 고정 3DGS 맵 배포) |
| 대상 | ORB3·RTAB·HDL **3종** | 1단계 **승자 1개만** |
| 편향 | 없음(셋 다 native online·공정) | 없음(승자 자기 맵 사용 → "누구 맵?" 순환편향 회피) |

- **online이 우리 목적의 1차 기준인 이유**: 실제 파이프라인이 *맵을 만들며 실시간 포즈를 넘기는* online이므로, 평가도 그 동작 모드 그대로 **맵 생성+추적 강건성**을 봐야 한다. **Tracking 성공률(지표 3)은 online 모드 측정값**이며, "맵이 깨질 것을 전제"하는 게 아니라 *깨지는지 여부 자체*를 측정한다(거친 모션에서 누가 추적을 유지/상실하는지가 결과).
- **각 bag은 독립 실행**: bag마다 SLAM을 빈 맵에서 새로 돌리므로 **한 bag의 실패가 다른 bag으로 전파되지 않는다**(맵·궤적은 한 실행의 두 산출물이지, 나쁜 맵을 다른 데이터에 재사용하는 구조가 아님). 단 *한 bag 내부*에서 추적이 끊기면 그 이후 프레임 추정은 오염되는데, 이것이 바로 Tracking 성공률·RPE로 정량화하려는 대상이다.
- **하지 말 것**: 지금 3종을 localization으로 풀 비교(도구 비대칭 ORB3/RTAB=mode vs HDL=별도 `hdl_localization`, "누구 맵?" 편향, 실험 ~2배). 얻는 것 대비 비용이 안 맞는다.
- **논문 문장 예**: *"SLAM 선택은 online SLAM 모드에서 ATE/RPE/Tracking률/MME로 수행했고, 선정된 X-SLAM에 대해 배포 환경(고정 3DGS 맵)을 모사한 localization 모드에서 재정합 안정성을 추가 검증했다."*

---

## 3. GT 생성 전략 (왜 Fast-LIO2인가)

### 3.1 방법 선정 근거
- 외부 GT 장비가 없으므로 **보유 센서로 pseudo-GT(유사 정답)** 를 만든다.
- **Fast-LIO2**(LiDAR-Inertial Odometry): VLP-16 + IMU만으로 동작, 소규모 실내에서 **MOCAP 대비 ~3 cm** 정확도 보고. 비교 대상 3종에 **포함되지 않는 외부 기법**이라 심판 자격이 있다.
- **왜 Fast-LIVO2(카메라 포함)가 아니라 Fast-LIO2인가**: GT에 카메라를 쓰면 평가 대상인 ORB-SLAM3·RTAB-Map(둘 다 카메라 기반)과 **센서·실패모드를 공유**해 오차 상관(편향)이 생긴다. 카메라를 쓰지 않는 **Fast-LIO2가 시각 SLAM 평가에 더 독립적**이고, 캘리브·동기 부담도 적다.

### 3.2 ⚠️ 치명적 주의 — HDL-Graph-SLAM 편향
- Fast-LIO2 GT의 입력 = VLP-16. HDL-Graph-SLAM 입력도 = **같은 VLP-16**. → GT와 평가대상이 **같은 센서·원시데이터·실패모드**를 공유 → **순환논리/편향**. HDL의 ATE/RPE가 실제보다 좋게 나온다.
- **금지**: 평가 대상(HDL) 자신의 odometry를 GT로 쓰는 것(자동 우승, 무의미).
- **해법**: HDL은 Fast-LIO2 GT로 채점하지 말고 **독립 기준**으로 교차검증:
  - **loop-drift**(아래 3.3) — 센서 무관 자기일관성
  - 필요 시 **AprilTag 소수 제어점**(카메라 기반, LiDAR와 독립)

### 3.3 GT 신뢰성 검증 (필수) — loop-drift / return-to-origin
- **정의**: 출발점 A에서 한 바퀴 돌아 **물리적으로 A로 정확히 복귀**. 완벽한 SLAM이면 추정 "시작 포즈 = 끝 포즈". 실제 어긋남 = **누적 드리프트**.
- **계산**: `drift = ‖p_end − p_start‖`, `drift(%) = drift / 총 궤적 길이 × 100`
- **용도 2가지(같은 측정)**: ① Fast-LIO2 자신의 drift → GT 오차 수치화("11 m당 ±X cm"), ② HDL 자신의 drift → LiDAR GT에 의존하지 않는 HDL 독립 평가.
- **합격 기준**: **GT 오차 ≪ 후보 간 차이** 이면 순위 결론 유효. 논문에는 GT를 "reference trajectory"로 부르고 그 오차 한계를 명시.

### 3.4 ❓ "Fast-LIO2가 GT급이면 그냥 그걸 배포 SLAM으로 쓰면 되지 않나?"
자주 나오는 질문(심사·발표 단골). 답: **"GT급"은 *짧고 검증된 폐궤적의 국소 정확도*라는 한정 주장이지, 시스템 최적이 아니다.** Fast-LIO2는 평가용 *reference*로만 쓰고 배포 SLAM 후보로는 별개로 본다. 이유 5가지:
1. **한정된 보증**: GT 자격은 loop-drift로 *검증된 그 bag*에서 ±N cm일 뿐. Fast-LIO2는 **odometry(LIO)라 기본 버전에 loop closure·전역최적화가 없어** 긴 주행에선 드리프트가 누적됨. "짧은 bag의 GT" ≠ "긴 배포 최적"(RTAB/HDL은 전역 일관성 설계).
2. **센서 강제**: Fast-LIO2 = **VLP-16 필수**. 본 비교의 목적 중 하나가 "카메라만(D455)으로 충분한가 → LiDAR를 빼서 비용·무게·전력 절감". LIO를 쓰면 이 선택지를 포기.
3. **맵 형태 불일치**: Fast-LIO2 맵 = **무색 LiDAR 포인트클라우드**. 본 파이프라인은 **3DGS(색·dense)·occupancy**가 필요 → RTAB(RGB-D dense)·ORB3(특징맵)이 더 맞음(지표 5의 존재 이유).
4. **배포 2단계 미지원**: 기본 Fast-LIO2는 **relocalization/고정맵 재정합이 없음** → MD 2.3절의 localization(배포검증) 모드를 못 함. ORB3·RTAB는 내장.
5. **다기준 결정**: SLAM 선택은 5지표(정확도·전역일관성·센서·맵형태·연산)의 절충. 정확도 한 축만 보고 LIO를 고르는 건 나머지 4축에서 시스템 요구와 어긋남.

> **논리적으로 주의**: "GT급이니 최선"은 *검증된 구간의 국소 정확도*를 *모든 배포 상황*으로 외삽하는 순환오류다. 다만 **LiDAR를 영구 탑재 + 포인트맵으로 충분**한 설계라면, loop closure 있는 LIO(예: LIO-SAM, FAST-LIO-SAM)를 **후보에 정식 추가**해 비교하는 것은 정당하다.

---

## 4. 캘리브레이션 — LI-Init (Fast-LIO2의 전제)

> Fast-LIO2는 IMU와 LiDAR가 **강결합·동기**되어 있다고 가정한다. D455f IMU와 VLP-16은 별도 장치(다른 클럭)이므로, **extrinsic + 시간오프셋을 반드시 보정**해야 GT가 신뢰된다. 이를 자동으로 푸는 도구가 **LI-Init**(HKU-MARS, Fast-LIO2와 같은 연구실).

### 4.1 왜 "가까운 데이터끼리 묶기"로는 부족한가
- **GT 생성 단계(IMU+LiDAR 융합)**: 근사동기 부족. 문헌상 LIO 내부 근사동기 = 11.36 m에서 drift 0.246 m vs LI-Init 시간보정 = 0.0102 m (**약 24배 차이**). → **반드시 LI-Init로 정식 보정.**
- **평가 단계(추정 궤적 vs GT 궤적 비교, EVO)**: 근사동기/보간으로 충분(EVO가 처리). → 여기서만 "가까운 시각끼리 묶기"가 정답.

### 4.2 강체 모듈 (필수 조건)
- D455f + VLP-16을 **하나의 단단한 판/프레임에 볼트로 고정**해 한 덩어리로 만든다.
- 접촉·근접은 불필요. **상대 움직임 0**만 필수(느슨한 케이블타이 ❌, 진동·유격이 캘리브·LIO를 망침).
- extrinsic은 그 고정 상태에서만 유효 → **handheld·Husky 두 단계 내내 모듈 내부 결합을 풀지 말 것**(그러면 캘리브 1회로 양 단계 커버).

### 4.3 D455f IMU 준비 (실전 함정)
- D455는 **자이로(200/400 Hz)와 가속도계(63/250 Hz)가 별도 스트림·다른 주파수**.
- LI-Init/Fast-LIO2는 **단일 `/imu` 토픽**을 기대 → `realsense-ros`의 **`unite_imu_method:=linear_interpolation`** 로 합쳐 ~200 Hz 단일 IMU 토픽 생성.
- IMU 출력 활성화(`enable_gyro:=true enable_accel:=true`).

### 4.4 LI-Init 핵심 파라미터
| 파라미터 | 값/권장 | 의미 |
|---|---|---|
| `mean_acc_norm` | **9.805** | 일반 IMU(D455). Livox만 1.0 |
| `data_accum_length` | 기본값↑ 권장 | 초기화 충분성 임계(작으면 품질 저하) |
| `online_refine_time` | **15~30 s** | Fast-LIO2로 extrinsic 미세조정 시간 |
| LiDAR type | **Velodyne** | VLP-16 |
| topic | IMU/LiDAR 토픽명 | 환경에 맞게 |

### 4.5 난이도 메모
- **최대 마찰점**: LI-Init 원본은 **ROS1(catkin)**. 작업 환경이 ROS2(humble)이면 **ROS1 도커/환경에서 실행**하거나 bag을 변환해야 한다.
- 알고리즘 자체는 자동 — 충분히 흔들면 `result/Initialization_result.txt`에 extrinsic·time offset 출력.

---

## 5. 데이터 수집 방법 (2번의 별도 녹화)

> **혼동 금지**: 캘리브(A)는 *격하게 흔들기*, GT/평가(B)는 *부드럽게 경로 주행*. 목적이 정반대다.

### 5.1 녹화 A — 캘리브용 (LI-Init 입력)
- **목표**: extrinsic + 시간오프셋 관측성 확보
- **동작**: 강체 모듈을 들고 **roll/pitch/yaw 3축 회전을 적극적으로 휘젓기** + 앞뒤좌우 병진 약간. "천천히 한 축만"은 실패.
- **시간**: **30 s ~ 1 분** (초기 축적 + `online_refine_time` 포함)
- **공간**: **제자리 회전으로 충분** → 4×1.4 m 소공간에서 전혀 문제없음
- **주의**: 너무 빠르면 소비자급 IMU 포화 → "격하지만 제어된" 수준

### 5.2 녹화 B — GT/평가용 (Fast-LIO2 + 3종 SLAM 입력)
- **목표**: 동일 bag으로 GT와 3종 SLAM 궤적을 모두 산출
- **동작 원칙**:
  1. **부드러운 등속 이동**(급회전·급정지 금지 → de-skew 오차↓). *흔들지 않는다.*
  2. **폐궤적 + return-to-origin**(시작점 바닥 표시 후 정확히 복귀) → GT drift 검증
  3. **다회 반복(예: 3랩 이상)** → 소공간은 1랩 drift가 너무 작아 변별 불가 → 오차 누적 필요
  4. **뷰포인트 다양화** → 곡선/루프 경로로 진행 방향(heading)을 다양하게 (자동 충족)
  5. **VLP-16 최소거리(~0.4 m) 이상** 유지
- **공간 메모**: 사방 벽이 가까운 밀폐 소공간은 평면 제약이 풍부해 **LiDAR에 오히려 유리**(퇴화는 긴 복도/개활지 문제, 본 환경 아님).

#### 5.2.1 자주 묻는 3가지 (원칙 2~4 정확한 해석)

**(a) 매 바퀴마다 정확히 출발점에 도착해야 하나?** → **아니다. 시작과 "최종" 끝, 두 지점만 정확하면 된다.**
- 중간 랩들은 **같은 영역을 다시 지나가기만** 하면 loop closure가 유발되고 drift가 누적된다(정확한 점 복귀 불필요).
- return-to-origin drift 공식 `‖p_end − p_start‖`은 *첫 포즈 = 마지막 포즈*를 가정 → **시작 시점과 최종 종료 시점**만 물리적으로 동일 지점·동일 방향이어야 한다.
- 실무: 바닥에 출발 **위치 + 방향(화살표)**을 테이프로 표시 → 시작은 그 위에서, 마지막은 **위치·방향 모두** 그 위로 복귀(방향까지 맞춰야 회전 drift도 측정).
- (선택) 매 랩 정밀 복귀 시 "랩별 drift 증가 곡선"을 얻지만 수고가 큼 → 보통 시작/최종만 정밀.

**(b) "벽 각도를 바꿔가며"가 무슨 뜻?** → **경로 모양 자체가 센서를 여러 방향으로 향하게 하라(단, 부드럽게).**
- 카메라는 강체 모듈에 고정 → "벽 향한 각도" = **모듈(로봇)의 진행 방향(heading)**. **곡선/루프 경로면 heading이 자연히 360° 변하므로 자동 충족**, 별도 조작 불필요.
- ❌ 일직선 주행하며 카메라 빙빙 돌리기(모션블러+de-skew 오차) / ❌ 랩마다·녹화마다 변경하라는 의미가 아님.
- 의도: "한쪽 벽만 한 각도로 계속 보는" 빈약한 궤적을 피하라. 곡선 경로면 해결, 직선 왕복만 한다면 그때만 패스별 heading을 달리.

**(c) 같은 루프 반복 외 다른 주행 패턴은?** → 있다. **패턴을 "지표"에 맞춰 나눠라.**

| 주행 패턴 | 주로 검증 | GT 품질 |
|---|---|---|
| 단일 루프 | 기본 정확도 | ✅ 좋음 |
| 다중 랩(three_laps) | 누적 drift, loop closure 반복 | ✅ 좋음 |
| 직선 왕복(back_and_forth) | 반대 방향 재방문, 순수 병진 | ✅ 좋음 |
| Figure-8(8자) | 빈번한 heading 변화 + 2개 교차점 loop closure | ✅ 좋음 |
| 나선/안팎(spiral) | 벽과의 거리 변화, 스케일 | ✅ 좋음 |
| CW/CCW 양방향 | 방향 의존성 | ✅ 좋음 |
| Stop-and-go(정지 포함) | 정지 중 drift, IMU bias | ✅ 좋음 |
| 순수 제자리 회전 | 회전-only 추적(대표적 실패모드) | ⚠️ GT 나쁨 |
| 빠른 속도/급기동 | 모션블러 내성 | ⚠️ GT 나쁨 |

> **핵심 구분 — 패턴을 지표에 매칭:** Fast-LIO2 GT는 *부드러운 운동*에서만 정확하다.
> - **GT 품질 런(부드러움)** → ATE/RPE용: 단일/다중 루프, 8자, 직선 왕복, 나선
> - **스트레스 런(거침/회전/고속)** → **Tracking 성공률(지표 3)**용: 순수 회전, 고속, 급기동. 이 런은 **정확한 GT가 불필요**(추적 끊김 여부만 보므로 GT 품질 저하 무관).
> 즉 부드러운 다양한 경로로 ATE/RPE를, 거친 경로로 robustness를 따로 수집하면 5대 지표를 효율적으로 모두 충족.

### 5.3 기록 토픽 (ros2 bag)
- LiDAR: VLP-16 pointcloud(`velodyne_points`)
- IMU: D455 통합 IMU(`/camera/imu`, `unite_imu_method` 적용)
- 카메라(3종 SLAM용): RGB(`/camera/color/image_raw`), Depth(정렬), CameraInfo
- 기록: `ros2 bag record <필요 토픽들>` (또는 `-a` 전체)

---

## 6. 처리 파이프라인 (순서)

```
[캘리브]  녹화 A(bag) ── LI-Init ──▶ extrinsic + time offset
                                         │
                                         ▼ (Fast-LIO2 config에 입력)
[GT 생성] 녹화 B(bag) ── Fast-LIO2(오프라인 재생) ──▶ GT 궤적(TUM)
[후보]    녹화 B(bag) ── ORB-SLAM3 / RTAB-Map / HDL ──▶ 추정 궤적(TUM)
[평가]    EVO: evo_ape / evo_rpe (GT vs 각 추정)  +  loop-drift 검증
                                                  +  연산비용 프로파일링
                                                  +  맵 정성(+MME)
```

### 6.1 대표 명령 (환경 따라 조정)
```bash
# Velodyne 네트워크(공장 기본): LiDAR IP 192.168.1.201, PC IP 192.168.1.100/24

# FAST-LIO2 (ROS2 포트 예: Ericsii/FAST_LIO_ROS2)
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git --recursive
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
ros2 launch fast_lio mapping.launch.py config_file:=velodyne.yaml   # config에 LI-Init 결과 반영

# EVO 평가
evo_ape tum gt.txt orbslam3.txt -as            # ATE RMSE (정렬 후)
evo_rpe tum gt.txt orbslam3.txt --delta 1 --delta_unit m   # RPE (구간 드리프트)
```
> ⚠️ HDL 평가 시 위 GT는 편향 가능 → loop-drift(및 필요 시 AprilTag)로 교차검증.

---

## 7. 리스크 & 한계 (정직한 서술)

| 리스크 | 영향 | 대응 |
|---|---|---|
| HDL ↔ Fast-LIO2 GT 센서 공유 | HDL ATE/RPE 편향 | loop-drift/AprilTag 독립 검증, 한계 명시 |
| D455 IMU + VLP-16 시간/extrinsic 미보정 | GT가 조용히 오염 | **LI-Init 필수**, 강체 고정 |
| GT 정확도 미검증 | "GT가 정답인가" 반박 | 폐궤적 drift로 GT 오차 한정 |
| **소공간(4×1.4 m)** | 절대 drift가 GT 오차에 묻혀 ATE 변별 불가 | 다회 랩으로 누적, RPE·강건성·연산 위주, "소공간 동등"도 결론, Husky 단계서 넓은 경로 |
| LI-Init ROS1 의존 | 환경 마찰 | ROS1 도커/bag 변환 |
| D455 자이로/가속도 분리 스트림 | IMU 토픽 누락 | `unite_imu_method` 설정 |

---

## 8. 실행 체크리스트

- [ ] D455f + VLP-16 **강체 모듈** 조립(볼트 고정, 유격 0)
- [ ] D455 IMU 활성화 + `unite_imu_method:=linear_interpolation`(~200 Hz 단일 IMU)
- [ ] Velodyne 네트워크 설정(IP) + ROS2 드라이버 구동 확인
- [ ] **녹화 A(캘리브)**: 제자리 3축 격하게 30 s~1 분 → bag
- [ ] LI-Init 실행(ROS1 환경, `mean_acc_norm=9.805`) → extrinsic + time offset 획득
- [ ] Fast-LIO2 config에 LI-Init 결과 반영
- [ ] **녹화 B(GT/평가)**: 4×1.4 m 부드럽게 다회 폐궤적(return-to-origin) → bag
- [ ] Fast-LIO2(오프라인 재생)로 GT 궤적(TUM) 생성
- [ ] 3종 SLAM을 동일 bag으로 실행 → 각 추정 궤적(TUM)
- [ ] EVO로 ATE/RPE(시각 SLAM은 GT, HDL은 독립 검증)
- [ ] GT loop-drift 측정 → GT 오차 한정
- [ ] 연산비용(시간·CPU·RAM) 동일 하드웨어 프로파일링
- [ ] 맵 정성(+MME) 평가 → 5대 지표 표 완성

---

## 9. 도구 & 저장소

| 용도 | 도구 | 링크 |
|---|---|---|
| LiDAR-IMU extrinsic+시간보정 | **LI-Init** | https://github.com/hku-mars/LiDAR_IMU_Init |
| GT 궤적 생성(LIO) | **FAST-LIO2 (ROS2 포트)** | https://github.com/Ericsii/FAST_LIO_ROS2 |
| 궤적 평가(ATE/RPE) | **EVO** | https://github.com/MichaelGrupp/evo |
| (선택) LiDAR-Camera extrinsic | direct_visual_lidar_calibration | https://github.com/koide3/direct_visual_lidar_calibration |
| (선택) IMU-Camera 캘리브 | Kalibr | https://github.com/ethz-asl/kalibr |
| (선택) 맵 품질(MME) | MapEval | https://github.com/JokerJohn/Cloud_Map_Evaluation |

---

## 10. 참고문헌 (출처 검증)

- FAST-LIO2: *Fast Direct LiDAR-Inertial Odometry* — arXiv 2107.06829
- LI-Init: *Robust Real-time LiDAR-inertial Initialization* (IROS 2022) — arXiv 2202.11006
- FAST-LIVO2: *Fast, Direct LiDAR-Inertial-Visual Odometry* (IEEE T-RO 2024) — arXiv 2408.14035
- 실내 GT(무 MOCAP): *Challenges of Indoor SLAM* — arXiv 2306.08522
- AprilTag GT: *TagSLAM* — arXiv 1910.00679
- GT-free 평가: *Look Ma, No Ground Truth!* — arXiv 2412.01116 / *MapEval* — arXiv 2411.17928
- Task 기반 평가: *Task-driven SLAM Benchmarking* — arXiv 2409.16573
- LiDAR-Camera 캘리브: Koide et al. *General, Single-shot, Target-less LiDAR-Camera Calibration* (ICRA 2023) — arXiv 2302.05094
- EVO 문서: https://michaelgrupp.github.io/evo/

---

*끝. 본 문서는 단독으로 작업 재현이 가능하도록 작성됨. 환경(ROS 배포판·토픽명·IP)은 대상 PC에 맞게 조정할 것.*
