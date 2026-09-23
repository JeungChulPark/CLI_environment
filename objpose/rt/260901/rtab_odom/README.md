# RTAB-Map — 오도메트리 소스와 링크 공분산 (260901_cbnu_bigeightcircle, 2026-09-23)

RTAB-Map의 매퍼는 그대로 두고 **오도메트리만 갈아끼웠을 때** 최종 정확도가 얼마나 달라지는지 잰
기록이다. 답을 찾는 과정에서 훨씬 큰 것이 나왔다 — `Rtabmap::process`의 **오도메트리 링크 공분산
기본값(단위행렬)이 오도메트리 개선을 통째로 삼키고 있었다.**

리포트: `html/260923_rtabmap_odometry_covariance.html` (자체 완결 HTML, 공분산·궤적 토글)

| 항목 | 위치 |
|---|---|
| IMU 주입 | `objpose/src/imu_source.h` — 디바이스 쿼터니언 → `SensorData::setIMU`, 공분산은 rig json에서 유도, `gravity_report()` 자가검증 |
| 외부 오도메트리 주입 | `objpose/src/rtab_aux.h` (`ExtOdom`) — TUM 궤적 → `Rtabmap::process(data, odom)`, VO 미실행 |
| 공분산 전달 | `objpose/src/rtab_stream.cc` — `rtabmap_ros CoreWrapper.cpp:1152-1178` 복제 |
| 실행·채점 | `run_compare.sh` (6회 실행 + `eval_traj.py` + `mapper_delta.py` + 리포트) |
| 순환 아닌 지표 | `mapper_delta.py` — 매퍼가 입력 궤적을 얼마나 밀어냈는지 |
| 원자료 | `covdefault/`, `covreal/` (`*.rtabmap.db`는 각 130 MB라 제외) |

## 1. 설계

같은 세션(9445 프레임, 29.98 Hz), **같은 매퍼 설정**, 두 축만 바꾼다. 셋 다 unpaced offline이라
매퍼가 데이터 시계 1 Hz로 동기 실행되고 결과가 결정론적이다 — 실시간 거동이 아니라 오도메트리
품질을 비교하는 실험이다. 기준은 VLP-16 KISS-ICP(경로 66.3 m, 315 s).

**오도메트리 소스 3종**

- `base` — RTAB-Map 기본 VO (`Odom/Strategy 0` = F2M, GFTT/BRIEF, 로컬 워드맵 2000)
- `imu` — 같은 VO + xsens MTi 자세. 스톡 RTAB-Map이 이를 `OdometryF2M`의 번들 조정 제약과
  노드별 `Link::kGravity`(`Optimizer/GravitySigma` 0.3)로 쓴다. ORB-SLAM3의 `gyro_aid.h`와 달리
  라이브러리를 패치하지 않는다.
- `ext` — VO를 끄고 KISS-ICP 궤적을 오도메트리로 주입. 카메라는 여전히 매퍼가 본다.

**링크 공분산 2종**

- `covdefault` — `Rtabmap::process`의 기본값 = 단위행렬 (σ 1 m / 1 rad)
- `covreal` — 오도메트리가 보고하는 값. VO는 `OdometryInfo::reg.covariance`,
  `ext`는 고정값(σ 1 cm / 0.001 rad, `--ext-cov`로 변경 가능)

## 2. 드러난 문제

`covdefault`에서 **최적화 궤적이 라이브 궤적보다 나빴다.** 매퍼가 좋은 오도메트리를 망가뜨린다는
뜻이고, `ext`에서는 주입한 KISS 궤적을 중앙값 11 cm, 최대 28 cm 밀어냈다.

원인은 한 줄이다:

```cpp
bool process(const SensorData & data, Transform odomPose,
             const cv::Mat & odomCovariance = cv::Mat::eye(6,6,CV_64FC1),   // σ = 1 m / 1 rad
             ...);
```

VO가 스스로 보고하는 링크 정확도는 **σ ≈ 2.4 cm**다. 40배 이상 헐겁게 들어가고 있었다. 결과는 둘:

1. 루프 링크는 `RegistrationVis`의 PnP 공분산(훨씬 작음)을 가지므로 **루프가 그래프를 지배**한다.
   오도메트리를 개선해도 그래프는 그 사실을 모른다.
2. `RGBD/OptimizeMaxError = 3.0`은 "잔차 ÷ 각 링크의 표준편차"를 보는데 분모가 1 m이면 어떤 루프도
   3σ를 못 넘는다 — **거짓 루프 기각이 사실상 꺼진다.**

## 3. 고친 방법

`rtabmap_ros`의 `CoreWrapper.cpp:1152-1178`을 그대로 복제했다: 매퍼 업데이트 사이 프레임들의
**최댓값**을 유지하고(*"Use largest covariance error (to be independent of the odometry frame rate)"*),
업데이트 후 리셋한다. `OdometryF2M`이 초기화 시 내는 `9999`
(*"a very high variance tells that the new pose is not linked with the previous one"*)는 그대로
통과시킨다 — 실행당 1회만 나온다.

`--no-odom-cov`로 예전(기본값) 동작을 재현할 수 있다.

## 4. 읽는 법

- **`ext`의 ATE는 순환이다** — 주입한 KISS-ICP가 곧 채점 기준이다. 정확도의 증거로 읽으면 안 된다.
  순환이 아닌 값은 `mapper_delta.py`가 내는 **입력 궤적 대비 이동량**이고, 그것이
  "매퍼의 결정이 좋은 오도메트리를 해칠 수 있는가"에 대한 답이다.
- `base` vs `imu`는 KISS가 독립 기준이므로 정상적으로 비교된다.
- **IMU 좌표계는 실행마다 자가검증된다.** `imu_source.h`의 `gravity_report()`가 정지 구간 중력을
  base 프레임에서 재고 `(0, 0, +9.8)`이 나와야 한다 — 260901 실측
  `imu (0.523 -0.051 -9.781) -> base (0.533 -0.041 9.781)`. 이게 틀리면 조용히 틀린다:
  중력이 옆을 향하고 중력 제약이 엉뚱하게 당겨서 "IMU가 오히려 나쁘다"처럼 보인다.

## 5. 결론

- **매퍼의 가중치가 틀려 있으면 오도메트리 개선이 최종 결과에 거의 도달하지 못한다.** 공분산 한
  줄을 고친 효과가 IMU를 얹은 효과보다 컸다.
- **가중치가 맞으면 오도메트리 품질이 그대로 최종 정확도로 이어진다.** 라이다 오도메트리의 우위가
  비로소 결과에 나타난다.
- 즉 **오도메트리 정밀도가 상한을 정하고, 매퍼 가중치가 그 상한에 도달할지를 정한다.** 순서는
  가중치가 먼저다 — 틀린 가중치 위에서는 오도메트리 개선의 효과를 측정조차 할 수 없다.
- **루프 횟수는 오도메트리 품질과 거의 무관하다** — 외형(BoW + 베이즈 필터) 기반 탐지이기 때문이다.
  공분산을 고치면 `ext`의 루프가 줄어드는데, 그것이 `OptimizeMaxError`의 거짓 루프 기각이 되살아난
  증거다.
- **VO가 런타임의 대부분을 쓴다.** VO를 끈 `ext`가 압도적으로 빠르다. 라이다 오도메트리가 따로
  있는 시스템이라면 카메라를 매퍼 전용으로 두는 것만으로 CPU가 크게 절약된다.

## 6. 실행 방법

```
# 6회 실행 + 채점 + 리포트 (호스트에서)
bash objpose/rt/260901/rtab_odom/run_compare.sh /path/to/260901_cbnu_bigeightcircle

# 리포트만 다시 만들기
python3 objpose/rt/260901/rtab_odom/make_report.py

# 개별 실행
~/objpose/run_slam.sh --slam rtabmap --session <SLAM dir> --mode offline --out out.txt          # base
~/objpose/run_slam.sh --slam rtabmap --session <SLAM dir> --mode offline --gyro <rig.json> ...  # imu
~/objpose/run_slam.sh --slam rtabmap --session <SLAM dir> --mode offline \
    --rig <rig.json> --ext-odom kiss/kiss_tum.txt ...                                           # ext
```

## 7. 남은 것

- `--ext-cov`의 기본값(σ 1 cm)은 KISS-ICP의 귀환 오차 15 cm / 66 m(0.23 %)에서 잡은 **보수적 가정**이지
  측정값이 아니다. 라이다 오도메트리가 자기 공분산을 내도록 하는 게 맞다.
- 이 실험은 unpaced offline이다. 실시간(`--pace 1.0`)에서 같은 결론이 나오는지는 확인하지 않았다.
- 한 세션(bigeightcircle)만 돌렸다. 나머지 세 세션(eightcircle, longcircle, digut)은 미실행.
