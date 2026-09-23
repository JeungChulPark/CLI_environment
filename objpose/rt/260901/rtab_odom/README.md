# RTAB-Map — 오도메트리 소스, 링크 공분산, 루프 제약 (260901_cbnu 4세션, 2026-09-23~24)

§1~7 은 bigeightcircle 한 세션의 기록이고, §8 이후가 네 세션으로 확장하면서 그 결론 대부분을
수정한다. §8 을 먼저 읽을 것.

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
- ~~실시간에서 같은 결론이 나오는지 미확인~~ → §11 에서 1.0× 실시간 3종 비교.
- ~~한 세션(bigeightcircle)만 돌렸다~~ → §8 에서 네 세션 전부 실행. 결론이 크게 바뀐다.

## 8. 네 세션으로 확장 (2026-09-24)

위 §5 의 결론은 bigeightcircle 한 세션에서 나온 것이다. 260901_cbnu 네 세션 전부에 같은 비교를
돌리자 그중 하나만 살아남았다.

실행: `run_sessions.sh`(세션마다 4 레인) · 채점: `compare_sessions.py` · 원자료 `sessions/`

**세션마다 자기 rig 를 쓴다.** T_lidar_cam 과 R_cam_imu 는 네 세션이 같다고 봐도 되지만(차이가 각
적합의 잔차와 같은 크기) **시간 오프셋은 녹화마다 실제로 다르다** — lidar_minus_cam 이
+0.706 / −0.344 / −5.005 / −3.045 ms 로 5.7 ms 에 걸쳐 흩어지고 상관은 모두 0.999 이상이다.
`calib_sessions.sh` 가 세션별로 다시 보정한다(`rigs/`). 부트스트랩 카메라 궤적은 자이로를 쓰지 않은
ORB-SLAM3 실행이라 순환이 없고, `--static-s` 는 세션마다 주행 시작 시각에 맞춰 다르게 준다.

수직 레버암은 rig 에서 쓰지 않는다. calib_rig.py 가 0 으로 정규화하는 값이라 세션마다 −487 /
−1226 / −744 / −1592 mm 로 제각각 나오는데 전부 정규화 기준점 차이일 뿐이다. `--lidar-z -0.45` 로
실측 기하(라이다 0.59 m, SLAM 카메라 1.04 m)를 직접 준다. bigeightcircle 에서 이 보정만으로 ICP
레인의 개선폭이 −2.2 % → −6.5 % 로 3 배가 됐다.

### ATE (cm, KISS 기준) — base/imu 만 유효, lidar 레인은 KISS 가 입력이라 순환

| 세션 | base | imu | lidar | lidicp |
|---|---|---|---|---|
| bigeightcircle | 6.8 | 5.8 | 1.2 | 1.6 |
| eightcircle | 5.8 | 4.6 | 0.9 | 1.6 |
| longcircle | 3.1 | 3.1 | 1.0 | 2.9 |
| digut | 8.1 | **21.7** | 3.9 | 3.3 |

### 재방문 일관성 — 입력 KISS 대비 (음수 = 매퍼가 개선)

| 세션 | 입력 cm | base | imu | lidar | lidicp |
|---|---|---|---|---|---|
| bigeightcircle | 4.89 | −3.8 % | +7.3 % | +10.7 % | **−4.2 %** |
| eightcircle | 11.03 | +0.4 % | −1.9 % | +4.5 % | **−2.9 %** |
| longcircle | 18.54 | +0.9 % | **−0.1 %** | +0.1 % | +8.0 % |
| digut | 7.62 | +46.3 % | +127.1 % | +26.8 % | **+19.1 %** |

절대값이 세션마다 4 배 차이 나는 것은 SLAM 오차가 아니라 로봇이 매 바퀴 얼마나 같은 선을 밟았는지를
반영한다. **세션 내 상대 변화만 읽을 것.**

### 무엇이 바뀌었나

- **IMU 는 미미한 게 아니라 위험하다.** 세 세션에서는 무해하거나 약간 낫지만 digut 에서 ATE 가
  8.1 → 21.7 cm(최대 63.6 cm, 회전 4.06°)로 무너진다. digut 은 경로 53 m 에 누적 yaw 1838° 로
  단위 거리당 회전이 가장 많다 — 중력 링크가 roll/pitch 를 묶어 잔차를 yaw 로 미는 §7 의 현상이
  회전이 많을수록 심해진다는 것과 맞는다.
- **ICP 루프가 매퍼를 이득으로 바꾸는 것은 2/4 세션뿐이다.** longcircle 에서는 오히려 +8.0 % 로
  가장 나쁘다.
- **LiDAR 오도메트리로 바꾸는 것만이 네 세션 모두에서 일관되게 좋다** — ATE 0.9~3.9 cm, 그리고
  (순환이 아닌) 실행 시간이 VO 대비 1/6.
- **digut 은 네 레인 모두 입력보다 나쁘다.** 회전이 많은 세션에서는 어떤 구성도 KISS 를 못 이긴다.

## 9. Icp/MaxTranslation 스윕 — 가설이 틀렸다

longcircle 에서 ICP 레인이 진 것은 `Icp/MaxTranslation = 0.2 m` 가 정당한 루프까지 거부해서일
것이라 보고 0.5 / 1.0 / 2.0 m 를 돌렸다(`sweep_icp.sh`, `compare_sweep.py`, 원자료 `sweep/`).

| 세션 | KISS 표류 | 0.2 m (기본) | 0.5 m | 1.0 m | 2.0 m |
|---|---|---|---|---|---|
| bigeightcircle | 15.3 cm | −4.4 % (54) | −6.7 % (103) | **−7.4 %** (78) | −7.1 % (56) |
| eightcircle | 12.8 cm | **−3.0 %** (8) | −0.2 % (32) | +4.2 % (17) | +9.9 % (35) |
| longcircle | 44.9 cm | **+8.0 %** (21) | +9.8 % (43) | +9.4 % (41) | +14.2 % (47) |
| digut | 35.9 cm | +19.6 % (37) | +27.1 % (14) | **+11.6 %** (24) | +48.4 % (14) |

괄호는 받아들인 루프 수.

상한을 올리면 longcircle 의 루프는 21 → 47 회로 예상대로 늘었지만 결과는 +8.0 % → +14.2 % 로 더
나빠졌다. **0.2 m 는 버그가 아니라 보호 장치였다** — 거부되던 링크는 못 붙은 정당한 루프가 아니라
틀린 링크였다. 표류량과 최적 상한에도 상관이 없다(15.3 cm 는 1.0 m 가, 12.8 cm 는 0.2 m 가 최선).
루프 수도 단조롭지 않은데, ICP 가 링크를 받으면 그래프가 바뀌고 바뀐 그래프가 다음 후보의 추측값을
바꾸는 피드백 때문이다.

남은 의심 후보는 `Icp/PointToPlane`(이 빌드에서 false)이지만, longcircle·digut 은 어떤 설정에서도
개선이 없어 원인이 ICP 파라미터보다 그 세션들의 루프 후보 자체(BoW 오탐)일 가능성이 크다.

**실용적 결론**: 정확한 라이다 오도메트리가 있으면 RTAB-Map 의 루프 클로저는 이 데이터셋에서 대체로
순손해다. 얻는 것은 정밀도가 아니라 외형 기반 장소 재인식과 시각 맵이다. 그게 필요 없으면 KISS-ICP
만 쓰는 편이 낫고, 쓴다면 `Icp/MaxTranslation` 은 기본값 0.2 m 를 유지하는 것이 안전하다.

## 10. 같이 고친 버그 — WAL bag 을 read-only 로 못 여는 문제

`--gyro` 가 bigeightcircle 을 빼고 세 세션 전부에서 죽었다. 메시지는
`has no topic /xsens/imu/data` 였지만 토픽은 멀쩡히 있었다.

WAL 모드로 남은 `.db3` 는 `-wal`/`-shm` 동반 파일 없이는 read-only 로 열리지 않는다(sqlite 가 공유
메모리 인덱스를 필요로 하는데 쓰기 권한 없이 만들지 못한다). bigeightcircle 만 예전 read-write
열기의 잔재로 동반 파일이 있어 동작했다. 게다가 `has_topic()` 이 열기 실패와 토픽 없음을 같은 false
로 보고해 원인이 가려졌다.

`raw_session.h` 의 네 군데(`Bag`, `has_topic`, `header_stamps`, 세션 로더)를 `ro_uri()` =
`file:<path>?immutable=1` 로 통일했다. `gyro_aid.h` 는 이미 이렇게 열고 있었고(주석에 NFS 에서
두 프로세스가 같은 bag 을 열던 2026-09-18 사례가 적혀 있다) 공용 헬퍼에만 빠져 있었다.
`slam_stream`(ORB-SLAM3)도 같은 헤더를 쓰므로 함께 재빌드했다.

## 11. 실시간 3D 뷰어 — live_view.py

`rtab_stream --mode live` 는 TCP 로 붙어 프레임마다 `pose`, 맵이 바뀔 때마다 `kf_update`(노드 그래프
전체)를 뉴라인 JSON 으로 보낸다. `live_view.py` 가 그 반대편에서 받아 SSE 로 브라우저에 중계하고
뷰어 페이지를 서빙한다. 디스크에는 아무것도 쓰지 않는다.

    python3 live_view.py --session <dataset>/SLAM [--load <캡처 디렉터리>]
    → http://127.0.0.1:8972/

페이지에서 세 오도메트리(기본 VO / IMU+VO / LiDAR)를 골라 한 번에 하나씩 돌린다 — 동시에 돌리면
CPU 를 나눠 써서 실시간 측정 자체가 무의미해진다. 끝난 실행의 궤적은 흐리게 남아 다음 실행과 겹쳐
보이고, 3D 뷰 아래에 세 실행의 개별 패널이 같은 시점·축척으로 그려진다. 서버가 실행별 궤적을
들고 있어 새로고침해도 잃지 않고, `--load` 로 캡처해둔 `.sse` 를 다시 돌리지 않고 불러올 수 있다.

3D 는 직접 만든 원근 투영이다(저장소 HTML 관례대로 CDN 없음). 좌표는 광학 프레임(x 오른쪽, y 아래,
z 앞)을 표시 프레임(y 위)으로 뒤집어 쓴다. 카메라는 시야뿔로 그려 어디를 보는지 드러낸다.

실시간(1.0×) 3 종 비교에서 확인한 것:

- 셋 다 **드롭 0** — 추적을 실제로 막는 오도메트리가 p99 18.6 ms 로 33.3 ms 예산의 절반이다.
- VO 가 프레임 비용의 거의 전부다: 15.4 ms → LiDAR 레인 0.01 ms. 매퍼도 100.7 → 85.4 ms.
- IMU 는 매퍼를 38 % 무겁게 한다(100.7 → 139.2 ms, 노드마다 중력 에지).
- 궤적 길이로 재는 떨림: 실제 경로 66.3 m 에 대해 VO 레인은 92 m(+39 %), LiDAR 레인은 66.7 m.
  프레임당 참값 0.70 cm 에 VO 는 0.97 cm — 차이 0.27 cm 가 순수한 떨림이다. 진행 방향과 수직으로
  나누면 cross 성분이 VO 0.442 cm vs LiDAR 0.075 cm. 다만 LiDAR 레인은 10 Hz 를 30 Hz 로 보간한
  것이라 10 Hz 로 다시 재면 0.238 cm 로, **진짜 우위는 6 배가 아니라 2 배**다.
- 큰 점프(>5 cm)는 base 6 회 / imu 16 회 / lidar 1 회이고 75 % 가 `map_changed` 직후다 —
  오도메트리가 튀는 게 아니라 매퍼 보정이 계단식으로 들어오는 것이다.
