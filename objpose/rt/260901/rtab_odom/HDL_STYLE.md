# hdl_graph_slam 의 루프 클로저를 RTAB-Map 에 옮기기 — hdl1 / hdl2 / hdl3

**세 실행 모두 RTAB-Map 이다.** hdl_graph_slam 을 돌린 것이 아니라, 그쪽이 루프를 닫는 방식을
RTAB-Map 의 파라미터로 한 단계씩 옮겨온 것이다. 이름에 `hdl` 이 붙은 것은 아이디어의 출처를
가리킬 뿐이다.

실행 `run_hdl_style.sh` · 채점 `compare_features.py` · 원자료 `hdl/<세션>/`
브라우저에서 보기: `live_view.py` (레인 이름은 `RTAB · +ICP …`)

## 1. 왜 옮겼나

앞선 실험(README §8)에서 이런 결과가 나왔다: KISS-ICP 를 오도메트리로 주입하면 궤적이
링크당 σ ≈ 1 cm 수준으로 정확한데, 그 위에 RTAB-Map 의 **시각** 루프 제약(특징점 ~300 개,
스테레오 IR depth)을 얹으면 재방문 일관성이 4.89 → 5.41 cm 로 **나빠진다.**

> 제약이 오도메트리보다 거칠면 체인이 나빠진다.

hdl_graph_slam 은 같은 상황에서 라이다로 루프를 닫는다. 그래서 그 방식을 가져왔다.

## 2. hdl_graph_slam 이 실제로 하는 일

`hdlgraphslam_ws/src/hdl_graph_slam/include/hdl_graph_slam/loop_detector.hpp` 와
`launch/hdl_graph_slam_imu.launch` 기준:

| 단계 | hdl 의 방식 | 근거 |
|---|---|---|
| 후보 선택 | 주행거리로 `accum_distance_thresh = 3 m` 이상 떨어지고, XY 로 `distance_thresh = 1 m` 안. **외형(BoW) 안 씀** | loop_detector.hpp:85-92 |
| 정합 | `GICP` (plane-to-plane, 양쪽 공분산) | launch:120 |
| 수락 | `fitness_score_thresh = 0.5` (정합 잔차) | loop_detector.hpp:150 |

오도메트리 쪽 스캔 매칭은 `NDT_OMP` 를 쓴다(launch:69). 바닥 평면 제약도 있지만 이 런치에서는
`enable_floor_detection = false` 다.

## 3. RTAB-Map 으로의 대응

| hdl | RTAB-Map | 옮겨졌나 |
|---|---|---|
| GICP | `Icp/PointToPlane=true` | △ 근사 — RTAB-Map 에 GICP 가 없다 |
| 1 m 근접 반경 | `RGBD/LocalRadius=1.0` (기본 10 m) | ○ |
| 외형 안 씀 | `Rtabmap/LoopThr=1.0` | △ 부분 — §6 참조 |
| fitness 문턱 | 대응 없음 (`Icp/MaxTranslation` 방식) | ✗ |

**GICP 는 point-to-plane 과 같지 않다.** point-to-plane 은 목표 쪽 표면 법선에만 벌점을 주고
소스 점은 무한히 정확한 점으로 본다. GICP 는 양쪽 점구름의 국소 공분산을 모두 써서
point-to-point 와 point-to-plane 을 특수 경우로 포함한다. RTAB-Map 의 registration 구현
(`util3d_registration.cpp:395, 461`)에는 `pcl::IterativeClosestPoint` 와
`IterativeClosestPointWithNormals` 만 있고 `GeneralizedIterativeClosestPoint` 는 없다.

## 4. 세 변형

한 번에 하나씩만 더해서 변화의 원인을 귀속할 수 있게 했다. 공통 부분은 네 실행 모두 같다 —
오도메트리는 KISS-ICP 주입(`--ext-odom`), 스캔은 `--lidar`, 수직 레버암은 `--lidar-z -0.45`,
`Reg/Strategy=1`(Icp, `--lidar` 가 자동 설정), unpaced offline.

```
기준  lidicp   (README §8 의 ICP 레인)        Icp/PointToPlane=false  ← 이 빌드의 기본값
hdl1           --param Icp/PointToPlane=true
hdl2  = hdl1 + --param RGBD/LocalRadius=1.0
hdl3  = hdl2 + --param Rtabmap/LoopThr=1.0
```

### hdl1 — 정합 방식만

**무엇을 검증하나.** 라이다는 점이 아니라 표면을 잰다. 같은 벽을 두 번 스캔하면 표본이 다른
자리에 떨어지므로 "이 점을 저 점에 붙여라"(point-to-point)는 벽을 따라 당기는 가짜 힘이 된다.
법선 방향 성분만 벌점을 주면 그 힘이 사라진다.

**예측.** ICP 가 가장 크게 실패한 세션(longcircle, +8.0 %)이 가장 많이 좋아져야 한다.

### hdl2 — hdl 의 근접 반경

`RGBD/LocalRadius` 가 근접(공간) 후보를 찾는 반경이다. 기본 10 m 는 hdl 의 1 m 보다 훨씬 넓어
멀리서 후보를 긁어온다.

### hdl3 — 외형 루프 끄기

`Rtabmap/LoopThr=1.0` 이면 베이즈 사후확률이 그 값을 넘을 수 없어 외형 루프가 수락되지 않는다.
남는 것은 `RGBD/ProximityBySpace` 의 거리 기반 링크뿐이고, 그게 hdl 의 후보 선택에 해당한다.

## 5. 결과

재방문 일관성, 입력 KISS 대비 % (음수 = 매퍼가 개선). 괄호는 (외형 루프 / 근접 링크).

| 세션 | 입력 cm | lidicp p2point | hdl1 p2plane | hdl2 +반경1m | hdl3 +외형끔 |
|---|---|---|---|---|---|
| bigeightcircle | 4.90 | −4.4 % (54/–) | −9.9 % (61/74) | **−14.7 %** (58/70) | +54.2 % (0/1) |
| eightcircle | 11.04 | −3.0 % (8/–) | +13.0 % (9/13) | **−3.2 %** (12/17) | +5.9 % (0/19) |
| longcircle | 18.55 | +8.0 % (21/–) | **+1.8 %** (24/21) | +3.0 % (13/14) | +7.2 % (0/35) |
| digut | 7.59 | +19.6 % (37/–) | +18.1 % (19/12) | **+6.5 %** (18/12) | +1.8 % (0/1) |

wall time: hdl1 은 lidicp 보다 16~41 % 느리다(법선 계산). hdl2 는 비슷, hdl3 은 루프가 적어 빠르다.

### hdl1 — 예측이 맞았다

longcircle 이 +8.0 → **+1.8 %** 로 악화가 거의 사라졌다. ICP 가 가장 크게 졌던 세션에서 가장
크게 개선됐다는 것이 가설의 핵심 예측이었다. bigeightcircle 도 −4.4 → −9.9 % 로 이득이 두 배다.

eightcircle 만 −3.0 → +13.0 % 로 반대로 갔는데, 이 세션은 루프가 8~9 개뿐이라 한두 링크가
결과를 좌우한다. hdl2 에서 −3.2 % 로 되돌아오는 것을 보면 틀린 링크 한둘이 원인이었을 가능성이
높다. 반복 실행이나 링크별 잔차 분석은 하지 않았으므로 단정하지 않는다.

### hdl2 — 반경을 좁히면 더 좋아진다

세 세션이 개선된다: bigeightcircle −9.9 → **−14.7 %**(이 시리즈 전체 최고),
eightcircle +13.0 → −3.2 %, digut +18.1 → +6.5 %. longcircle 만 +1.8 → +3.0 % 로 소폭 나빠진다.
10 m 반경이 멀리서 후보를 끌어와 틀린 링크를 만들고 있었다는 설명과 맞는다.

**두 세션(bigeightcircle, eightcircle)에서 hdl2 가 최선이다.**

### hdl3 — 두 세션에서만 hdl 방식이 실제로 돌았다

`loops` 카운터는 외형 루프만 센다(`rtab_stream` 은 `loopClosureId` 와 `proximityDetectionId` 를
따로 집계하는데 offline 의 `done:` 줄은 앞의 것만 찍는다). json 을 보면 근접 링크는 살아 있다:

| 세션 | 외형 루프 | 근접 링크 | 실제로 무엇이 측정됐나 |
|---|---|---|---|
| bigeightcircle | 0 | **1** | 사실상 루프 보정 없음 |
| eightcircle | 0 | **19** | hdl 방식이 실제로 돎 |
| longcircle | 0 | **35** | hdl 방식이 실제로 돎 |
| digut | 0 | **1** | 사실상 루프 보정 없음 |

즉 hdl3 은 네 세션 중 **둘에서만** "거리 기반 후보만으로 루프 닫기"를 측정했고, 그 둘에서
**hdl2 보다 나빴다**: eightcircle −3.2 → +5.9 %, longcircle +3.0 → +7.2 %.

→ **외형 탐지를 끄는 것은 이 데이터셋에서 손해다.** BoW 후보를 유지한 채 정합만 라이다로
바꾸는 편(hdl2)이 낫다.

나머지 두 세션(근접 링크 1 개)은 루프 보정이 사실상 없는 상태를 잰 셈인데, 그것대로 읽을 것이
있다: digut 은 **+1.8 % 로 그게 최선**(루프를 안 거는 편이 낫다), bigeightcircle 은 +54.2 % 로
대폭 악화(루프가 반드시 필요하다).

## 6. 정정 기록

이 문서를 쓰면서 앞서 보고했던 것 두 가지를 고쳤다.

1. **"외형을 끄면 근접 링크도 사라진다 — RTAB-Map 의 구조적 한계"** → 틀렸다. `Rtabmap.cpp` 에서
   근접 탐지(line 1900)는 외형 가설 평가(line 2128 이후)보다 **먼저** 실행된다. 독립적이다.
2. **"hdl3 은 루프 0 개"** → 외형 루프만 0 이었다. 근접 링크는 세션에 따라 1~35 개였다.
   README §12 의 "구조적으로 재현 불가"라는 문장은 이 문서가 대체한다.

왜 bigeightcircle 과 digut 에서 근접 링크가 1 개뿐이었는지는 아직 모른다. 후보:
`RGBD/ProximityPathMaxNeighbors = 0`(문서상 *"Set to 0 to disable one-to-many proximity
detection"* — 근접 탐지가 반쪽만 켜진 상태), `RGBD/ProximityMaxGraphDepth = 50`,
`RGBD/LocalRadius = 1.0`(hdl 의 1 m 는 XY 거리인데 RTAB-Map 의 정의와 다를 수 있다).

## 7. 재현

```bash
# 네 세션 × 세 변형
bash objpose/rt/260901/rtab_odom/run_hdl_style.sh

# 한 세션만
bash objpose/rt/260901/rtab_odom/run_hdl_style.sh bigeightcircle

# 채점 (입력 KISS 대비 재방문 일관성)
python3 objpose/rt/260901/rtab_odom/compare_features.py

# 브라우저에서 보기 — 여섯 레인이 겹쳐 그려지고 개별 패널도 나온다
python3 objpose/rt/260901/rtab_odom/live_view.py \
    --session <dataset>/SLAM --rig rigs/rig_bigeightcircle.json --ext-odom <kiss_tum.txt> \
    --load-tum hdl1=hdl/bigeightcircle/hdl1_tum.txt.optimized.txt \
    --load-tum hdl2=hdl/bigeightcircle/hdl2_tum.txt.optimized.txt \
    --load-tum hdl3=hdl/bigeightcircle/hdl3_tum.txt.optimized.txt
# → http://127.0.0.1:8972/
```

## 8. 남은 것

- **hdl4** — 근접 탐지를 제대로 켠 hdl3. `RGBD/ProximityPathMaxNeighbors=10`,
  `RGBD/ProximityMaxGraphDepth=0`, `RGBD/LocalRadius=3.0`. 그래야 "거리 기반 후보만"이 네 세션
  모두에서 실제로 측정된다.
- **진짜 GICP** — RTAB-Map 에는 없다. PCL 의 `GeneralizedIterativeClosestPoint` 를 `rtab_stream`
  에 직접 붙이거나, `hdl_graph_slam` 자체를 돌려야 한다. 후자는 `objpose/docs/orb_imu_260918.md`
  §7 에 bigeightcircle live ATE 3.5 cm 로 기록이 있다.
- **fitness 문턱** — hdl 의 수락 기준은 정합 잔차인데 RTAB-Map 은 이동량 상한(`Icp/MaxTranslation`)
  이다. 후자는 스윕(README §9)에서 거짓 링크를 막는 보호 장치로 작동하고 있었다.
- **법선 품질** — VLP-16 은 수직 16 채널이라 위아래 이웃이 성기다. `Icp/PointToPlaneK = 5` 가
  적절한지 확인하지 않았다.
- eightcircle 의 hdl1 역전(−3.0 → +13.0 %)은 표본이 작아 설명하지 못했다.

## 9. 읽을 때 주의

- **재방문 일관성의 절대값은 세션 간 비교 불가다.** 4.90 / 11.04 / 18.55 / 7.59 cm 의 차이는
  SLAM 오차가 아니라 로봇이 매 바퀴 얼마나 같은 선을 밟았는지를 주로 반영한다. 세션 내 상대
  변화(%)만 읽을 것.
- **이 지표는 매퍼에게 유리하게 편향돼 있다.** 루프 클로저가 최적화하는 목적함수 자체이므로,
  지면 확실히 진 것이고 이기는 것은 어느 정도 당연하다.
- ATE 로 이 레인들을 채점하면 **순환**이다 — KISS 가 이들의 오도메트리이자 기준이다.
