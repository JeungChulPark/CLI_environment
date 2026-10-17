# OpenLORIS-Scene: 원본 ORB-SLAM3 vs 수정본 (GT 비교)

GT가 있는 공개 데이터셋에서 원본 ORB-SLAM3 v1.0과 이 저장소의 수정본을 같은 조건으로 비교한다.
OpenLORIS-Scene은 지상 로봇에 RealSense D435i(RGB-D + IMU)를 단 데이터라 우리 구성과 거의 같고,
자이로 보조를 실험할 수 있다.

## 데이터

- 받는 곳: Hugging Face 미러 `shixuesong/openloris-scene` (로그인 불필요). 공식 페이지는 등록 양식 작성을 요청한다:
  https://lifelong-robotic-vision.github.io/dataset/scene.html
- 위치: `~/Documents/DefenseMeta/Dataset/openloris/`
  - `office/office1-1 … office1-7` (package 형식: `color/`, `aligned_depth/`, `color.txt`, `aligned_depth.txt`,
    `d400_gyroscope.txt` 400 Hz, `trans_matrix.yaml`, `sensors.yaml`)
  - `gt/per-scene/gt_<scene>_1.txt` (공식 평가용), `gt/per-sequence/<seq>/groundtruth.txt`
- GT: office = OptiTrack 모션캡처, corridor/home/cafe/market = 오프라인 라이다 SLAM. GT 프레임은 `base_link`.
- 다른 장면(package): corridor 33.6 GB, home 19.0 GB, cafe 7.5 GB, market 41.5 GB.

## 비교 대상

| 이름 | 빌드 | 설정 |
|---|---|---|
| `upstream` | UZ-SLAMLab/ORB_SLAM3 `v1.0-release` + macOS 이식 수정 2개(g2o `tr1`, `stdint-gcc.h`) | `openloris_d400.yaml` |
| `ours` | `orbslam_ws/src/ORB_SLAM3` | 같은 yaml + 실시간 설정 C (`Optimizer.PoseIterations [10,5,5,5]`, `PoseEarlyExit 1`) |
| `ours_gyro` | 같은 빌드 | `ours` + D435i 자이로 보조 (`--gyro`, `make_rig.py`) |

- 세 버전 모두 같은 실행기(`openloris_rgbd.cc`), 같은 컴파일 옵션, 같은 의존성(OpenMP 포함)으로 빌드한다.
- 주의: 저장소의 "초기 커밋"(a4aff56d)은 원본이 아니다. 루프 검출 임계값이 이미 완화돼 있다
  (BoW 0.8→0.5, 연속 일치 3→1, 매칭 수 20→12 등, `KeyFrameDatabase.cc`, `LoopClosing.cc`). 그래서 기준은 v1.0 태그다.

## 실행

```bash
orbslam_ws/openloris/build.sh                       # output/openloris/build_{upstream,ours}/openloris_rgbd
python3 -m venv orbslam_ws/output/openloris/venv && orbslam_ws/output/openloris/venv/bin/pip install \
    numpy matplotlib transformations opencv-python-headless
git clone https://github.com/lifelong-robotic-vision/openloris-scene-tools.git orbslam_ws/output/openloris/tools
orbslam_ws/openloris/run_openloris.sh ~/Documents/DefenseMeta/Dataset/openloris/office \
    orbslam_ws/output/openloris/runs/office_nf1000 3 1000
```

- 기본은 실시간 재생이다. 30 Hz 벽시계로 프레임을 내보내고, 추적이 못 따라가면 KEEP_LAST(5) 큐에서 오래된 프레임을 버린다.
  `REALTIME=0`이면 모든 프레임을 차례로 처리한다(일반 오프라인 벤치마크 방식).
- 반복은 버전을 번갈아 돌린다(rep1의 세 버전 → rep2 …). 기계 부하 변화가 한쪽에만 걸리지 않게 하려는 것이다.
- 중단 후 다시 실행하면 끝난 실행은 건너뛴다.

## 평가 (`eval_openloris.py`)

공식 평가 도구(`openloris-scene-tools/benchmark/evaluate.py`)를 그대로 호출한다. ROS의 `tf.transformations`는
`tf_shim/`으로 대신한다. pip `transformations` 패키지와 쿼터니언 순서가 달라서(ROS [x,y,z,w], pip [w,x,y,z]) shim에서 바꿔 준다.

| 지표 | 뜻 |
|---|---|
| `ate_rmse` | 공식 ATE. GT(base_link)를 d400_color로 변환한 뒤 SE(3) 정렬 |
| `ate_rmse_sim3` | 같은 계산에 스케일까지 맞춘 값(공식 `--scale`) |
| `aoe_rmse` | 방향 오차 [deg] |
| `cr`, `cr_0.1m_5deg` | 공식 correct rate. 포즈가 있는 시간 비율, 그리고 0.1 m·5° 안에 든 시간 비율 |
| `live` / `final` | 프레임마다 그 순간 돌려준 포즈 / 종료 후 최적화 궤적 |
| `seg_*` | 맵 구간별 평가(아래 home/corridor 절 참고) |

- office1-1 시험 실행에서 모든 버전의 궤적이 GT보다 일정하게 약 5% 작았다(1/3/5초 구간 이동 거리 비 1.045–1.052).
  D435i depth나 GT 쪽 스케일 문제로 보이며 모든 버전에 똑같이 작용한다. 이 때문에 SE(3) ATE는 약 8 cm,
  Sim(3) ATE는 약 1 cm로 나온다. 시간 오프셋은 원인이 아니다(±0.3 s 범위에서 ATE 변화 1 cm 미만).

## 결과: office 7개 시퀀스, 3회 반복 (2026-10-17, Mac M4 Pro)

조건: nf1000, 30 Hz 실시간 재생. 2배·3배 = `STRIDE=k RATE=k`(k장마다 1장, 30 Hz) → 프레임 간 움직임 k배.
원자료: `results/office_nf1000{,_x2,_x3}.csv` (실행마다 한 줄, `eval_openloris.py` 출력).
값은 7개 시퀀스 평균, ± = 반복 3회 사이 표준편차.

Sim(3) ATE [cm] (스케일까지 맞춘 궤적 모양 오차)

| 조건 | 포즈 | upstream | ours | ours_gyro | gyro 효과(ours 대비) | gyro가 나은 시퀀스 |
|---|---|---|---|---|---|---|
| 1배 | live | 3.29 ± 0.70 | 2.98 ± 0.37 | **2.08 ± 0.16** | −30 % | 6/7 |
| 2배 | live | 3.12 ± 0.51 | 3.06 ± 0.40 | **2.03 ± 0.07** | −34 % | 6/7 |
| 3배 | live | 2.30 ± 0.79 | 2.55 ± 0.11 | **1.81 ± 0.07** | −29 % | 5/7 |
| 1배 | final | 2.90 ± 0.78 | 2.55 ± 0.46 | **1.84 ± 0.19** | −28 % | 5/7 |
| 2배 | final | 2.67 ± 0.57 | 2.53 ± 0.22 | **1.74 ± 0.07** | −31 % | 5/7 |
| 3배 | final | 2.00 ± 0.77 | 2.30 ± 0.15 | **1.55 ± 0.09** | −33 % | 5/7 |

1배 live, 그 밖의 지표: 공식 SE(3) ATE 7.87 / 7.70 / 7.26 cm, 최대 오차 16.0 / 15.3 / 12.5 cm,
AOE 2.18 / 2.08 / 1.84°, CR(0.1 m, 5°) 0.77 / 0.79 / 0.81, 추적 시간 평균 7.9 / 4.6 / 4.6 ms.

- 같은 시퀀스를 반복했을 때의 편차는 중앙값 0.05 cm, 자이로에 따른 차이는 시퀀스당 평균 0.7–1.0 cm다.
- 공식 SE(3) ATE에서는 자이로 효과가 −6 %로 작게 보인다. 데이터셋 공통의 약 5 % 스케일 편향이 그 지표를 지배하기 때문이다.
- upstream과 ours(자이로 없음)의 정확도 차이는 반복 편차 안이다. 속도 개선(추적 시간 −40 %)은 정확도를 바꾸지 않았다.
- 1배·2배·3배 모두 추적 실패 0, 프레임 손실 0. office는 로봇이 느려서(최대 0.4 m/s) 3배에서도 영상 추적이 무너지지 않는다.

비정상 종료: upstream 63회 중 4회, ours·ours_gyro 126회 중 0회.
- 종료 중 `mutex lock failed` 3회 (2배 office1-2, office1-7 ×2): 원본 `System::Shutdown()`이 스레드 종료를 기다리지 않는다.
- 실행 중 `SO3::exp failed! omega: nan` 1회 (3배 office1-5, 530프레임 중 422번째): 원본 `Sim3Solver`의 0/0 → NaN.
- 둘 다 수정본에서 고친 버그다. 실패한 실행은 위 표의 upstream 평균에서 빠져 있다(2배 n=19, 3배 n=20; 나머지 n=21).

## 결과: home 5개, corridor 5개 시퀀스, 3회 반복 (2026-10-17~18)

GT: home·corridor는 2D 라이다(Hokuyo) Hector SLAM이다. 평면 3자유도이고 오차가 cm 수준이라 office 모션캡처보다 거칠다.
home1-3에서는 GT 시간을 +0.3 s 옮기면 ATE가 16 → 13 cm로 줄었다. 세 버전에 똑같이 작용하므로 버전 비교는 유효하지만,
절대값은 office와 직접 비교하지 않는다. 원자료는 `results/{home,corridor}_nf1000{,_x2}.csv`.

### 평가 방식: 맵 구간별 (`seg_*` 열)

두 장면 모두 추적을 잃고 새 맵을 만든다(시퀀스당 1–7개). 새 맵마다 원점이 달라서, 공식 단일 정렬로는 ATE가 수 m,
AOE가 약 180°로 나와 의미가 없다. 수정본은 루프 임계값이 완화돼 있어 끊긴 맵을 옛 맵에 병합하기도 한다.
이때 live 포즈가 옛 맵 좌표계로 점프한다.
그래서 새 맵이 시작될 때와 포즈가 0.25 m 넘게 점프할 때 궤적을 나눈다. 그다음 5초 이상인 조각을 각각 Sim(3)로 정렬하고,
모든 포즈에 대한 RMSE로 합친다(`seg_ate_sim3`). `seg_cr`은 점수를 매긴 조각이 덮는 시간 비율이다.
office는 맵이 하나라 이 값이 공식 값과 같다.

### Sim(3) ATE, 맵 구간별 평가 [cm], 5개 시퀀스 평균 (각 3회 평균)

| 장면 | 조건 | 포즈 | upstream | ours | ours_gyro | gyro 효과(ours 대비) | gyro가 나은 시퀀스 |
|---|---|---|---|---|---|---|---|
| home | 1배 | live | 13.70* | 12.84 | **11.00** | −14 % | 4/5 |
| home | 1배 | final | 13.86* | 13.01 | **11.27** | −13 % | 4/5 |
| home | 2배 | live | 13.66 | 13.31 | **10.57** | −21 % | 5/5 |
| home | 2배 | final | 13.70 | 13.54 | **10.90** | −20 % | 5/5 |
| corridor | 1배 | live | 62.7 | 63.8 | **42.5** | −33 % | 4/5 |
| corridor | 1배 | final | 61.4 | 51.4 | **41.5** | −19 % | 4/5 |
| corridor | 2배 | live | 55.4 | 52.7 | **42.0** | −20 % | 4/5 |
| corridor | 2배 | final | 51.1 | 49.7 | **40.5** | −19 % | 4/5 |

\* home1-4 upstream 1배는 3회 모두 크래시했다(Sim3Solver NaN). 그래서 4개 시퀀스 평균이다.

- 자이로 효과는 세 장면 모두에서 같은 방향이다(office −30 %, home −14~21 %, corridor −19~33 %).
  corridor1-5(150 → 75 cm)와 corridor1-4(43 → 25 cm)처럼 오차가 큰 시퀀스에서 가장 크다.
- 자이로가 낫지 않은 시퀀스는 home1-4 1배(10.6 vs 11.0)와 corridor1-1(31 vs 31–34)이다.
- 추적 구간 비율(`seg_cr`, 약 0.90)은 버전 사이에 거의 같다. 자이로는 추적 상실 자체를 막지는 못하고, 추적 중인 구간의 정확도를 높인다.
- 추적 시간: upstream 7.5–8.0 ms, ours·ours_gyro 4.6–5.1 ms.

### 크래시와 수정

| 원인 | 영향 | 조치 |
|---|---|---|
| `MapPoint::ComputeDistinctiveDescriptors`의 N×N 스택 배열. macOS 보조 스레드 스택은 512 KB라 N > 약 360이면 SIGBUS가 난다 | corridor, 세 버전 모두 37회 | 힙으로 옮김. 수정본 소스를 고쳤고 upstream에는 `build.sh` prep()의 이식 수정으로 적용. 해당 실행은 재실행 |
| upstream `Sim3Solver` 0/0 → NaN (`SO3::exp failed`) | upstream만 7회 (home 4, corridor 3) | 수정본에서 이미 고친 버그. 실패한 실행은 평균에서 제외 |
| `LocalMapping::Release()`가 대기열 키프레임을 `delete`. RGB-D 초기 KF는 이미 맵에 있어서, 다음 `ResetActiveMap` → `Map::clear()`가 해제된 mutex를 잠근다 | ours corridor1-1 1회 (원본 코드에서 물려받은 버그) | 수정본에서 delete를 제거하고 재실행 |

남은 것: 자이로 효과가 없는 시퀀스의 원인, home GT 시간 오프셋 보정,
루프 임계값 완화·조기 종료를 원본 값으로 되돌린 분리 실험.
