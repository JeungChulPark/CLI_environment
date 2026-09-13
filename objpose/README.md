# objpose — SLAM 기반 객체 위치 추정 + localhost 시각화

두 대의 RealSense(SLAM 카메라, SAM 카메라)로 녹화한 `Dataset/260826_etri_eightcircle_dark`를
입력으로, **Mac(219.113)의 ORB-SLAM3 위치**와 **PC(219.100)의 SAM-6D 객체 포즈**를 시간 동기화·보간해
SLAM 지도 좌표계의 객체 위치를 `http://localhost:8765`에 실시간으로 보여준다.

```
PC 219.100                                                     Mac 219.113
hub.py ── ssh -R 17001 ─ "run_slam.sh --features 2000 --mode live" ─► slam_stream (ORB-SLAM3 f2000)
   ▲                                                               │ SLAM 원본 세션 1x 재생
   └──────────── pose JSON (t_ns, state, T_wc) ◄── TCP 터널 ───────┘
   │
   ├─ SAM 원본 세션 1x 재생 ─► 공유메모리 ─► sam6d_infer.py (수정 없음)
   ├─ 결과 ─► fusion.py:  T_w_obj = T_w_slam(s)·X·T_sam_obj,  T_sam_obj(t) = (T_w_slam(t)·X)⁻¹·T_w_obj
   └─ localhost:8765 ─ 영상(객체 박스·3축) + 3D 지도(월드 3축·궤적·객체 박스) + 상태
```

## 실행

```bash
# (한 번) 카메라 RT 추정 — Mac ORB-SLAM3 f2000 궤적 두 개(objpose/rt/*_traj_tum.txt)로
/home/jucpark/anaconda3/envs/sam6d/bin/python objpose/pc/estimate_rt.py

# 전체 실행 — Mac 에 SLAM 명령을 내리고 SAM-6D 를 띄운 뒤 같은 데이터 시각에서 동시에 시작
bash objpose/run.sh                     # 브라우저: http://localhost:8765 , 종료: Ctrl+C
```

주요 옵션(`objpose/pc/hub.py --help`): `--duration-s`, `--start-s`, `--rate`, `--overlay-max-age`(기본 10 s),
`--display-delay`(기본 0.25 s), `--sam-feed-hz`(기본 10), `--no-mac`(외부 SLAM 클라이언트 대기).

## 구성 요소

| 파일 | 역할 |
|---|---|
| `pc/hub.py` | Mac 명령·SSH 터널, SLAM 포즈 수신, SAM 재생·SAM-6D 공급, 결과 융합, HTTP/SSE |
| `pc/rs_session.py` | RealSense SDK 원본 세션 리더 + depth→color 정렬 (변환본과 비트 동일 검증) |
| `pc/fusion.py` | SE(3) 보간 포즈 버퍼, 키프레임 기준 객체·카메라 포즈, 루프 클로저 보정 |
| `pc/memory.py` | 객체 메모리(`objectmemory_ws`) 어댑터, 키프레임 보정 시 재구성 |
| `pc/clock.py` | 프레임 시계(color 메타데이터) 테이블, TUM 궤적 시각 재부여 |
| `pc/estimate_rt_offline.py` | **최종 RT**: 궤적 평면 + 평면 hand-eye + 바닥 높이 → `rt/X_slam_sam.json` |
| `pc/estimate_rt.py`, `pc/estimate_rt_loc.py` | 비교용(두 지도 AX=ZB, 공유 지도 localization) |
| `pc/web/index.html` | 시각화 페이지 (three.js) |
| `mac_slam/` | Mac 용 `slam_stream.cc`, CMake(macOS clang 호환 shim), `run_slam.sh`, f2000 설정 |
| `rt/` | Mac ORB-SLAM3 f2000 궤적, rig 적합 결과, `X_slam_sam.json` |
| `output/<run>/` | `slam_poses.jsonl`, `sam6d_estimates.jsonl`, `display_objects.jsonl`, `summary.json`, 로그 |

## 동기화 규약

- 모든 시각은 **SLAM 호스트 시계**로 통일하고, 프레임 시각은 **color 메타데이터 `timestamp`**(`pc/clock.py`)를 쓴다.
  SLAM 세션의 `rgbd_timestamp_associations.json` 스탬프는 프레임 드롭 때문에 녹화 구간에 따라 **−2.1 ~ +3.8 s** 틀어져 있어 쓰지 않는다
  (Mac `slam_stream --time-source frame` 이 기본, 허브도 `frame_idx` 로 다시 매긴다).
- SAM 시각 = SAM 메타데이터 시각 − 18 225 662 057 ns(피어 시계 차) + τ(−48 ms, SAM "System Time" 은 도착 시각이라 늦음; RT 추정에서 측정).
- SAM-6D 추정 프레임 시각 s 의 SLAM 포즈는 앞뒤 포즈(간격 ≤ 0.25 s) 사이 SE(3) 보간.
- 추정 사이의 표시 프레임 t 에서는 객체를 월드에 고정하고 SLAM 움직임으로 카메라 기준 포즈를 옮긴다.
- 지도 번호(map_id)가 바뀔 때만 포즈 버퍼를 비운다(루프 클로저는 같은 지도라 유지).
- **키프레임 기준 객체** (`ORB_POSE_CONSISTENCY_FOR_SAM6D.md` 권장 2): 포즈마다 기준 키프레임(`ref_kf`, `T_w_kf`)을 받고
  객체는 `T_kf_obj` 로 저장한다. Mac 이 1 s 마다·루프 클로저 직후(+0.5 s, +3 s) `kf_update` 로 전체 키프레임 포즈를 보내면
  월드 좌표(`T_w_obj = T_w_kf(최신)·T_kf_obj`)와 카메라 포즈를 다시 계산한다. 컬링된 키프레임은 직전 스냅샷으로 가장 가까운
  키프레임에 옮긴다. 루프 클로저 ±1 s 추정은 표시하고, 깨끗한 앵커를 대체하지 않는다.

## 카메라 RT (오프라인, `pc/estimate_rt_offline.py` → `rt/X_slam_sam.json`)

1. 각 카메라 궤적 평면의 법선 = "아래" 방향 (평면 이탈 RMS SLAM 0.6 cm / SAM 0.2 cm)
2. 수평 오프셋·yaw: 수평화한 두 궤적의 **상대 움직임**(0.5–4 s 창)으로 SE(2) hand-eye (AX = XB), Huber
3. 높이 차: 각 카메라 depth 의 바닥 평면(RANSAC, 법선 제약)

결과(2026-09-13): 잔차 중앙 0.41 cm, 구간 편차 ≤ 6.6 mm / 0.15°, 부트스트랩 sd 약 2 mm / 0.05°, URDF 대비 3.5 cm / 0.54°.
기각한 방법: 두 지도 AX = ZB(수평 회전만이라 조건 불량), SAM 을 SLAM 지도에 localization(지도 휘어짐, 잔차 p90 12 cm).

## 2026-09-13 검증 결과 (실제 Mac 연동 전체 실행)

| 지표 | 처음 (두 지도 AX=ZB RT, 연관 스탬프) | 최종 (오프라인 RT, 프레임 시계) |
|---|---|---|
| 정지 객체 월드 위치 흩어짐 | 중앙 5.6 / p90 16.7 cm | **중앙 1.5 / p90 7.1 cm** |
| 보간 예측 오차, 간격 ≤ 3 s | 중앙 1.6–1.8 cm | **중앙 0.5 cm, p90 1.4 cm** |
| 보간 예측 오차, 간격 10–60 s | 중앙 19.8 cm | **중앙 5.4 cm, p90 9.5 cm** |
| SLAM (Mac M4 Pro, f2000) | 7,074 포즈 OK, 버림 0 | 7,074 포즈 OK, 버림 0 |

키프레임 기준 객체 적용 후 (v4, 같은 조건):

| 지표 | 월드 고정 (v3) | 키프레임 기준 (v4) |
|---|---|---|
| 루프 클로저(86 s) 전후 같은 객체 위치 차 | 평균 5.4 cm (2.8–7.7) | **평균 0.8 cm (0.3–2.0)** |
| 정지 객체 흩어짐 | 중앙 1.5 / p90 7.1 cm | **중앙 0.8 / p90 2.2 cm** |

### 객체 메모리 (`pc/memory.py`, v5)

`objectmemory_ws` 의 `StreamingObjectMemory`(연관 게이트, 임시→활성 승격, 시야 기반 존재 확률, SE(3) 융합, 다중 개체)를
그대로 쓴다. SAM-6D 가 처리한 모든 프레임(검출 0 건 포함)을 키프레임 기준 카메라 포즈와 함께 보관하고, `kf_update` 로
키프레임이 바뀌면 보정된 좌표로 처음부터 재생해 메모리를 다시 만든다(308 프레임 75 ms). 연관은 **위치만**(0.15 m)으로 한다 —
상자형 물체는 SAM-6D 회전이 180° 뒤집혀 회전 게이트(45°)가 한 물체를 두 개체로 쪼갰다.

v5 결과: 남은 개체 **8 개 = 8 종**, 각 개체 위치가 그 물체 관측 중앙값과 0.1–1.3 cm. 1.84 m 떨어진 가짜 `milk` 관측 4 건은
전부 삭제. 끝에 시야 밖 물체는 `remembered`(기억) 상태로 남아 지도에만 흐리게 표시된다.

## ORB-SLAM3 vs RTAB-Map (2026-09-13, 같은 데이터·RT·SAM-6D, `--features 2000`)

`bash objpose/run.sh --slam rtabmap` 로 Mac 백엔드만 바꾼다(`mac_slam/rtab_stream.cc`, Homebrew rtabmap 0.23.8, F2M 오도메트리,
GFTT/BRIEF, `Vis/MaxFeatures 2000`, 매퍼 1 Hz). 비교: `pc/compare_runs.py` → `output/compare_orb_v7_vs_rtab_v1.json`.

| 지표 | ORB-SLAM3 | RTAB-Map | (ORB 반복 실행 차) |
|---|---|---|---|
| 포즈 OK / 버림 | 7,074 / 0 | 7,073 / 1 | |
| 추적 시간 평균 (오도메트리) | 11.1 ms | 18.8 ms (매퍼 1 Hz 평균 126, 최대 710 ms 별도 스레드) | |
| 루프 클로저·지도 보정 이벤트 | 2 | 165 (루프 124 + 근접 157) | |
| 지도 보정 최대 이동 | 9.0 cm | 17.2 cm | |
| 실시간 포즈 vs 최종 포즈 | 중앙 0.6 / p95 8.3 cm | 중앙 4.7 / p95 17.9 cm | |
| 정지 객체 흩어짐 | **중앙 0.8 / p90 2.4 cm** | 중앙 2.7 / p90 6.3 cm | |
| 보간 예측 오차 (간격 ≤ 3 s) | **중앙 0.47 cm** | 중앙 1.08 cm | |
| 객체 메모리 | 8 개 = 8 종 | 8 개 = 8 종 | |
| 최종 궤적 차 (정렬 후) | — | RMSE 2.6 cm | 0.43 cm |
| 같은 객체 위치 차 (정렬 후) | — | 1.0–3.2 cm | 0.1–0.8 cm |
| 1 s 상대 움직임 차 (p95) | — | 1.0° / 3.2 cm | 0.24° / 0.9 cm |

### RTAB-Map 파라미터 조정 (오프라인 스크리닝 8 설정 → 실시간 확인)

`pc/tune_eval.py` 로 Mac 오프라인 궤적(`rt/rtab_tune/c*.txt`)을 채점: v7 의 SAM-6D 검출을 각 궤적에 올린 정지 객체 흩어짐,
ORB-SLAM3 기준 1 s 상대 움직임 차, 정렬 후 ATE. 최선 **c3 = `Vis/MaxDepth=4` + `Vis/FeatureType=10`(ORB-OCTREE)**
(최종 포즈 흩어짐 3.94/13.3 → 2.35/6.6 cm). 서브픽셀·근접검출 끄기·MaxDepth 3·PnP 1 px 조합은 더 나아지지 않았다.

실행: `bash objpose/run.sh --slam rtabmap --slam-param Vis/MaxDepth=4 --slam-param Vis/FeatureType=10`

| 지표 (실시간 전체 실행) | ORB-SLAM3 | RTAB-Map 기본 | RTAB-Map 조정(c3) |
|---|---|---|---|
| 오도메트리 시간 평균 | 11.1 ms | 18.8 ms | 21.2 ms |
| 정지 객체 흩어짐 | **0.8 / 2.4 cm** | 2.7 / 6.3 cm | 2.1 / 5.0 cm |
| 보간 예측 오차 ≤ 3 s / 3–10 s | **0.47 / 1.06 cm** | 1.08 / 3.71 cm | 1.05 / 2.16 cm |
| 최종 궤적 vs ORB (RMSE) | — | 2.6 cm | 2.1 cm |
| 같은 객체 위치 vs ORB | — | 1.0–3.2 cm | 0.6–2.6 cm |
| 메모리 삭제(가짜·임시) | 9 | 9 | 5 |

## LiDAR SLAM (VLP-16) vs ORB-SLAM3 — 260910_object (2026-09-13)

- 데이터: `Dataset/260910_object/lidar` (rosbag2 sqlite, `/velodyne_points` PointCloud2 x,y,z,intensity,ring,time · 10 Hz · 1,583 스캔, IMU 없음).
  SLAM/SAM 폴더는 이미 변환된 bag(표준 토픽, depth 정렬됨, color Global Time). (fx 값이 260826 과 달라 카메라 역할이 바뀐 것으로 의심했으나, 두 라이다–카메라 RT 로 합성한 카메라 간 RT 가 260826 카메라 RT 와 2.7 cm / 1.6° 로 일치해 리그 구성은 동일함을 확인.)
- 라이다: Mac `~/objpose/lidar/venv` KISS-ICP 1.3 (deskew, min 0.5 m, voxel 0.1) — `lidar/lidar_feasibility.py`. 9.9 ms/스캔.
- 영상: Mac `slam_stream` 이 변환 bag 을 자동 인식 (`--features 2000 --pace 1.0`) → `rt/260910/slam_orb_tum.txt*`. 10.6 ms/프레임, 루프 클로저 1회(2.19 m 드리프트 보정).
- 라이다–카메라 RT (`lidar/estimate_lidar_cam_rt.py` → `rt/260910/T_cam_lidar_final.json`): 평면 hand-eye(3,122 쌍, 회전 증분 일치 0.11°,
  잔차 1.18 cm, τ +4 ms) + 높이는 라이다 점–depth 표면 겹침 최대(−0.30 ± 0.1 m). 6-DoF depth 정합은 구간별로 19.5 cm 까지 달라 채택하지 않음.
  VLP-16(최하 −15°)은 실내에서 바닥을 거의 못 봄. 높이는 강체 정렬 후 궤적 비교에 영향 없음.
- 비교 (`lidar/compare_lidar_orb.py` → `lidar/compare_260910/`):

| 지표 (라이다 기준) | ORB-SLAM3 최종(루프 반영) | ORB-SLAM3 실시간 |
|---|---|---|
| ATE (강체 정렬) | RMSE 5.8 cm, p95 9.4 cm | RMSE 11.4 cm, 최대 2.2 m(루프 전 드리프트) |
| 방향 차 | 중앙 0.71° | 중앙 1.41° |
| RPE 1 s | 1.1 cm / 0.20° | 1.3 cm / 0.25° |
| RPE 10 s | 6.3 cm / 0.59° | 7.2 cm / 0.87° |
| 척도 | ORB 궤적이 3.8 % 작음 (척도 보정 시 ATE 3.4 cm) | |
| 귀환 오차 | 11.3 cm (라이다 9.5 cm) | |

### 라이다 SLAM 실시간 연결 (`--slam lidar`)

- Mac `~/objpose/lidar/lidar_stream.py`(KISS-ICP, venv)가 같은 프로토콜로 라이다 포즈(velodyne 좌표, 첫 스캔 = 월드)를 보낸다.
  루프 클로저·키프레임이 없어 객체는 월드 기준으로 고정한다.
- 허브는 변환 bag SAM 입력(`pc/conv_session.py`)을 자동 인식하고, 라이다–SAM 카메라 RT `rt/260910/X_lidar_sam.json`
  (평면 hand-eye 잔차 0.64 cm + 겹침 높이 −0.386 m)를 쓴다. 뷰어는 라이다 월드를 z-up 으로 표시한다.
- 실행 (Mac 경로는 따옴표로 감쌀 것 — 안 그러면 PC 셸이 ~ 를 PC 홈으로 바꾼다):

```bash
bash objpose/run.sh --slam lidar \
  --sam-session /home/jucpark/DeepLearning/Dataset/260910_object/SAM \
  --slam-session /home/jucpark/DeepLearning/Dataset/260910_object/lidar \
  --mac-slam-session '~/Documents/DefenseMeta/Dataset/260910_object/lidar' \
  --extrinsic objpose/rt/260910/X_lidar_sam.json
```

- 결과(`output/live_260910_lidar_v2`): 라이다 포즈 1,577 전부 OK·버림 0, 도착 지연 중앙 102 / p99 138 ms,
  SAM-6D 736 회, 객체 메모리 8 개 = 8 종(삭제 1), 관측 흩어짐 중앙 1.08 / p90 1.83 cm, 보간 오차(≤3 s) 0.98 cm.

### 같은 데이터 실시간 비교: 라이다 vs ORB-SLAM3 (260910_object, SAM-6D·메모리·화면 동일)

`pc/compare_runs.py ... --extrinsic-a rt/260910/X_lidar_sam.json --extrinsic-b rt/260910/X_slam_sam_260910.json`
(두 SLAM 센서가 달라 같은 SAM 카메라 궤적으로 환산해 비교) → `output/compare_260910_lidar_vs_orb.json`

| 지표 | 라이다 KISS-ICP | ORB-SLAM3 f2000 |
|---|---|---|
| 포즈 OK / 버림 | 1,577 / 0 (10 Hz) | 4,772 / 0 (30 Hz) |
| 처리 시간 | 32 ms/스캔 | 10.6 ms/프레임 |
| 도착 지연 중앙 / p99 | 102 / 138 ms | 19 / 34 ms |
| 지도 보정 | 없음(드리프트 작음) | 루프 2회, 최대 2.0 m 보정(루프 전 2.3 m 드리프트) |
| 정지 객체 흩어짐 | **1.08 / 1.83 cm** | 3.66 / 10.0 cm |
| 보간 오차 ≤3 s / 3–10 s / 10–60 s | **0.98 / 1.64 / 1.84 cm** | 2.18 / 6.21 / 10.9 cm |
| 객체 메모리 | 8 = 8 종 (삭제 1) | 8 = 8 종 (삭제 0) |
| SAM 카메라 최종 궤적 차 | RMSE 5.9 cm (정렬 후) | |
| 같은 객체 위치 차 | 1.9–8.1 cm; 객체 간 거리는 ORB 가 3.0 % 짧음(척도 보정 시 1.9 cm) | |

### 라이다 궤적 흔들림 원인과 부드럽게 만들기 (260910_object, Dataset 폴더, KISS-ICP)

Dataset 폴더에서 돌린 라이다 SLAM 은 KISS-ICP 다. hdl_graph_slam 결과는 이 데이터에 없고, PC·Mac 어디에도 빌드돼 있지 않다
(Mac `hdlgraphslam_ws` 는 소스만 있고 ROS 도 없음).

- 흔들림 크기: 1 s 이동평균 잔차 수평 0.62 cm(ORB 최종 0.69 cm). 스캔 간 가속 중앙 0.29 cm, 튀는 스캔(상위 1 %)
  1.5–2.7 cm 가 67.8–68.1 s, 102–103 s, 108–110.4 s, 123.8 s 에 몰려 있고, 같은 순간 ORB 는 매끄럽다.
- 원인이 **아닌** 것: 퇴화(수평 제약 고유값 비 0.73, 가속과 상관 −0.01), 점 수·가림(스파이크 스캔도 2.8 만 점, 1 m 이내 0–0.5 %),
  스캔 누락·시간 필드(모든 스캔 −99.5…+1.3 ms, 방위 360°), 스탬프 기준(KISS-ICP deskew 는 스캔 끝 기준 = 헤더 스탬프).
- 원인: **등속 가정 deskew**. 가속 크기의 순위상관이 선가속 0.70, 요 각가속 0.63, 속도 0.48, 요 속도 0.53 이고,
  스파이크는 각가속 40–130 °/s² 인 출발·정지·회전 전환 순간이다. KISS-ICP 는 직전 스캔의 움직임으로 현재 스캔을 펴는데, 움직임이
  갑자기 바뀌면 스캔이 잘못 펴져 정합이 1–2 cm 밀린다. deskew 를 끄면 그 순간 가속이 1.84 → 0.63 cm 로 사라지는 것으로 확인.
- 측정(`lidar/kiss_variants.py` Mac 실행 → `lidar/eval_lidar_variants.py`, `lidar/eval_lidar_variants_260910.json`).
  고주파 차 = ORB-SLAM3 최종 궤적과의 차에서 1 s 이동평균을 뺀 것(라이다 잡음이 줄면 작아지고, 실제 움직임을 뭉개면 급회전에서 커짐).

| 방식 | ms/스캔 | 지연 | 가속 p99 | 고주파 차 전체 / 스파이크 / 급회전 | ORB 대비 저주파 RMSE | 객체 흩어짐 중앙 / p90 |
|---|---|---|---|---|---|---|
| 기존 (등속 deskew) | 9.5 | 0 | 1.50 | 0.51 / 2.30 / 0.64 | 3.27 | 1.14 / 1.68 |
| deskew 끔 | 11.1 | 0 | 1.37 | 0.39 / 0.91 / 0.64 | **5.25** | 1.35 / 2.39 |
| 반복 deskew 2회 | 14.8 | 0 | 1.36 | 0.35 / 0.70 / 0.46 | 3.26 | 1.02 / 1.73 |
| 복셀 0.05 | 37.4 | 0 | 1.77 | 0.58 / 2.29 / 0.68 | 3.85 | 1.07 / 1.87 |
| 기존 + 5 스캔 평균 | 9.5 | 0.2 s | 0.50 | 0.28 / 0.65 / 0.28 | 3.25 | 1.04 / 1.63 |
| **반복 deskew 2회 + 5 스캔 평균** | 14.8 | 0.2 s | 0.52 | **0.26 / 0.43 / 0.32** | 3.25 | **1.03** / 1.67 |

- 가우시안 σ2(0.6 s)·CA-RTS 스무더도 시험(`lidar/smooth_lidar_traj.py`): 5 스캔 평균보다 낫지 않거나 지연이 표시 지연 0.3 s 를 넘는다.
- 결론: 스파이크는 반복 deskew 로 원인을 줄이고, 남은 스캔 잡음은 표시 지연(0.3 s) 안에 들어가는 5 스캔 평균으로 없앤다.
  저주파 차 3.3 cm(드리프트·RT 수준)는 어떤 스무딩으로도 줄지 않는다 — 루프 클로저 백엔드가 필요하다.

#### 실시간 적용 (`lidar_stream.py --deskew-passes 2 --smooth-scans 5`, 기본값)

- 1차 시도(`output/live_260910_lidar_v4`): 평균낸 포즈만 보냈더니 도착 지연이 19 → 328 ms 로 늘었다. 화면 프레임을 보간하려면
  그 다음 스캔(최대 +0.1 s)도 필요해서 표시 지연 0.3 s 를 넘었고, **화면 프레임의 95 % 에서 포즈가 없어 박스가 사라졌다**
  (SAM-6D 검출 수는 그대로; Sikhye 첫 표시 41.8 → 57.9 s, 메모리에서 Mugcup lost·Dinosaur 둘로 쪼개짐).
- 수정(v5): 원시 포즈는 즉시 보내고(지연 22 ms), 0.2 s 뒤 `pose_refine` 으로 평균낸 포즈를 보내 허브 버퍼(`PoseBuffer.refine`)에서
  바꿔 끼운다(지연 223 ms). `slam_poses.jsonl` 의 `T_wc` = 최종(평균) 포즈, `T_wc_raw` = 스트리밍된 원시 포즈.
  라이다 외 옵션은 `--slam-param deskew_passes=1` 처럼 넘긴다.

| 실시간 (260910_object) | v3 기존 | v4 평균만 전송 | **v5 원시+보정 전송** |
|---|---|---|---|
| 포즈 도착 지연 중앙 | 19 ms | 328 ms | 22 ms (보정 223 ms) |
| 포즈 없는 화면 프레임 | 0 % | 95 % | 0 % |
| 화면 객체 수 (50–160 s) | 8.00 | 7.7 | 8.00 |
| 객체 메모리 | 8 = 8 종 | 7 active + lost 1 + 분열 | 8 = 8 종 |
| 스캔 간 가속 중앙 / p99 | 0.29 / 1.56 cm | | **0.11 / 0.52 cm** |
| ORB 대비 고주파 차 전체 / 스파이크 | 0.51 / 2.24 cm | | **0.26 / 0.46 cm** |
| 관측 흩어짐 중앙 / p90 | 1.12 / 1.62 cm | | 1.04 / 1.68 cm |
| 보간 오차 ≤3 s | 0.95 cm | | 0.82 cm |
| 처리 시간 | 11.0 ms/스캔 | | 15.6 ms/스캔 |

v5 와 v3 의 객체 위치 차 0.08–0.50 cm, 궤적 차 RMSE 0.43 cm — 저주파(위치·지도)는 그대로이고 흔들림만 줄었다.
