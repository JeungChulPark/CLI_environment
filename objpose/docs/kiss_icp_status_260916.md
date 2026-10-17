# KISS-ICP 라이다 백엔드 — 지금까지의 변경 정리 (2026-09-16)

objpose 의 라이다 SLAM 백엔드 두 가지 중 하나가 KISS-ICP 다(다른 하나는 hdl_graph_slam). 이 문서는 KISS-ICP 가
어떻게 들어왔고, 무엇을 고쳤고, 지금 어떤 상태로 어디에서 돌아가는지를 한 곳에 모은 것이다. 근거 파일과 수치는
`objpose/README.md`(라이다 절), `objpose/lidar/*.py`, `objpose/output/one_*kiss*` 에 있다.

## 1. 한 줄 요약

| 항목 | 내용 |
|---|---|
| 라이브러리 | `kiss_icp` 1.3.0 (PyPI), 파이썬 venv — ROS 불필요 |
| 역할 | VLP-16 스캔 → 라이다 오도메트리(루프 클로저 없음) → 허브 프로토콜로 포즈 전송 |
| 스트리머 | `objpose/lidar/lidar_stream.py` (허브의 `--slam lidar`, `~/objpose/lidar/venv/bin/python` 으로 원격 실행) |
| 실행 호스트 | Mac 192.168.20.55 (`~/objpose/lidar/venv`, 2026-09-13), Ubuntu .3 192.168.20.3 (`~/objpose/lidar/venv`, 2026-09-15) |
| 페이지 버튼 | LiDAR (Mac) / LiDAR (192.168.20.3) 카드의 **KISS-ICP** (백엔드 `kiss-mac` 포트 17006, `kiss-3` 포트 17007) |
| 기본 옵션 | `--deskew-passes 2 --smooth-scans 5 --min-range 0.5 --max-range 30 --voxel 0.10` |
| 260910 성능 | 정합 11–18 ms/스캔, 도착 지연 11–14 ms, 객체 흩어짐 1.05 cm(hdl 1.62, ORB 4–7) |

## 2. 변경 이력

### 2-1. 도입 (2026-09-13, `lidar/lidar_feasibility.py`, `lidar/lidar_stream.py`)
- 260910_object 의 VLP-16 bag(`/velodyne_points`, 10 Hz, 1,583 스캔)으로 Mac 에서 타당성 확인: 9.9 ms/스캔,
  귀환 오차 9.5 cm (`lidar/feasibility_mac/`, `lidar/feasibility_pc/`).
- 스트리머 `lidar_stream.py`: rosbag2 sqlite 에서 스캔을 직접 읽어(ROS 없이) bag 속도로 재생하고,
  ORB 스트리머(`slam_stream`)와 같은 TCP 줄 단위 JSON 프로토콜(hello → start → pose… → end)로 허브에 보낸다.
  포즈는 velodyne 좌표, 첫 스캔이 월드 원점. 키프레임이 없으므로 허브는 객체를 월드에 고정한다(`ref_kf: null`).
- 라이다–SAM 카메라 RT `rt/260910/X_lidar_sam.json`(평면 hand-eye + 겹침 높이, `lidar/estimate_lidar_cam_rt.py --camera SAM`).
- 뷰어는 hello 의 `source` 가 `lidar_*` 이면 지도를 z-up 으로 그린다.

### 2-2. 궤적 흔들림 분석과 스무딩 (2026-09-13, v3 → v5)
`README.md` "라이다 궤적 흔들림 원인과 부드럽게 만들기".
- 문제: 1 s 이동평균 잔차 0.62 cm 수준이지만 출발·정지·회전 전환 순간에 1.5–2.7 cm 스파이크.
- 원인: KISS-ICP 의 **등속 가정 deskew**(직전 스캔 움직임으로 현재 스캔을 폄) — 가속 순위상관 0.63–0.70. 퇴화·점 수·시간 필드는 원인이 아님.
- 실험 (`lidar/kiss_variants.py` Mac 실행, `lidar/eval_lidar_variants.py`, `lidar/eval_lidar_variants_260910.json`):

  | 방식 | ms/스캔 | 가속 p99 | 고주파 차(전체/스파이크) cm | 객체 흩어짐 중앙 cm |
  |---|---|---|---|---|
  | 기존(등속 deskew) | 9.5 | 1.50 | 0.51 / 2.30 | 1.14 |
  | deskew 끔 | 11.1 | 1.37 | 0.39 / 0.91 (저주파 RMSE 악화 5.25) | 1.35 |
  | **반복 deskew 2회** | 14.8 | 1.36 | 0.35 / 0.70 | 1.02 |
  | **반복 deskew 2회 + 5스캔 평균** | 14.8 | 0.52 | 0.26 / 0.43 | 1.03 |

- 적용(스트리머 기본값): `--deskew-passes 2`(정합 후 자기 움직임으로 다시 펴서 재정합) + `--smooth-scans 5`(중앙 이동평균).
- v4 실패 → v5 수정: 평균한 포즈만 보내면 도착 지연이 19 → 328 ms 로 늘어 화면 프레임 95 % 에 포즈가 없었다.
  그래서 **원시 포즈를 즉시 보내고**(지연 22 ms) 0.2 s 뒤 `pose_refine` 메시지로 평균 포즈를 바꿔 끼운다
  (허브 `PoseBuffer.refine`; `slam_poses.jsonl` 에 `T_wc` = 최종, `T_wc_raw` = 원시). 결과: 포즈 없는 프레임 0 %, 8 객체 유지.
- macOS 특이 처리: 스캔 사이 sleep 하면 코어가 저속으로 떨어져 5배 느려지므로 `--spin-ms 50` 동안 busy-wait.

### 2-3. LAN 직결과 hdl_graph_slam 병행 (2026-09-14)
- 허브 `--link direct|manual|local|tunnel`, 레인 구조(`--lane …`) 도입. KISS-ICP 는 `--slam lidar` 레인으로
  Mac 에서 `--connect 192.168.20.5:<port>` 로 직접 접속(역터널 불필요).
- 같은 날 hdl_graph_slam 을 별도 백엔드(`--slam hdl`, `lidar/hdl_stream.py`)로 추가. KISS-ICP 코드는 그대로.
- 허브의 라이다 레인 색은 주황(`BACKEND_COLOR["lidar"] = "#f2a03d"`).

### 2-4. .3 호스트 추가와 페이지 버튼 (2026-09-15)
- Ubuntu .3 에 `~/anaconda3/bin/python3 -m venv ~/objpose/lidar/venv` + `pip install kiss-icp==1.3.0 numpy scipy`
  (시스템 `python3 -m venv` 는 pip 이 없어 실패했던 것을 anaconda Python 3.13 으로 대체). `lidar_stream.py` 복사.
- `run_one.sh`: `kiss-3`(`--slam lidar --link direct --slam-port 17007`, 세션 `$PC3_DATASET/lidar`),
  `kiss-mac`(포트 17006, `$MAC_DATASET/lidar`). `launcher.py`: 호스트별 "LiDAR (…)" 카드에 `algo: KISS-ICP` 버튼.
- 260915 데이터셋 블록(dense/distance/eightcircle/inlong_*/outlong_*/height)도 같은 백엔드를 쓰며 RT 는
  `rt/260915/X_lidar_sam_260915.json`(2026-09-16 재추정: 260910 대비 0.34 cm / 0.61° 차이 — 라이다·SAM 카메라 리그는 그대로).

## 3. 현재 결과 (260910_object, 2026-09-15 실행, 같은 센서 좌표계)

| 지표 | KISS-ICP Mac | KISS-ICP .3 | hdl_graph_slam Mac |
|---|---|---|---|
| 포즈 / 버림 | 1577 / 0 | 1577 / 0 | 1577 / 0 |
| 처리 / 스캔 | 11.2 ms | 17.6 ms | 13.4 ms |
| 도착 지연 중앙 | 11 ms | 14 ms | 55 ms |
| 루프 클로저 | 없음 | 없음 | 12회 (최대 2.7 cm) |
| 객체 흩어짐 중앙 / p90 | 1.05 / 1.74 cm | 1.06 / 1.77 cm | 1.62 / 2.84 cm |
| 보간 오차 ≤3 s | 0.65 cm | 0.62 cm | 0.90 cm |
| hdl(Mac) 대비 객체 위치 차 중앙 | 0.84 cm | 0.82 cm | – |
| 궤적 RMSE (hdl Mac 대비) | 2.1 cm | 2.1 cm | – |

- 짧은 실내 세션에서는 KISS-ICP 가 hdl 과 같은 위치(1 cm 이내)를 더 매끄럽고(흩어짐 −35 %) 더 빠르게(지연 1/4) 낸다.
- hdl 의 이점(루프 보정, 키프레임 앵커 재배치)은 드리프트가 수십 cm 로 커지는 긴 주행에서만 드러난다.
- 두 라이다 방식 모두 ORB-SLAM3(흩어짐 4–8 cm, 라이다 대비 객체 오차 3–5 cm)보다 한 단계 안정적이다.

## 4. 알려진 문제·주의

1. **260915 라이다 bag 의 중복 스캔**: 같은 스캔이 바이트 동일하게 두 번 녹화되고 10 Hz 퍼블리셔 2개가 겹쳐
   40 Hz 처럼 보인다(dense 5,414 스캔/136 s). `lidar_stream.py` 는 중복을 거르지 않아 "behind schedule" 로 스캔의
   약 50 % 를 `dropped` 로 세며, 실제 처리는 정상 10 Hz 다(예: dense KISS .3 2,459 처리 / 2,933 버림, 결과 8 = 8종,
   흩어짐 0.75 cm). 녹화 설정을 고치는 것이 우선이고, 스트리머에 헤더 스탬프 중복 제거를 넣는 것도 간단하다
   (`rt/260915/kiss_260915.py` 에 오프라인 중복 제거 구현 있음).
2. **드리프트 미보정**: 오도메트리라 되돌아와도 지도가 고쳐지지 않는다. 260910(160 s)에서는 hdl 대비 2 cm 차이로
   문제 없었지만 긴 경로에서는 hdl 또는 루프 클로저가 필요하다.
3. **.3 처리 시간**: Python 3.13 venv 에서 17.6 ms/스캔(Mac 11.2 ms). 10 Hz(100 ms)에는 충분하다.
4. **첫 스캔 = 월드 원점**: 실행마다 라이다 월드 좌표계가 첫 스캔 자세로 정해진다. 실행 간 객체 위치를 비교할 때는
   `pc/compare_runs.py` 가 궤적으로 정렬한다(같은 센서면 외부 파라미터 없이).

## 5. 관련 파일

| 파일 | 내용 |
|---|---|
| `objpose/lidar/lidar_stream.py` | 스트리머(재생·KISS-ICP·반복 deskew·5스캔 평균·pose_refine) |
| `objpose/lidar/lidar_feasibility.py`, `feasibility_mac/`, `feasibility_pc/` | 타당성 실험(속도·지도) |
| `objpose/lidar/kiss_variants.py`, `eval_lidar_variants.py`, `eval_lidar_variants_260910.json` | deskew/스무딩 변형 실험과 채점 |
| `objpose/lidar/smooth_lidar_traj.py` | 가우시안·RTS 스무더 비교(채택 안 함) |
| `objpose/lidar/estimate_lidar_cam_rt.py`, `rt/260910/X_lidar_sam.json`, `rt/260915/X_lidar_sam_260915.json` | 라이다–카메라 RT |
| `objpose/pc/compare_runs.py` | 실행 간 비교(KISS vs hdl vs ORB) |
| `objpose/run_one.sh`, `pc/launcher.py` | 백엔드 `kiss-3` / `kiss-mac`, 페이지 카드 |
| `objpose/output/one_*kiss*` , `output/live_260910_lidar_v1~v5` | 실행 로그(v1–v5 는 스무딩 개발 과정) |
