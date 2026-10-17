# Mac 의 ORB-SLAM3 — 변경 이력 정리 (2026-09-16)

Mac(192.168.20.55)은 ROS 없이 ORB-SLAM3 를 네이티브로 빌드해 `~/objpose/run_slam.sh` 로 실행한다. 카메라 bag 을 직접 읽어
포즈를 이 PC 의 허브로 TCP 스트리밍하는 `slam_stream` 이 그 위에 있다. 아래는 소스(`orbslam_ws/src/ORB_SLAM3`),
스트리머(`objpose/mac_slam/`), 실행 스크립트, 설정의 변경 이력이다. 근거: git log, `objpose/README.md`,
`objpose/mac_slam/slam_stream.cc` 주석, `docs/orb_improvement_plan_260914.md`, `docs/loop_split_analysis_260914.md`.

## 1. 구성 한눈에

| 구성 | 내용 |
|---|---|
| 빌드 | `objpose/mac_slam/CMakeLists.txt`: macOS arm64, ROS/ament 없이 ORB-SLAM3 + slam_stream. 소스는 repo 클론의 **복사본**에 패치를 적용해 빌드(`patches/`). clang 호환 shim `compat/` (`stdint-gcc.h`, `tr1/`). Pangolin 없이 뷰어 비활성 |
| 실행 | `~/objpose/run_slam.sh --slam orbslam3|rtabmap --session <dir> --features N --time-source frame --mode live --connect <pc>:<port> --rate 1.0` (RTAB-Map 은 같은 CLI 의 `rtab_stream`) |
| 입력 | RAW RealSense SDK 세션(`*.db3` + `rgbd_timestamp_associations.json`)과 변환 bag(표준 토픽) 둘 다 `raw_session.h` 가 읽음. depth 는 컬러 프레임에 정렬(변환기 `align()` 의 C++ 이식, 비트 동일 검증) |
| 설정 | `settings/orbslam3_260826_dark_nf2000.yaml`(SLAM 카메라), `settings/sam_camera_nf2000.yaml`(SAM 카메라): nFeatures 2000, nLevels 8, FAST 20/7, `Stereo.ThDepth` 40. 내부 파라미터는 실행 시 세션 bag 의 `camera_info` 로 덮어씀(`settings_with_camera()`) |
| 출력 | `--mode offline --out traj.txt`(TUM, 최적화 궤적 `.optimized.txt` 포함) / `--mode live`(JSON 포즈 스트림 + `kf_update`) |

## 2. 이력 (시간순)

### 2026-06-30 — 밀집 지도 고스팅 수정 (`ea1ce23`)
- 종료 시 저장하는 dense map 을 추적 중 누적한 실시간 포즈가 아니라 **최적화된(루프·BA 반영) 프레임 포즈**로 depth 를
  재투영해 다시 만든다(`dense_map_reproject_optimized`, 기본 ON). 실시간 공개 경로는 그대로.

### 2026-09-13 — 30 fps 실시간화 (`f912e95`, `3f7dd82`, `8777cbc`, `bf20cfe`, `0778dc8`)
- **추적 경로 오버헤드 제거** (`f912e95`): nFeatures 2000 에서 추적 24.7 ms(P99 35.6 ms, 7,142 프레임 중 170 프레임 33 ms 초과).
  특징 수가 아니라 지도점 수(12k → 20k)에 비례해 늘던 비용을 잘라냄 — 뷰어가 없는데도 매 프레임 하던 Frame 깊은 복사,
  `SearchLocalPoints()` 의 지도점별 red-black-tree 삽입 등. 결과 P99 가 33 ms 예산 안으로.
- **PoseOptimization 반복 스케줄 설정화** (`3f7dd82`): `Optimizer.PoseIterations: [10,5,5,5]`, `Optimizer.PoseEarlyExit: 1`
  (기본은 기존 동작). 이상치 집합이 수렴하면 남은 라운드를 건너뜀.
- **프레임별 추적 시간·상태 기록** (`8777cbc`): `System::InsertTrackTime()` 호출 누락 수정, `TrackPerFrame.csv`(추적 ms,
  상태 OK/RECENTLY_LOST/LOST), 요약에 P99. 이 로그로 "타임스탬프가 6번 뒤로 가서 지도가 7개 생긴 재생" 을 잡아냄.
- **재생 타임스탬프 단조화** (`bf20cfe`): WSL2 벽시계 역행이 Atlas 새 지도 생성을 유발 → 한 번 읽은 시각에서 1/30 s 씩
  더하는 구조적 단조 스탬프로. 이후 지도 1개, 7,143 프레임, 최대 30.25 ms.
- **타당성 보고** (`0778dc8`): ORB-SLAM3 vs RTAB-Map 30 fps 비교 문서(`slam_comparison_report/`).

### 2026-09-13 — Mac 네이티브 빌드 + 스트리머 (`6290e91`, `8f15f8d`)
- `slam_stream.cc`: ROS 없이 세션을 읽고 ORB-SLAM3 RGB-D 를 돌려 포즈를 TCP 로 보냄. 허브가 ssh 로 Mac 에 명령을 내리고
  **역터널(ssh -R 17001)** 로 포즈를 받음(당시 PC 가 WSL2/NAT 뒤). 프로토콜: `hello`(first/last_ns, n_frames, time_source)
  → `start`(t0, rate) → `pose`(t_ns, state, T_wc, track_ms, frame_idx, map_id, map_changed) → `end`.
- 시각 기준 `--time-source frame|assoc`: 연관 파일 스탬프가 프레임 드롭으로 −2.1~+3.8 s 어긋나 **컬러 메타데이터 프레임
  시계**를 기본으로(허브도 `frame_idx` 로 재매김).
- Atlas 접근: ORB-SLAM3 를 패치하지 않고 명시적 템플릿 인스턴스화로 `System::mpAtlas` 를 읽어 **지도 전환(map_id)** 을 정확히 보고.
- `--save-atlas` / `--load-atlas --localization`: 지도 저장·불러오기와 **사전 지도 위 localization** 모드
  (`patches/01-localization-prior-map-tracking.patch` + `Tracking.cc` 수정: 불러온 지도를 활성화해 첫 프레임을
  Relocalization 으로 보내고, 추적 상실 시 새 지도를 만들지 않고 재국지화 유지; `--print-times`, `--dump-aligned` 디버그 옵션).
- 업스트림 버그 수정 두 건: `Map::PreSave` 의 반복자 무효화(저장 중 SIGSEGV 로 지도 소실, 3회 중 2회 재현 → 스냅샷 반복),
  `System::Shutdown` 이 스레드 종료를 기다리지 않고 `SaveAtlas` 로 진입하던 문제(주석 처리된 대기 루프 복원).
- `Sim3Solver.cc`, `LoopClosing.cc` 소폭 수정(루프 검출 진단 출력 포함; 아래 LOOP_GAP).

### 2026-09-13/14 — 키프레임 앵커 프로토콜 (`ORB_POSE_CONSISTENCY_FOR_SAM6D.md` 권장 2)
- 실시간 포즈는 루프 클로저 전 드리프트(최대 11 cm, 260910 에서는 0.4–3.6 m)를 안고 있으므로 객체를 월드에 고정하지 않고
  **키프레임 기준**으로 저장하도록 스트리머가 포즈마다 `ref_kf`, `T_w_kf`, `kf_map_id` 를 붙이고, 1 s 마다·루프 직후(+0.5 s,
  +3 s) 전체 키프레임 포즈를 `kf_update`(reason periodic/loop/map_changed) 로 보냄. 컬링된 키프레임은 허브가 재앵커.
- 루프 진단 로그 `LOOP_GAP|cur_id|matched_id|ts|cur_xyz|matched_xyz|gap_m|sim3t|sim3_scale` (`LoopClosing.cc`) — 루프 검출
  순간 두 키프레임의 지도상 거리(= 루프 전 누적 오차)를 남겨 오늘의 분석(gap 1.4–2.9 m)에 쓰였다.

### 2026-09-14 — LAN 직결과 다중 호스트 (`run_lan.sh`, `run_pc3.sh`, `linux_slam/`)
- PC 가 Ubuntu 워크스테이션(192.168.20.5)으로 바뀌어 **역터널 없이 Mac 이 PC 로 직접 접속**(`--link direct`,
  `--connect 192.168.20.5:17001`). `run_lan_slam.sh` 는 Mac 쪽 편의 래퍼.
- 같은 `slam_stream.cc` 를 Linux 로도 빌드(`linux_slam/`, Ubuntu .3 과 이 PC) — Mac 전용이던 스트리머가 호스트 독립이 됨.
- 허브 `--features N` → `run_slam.sh --features N` → `settings_override()` 가 임시 yaml 의 `ORBextractor.nFeatures` 를 바꾸므로
  **재컴파일 없이 특징 수 변경** 가능. 반면 `--slam-param Key=Value` 는 스트리머에 분기가 없어 실행이 죽는다(미구현, 문서 §1.5a).

### 2026-09-14/15 — 특징 수 스윕과 루프 분열 분석 (실행·문서)
- Mac f2000/f3000/f4000, .3 f2000/f3000/f4000 스윕: Mac 은 f4000 이 최선(흩어짐 5.7 → 4.1 cm, 추적 p99 20.6 ms), .3 은 f3000
  까지(f4000 은 p99 32 ms 로 예산 한계). 런처 기본값을 그렇게 두었다. 어떤 설정도 첫 회전 구간(50–70 s) 드리프트 10–36 cm 는
  못 없앤다(`docs/orb_improvement_plan_260914.md`).
- Mac f3000 실행 한 번에서 두 번째 루프가 드리프트를 남긴 채 닫혀 지도가 0.58 m + 17.5° 로 갈라짐 → 객체 8종이 모두 둘로
  (`docs/loop_split_analysis_260914.md`). 허브 쪽 F1(거짓 컬링 제거)·F2(게이트 0.30 m) 적용; SLAM 쪽 근본 수정(F4: `LOOP_CORR`
  계측, 루프 후 정합 검사, 초반 루프 거부)은 미착수.

### 2026-09-15/16 — 데이터셋·리그
- 260915 녹화(dense/distance/eightcircle/inlong/outlong/height)에서 Mac ORB 는 `~/Documents/DefenseMeta/Dataset/<name>/SLAM`
  (변환 bag)을 읽음. 리그 RT 재추정 결과 **SLAM 카메라만 260910 대비 5 cm / 1.7° 이동**(SAM 카메라·라이다는 그대로) →
  `rt/260915/X_slam_sam_260915.json` 적용(`docs/rig_rt_260915.md`). 카메라 기기 배정이 260826 과 반대라는 점도 확인
  (260915 SLAM 폴더 = `sam_camera_nf2000.yaml` 의 기기; 내부 파라미터는 bag 에서 덮어쓰므로 결과에는 영향 없음).

## 3. 지금 남은 것

| 항목 | 상태 |
|---|---|
| `--slam-param` 실제 전달 (`ThDepth` 등 스윕) | 미구현 — `slam_stream.cc` 인자 분기 추가 필요 |
| `n_tracked_mp` 를 포즈 메시지에 포함 | 1줄, 미적용 (`slam_stream.cc` 의 `json pose`) |
| 루프 보정 검증(F4) | 미착수 — 정상 보정은 새 구간이 m 단위로 이동, 실패 보정은 옛 구간만 소폭 변형 |
| IMU(RGB-D-Inertial) | 데이터에 IMU 없음, 새 녹화 전제 |
| 성능 | Mac f4000 추적 15 ms(p99 21), 도착 지연 20–30 ms; .3 f3000 22 ms(p99 26) |
