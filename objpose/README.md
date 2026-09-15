# objpose — SLAM 기반 객체 위치 추정 + localhost 시각화

두 대의 RealSense(SLAM 카메라, SAM 카메라)로 녹화한 `Dataset/260826_etri_eightcircle_dark`를
입력으로, **Mac(192.168.20.55)의 ORB-SLAM3 위치**와 **PC(192.168.20.5)의 SAM-6D 객체 포즈**를 시간 동기화·보간해
SLAM 지도 좌표계의 객체 위치를 `http://localhost:8765`에 실시간으로 보여준다.

```
PC 192.168.20.5                                                     Mac 192.168.20.55
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

## LAN 직결 프로필 (2026-09-14, `run_lan.sh`) — 기존 `run.sh` 는 그대로

이 PC 가 WSL2(.100, NAT 뒤)에서 **Ubuntu 워크스테이션 192.168.20.5** 로 바뀌어 들어오는 연결을 받을 수 있게 됐다.
그래서 역터널 없이 **Mac 이 PC 로 직접 TCP 접속**하는 설정을 따로 만들었다. `run.sh`(ssh -R 터널, `localhost:8765`)는
손대지 않았고, 두 스크립트는 같은 `hub.py` 를 기본값만 달리해 부른다.

```
Mac 192.168.20.55                                  PC 192.168.20.5
run_slam.sh --mode live                            hub.py --link direct|manual
  --connect 192.168.20.5:17001  ──TCP──▶  SLAM 포즈 서버 0.0.0.0:17001
  (SLAM 세션 1x = 30 fps 재생)                       ├─ SAM 세션(변환본) 1x 재생 ─▶ SAM-6D
                                                    ├─ 융합: T_w_obj = T_w_slam(s)·X·T_sam_obj
                                                    └─ http://192.168.20.5:8765  (영상 박스·3축 + 3D 지도)
```

| | `run.sh` (기존) | `run_lan.sh` (신규) |
|---|---|---|
| 연결 | `ssh -R 17001` 역터널, Mac 은 `127.0.0.1:17001` 로 접속 | `--link direct`: ssh 로 스트리머만 띄우고 `--connect 192.168.20.5:17001` / `--link manual`: Mac 에서 사람이 실행 |
| 포즈 서버 바인드 | `127.0.0.1` | `0.0.0.0` (LAN) |
| 뷰어 | `http://localhost:8765` | `http://192.168.20.5:8765` (LAN 어디서나) |
| PC 입력 | `~/DeepLearning/Dataset/260826_etri_eightcircle_dark/{SAM,SLAM}` (SDK 원본) | `/mnt/synology/Dataset/data_slam_converted/260826_etri_eightcircle_dark/{SAM,SLAM}` (변환본, `--sam-tau-s -0.0474`) |
| Mac 입력 | `~/Documents/DefenseMeta/Dataset/…/SLAM` | 같은 경로 (Mac 에 `~/Dataset` 은 없음, 2026-09-14 확인) |

```bash
bash objpose/run_lan.sh                  # ssh 키가 등록돼 있으면 direct, 아니면 manual 로 자동 전환
LINK=manual bash objpose/run_lan.sh      # 허브가 출력한 명령을 Mac 에서 실행 (output/<run>/mac_command.txt 에도 남음)
```

manual 모드에서 Mac 에서 실행할 명령(허브가 출력하는 것과 같다):

```bash
~/objpose/run_slam.sh --slam orbslam3 --session ~/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark/SLAM --features 2000 \
    --time-source frame --mode live --connect 192.168.20.5:17001 --rate 1.0
```

허브를 **먼저** 띄운다(포즈 서버가 열려 있어야 Mac 이 붙는다). 허브는 SAM-6D 모델 적재와 Mac 의 hello 를 모두 기다린 뒤
같은 데이터 시각 t0 에서 두 재생을 동시에 시작한다. direct 모드를 쓰려면 이 PC 의 공개키가 Mac 에 등록돼 있어야 한다
(`ssh-copy-id mac`, 현재는 미등록이라 auto 는 manual 로 간다).

변환본 SAM bag 의 시각: 변환기가 컬러 메타데이터 시각에서 18 226 215 158 ns 를 뺐다(`info.json`). 카메라 RT 는
프레임 시계(18 225 662 057 ns 차감)에 τ = −48 ms 를 더한 시각으로 추정됐으므로 변환본에는 `--sam-tau-s -0.0474` 를 준다
(변환본 첫 프레임이 원본 프레임 시계보다 0.553 ms 이르다는 것을 실측으로 확인).

Mac 없이 PC 쪽만 점검: `tools/fake_slam_stream.py` 가 저장된 ORB-SLAM3 궤적(`rt/sim/slam_orb_frameclock_tum.txt`)을
Mac 과 같은 규약으로 실시간 전송한다.

```bash
LINK=manual bash objpose/run_lan.sh --duration-s 60          # 터미널 1
python3 objpose/tools/fake_slam_stream.py --connect 192.168.20.5:17001   # 터미널 2
```

## SLAM 호스트를 Ubuntu 박스 192.168.20.3 으로 (2026-09-14, `run_pc3.sh` + `linux_slam/`)

맥 대신 같은 LAN 의 **Ubuntu 22.04 PC 192.168.20.3**(`jucpark-device`, sshd 10022, ssh 별칭 `slam3`)이 ORB-SLAM3 를 돌리고
포즈를 이 PC(192.168.20.5)로 보낸다. 파이프라인은 `run_lan.sh` 그대로이고(허브·SAM-6D·융합·뷰어 `http://192.168.20.5:8765`
모두 이 PC), SLAM 쪽 실행 파일만 Linux 로 빌드한 것이다.

```
192.168.20.3 (Ubuntu, ~/objpose)                       192.168.20.5 (이 PC)
run_slam.sh -> build/slam_stream (ORB-SLAM3 f2000)     hub.py --link direct|manual  (run_pc3.sh = run_lan.sh + 호스트 값)
  --session ~/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark/SLAM   ├─ SLAM 포즈 서버 0.0.0.0:17001
  --connect 192.168.20.5:17001  ──TCP──▶                  ├─ SAM 세션(변환본) 재생 ─▶ SAM-6D ─▶ 융합·객체 메모리
                                                          └─ http://192.168.20.5:8765
```

| 파일 | 역할 |
|---|---|
| `linux_slam/CMakeLists.txt` | `mac_slam/CMakeLists.txt` 의 Linux 판. 같은 `mac_slam/slam_stream.cc`·`raw_session.h` 를 빌드. `-march=native` 필수. conda 가 PATH 에 있으면 그 lib 을 무시(conda libcurl/libtiff 가 OpenCV→gdal 링크를 깬다) |
| `linux_slam/build_host.sh` | **SLAM 호스트에서** 실행. apt 의존성 → Pangolin 0.9.2(`deps/install`) → ORB-SLAM3 소스 패치 → 빌드 → `~/objpose/run_slam.sh` 심링크. 멱등 |
| `linux_slam/run_slam.sh` | `mac_slam/run_slam.sh` 와 같은 CLI 의 Linux 래퍼(`LD_LIBRARY_PATH`), 허브가 `~/objpose/run_slam.sh` 로 부른다 |
| `linux_slam/deploy.sh` | **이 PC 에서** 실행. 번들(스트리머 소스·ORB-SLAM3 소스·Pangolin 소스·어휘)을 `slam3:~/objpose` 로, 원본 SDK SLAM 세션(12 GB, `~/DeepLearning/dataset/…/SLAM`)을 `slam3:~/Dataset/…/SLAM` 으로 rsync 한 뒤 원격 빌드 |
| `run_pc3.sh` | `SLAM_HOST=192.168.20.3 MAC_SSH=slam3` 로 `run_lan.sh` 를 부른다 |

스트리머는 **원본 SDK 세션**(db3 + `rgbd_timestamp_associations.json`)을 읽으므로 .3 에는 변환본이 아니라 원본을 둔다.

```bash
# (한 번) .3 에 이 PC 의 공개키 등록 — .3 에서:
ssh -p 10022 jucpark@192.168.20.5 cat ~/.ssh/id_ed25519.pub >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys
# (한 번) 배포 + 원격 빌드 — 이 PC 에서 (Pangolin + ORB-SLAM3 빌드, 수 분)
bash objpose/linux_slam/deploy.sh
# 실행 — 이 PC 에서. 뷰어 http://192.168.20.5:8765
bash objpose/run_pc3.sh                     # ssh 가 되면 direct(허브가 .3 에 스트리머를 띄움), 아니면 manual
LINK=manual bash objpose/run_pc3.sh         # .3 에서 허브가 출력한 ~/objpose/run_slam.sh … 명령을 직접 실행
```

이 PC 에서의 자기 검증(2026-09-14): `HOST=local bash objpose/linux_slam/deploy.sh` 로 같은 번들을 `~/objpose` 에 빌드하고,
`LINK=manual` 허브에 그 Linux `slam_stream` 을 붙여 60 s 실행(`output/lan_selftest_260914_090349`):
SLAM 포즈 1,515 OK / 버림 76(추적 실패 프레임), 추적 11.8 ms/프레임, 수신 지연 16 ms, 키프레임 60,
SAM-6D 공급 509 / 처리 157 프레임(추정 나이 중앙 0.46 s), 객체 메모리 활성 5 + 기억 1, 미배치 0, 뷰어 `http://192.168.20.5:8765` 200.

### 2026-09-14 .3 ORB-SLAM3 + .5 SAM-6D 실제 실행 결과 (`output/pc3_260914_100128`, 260826_etri_eightcircle_dark 전체)

| 지표 | 맥 → WSL2 역터널 (09-13 v7) | **.3 → .5 LAN 직결 (09-14)** |
|---|---|---|
| SLAM 포즈 | 7,074 OK / 버림 0 | 7,074 OK / 버림 0 |
| 추적 시간 / 도착 지연 | 10.5 ms / 170 ms | 15.5 ms (i9-11900K) / **26 ms** |
| 키프레임 / kf_update / 루프 클로저 | 162 / 288 / 2 | 157 / 289 / 2 (최대 보정 7.5 cm) |
| SAM-6D 처리 프레임 (공급) | 308 (2,387) | **691** (2,429), 추정 나이 중앙 0.49 s |
| 객체 | 8 종 배치, 미배치 1 | 8 종 배치, 미배치 1 |

같은 8 개 객체의 월드 위치가 두 실행에서 0.4–3.3 cm 안에서 일치한다(Bear 3.3 cm, Dinosaur 2.3 cm, 나머지 ≤ 1.2 cm).
SAM-6D 가 4090 워크스테이션에서 2.2 배 많은 프레임을 처리했고, 도착 지연이 역터널의 1/6 이라 보간 없이도 화면이 따라간다.

### 2026-09-14 두 레인 실행: .3 ORB-SLAM3 + 이 PC hdl_graph_slam, 한 화면 (`output/lidar_orb_260910_260914_101044`)

`run_lidar_orb_260910.sh` 를 ORB 레인만 .3 으로 바꿔 실행했다(`MAC_SSH=slam3 SLAM_HOST=192.168.20.3
MAC_DATASET=/home/jucpark/DeepLearning/Dataset/data_slam_converted/260910_object`, .3 의 스트리머는 변환본 bag 을 그대로 읽는다).
PC 입력은 NAS `data_slam_converted/260910_object` 의 SAM·lidar. 뷰어 `http://192.168.20.5:8765` 에서 레인마다 영상 창과 카드
(ORB-SLAM3 파랑 `#56c2e0`, LiDAR hdl_graph_slam 주황 `#f2a03d`)가 따로 있고, 3D 지도는 LiDAR 월드(z-up)가 기준이며 ORB 궤적은
SAM 카메라 궤적으로 강체 정렬해 겹쳐 그린다. **주의:** 이 실행의 정렬(Kabsch, 처음 15 s, 76 쌍, RMSE 0.04 cm)은 퇴화했다 —
260910 은 처음 약 35 s 동안 카트가 정지해 있어 76 쌍이 사실상 한 점이고 회전이 임의로 정해진다(허브 쪽에서 카메라가 실제로 이동한 뒤
정렬을 맞추도록 수정 중, 다른 세션 담당).

| 레인 | 호스트 | 포즈 | 추적 / 지연 | 키프레임 / kf_update | 루프 | 최대 키프레임 보정 |
|---|---|---|---|---|---|---|
| ORB-SLAM3 f2000 | 192.168.20.3 | 4,772 OK / 버림 0 | 16.6 ms / 20 ms | 178 / 164 | 2 (+72 s, +87 s) | **84.3 cm** |
| hdl_graph_slam | 192.168.20.5 | 1,577 OK / 버림 0 | 62.7 ms / 78 ms | 105 / 80 (loop 12) | 12 | 2.7 cm |

SAM-6D 830 프레임(추정 나이 중앙 0.08 s), 두 레인 모두 8 종 객체 배치(ORB 활성 7, hdl 활성 6·잃음 2, hdl 미배치 1).
재생 끝 시점에 같은 객체의 카메라 기준 위치가 두 레인 사이에서 3.9–11.3 cm 벌어지지만(Bear·Dinosaur·Febreze 10–11 cm),
위 정렬 퇴화 때문에 이 수치는 두 SLAM 의 차이로 해석하지 말 것 — 정렬 수정 후 재실행해서 다시 잰다. hdl 의 "추적" 62.7 ms 는
scan_matching_odometry 의 등록 시간으로, LiDAR 단독 스모크(4 ms)보다 큰 것은 같은 PC 에서 SAM-6D·허브가 함께 돌아 CPU 를 나눠 쓴 탓이다.

## 한 번에 하나씩: `run_one.sh` + 런처 UI (2026-09-14, `http://192.168.20.5:8765`)

두 SLAM 을 동시에 돌리지 않고 **하나씩** 실행하며, 페이지 맨 위 모드 바가 지금 어떤 SLAM 이 돌고 있는지 색과 이름으로 보여 준다.

```
브라우저 ── 8765 ── pc/launcher.py ──중계──► pc/hub.py (127.0.0.1:8766)  뷰어 페이지 · /events · /stats
                        │ /api/state  /api/run?backend=orb|hdl  /api/stop
                        └── run_one.sh orb     : ORB-SLAM3 f2000, Ubuntu 192.168.20.3 (ssh slam3, direct), 포즈 17001
                            run_one.sh hdl     : hdl_graph_slam (VLP-16), 이 PC (conda hdl_graph_slam_jazzy), 포즈 17002
                            run_one.sh hdl-mac : hdl_graph_slam, Mac 192.168.20.55 (ssh mac, direct; ROS 2 는 Mac 안), 포즈 17003
```
런처 제어: `bash objpose/pc/launcherctl.sh status|start [<dataset>] <backend>|stop|restart` (런처 프로세스를 스크립트 경로로만 찾는다).

**입력 데이터 탭 (2026-09-14 추가).** 모드 바 첫 줄은 **입력 데이터셋 탭**이다. 탭을 누르면 그 데이터셋의 백엔드 버튼만 보이고,
실행 중인 데이터셋 탭에는 `● 실행 중` 표시가 붙는다. API 는 `POST /api/run?dataset=<D>&backend=<B>` (dataset 을 생략하면
기존처럼 `260910_object`), 상태는 `GET /api/state` 의 `dataset` · `last_dataset` · `datasets`.
- `260910_object` — orb(.3, 17001) · orb-mac(17004) · hdl-3(17005) · hdl-mac(17003), 외부 파라미터 `rt/260910/*`
- `260826_etri_eightcircle_dark` — 카메라만(lidar 폴더 없음). orb: `.3` 의 `~/Dataset/260826_etri_eightcircle_dark/SLAM`,
  orb-mac: Mac 의 `~/Documents/DefenseMeta/Dataset/260826_etri_eightcircle_dark/SLAM`. 외부 파라미터 `rt/X_slam_sam.json`,
  SAM τ −47.4 ms (= run_pc3.sh / run_lan.sh 와 같은 설정). 실행: `bash objpose/run_one.sh 260826_etri_eightcircle_dark orb`.
12:44 사용자 지시로 페이지의 버튼은 **ORB-SLAM3(.3)** 와 **LiDAR hdl_graph_slam(Mac)** 둘만 남겼다(`launcher.py` 의 `hdl` 항목 주석 처리;
`run_one.sh hdl` 은 명령줄에서 여전히 된다). 3D 지도 창은 영상 창 옆 같은 줄에 놓인다.

- `pc/launcher.py` 는 허브 자식 프로세스를 **하나만** 가진다. 다른 백엔드 버튼을 누르면 현재 허브에 SIGINT(summary.json 저장) →
  종료 확인 → 새 허브 시작. `run_one.sh` 도 다른 hub.py 가 살아 있으면 시작을 거부한다.
- 페이지는 허브의 `web/index.html` 그대로(디스크 수정 없음)이고, 런처가 응답에 `<div id="launcher-bar">` 와 `web/launcher.js` 를
  끼워 넣는다. 모드 바: "실행 중: ORB-SLAM3 / LiDAR hdl_graph_slam"(백엔드 색: ORB 파랑 `#56c2e0`, hdl 주황 `#f2a03d`),
  백엔드별 버튼(바꾸기·다시 시작), 멈춤, 결과 디렉터리. 실행이 바뀌면 페이지가 스스로 새로고침해 이전 레인이 남지 않는다.
- 입력: NAS `data_slam_converted/260910_object` 의 SAM/lidar(이 PC), `.3:/home/jucpark/DeepLearning/Dataset/data_slam_converted/260910_object/SLAM`(ORB).

```bash
/home/jucpark/anaconda3/envs/sam6d/bin/python objpose/pc/launcher.py --backend hdl      # 런처 + hdl 부터 시작 (8765)
bash objpose/run_one.sh orb   /   bash objpose/run_one.sh hdl                            # 런처 없이 하나만 (8765 직접)
```

hdl 단독 실행(`output/lidar_orb_260910_260914_103254`, 10:33): 포즈 1,576 OK / 버림 1, 등록 3.4 ms(단독), 도착 지연 29 ms,
키프레임 99 / 갱신 79, 최대 보정 2.7 cm, SAM-6D 831 프레임, 객체 메모리 활성 4 · 놓침 4 · 삭제 1.

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

## 라이다 = hdl_graph_slam, ORB 와 같은 화면에서 (260910_object, 2026-09-14, `run_lidar_orb_260910.sh`)

"라이다 SLAM" 은 KISS-ICP 가 아니라 **hdl_graph_slam** 이다. Mac 에는 ROS 가 없고 .3 은 ssh 가 막혀 있어 **이 PC(.5)** 에 올렸다.

```
Mac 192.168.20.55                                    PC 192.168.20.5 (이 PC)
run_slam.sh --slam orbslam3 (SLAM bag)  ──TCP──▶ :17001 ─┐
                                                          ├─ hub.py  레인 orb  (rt/260910/X_slam_sam_260910.json)
lidar/run_hdl_stream.sh (이 PC, conda hdl_graph_slam_jazzy)│          레인 hdl  (rt/260910/X_lidar_sam.json)
  bag 재생 ─▶ prefiltering ─▶ fast_gicp odometry ─▶       │   SAM bag 재생 ─▶ SAM-6D (한 번) ─▶ 두 레인에 각각 융합·객체 메모리
  graph_slam_node (g2o 2 s, 루프 클로저)  ──TCP──▶ :17002 ─┘   http://192.168.20.5:8765  영상 2개 + 3D 지도 1개 (색으로 구분)
```

| 구성 | 내용 |
|---|---|
| conda `hdl_graph_slam_jazzy` | robostack Jazzy `ros-jazzy-ros-base` + pcl 1.15 + g2o + suitesparse (`_env_specs/hdl_graph_slam_humble.requested_cmds.txt` 의 Jazzy 판). conda g2o 에 cholmod 솔버가 없어 `CMakeLists.txt`/`graph_slam.cpp` 를 eigen+csparse 로 바꿈 |
| `hdlgraphslam_ws` | ndt_omp · fast_gicp · hdl_graph_slam(ROS 2 포트) colcon Release 빌드. `graph_slam_node` 에 `/hdl_graph_slam/keyframes`(nav_msgs/Path, 최적화 때마다 전체 키프레임 포즈) 추가 |
| `lidar/hdl_stream.py`, `lidar/run_hdl_stream.sh` | 허브 프로토콜 스트리머. `ros2 launch hdl_graph_slam_501.launch.py` 를 띄우고 bag 의 `/velodyne_points` 를 직렬화된 채로 10 Hz 재생(+`/clock`), `/odom` 마다 포즈 = `T_map_kf(ref)·T_odom_kf(ref)⁻¹·T_odom_c` (ref = 직전 키프레임, ORB 와 같은 키프레임 앵커), keyframes Path 마다 `kf_update`(loop / optimize / periodic). rclpy 는 **SingleThreadedExecutor** 여야 한다 — MultiThreaded 는 GIL 을 잡아 재생 루프가 1 스캔/s 로 떨어졌다 |
| `pc/hub.py` 레인 | 기존 인자 = 레인 0, `--lane name=hdl,slam=hdl,port=17002,ssh=local,session=…,extrinsic=…` 로 추가. `--link local`. 레인마다 포즈 버퍼·융합·객체 메모리·로그(`slam_poses_<name>.jsonl` …). 표시용 정렬: 기준 = 첫 라이다 레인(z-up), 다른 레인은 SAM 카메라 궤적으로 Kabsch 정렬(겹침 15 s 뒤 고정, `--align-span-s`), 그 뒤 두 SLAM 의 차이가 그대로 보인다 |
| `pc/web/index.html` | 레인마다 영상 패널 + SLAM 카드, 3D 지도 하나(궤적·카메라·박스 테두리 = 레인 색, 박스 채움 = 객체 색, 레인 켜고 끄기), 객체 표에 SLAM 열 |

```bash
bash objpose/run_lidar_orb_260910.sh                 # ORB(Mac, ssh direct) + hdl(이 PC), 뷰어 http://192.168.20.5:8765
LANES=hdl bash objpose/run_lidar_orb_260910.sh       # 라이다만 (Mac 불필요)
LANES=orb bash objpose/run_lidar_orb_260910.sh       # ORB 만
```

입력: 이 PC 는 NAS `/mnt/synology/Dataset/data_slam_converted/260910_object/{SAM,SLAM,lidar}`, Mac 은 `~/Documents/DefenseMeta/Dataset/260910_object/SLAM`.
hdl 옵션은 `params=kf_trans=0.5;kf_angle=0.25;far_thresh=30;reg_threads=8;floor=true` (0807 실행값과 같음). 실행이 끝나면
`output/<run>/hdl_dump/`(최적화 그래프, `keyframes_tum.txt`) 가 남는다.

### hdl_graph_slam 을 Mac(.55)에서 (2026-09-14, `lidar/build_hdl_mac.sh`)

사용자 지시: 라이다 SLAM 은 Mac 에서, SAM-6D·허브·뷰어는 이 PC. Mac 에는 ROS 가 없었으므로(brew 에 g2o·pcl·boost·eigen 만 있었음):

| 단계 | 내용 |
|---|---|
| micromamba `~/micromamba`, env `hdl_graph_slam_jazzy` | robostack-jazzy osx-arm64: ros-jazzy-ros-base, pcl 1.15, suitesparse, llvm-openmp, colcon … (conda-forge 에 osx-arm64 **g2o 가 없다**) |
| `lidar/build_hdl_mac.sh` (Mac 에서 실행) | g2o `20200410_git` 를 소스 빌드해 env 에 설치(CMake 정책 3.5, x86 `-msse*` 줄 제거) → `~/objpose/hdlgraphslam_ws` colcon 빌드. **Apple clang + conda libomp** 로 빌드한다: conda 의 ld64 는 macOS 26 SDK 의 libSystem.tbd(`arm64e.x1`)를 읽지 못한다. 노드 실행 파일에 `-lpython3.12` 를 링크한다: robostack 의 `*__rosidl_generator_py` dylib 가 flat namespace 로 Python 심볼을 요구해 `_PyExc_RuntimeError` 로 죽는다. `deps/COLCON_IGNORE` 로 g2o 체크아웃을 colcon 에서 제외 |
| 소스 패치 (Linux 도 공유) | `ndt_omp`·`hdl_graph_slam` CMake 의 `-msse*` 를 x86 에서만 붙이도록(arm64 제외) |
| `lidar/run_hdl_stream.sh` | micromamba 면 `micromamba activate`, 아니면 anaconda; 워크스페이스는 `../../hdlgraphslam_ws` 또는 `../hdlgraphslam_ws`(Mac 배포본) |
| `lidar/hdl_stream.py` 예열 | hello 전에 첫 스캔을 오도메트리가 응답할 때까지 보낸다(`--warmup-s 20`). Mac 에서 첫 스캔이 DDS·fast_gicp 초기화로 ~10 s 걸려 시작 직후 91 스캔을 버렸던 문제 해결 |

Mac 스트리머 단독 검증(이 PC 의 테스트 허브로 200 스캔): 버림 0, 정합 12.7 ms, 도착 지연 중앙 55 ms(최대 67), kf_update 정상.
실행: `LANES=hdl LIDAR_HOST=mac bash objpose/run_lidar_orb_260910.sh` (포즈 포트 17003) 또는 launcher 의 `hdl-mac` 백엔드.

### KISS-ICP vs hdl_graph_slam (260910_object, 2026-09-14)

같은 SAM-6D 검출(오늘 hdl 실행의 830 프레임)을 두 궤적에 얹고 ORB-SLAM3 최종 궤적을 기준으로 `lidar/eval_lidar_variants.py` 로 채점(cm):

| 지표 | KISS-ICP (v5) | hdl 최종(루프 반영) | hdl 실시간 |
|---|---|---|---|
| 처리 / 스캔 | 12 ms (Mac) | 4 ms 단독 · 25 ms ORB+SAM-6D 동시(이 PC) | |
| 도착 지연 중앙 | 24 ms | 58 ms | |
| 고주파 흔들림 RMS | 0.43 | 0.46 | 0.47 |
| ORB 대비 고주파 차 RMS | 0.26 | 0.33 | 0.36 |
| ORB 대비 저주파 RMSE | 3.25 | 4.40 | 4.93 |
| 객체 흩어짐 중앙 / p90 | 1.06 / 1.80 | 1.56 / 2.84 | 2.07 / 3.46 |
| 루프 클로저 | 없음 | 12회, 최대 보정 2.7 cm | |

짧은 실내 세션이라 KISS-ICP 가 더 매끄럽다; hdl 의 이점(루프 보정, 키프레임 앵커 재배치)은 긴 주행에서 나온다. hdl 수치는 ORB(.3)·SAM-6D 와 같은 PC 에서 동시에 돈 조건.

### 루프 클로저 후 객체 분열 — F1·F2 적용과 "루프 전 관측 가중치" 실측 (2026-09-14, `docs/loop_split_analysis_260914.md`)

- 원인: 허브 버그가 아니라 ORB-SLAM3 의 두 번째 루프(85.5 s)가 드리프트를 남긴 채 닫혀 지도가 0.58 m + 17.5° 로 갈라진 것(로그만으로 허브 파이프라인을 mm 단위로 재현해 확인).
- **F1** `pc/fusion.py` KeyframeMap.apply_update: kf_update 목록의 최대 id 보다 큰 키프레임은 컬링이 아니라 "메시지 이후 생성" (거짓 컬링·잘못된 리다이렉트 제거, 결과 무회귀).
- **F2** `--assoc-gate-m` 기본 0.15 → 0.30 m, objectmemory `core/fusion.py` 러닝 평균 가중치 하한 0.05 (`OBJMEM_MIN_FUSE_WEIGHT`, 0 = 예전). 오프라인 재생: 정상 ORB 실행 4개·라이다 2개 모두 8 = 8종 클래스당 1개(이전 125210 머그컵 2개), 지도가 갈라진 162947 만 2개씩 남음. 실기 재실행(Mac f3000, `one_260910_object_orb-mac_f3000_260914_170121`): 루프 2회(1.42 + 2.52 m)에도 8 = 8종, 라이다 대비 객체 오차 중앙 4.8 cm.
- **루프 전 관측 가중치** `w = 1/(1+(d/d0)²)` (d = 앵커 키프레임 누적 보정, `pc/memory.py` `OBJPOSE_LOOP_W_D0_M`, objectmemory 검출 `weight` 필드): 구현했지만 **기본은 끔(0)**. 오프라인(ORB 실행 4개)에서는 어떤 d0 에서도 라이다 대비 오차가 나빠졌고(4.3–4.8 → 4.7–5.9 cm), 실기 1회(`…_170910`)는 중앙 4.6 → 3.3 cm 로 좋아졌으나 평균은 같음(4.1 cm; 가까운 물체 개선, 먼 물체 악화). 이유: 최종 지도에서 루프 전 관측 무리가 루프 후 무리보다 라이다 정답에 가까운 물체가 많다 — ORB 지도는 루프 후에도 국소적으로 5–9 cm 뒤틀려 있어 "루프 전 = 나쁨" 이 성립하지 않는다. 남은 오차의 주범은 지도 뒤틀림이므로 F3(갈라짐 탐지)·F4(SLAM 측 루프 검증)·ThDepth 로 간다.
