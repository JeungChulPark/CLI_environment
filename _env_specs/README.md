# conda 환경 재구축 가이드

이 워크스페이스 묶음(orbslam_ws / hdlgraphslam_ws / sam6d_ws / objectmemory_ws)을
새 PC에서 돌리기 위한 conda 환경 사양서.

## 전제: ROS2는 시스템이 아니라 conda에 있다

원본 PC에 `/opt/ros` 가 **없다**. ROS2 Humble은 전부 conda-forge + **robostack-humble**
채널로 설치돼 있다. 따라서 새 PC에서도 apt로 ROS를 깔 필요가 없고, 아래 환경만 만들면 된다.
(`sam_yolo` 만 예외로 ROS 없이 pip 위주)

## 환경 5개

| env | 원본 크기 | ROS | 쓰는 곳 |
|---|---|---|---|
| `orbslam3` | 832 MB | Humble | ORB-SLAM3 빌드 + 실행 (`orbslam_ws`) |
| `hdl_graph_slam_humble` | 4.2 GB | Humble | hdl_graph_slam 빌드 + 실행 (`hdlgraphslam_ws`) |
| `sam_yolo` | 5.6 GB | 없음 | SAM-6D ISM 배치 + YOLO-World 사이드카 |
| `sam6d_ros_humble` | 15 GB | Humble | SAM-6D PEM 배치, ROS 노드, `objectmemory_ws` (pytest 포함) |
| **`sam6d`** | **17 GB** | **Jazzy** | **SAM-6D 실시간 노드(ISM+PEM 한 프로세스, 사이드카 불필요)** — 2026-08-13 신설 |

`sam6d` 는 위 `sam_yolo` + `sam6d_ros_humble` 조합을 **하나로 합친 Jazzy 판**이다.
이식 대상 노트북이 Jazzy이기 때문에 만들었고, 실시간 경로만 이 환경을 쓴다.
오프라인 배치 경로(`integration --stage sam`)는 여전히 humble 두 환경을 쓴다.
자세한 구축 순서와 함정은 `sam6d.requested_cmds.txt` 참조 — 특히 **pointnet2 재빌드는
반드시 `conda activate sam6d` 상태에서** 해야 한다(conda-forge torch가 헤더 경로를
`CONDA_PREFIX` 기준으로 잡기 때문).

## 파일 설명 (env 당 최대 4종)

- **`<env>.requested_cmds.txt`** — 원본 PC에서 이 환경을 만들 때 실제로 친 conda/mamba 명령.
  **재구축은 이걸 그대로 실행하는 게 1순위.** 버전 핀이 느슨해 새 드라이버/GPU에 맞게 풀린다.
- `<env>.full.yml` — 빌드 문자열까지 박힌 완전 고정본. `conda env create -f`.
  같은 플랫폼(linux-64)에서 원본과 100% 동일하게 재현되지만, 아래 "GPU 주의" 참조.
- `<env>.explicit.txt` — 패키지 URL 목록. `conda create -n <env> --file <파일>`. 가장 엄격.
- `<env>.pip.txt` — `pip freeze` 결과. conda 환경을 만든 뒤 `pip install -r` 로 얹는다.

`sam6d_ros_humble.history.yml` 은 **없다**. conda의 알려진 버그
(`CondaValueError: Requested package 'pillow' is not found in 'explicit_packages'`)로
`--from-history` export가 실패한다. 대신 `requested_cmds.txt` 를 쓸 것.

## 권장 절차

```bash
# 1) 원본 생성 명령 그대로 (권장)
cat orbslam3.requested_cmds.txt          # 내용 확인 후 한 줄씩 실행
cat hdl_graph_slam_humble.requested_cmds.txt
cat sam6d_ros_humble.requested_cmds.txt

# 2) sam_yolo 는 conda 생성 후 pip 로 채운다
conda create -n sam_yolo python=3.11 -y
conda activate sam_yolo
pip install -r sam_yolo.pip.txt

# 3) 나머지 env 의 pip 의존성 얹기
conda activate sam6d_ros_humble && pip install -r sam6d_ros_humble.pip.txt
```

1번이 실패하면 `conda env create -f <env>.full.yml` 로 대체.

## GPU 주의 — 여기가 제일 깨지기 쉽다

원본 PC: **NVIDIA RTX PRO 6000 Blackwell Max-Q**, 드라이버 **580.159.04**.

- `sam6d_ros_humble`: `pytorch=2.7.1=cuda129_generic_py311` (conda-forge, CUDA 12.9)
- `sam_yolo`: `torch==2.12.0`, `torchvision==0.27.0` (pip, CUDA 13 계열)

두 환경의 CUDA 계열이 **서로 다르다**(12.9 vs 13). 의도된 분리이므로 통일하려 하지 말 것.
새 PC의 GPU/드라이버가 다르면 `full.yml`/`explicit.txt` 의 고정 빌드가 오히려 안 맞을 수 있다.
그 경우 `requested_cmds.txt` 로 느슨하게 다시 풀어서 설치하는 쪽이 안전하다.

`sam_yolo.pip.txt` 에는 git 의존성이 하나 있다:
`clip @ git+https://github.com/ultralytics/CLIP.git@577b3cfa...` — 설치 시 네트워크 필요.

## ORB 빌드 관련

`orbslam_ws/src/orbslam3_ros2/CMakeLists.txt` 가 `$ENV{CONDA_PREFIX}/lib/libOpenGL.so.0` 를
참조한다. 즉 **`conda activate orbslam3` 한 상태에서 `colcon build`** 해야 OpenGL/Pangolin이
잡힌다. `orbslam3.requested_cmds.txt` 에 glew/libgl-devel/libegl-devel/xorg-* 설치 줄이
들어 있으니 빠뜨리지 말 것.
