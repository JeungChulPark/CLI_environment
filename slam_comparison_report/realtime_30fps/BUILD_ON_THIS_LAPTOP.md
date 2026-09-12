# 이 노트북에서 ORB-SLAM3 / RTAB-Map 빌드하기 (검증 완료 2026-09-12)

대상: i9-12900HK (AVX2, **AVX-512 없음**) + RTX 3080 Ti Laptop + WSL2 + anaconda3.
`_env_specs/`의 원본 사양은 **이 머신에서 그대로 쓸 수 없다** (사유는 8장).

## 1. 결과

| 항목 | 상태 |
|---|---|
| conda env `orbslam3` (3.6 GB) | ✅ |
| ORB-SLAM3 Thirdparty (DBoW2 / g2o / Sophus) | ✅ |
| `orbslam3_core` + `orbslam3_ros2` | ✅ undefined symbol **0**, 런타임 기동 확인 |
| conda env `rtabmap` (6.3 GB) | ✅ |
| RTAB-Map 9개 패키지 | ✅ undefined symbol **0**, 런타임 기동 확인 |
| **Phase 0.1 (`Version.h` 삭제)** | ✅ 적용 및 **런타임 검증** |

## 2. ORB-SLAM3 환경 생성

```bash
source ~/anaconda3/etc/profile.d/conda.sh
conda create -y -n orbslam3 --solver=libmamba --override-channels \
  -c conda-forge -c robostack-humble --strict-channel-priority \
  python=3.11 pip git wget cmake ninja make cxx-compiler pkg-config \
  colcon-common-extensions \
  ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport \
  opencv=4.11 eigen=3.4.0 suitesparse pangolin-opengl
conda install -y -n orbslam3 --solver=libmamba --override-channels \
  -c conda-forge -c robostack-humble --strict-channel-priority \
  libboost-devel libgl-devel libegl-devel libopengl-devel libglvnd-devel epoxy glew
```

- `boost-cpp`를 **넣지 말 것** (humble의 `libboost-devel 1.86`과 충돌).
- `pangolin-opengl` 이 conda-forge에서의 Pangolin 이름이다 (`pangolin` 아님).
- `libglvnd-devel` 필수 — `orbslam3_ros2`의 `find_library(GLdispatch)`가 `.so` 심볼릭 링크를 요구하는데
  런타임 패키지엔 `.so.0` 만 있다.

## 3. ORB-SLAM3 누락 파일 보충 (필수)

```bash
cd orbslam_ws/src/ORB_SLAM3
# (a) vocabulary (gitignore 되어 있음)
mkdir -p Vocabulary && cd Vocabulary
curl -sL -o ORBvoc.txt.tar.gz \
  https://raw.githubusercontent.com/UZ-SLAMLab/ORB_SLAM3/master/Vocabulary/ORBvoc.txt.tar.gz
tar -xf ORBvoc.txt.tar.gz     # -> ORBvoc.txt 145 MB
# (b) 전송 과정에서 빠진 소스 4개
cd ../Thirdparty/DBoW2
B=https://raw.githubusercontent.com/UZ-SLAMLab/ORB_SLAM3/master/Thirdparty/DBoW2
curl -sL -o DUtils/Timestamp.cpp $B/DUtils/Timestamp.cpp
curl -sL -o DUtils/Timestamp.h   $B/DUtils/Timestamp.h
cd ../g2o
B=https://raw.githubusercontent.com/UZ-SLAMLab/ORB_SLAM3/master/Thirdparty/g2o
curl -sL -o g2o/stuff/timeutil.cpp $B/g2o/stuff/timeutil.cpp
curl -sL -o g2o/stuff/timeutil.h   $B/g2o/stuff/timeutil.h
```

## 4. ORB-SLAM3 빌드

```bash
conda activate orbslam3
export CC=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc
export CXX=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++   # CC/CXX가 gcc-12로 미리 설정돼 있으므로 덮어써야 함
export MAKEFLAGS="-j6"
cd orbslam_ws/src/ORB_SLAM3
for d in DBoW2 g2o Sophus; do
  cd Thirdparty/$d && rm -rf build && mkdir build && cd build
  extra=""; [ "$d" = Sophus ] && extra="-DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF"
  cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
           -DCMAKE_C_COMPILER=$CC -DCMAKE_CXX_COMPILER=$CXX $extra && make -j6
  cd ../../..
done
cd ../..   # orbslam_ws
colcon build --parallel-workers 2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_C_COMPILER=$CC -DCMAKE_CXX_COMPILER=$CXX \
  -DPython_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=BOTH \
  -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH \
  -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH
```

실행 (기본 경로가 `$HOME/orbslam3_ws/...` 로 하드코딩돼 있어 **반드시 명시**):

```bash
source install/setup.bash
ros2 run orbslam3_ros2 rgbd_node --ros-args \
  -p vocabulary_path:=$PWD/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  -p settings_path:=$PWD/data/orbslam3_260714_105018_rgbd.yaml
```
(`Examples/` 디렉터리는 이 저장소에 없다. `data/`의 D455 설정을 쓴다.)

## 5. RTAB-Map 환경 생성

`_env_specs/`에 rtabmap 사양이 **없어** 새로 구성했다.

```bash
conda create -y -n rtabmap --solver=libmamba --override-channels \
  -c conda-forge -c robostack-humble --strict-channel-priority \
  python=3.11 pip git cmake ninja make cxx-compiler pkg-config \
  colcon-common-extensions numpy \
  ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-pcl-conversions ros-humble-image-geometry ros-humble-laser-geometry \
  ros-humble-diagnostic-updater ros-humble-octomap-msgs ros-humble-grid-map-ros \
  ros-humble-apriltag-msgs ros-humble-nav2-msgs ros-humble-tf2-eigen \
  ros-humble-tf2-geometry-msgs ros-humble-stereo-msgs ros-humble-pluginlib \
  ros-humble-rclcpp-components ros-humble-message-filters ros-humble-visualization-msgs \
  opencv=4.11 eigen=3.4.0 libboost-devel \
  pcl vtk sqlite zlib octomap g2o ceres-solver suitesparse
conda install -y -n rtabmap --solver=libmamba --override-channels -c conda-forge "empy=3.3.4"
```

- `empy` 는 **3.3.4로 고정**해야 한다. 4.x는 `rosidl_adapter`와 비호환.
- `libpointmatcher` 는 conda-forge에 없다 → ICP는 PCL 폴백.
- `aruco_msgs` / `aruco_opencv_msgs` 없어도 된다 (조건부 컴파일, 아루코 기능만 빠짐).

## 6. Phase 0.1 — 반드시 먼저 (안 하면 g2o·GTSAM·ORB_SLAM 전부 꺼짐)

```bash
git rm --cached rtabmap_ws/src/rtabmap/corelib/include/rtabmap/core/Version.h
rm -f          rtabmap_ws/src/rtabmap/corelib/include/rtabmap/core/Version.h
echo 'rtabmap_ws/src/rtabmap/corelib/include/rtabmap/core/Version.h' >> .gitignore
```

**검증된 효과** (빌드 전 → 후):

| | 커밋본 | CMake 생성본 | 런타임 |
|---|---|---|---|
| `RTABMAP_G2O` | `//#define` | **`#define`** | `Optimizer/Strategy = 1` (g2o) |
| 이전 상태 | — | — | `0` (TORO, 100 iteration) |

`ldd librtabmap_core.so` 에 g2o 라이브러리 10개가 실제로 링크된 것으로 최종 확인.

## 7. RTAB-Map 빌드

```bash
conda activate rtabmap
# WSL2: Windows PATH를 제거해야 CMake가 Windows SDK를 오검출하지 않는다
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '^/mnt/[a-z]/' | paste -sd:)"
export CC=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc
export CXX=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++
export MAKEFLAGS="-j6"
cd rtabmap_ws && rm -rf build install log
colcon build --parallel-workers 3 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_C_COMPILER=$CC -DCMAKE_CXX_COMPILER=$CXX \
  -DWITH_ZED=OFF -DWITH_ZEDOC=OFF \
  -DCMAKE_CXX_FLAGS="-I$CONDA_PREFIX/include/g2o/EXTERNAL/csparse -include cstdint" \
  -DPython_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPython_INCLUDE_DIR="$CONDA_PREFIX/include/python3.11" \
  -DPython_LIBRARY="$CONDA_PREFIX/lib/libpython3.11.so" \
  -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=BOTH \
  -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH \
  -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH
```
소요 6분 40초, 9개 패키지 전부 성공.

**두 CXX 플래그의 이유를 반드시 이해할 것:**

- `-I .../g2o/EXTERNAL/csparse` — `cs.h` 가 두 곳에 있고 **버전이 다르다**.
  `suitesparse/cs.h` 는 CXSparse **v4**, g2o 번들은 CSparse **v3**.
  `ldd libg2o_csparse_extension.so` 는 `libg2o_ext_csparse.so`(v3)를 가리킨다.
  suitesparse 쪽을 쓰면 **컴파일은 되지만 struct 레이아웃이 달라 런타임에 메모리가 깨진다.**
- `-include cstdint` — rosidl 생성 헤더
  (`builtin_interfaces/msg/detail/time__struct.hpp`)가 `uint32_t` 를 쓰면서 `<cstdint>` 를
  include하지 않는다. libstdc++ 15는 전이적으로 주지 않는다 (GCC 13+ 회귀).
  ORB-SLAM3는 include 순서 덕에 우연히 피해 간다.

## 8. 원본 사양을 쓸 수 없는 이유 + 해결한 결함 10건

| # | 문제 | 성격 | 해결 |
|---|---|---|---|
| 1 | 🔴 `orbslam3.full.yml` 이 `_x86_64-microarch-level=4` (**AVX-512**) 요구 | 원본 PC는 AVX-512 CPU. Alder Lake는 v3 | 사양 폐기, v3로 재구성 |
| 2 | `explicit.txt` → `InvalidVersionSpec: '1%21164.3095'` (x264 epoch) | conda 24.1.2 파서 | 버전 조합만 추출 |
| 3 | `requested_cmds.txt` solver 충돌 | `boost-cpp` ↔ `libboost-devel 1.86` | `boost-cpp` 제외 |
| 4 | `Vocabulary/ORBvoc.txt` 없음 | gitignore (대용량) | upstream 다운로드 |
| 5 | `DBoW2/DUtils/Timestamp.{cpp,h}` 없음 | 전송 누락 | upstream 보충 |
| 6 | `g2o/stuff/timeutil.{cpp,h}` 없음 | 전송 누락 | upstream 보충 |
| 7 | `empy 4.2.1` ↔ `rosidl_adapter` | `_interpreter.shutdown()` AttributeError | `empy=3.3.4` |
| 8 | 🔴 WSL2 Windows `PATH` 44개 유입 → ZED SDK 오검출 | **이 환경 고유** | PATH 필터 + `WITH_ZED=OFF` |
| 9 | 🔴 `cs.h` CSparse v3 vs CXSparse v4 | **조용히 런타임에 깨지는 유형** | g2o 번들 헤더 사용 |
| 10 | rosidl 헤더 `<cstdint>` 누락 | libstdc++ 15 회귀 | `-include cstdint` |

1·9번은 **빌드가 성공한 뒤 런타임에야 드러났을** 종류다.

## 9. 런타임으로 확인된 기본값 (분석 보고서의 소스 판독과 일치)

```
Optimizer/Strategy     = 1     (g2o)   <- Phase 0.1 효과. 이전엔 0 (TORO)
Optimizer/Iterations   = 20
Rtabmap/DetectionRate  = 1     (Hz)    <- 30 FPS 중 29프레임은 매핑에 안 들어감
Rtabmap/TimeThr        = 0     (무제한) <- Phase 5.1 대상
Mem/STMSize            = 10
Reg/Strategy           = 0     (Vis)   <- ICP가 odometry 경로에 없음. 좋음
Vis/FeatureType        = 6     (GFTT/BRIEF) <- Phase 5.2 대상 (→2 ORB)
Kp/DetectorStrategy    = 6     (GFTT/BRIEF) <- 반드시 위와 함께 변경
Kp/MaxFeatures         = 500
Vis/CorType            = 0     (Features Matching)
Vis/MinInliers         = 20
wait_for_transform     = 0.2   (초)    <- Phase 3.2 대상. 200 ms P99 위험
```

## 10. 다음 단계

Phase 1 프로파일링. 두 시스템 모두 빌드·기동되므로 이제 **추정이 아닌 실측**이 가능하다.
남은 준비물은 입력 데이터(bag) 뿐이며,
`rtabmap_ws/scripts/run_four_slam_bags.py` 의 `/home/etri/...` 하드코딩 경로 수정이 필요하다
(`TRANSFER_NOTES.md` 참고).
