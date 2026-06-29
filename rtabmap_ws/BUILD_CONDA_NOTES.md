# rtabmap_ws build notes (conda `rtabmap` environment)

이 문서는 시스템 ROS(`/opt/ros/humble`)가 아니라 **conda `rtabmap` 환경(robostack 스타일 ROS 2 Humble)**으로
`rtabmap_ws`를 빌드·검증한 절차와, 그 과정에서 해결한 문제를 기록한다.
(시스템 ROS 기준 절차는 `TRANSFER_NOTES.md` 참고.)

검증 완료: 9개 패키지 전부 빌드 성공, `rtabmap` 노드 정상 기동 확인 (2026-06-17).

## 환경 전제

- conda env `rtabmap` 사용 (`conda activate rtabmap`).
  - robostack 스타일이라 **activate 시 ROS 2 Humble이 자동 source**된다
    (`ROS_DISTRO=humble`, `rclpy` import OK). 별도 `source /opt/ros/humble/setup.bash` 불필요.
  - colcon / ros2 / cmake(4.2.3) / gcc(conda cross-compiler) / PCL / OpenCV 4.11 모두 env 안에 존재.
  - Python 3.11, numpy 1.26 + 개발 헤더(`include/python3.11/Python.h`, `lib/libpython3.11.so`) 존재.

## 빌드 명령 (이대로 재현 가능)

```bash
source /home/ldh9501/miniconda3/etc/profile.d/conda.sh
conda activate rtabmap
cd /home/ldh9501/temp_ws/CLI_environment/rtabmap_ws
rm -rf build install log              # 클린 빌드 시

PYV=3.11
export MAKEFLAGS="-j6"                 # 가용 메모리(~18GB) 고려한 병렬도 제한 (코어 빌드 OOM 방지)
colcon build --parallel-workers 4 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python" \
  -DPython_INCLUDE_DIR="$CONDA_PREFIX/include/python${PYV}" \
  -DPython_LIBRARY="$CONDA_PREFIX/lib/libpython${PYV}.so" \
  -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=BOTH \
  -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH \
  -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH
```

- 소요 시간: 약 20분 (rtabmap 코어 ~7분이 최대). 48코어지만 메모리 때문에 `-j6`로 제한.
- 빌드 후: `source install/setup.bash`

## 해결한 문제

### 1) `find_package(Python)` 실패 → Python 힌트 주입 (필수)

증상: 무수정 `colcon build` 시 `rtabmap_msgs`의 `rosidl_generator_py` 단계에서 cmake config 실패.
```
CMake Error: Could NOT find Python (missing: Python_EXECUTABLE Python_INCLUDE_DIRS
  Python_LIBRARIES ... Interpreter Development NumPy ...)
```

원인: robostack의 conda 크로스 컴파일러 활성화가 `CONDA_BUILD_SYSROOT`/`CMAKE_PREFIX_PATH`에
sysroot를 설정 → ament/rosidl 빌드 중 `CMAKE_FIND_ROOT_PATH`가 sysroot로 제한되어
`find_package(Python)`이 **인터프리터조차 sysroot 안에서만 찾다 실패**한다
(그래서 `Python_EXECUTABLE` 포함 전 컴포넌트가 missing).
필요한 파일(Python.h, libpython3.11.so, numpy 헤더)은 env에 전부 존재하므로 누락이 아니라 **탐색 제한** 문제.

해결: 위 빌드 명령처럼 Python 실행파일/헤더/라이브러리를 명시 힌트로 주고
`CMAKE_FIND_ROOT_PATH_MODE_{PROGRAM,LIBRARY,INCLUDE}=BOTH`로 sysroot 밖도 탐색하게 한다.

### 2) `aruco_msgs` / `aruco_opencv_msgs` 누락 → 무시 가능 (조치 불필요)

`rtabmap_slam/package.xml`이 두 패키지를 `<depend>`하지만 conda env엔 없다.
그러나 `rtabmap_slam/CMakeLists.txt`에서 `find_package(aruco_msgs)`는 **REQUIRED가 아니고**
`IF(aruco_msgs_FOUND)` 조건부 컴파일이라, 없어도 빌드는 통과하고 **아루코 마커 기능만 빠진다**.
SLAM 동작/검증에는 불필요. (필요하면 robostack 채널에서 별도 설치.)

### (참고) 남는 경고

`CMP0148`(FindPythonInterp/Libs deprecated) dev 경고는 무해. 빌드/실행에 영향 없음.

## 작동 검증 결과 (2026-06-17)

`conda activate rtabmap && source install/setup.bash` 상태에서:

- ros2 패키지 인식: `rtabmap_msgs, rtabmap_conversions, rtabmap_sync, rtabmap_odom,
  rtabmap_slam, rtabmap_util, rtabmap_launch, rtabmap_ros` 8개 전부 OK.
- launch 파일: `rtabmap_launch/launch/rtabmap.launch.py` 존재.
- 실행 노드: `rtabmap_slam rtabmap`, `rtabmap_odom {rgbd,icp,stereo}_odometry`.
- 메시지 로드: `ros2 interface show rtabmap_msgs/msg/Info` OK.
- 노드 실제 기동: `ros2 run rtabmap_slam rtabmap --help` → 정상 출력
  (공유 라이브러리 전부 로드, undefined symbol 없음). **링크/런타임 정상의 결정적 증거.**

검증 스크립트는 `/tmp/verify_rtabmap.sh` 참고.

## 실행 시 주의 (scripts 경로 하드코딩)

`scripts/run_four_slam_bags.py`는 절대경로/`ENV_PREFIX`가 `/home/etri/...`로 하드코딩되어 있어
다른 PC에서는 수정 필요(상세는 `TRANSFER_NOTES.md`의 "Hard-coded path issue" 절 참고).
