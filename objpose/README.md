# objpose (Mac side)

Mac-side SLAM streamers for the object-pose hub. The hub on 192.168.219.100 runs them over ssh and receives poses through `ssh -R` tunnels.
This folder mirrors `~/objpose` on the Mac, so copying it there gives a runnable setup.

It was captured on 2026-09-13 from a MacBook Pro (M4 Pro) running macOS 26.6.2, arm64.

## Layout

| Path | What it is |
|---|---|
| `run_slam.sh` | Wrapper that runs `--slam orbslam3` (default) or `--slam rtabmap` |
| `build/slam_stream`, `build/lib{ORB_SLAM3,DBoW2,g2o}.dylib` | Prebuilt ORB-SLAM3 streamer (arm64) |
| `build_rtab/rtab_stream` | Prebuilt RTAB-Map streamer (arm64) |
| `deps/install/` | Pangolin 0.9.3 headers and dylibs, built from stevenlovegrove/Pangolin @ `73967b39` |
| `Vocabulary/ORBvoc.txt.tar.gz` | ORB vocabulary, compressed from 145 MB to 42 MB so it fits GitHub's 100 MB file limit |
| `ORB_SLAM3_src/` | The ORB-SLAM3 copy the streamer is built from. `System.h/.cc` and `Tracking.cc` differ from `orbslam_ws/src/ORB_SLAM3` |
| `src/` | Streamer sources (`slam_stream.cc`, `rtab_stream.cc`, `raw_session.h`), CMake files and patches |
| `settings/` | Camera settings (SLAM camera and SAM camera, 2000 features) |
| `lidar/` | KISS-ICP LiDAR streamer and the offline checks. `requirements.txt` pins the venv |

## Setup

```bash
cp -R objpose ~/objpose
cd ~/objpose/Vocabulary && tar -xzf ORBvoc.txt.tar.gz
brew install opencv@4 eigen@3 boost openssl@3 sqlite nlohmann-json glew libomp rtabmap
cd ~/objpose/lidar && /usr/bin/python3 -m venv venv && ./venv/bin/pip install -r requirements.txt
```

The prebuilt binaries look for their libraries in `/Users/user/objpose/build`, `/Users/user/objpose/deps/install/lib` and Homebrew.
On a different user or path, rebuild instead:

```bash
cmake -S ~/objpose/src -B ~/objpose/build && cmake --build ~/objpose/build -j12
cmake -S ~/objpose/src/rtabmap -B ~/objpose/build_rtab && cmake --build ~/objpose/build_rtab -j12
```

Homebrew versions at capture time: opencv@4 4.14.0_7, eigen@3 3.4.1, boost 1.92.0, rtabmap 0.23.8_3, pcl 1.15.1_8, glew 2.3.1, libomp 23.1.1, nlohmann-json 3.12.0.

## Run

```bash
~/objpose/run_slam.sh --session <dataset>/SLAM --features 2000 --mode live --connect 127.0.0.1:17001
~/objpose/run_slam.sh --slam rtabmap --session <dataset>/SLAM --features 2000 --mode live --connect 127.0.0.1:17001
~/objpose/lidar/venv/bin/python ~/objpose/lidar/lidar_stream.py --session <dataset>/lidar --mode live --connect 127.0.0.1:17001
```

Not included:
- Datasets.
- Run outputs (`rt/`, `lidar/*_260910/`).
- The Pangolin source and build tree.
- CMake build trees.
- The Python venv itself.
