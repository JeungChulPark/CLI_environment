# rtab_stream on a Linux SLAM host, against the RTAB-Map core of a conda env (robostack ros-jazzy-rtabmap).
# Same source as mac_slam/rtabmap/CMakeLists.txt (Homebrew); built by linux_slam/build_rtab_mamba.sh into ~/objpose/build_rtab.
cmake_minimum_required(VERSION 3.16)
project(objpose_rtab_stream_linux CXX)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -Wno-deprecated-declarations")

get_filename_component(OBJPOSE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(STREAM_SRC "${OBJPOSE_ROOT}/mac_slam" CACHE PATH "rtab_stream.cc / raw_session.h")
find_package(RTABMap REQUIRED)
find_package(OpenCV REQUIRED)            # the env's OpenCV (5.0 in robostack jazzy), the one the RTAB-Map core was built with
find_package(SQLite3 REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(rtab_stream ${STREAM_SRC}/rtab_stream.cc)
target_include_directories(rtab_stream PRIVATE ${STREAM_SRC})
target_link_libraries(rtab_stream rtabmap::core ${OpenCV_LIBS} SQLite::SQLite3 nlohmann_json::nlohmann_json)
