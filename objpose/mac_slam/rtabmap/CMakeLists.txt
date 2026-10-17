# rtab_stream against Homebrew's rtabmap (PCL, Eigen 5), as orbslam_ws/mac_harness/rtabmap does.
# Separate project from ../CMakeLists.txt, whose include path pins Eigen 3 for ORB-SLAM3.
cmake_minimum_required(VERSION 3.16)
project(objpose_rtab_stream CXX)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mcpu=native -Wno-deprecated-declarations")

get_filename_component(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
find_package(RTABMap REQUIRED PATHS /opt/homebrew/opt/rtabmap/lib/rtabmap-0.23)
find_package(OpenCV 4 REQUIRED PATHS /opt/homebrew/opt/opencv@4/lib/cmake/opencv4)
find_package(SQLite3 REQUIRED)
find_package(nlohmann_json REQUIRED PATHS /opt/homebrew)

add_executable(rtab_stream ${SRC_DIR}/rtab_stream.cc)
target_include_directories(rtab_stream PRIVATE ${SRC_DIR})
target_link_libraries(rtab_stream rtabmap::core ${OpenCV_LIBS} SQLite::SQLite3 nlohmann_json::nlohmann_json)
