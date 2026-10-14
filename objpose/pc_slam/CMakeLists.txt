cmake_minimum_required(VERSION 3.16)
project(objpose_pc_slam CXX C)

# Linux x86_64 counterpart of objpose/mac_slam/CMakeLists.txt: ORB-SLAM3 (no ROS) +
# the ROS-free slam_stream, built from a patched COPY of orbslam_ws/src/ORB_SLAM3
# (ORB_SLAM3_SRC) against the conda env "orbslam3". slam_stream.cc itself is shared
# with the Mac build (../mac_slam). Build dir lives outside the repo:
#   cmake -S objpose/pc_slam -B ~/objpose_pc/build && cmake --build ~/objpose_pc/build -j16

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(ORB_SLAM3_SRC "$ENV{HOME}/objpose_pc/ORB_SLAM3_src" CACHE PATH "Patched copy of orbslam_ws/src/ORB_SLAM3")
set(CONDA_ENV "$ENV{HOME}/anaconda3/envs/orbslam3" CACHE PATH "conda env with OpenCV/Eigen/Pangolin/Boost")
set(MAC_SLAM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../mac_slam")

set(CMAKE_CXX_STANDARD 14)            # ORB-SLAM3 increments bools (removed in C++17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -Wno-deprecated-declarations -Wno-unused-but-set-variable -Wno-unused-variable")
add_definitions(-DCOMPILEDWITHC11)

set(CMAKE_PREFIX_PATH "${CONDA_ENV}" ${CMAKE_PREFIX_PATH})

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(OpenMP REQUIRED)
# nlohmann_json: header-only, from ~/.local/include (or anywhere on the include path).
find_path(NLOHMANN_JSON_INCLUDE nlohmann/json.hpp HINTS "$ENV{HOME}/.local/include" "${CONDA_ENV}/include" REQUIRED)

# Pangolin headers include <GL/glew.h>; the conda env carries it.
include_directories(${CONDA_ENV}/include)

# Run without activating the env: bake the build dir and the env's lib dir into RPATH.
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR};${CONDA_ENV}/lib")
set(CMAKE_SKIP_BUILD_RPATH OFF)

if(NOT EXISTS ${ORB_SLAM3_SRC}/Thirdparty/g2o/config.h)
  file(WRITE ${ORB_SLAM3_SRC}/Thirdparty/g2o/config.h
"#ifndef G2O_CONFIG_H\n#define G2O_CONFIG_H\n#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR\n#  error \"g2o requires column major Eigen matrices\"\n#endif\n#endif\n")
endif()

# --- Thirdparty (inline, our flags, out-of-tree) ---------------------------------
file(GLOB DBOW2_SRCS ${ORB_SLAM3_SRC}/Thirdparty/DBoW2/DBoW2/*.cpp ${ORB_SLAM3_SRC}/Thirdparty/DBoW2/DUtils/*.cpp)
add_library(DBoW2 SHARED ${DBOW2_SRCS})
target_include_directories(DBoW2 PUBLIC ${ORB_SLAM3_SRC}/Thirdparty/DBoW2 ${OpenCV_INCLUDE_DIRS})
target_link_libraries(DBoW2 PUBLIC ${OpenCV_LIBS} Boost::serialization)

file(GLOB G2O_SRCS
  ${ORB_SLAM3_SRC}/Thirdparty/g2o/g2o/types/*.cpp
  ${ORB_SLAM3_SRC}/Thirdparty/g2o/g2o/core/*.cpp
  ${ORB_SLAM3_SRC}/Thirdparty/g2o/g2o/stuff/*.cpp
  ${ORB_SLAM3_SRC}/Thirdparty/g2o/g2o/stuff/*.c
  ${ORB_SLAM3_SRC}/Thirdparty/g2o/g2o/solvers/*.cpp)
add_library(g2o SHARED ${G2O_SRCS})
target_include_directories(g2o PUBLIC ${ORB_SLAM3_SRC}/Thirdparty/g2o ${ORB_SLAM3_SRC}/Thirdparty/g2o/g2o)
target_link_libraries(g2o PUBLIC Eigen3::Eigen)
target_compile_definitions(g2o PRIVATE UNIX)

# --- ORB_SLAM3 --------------------------------------------------------------------
set(ORB_SRCS
  System.cc Tracking.cc LocalMapping.cc LoopClosing.cc ORBextractor.cc ORBmatcher.cc
  FrameDrawer.cc Converter.cc MapPoint.cc KeyFrame.cc Atlas.cc Map.cc MapDrawer.cc
  Optimizer.cc Frame.cc KeyFrameDatabase.cc Sim3Solver.cc Viewer.cc ImuTypes.cc
  G2oTypes.cc CameraModels/Pinhole.cpp CameraModels/KannalaBrandt8.cpp
  OptimizableTypes.cpp MLPnPsolver.cpp GeometricTools.cc TwoViewReconstruction.cc
  Config.cc Settings.cc)
list(TRANSFORM ORB_SRCS PREPEND ${ORB_SLAM3_SRC}/src/)
add_library(ORB_SLAM3 SHARED ${ORB_SRCS})
target_include_directories(ORB_SLAM3 PUBLIC
  ${ORB_SLAM3_SRC} ${ORB_SLAM3_SRC}/include ${ORB_SLAM3_SRC}/include/CameraModels
  ${ORB_SLAM3_SRC}/Thirdparty/Sophus ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(ORB_SLAM3 PUBLIC ORBEXTRACTOR_OMP_THREADS=4)
target_link_libraries(ORB_SLAM3 PUBLIC
  ${OpenCV_LIBS} Eigen3::Eigen ${Pangolin_LIBRARIES} DBoW2 g2o
  Boost::serialization OpenSSL::Crypto OpenMP::OpenMP_CXX)

# --- slam_stream ------------------------------------------------------------------
add_executable(slam_stream ${MAC_SLAM_DIR}/slam_stream.cc)
target_include_directories(slam_stream PRIVATE ${MAC_SLAM_DIR} ${NLOHMANN_JSON_INCLUDE})
target_link_libraries(slam_stream PRIVATE ORB_SLAM3 SQLite::SQLite3 pthread)
