cmake_minimum_required(VERSION 3.16)
project(objpose_linux_slam CXX C)

# Linux (Ubuntu 22.04, x86_64) build of the ROS-free ORB-SLAM3 core + slam_stream, the same
# streamer that mac_slam/CMakeLists.txt builds on the Mac. Layout on the SLAM host (~/objpose):
#   ORB_SLAM3_src/   copy of orbslam_ws/src/ORB_SLAM3 with mac_slam/patches applied (never the clone)
#   mac_slam/        slam_stream.cc, raw_session.h, settings/  (shared with the Mac build)
#   deps/install/    Pangolin v0.9.2 (built by linux_slam/build_host.sh)
#   Vocabulary/      ORBvoc.txt
#   build/           this project's build dir (libORB_SLAM3.so, libDBoW2.so, libg2o.so, slam_stream)
# -march=native on EVERY target: the Eigen alignment ABI must match between the Thirdparty
# libraries and ORB_SLAM3, or it segfaults (see memory: orbslam3 standalone build).

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

get_filename_component(OBJPOSE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(ORB_SLAM3_SRC "${OBJPOSE_ROOT}/ORB_SLAM3_src" CACHE PATH "Patched copy of orbslam_ws/src/ORB_SLAM3")
set(STREAM_SRC "${OBJPOSE_ROOT}/mac_slam" CACHE PATH "slam_stream.cc / raw_session.h")
set(DEPS_INSTALL "${OBJPOSE_ROOT}/deps/install" CACHE PATH "Pangolin install prefix")

set(CMAKE_CXX_STANDARD 14)            # ORB-SLAM3 increments bools (removed in C++17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -Wno-deprecated-declarations -Wno-unused-but-set-variable -Wno-unused-variable -Wno-unused-result")
add_definitions(-DCOMPILEDWITHC11)

# A conda on PATH makes CMake find its libssl/libsqlite3 and put $CONDA_PREFIX/lib on the link line,
# where conda's libcurl/libtiff shadow the system ones the OpenCV -> gdal chain needs (undefined
# CURL_OPENSSL_4 / LIBTIFF_4.0 symbol versions when linking slam_stream). Only look in the system.
set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH OFF)
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_IGNORE_PATH "$ENV{CONDA_PREFIX}" "$ENV{CONDA_PREFIX}/bin" "$ENV{CONDA_PREFIX}/lib" "$ENV{CONDA_PREFIX}/include")
  list(APPEND CMAKE_IGNORE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED PATHS ${DEPS_INSTALL}/lib/cmake/Pangolin NO_DEFAULT_PATH)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenMP REQUIRED)

# g2o's config.h is a configure-time artifact (gitignored); recreate it next to the sources.
if(NOT EXISTS ${ORB_SLAM3_SRC}/Thirdparty/g2o/config.h)
  file(WRITE ${ORB_SLAM3_SRC}/Thirdparty/g2o/config.h
"#ifndef G2O_CONFIG_H\n#define G2O_CONFIG_H\n#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR\n#  error \"g2o requires column major Eigen matrices\"\n#endif\n#endif\n")
endif()

# --- Thirdparty (built inline with our flags, out of tree) --------------------------
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

# --- ORB_SLAM3 ------------------------------------------------------------------------
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

# --- slam_stream ----------------------------------------------------------------------
add_executable(slam_stream ${STREAM_SRC}/slam_stream.cc)
target_include_directories(slam_stream PRIVATE ${STREAM_SRC})
target_link_libraries(slam_stream PRIVATE ORB_SLAM3 SQLite::SQLite3 nlohmann_json::nlohmann_json)
set_target_properties(slam_stream PROPERTIES BUILD_RPATH "${CMAKE_BINARY_DIR};${DEPS_INSTALL}/lib")
