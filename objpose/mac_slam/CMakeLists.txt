cmake_minimum_required(VERSION 3.16)
project(objpose_mac_slam CXX C)

# Native macOS arm64 build of ORB-SLAM3 (no ROS / ament) + the ROS-free slam_stream
# program. Sources come from a COPY of the repo clone (ORB_SLAM3_SRC), never the clone.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(ORB_SLAM3_SRC "$ENV{HOME}/objpose/ORB_SLAM3_src" CACHE PATH "Copy of orbslam_ws/src/ORB_SLAM3")
set(CMAKE_CXX_STANDARD 14)            # ORB-SLAM3 increments bools (removed in C++17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -mcpu=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mcpu=native -Wno-deprecated-declarations -Wno-unused-but-set-variable -Wno-unused-variable")
add_definitions(-DCOMPILEDWITHC11)

set(CMAKE_PREFIX_PATH
  "$ENV{HOME}/objpose/deps/install"
  "/opt/homebrew/opt/opencv@4"
  "/opt/homebrew/opt/eigen@3"
  "/opt/homebrew/opt/boost"
  "/opt/homebrew/opt/openssl@3"
  ${CMAKE_PREFIX_PATH})

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(nlohmann_json REQUIRED PATHS /opt/homebrew)

# OpenMP through Homebrew libomp (Apple clang has no bundled OpenMP runtime).
set(LIBOMP /opt/homebrew/opt/libomp)
set(OMP_FLAGS -Xpreprocessor -fopenmp -I${LIBOMP}/include)
set(OMP_LIB ${LIBOMP}/lib/libomp.dylib)

# GCC/TR1-only includes in the Thirdparty copies are satisfied by shim headers instead of
# patching sources.
include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/compat)
# g2o includes "eigen3/Eigen/..."; pin Eigen 3.4 ahead of /opt/homebrew/include, where the
# default brew "eigen" formula is Eigen 5.
include_directories(BEFORE /opt/homebrew/opt/eigen@3/include /opt/homebrew/opt/eigen@3/include/eigen3)
# Pangolin headers include <GL/glew.h> without exporting GLEW's include path.
include_directories(/opt/homebrew/opt/glew/include)
# g2o's config.h is a configure-time artifact (gitignored); recreate it next to the sources.
if(NOT EXISTS ${ORB_SLAM3_SRC}/Thirdparty/g2o/config.h)
  file(WRITE ${ORB_SLAM3_SRC}/Thirdparty/g2o/config.h
"#ifndef G2O_CONFIG_H\n#define G2O_CONFIG_H\n#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR\n#  error \"g2o requires column major Eigen matrices\"\n#endif\n#endif\n")
endif()

# --- Thirdparty -----------------------------------------------------------------
# Built from their own sources but with our flags (their CMakeLists force -march=native
# and write into the source tree; building them inline keeps everything out-of-tree).
file(GLOB DBOW2_SRCS ${ORB_SLAM3_SRC}/Thirdparty/DBoW2/DBoW2/*.cpp ${ORB_SLAM3_SRC}/Thirdparty/DBoW2/DUtils/*.cpp)
add_library(DBoW2 SHARED ${DBOW2_SRCS})
target_include_directories(DBoW2 PUBLIC ${ORB_SLAM3_SRC}/Thirdparty/DBoW2)
target_link_libraries(DBoW2 PUBLIC ${OpenCV_LIBS} Boost::serialization)
target_include_directories(DBoW2 PUBLIC ${OpenCV_INCLUDE_DIRS})

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
target_compile_options(ORB_SLAM3 PRIVATE ${OMP_FLAGS})
target_link_libraries(ORB_SLAM3 PUBLIC
  ${OpenCV_LIBS} Eigen3::Eigen ${Pangolin_LIBRARIES} DBoW2 g2o
  Boost::serialization OpenSSL::Crypto ${OMP_LIB})

# --- slam_stream ------------------------------------------------------------------
add_executable(slam_stream ${CMAKE_CURRENT_SOURCE_DIR}/slam_stream.cc)
target_link_libraries(slam_stream PRIVATE ORB_SLAM3 SQLite::SQLite3 nlohmann_json::nlohmann_json)
set_target_properties(slam_stream PROPERTIES BUILD_RPATH "${CMAKE_BINARY_DIR};$ENV{HOME}/objpose/deps/install/lib")
