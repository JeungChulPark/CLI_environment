cmake_minimum_required(VERSION 3.10)
project(openloris_orb CXX C)
# One ORB-SLAM3 source tree (ORB_SRC) + the OpenLORIS runner, built the same way for every variant:
#   upstream  = UZ-SLAMLab/ORB_SLAM3 v1.0-release
#   ours      = orbslam_ws/src/ORB_SLAM3 (WITH_GYRO=ON adds the --gyro path; it needs System::SetFrameGyro)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mcpu=native -Wno-deprecated-declarations -Wno-unused")
add_definitions(-DCOMPILEDWITHC11)

set(ORB_SRC "" CACHE PATH "ORB-SLAM3 source tree")
option(WITH_GYRO "build the gyro-aid path (modified tree only)" OFF)
set(ORBEXTRACTOR_OMP_THREADS "4" CACHE STRING "")
find_package(OpenMP REQUIRED COMPONENTS CXX)
add_definitions(-DORBEXTRACTOR_OMP_THREADS=${ORBEXTRACTOR_OMP_THREADS})

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)

include_directories(
  ${ORB_SRC}
  ${ORB_SRC}/include
  ${ORB_SRC}/include/CameraModels
  ${ORB_SRC}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR})

add_subdirectory(${ORB_SRC}/Thirdparty/DBoW2 DBoW2)
add_subdirectory(${ORB_SRC}/Thirdparty/g2o g2o)

file(GLOB ORB_SRCS ${ORB_SRC}/src/*.cc ${ORB_SRC}/src/*.cpp ${ORB_SRC}/src/CameraModels/*.cpp)
add_library(ORB_SLAM3 SHARED ${ORB_SRCS})
target_link_libraries(ORB_SLAM3 ${OpenCV_LIBS} pango_display pango_opengl Eigen3::Eigen
  DBoW2 g2o Boost::serialization OpenSSL::Crypto OpenMP::OpenMP_CXX)

add_executable(openloris_rgbd ${CMAKE_CURRENT_SOURCE_DIR}/openloris_rgbd.cc)
target_link_libraries(openloris_rgbd ORB_SLAM3)
if(WITH_GYRO)
  find_package(SQLite3 REQUIRED)
  find_package(nlohmann_json REQUIRED)
  target_compile_definitions(openloris_rgbd PRIVATE WITH_GYRO)
  target_include_directories(openloris_rgbd PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../objpose/src)
  target_link_libraries(openloris_rgbd SQLite3::SQLite3 nlohmann_json::nlohmann_json)
endif()
