cmake_minimum_required(VERSION 3.10)
project(orb3_mac CXX C)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mcpu=native -Wno-deprecated-declarations -Wno-unused")
add_definitions(-DCOMPILEDWITHC11)

set(ORBEXTRACTOR_OMP_THREADS "4" CACHE STRING "")
find_package(OpenMP REQUIRED COMPONENTS CXX)
add_definitions(-DORBEXTRACTOR_OMP_THREADS=${ORBEXTRACTOR_OMP_THREADS})
option(ORBSLAM3_REGISTER_TIMES "" OFF)
if(ORBSLAM3_REGISTER_TIMES)
  add_definitions(-DREGISTER_TIMES)
endif()

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)
find_package(SQLite3 REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/CameraModels
  ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR})

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

file(GLOB ORB_SRCS src/*.cc src/*.cpp src/CameraModels/*.cpp)
add_library(ORB_SLAM3 SHARED ${ORB_SRCS})
target_link_libraries(ORB_SLAM3 ${OpenCV_LIBS} pango_display pango_opengl Eigen3::Eigen
  DBoW2 g2o Boost::serialization OpenSSL::Crypto OpenMP::OpenMP_CXX)

add_executable(rt_harness ${HARNESS_DIR}/rt_harness.cc)
target_link_libraries(rt_harness ORB_SLAM3 SQLite3::SQLite3)

add_executable(live_orbslam ${HARNESS_DIR}/live_orbslam.cc)
target_link_libraries(live_orbslam ORB_SLAM3 SQLite3::SQLite3 OpenSSL::Crypto)
target_compile_definitions(live_orbslam PRIVATE LIVE_WEB_DIR="${HARNESS_DIR}/web")
