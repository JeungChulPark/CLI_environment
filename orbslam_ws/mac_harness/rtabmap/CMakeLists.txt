# live_rtabmap against Homebrew's rtabmap (PCL, Eigen 5). Kept apart from the ORB-SLAM3 project,
# whose include path pins Eigen 3.
cmake_minimum_required(VERSION 3.16)
project(live_rtabmap CXX)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mcpu=native -Wno-deprecated-declarations")

get_filename_component(HARNESS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
find_package(RTABMap REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(SQLite3 REQUIRED)

add_executable(live_rtabmap ${HARNESS_DIR}/live_rtabmap.cc)
target_include_directories(live_rtabmap PRIVATE ${HARNESS_DIR})
target_link_libraries(live_rtabmap rtabmap::core ${OpenCV_LIBS} SQLite3::SQLite3 OpenSSL::Crypto)
target_compile_definitions(live_rtabmap PRIVATE LIVE_WEB_DIR="${HARNESS_DIR}/web")
