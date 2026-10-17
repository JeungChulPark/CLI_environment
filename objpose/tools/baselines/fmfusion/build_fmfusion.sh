#!/usr/bin/env bash
# FM-Fusion (HKUST, RA-L 2024) 을 sudo 없이 ~/DeepLearning/third_party 에 빌드한다. 2026-09-28.
#   Open3D 0.18.0 C++ 배포본(cxx11-abi), glog 0.6.0, jsoncpp 1.9.5 → third_party/install
#   시스템에 없는 헤더·라이브러리(GL/glu.h, GL/gl.h, libc++ 14)는 apt-get download 한 deb 에서 꺼내 install/ 에 둔다.
#   conda 의 CC/CXX/CFLAGS/LDFLAGS 가 섞이면 anaconda3/lib 가 RPATH 에 들어가 libtiff 충돌이 나므로 env -i 로 빌드한다.
# ROS 시각화 노드는 만들지 않는다(Ubuntu 22.04 에 ROS Noetic 없음). 실행 파일 build/src/IntegrateInstanceMap 만 쓴다.
set -eu
TP=$HOME/DeepLearning/third_party
P=$TP/install
S=$TP/src
mkdir -p $P $S
cd $S
[ -d $TP/FM-Fusion ] || git clone -q https://github.com/HKUST-Aerial-Robotics/FM-Fusion.git $TP/FM-Fusion   # b82d64c 사용
[ -f o3d.tar.xz ] || curl -sL -o o3d.tar.xz https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz
[ -f $P/lib/libOpen3D.so ] || tar xf o3d.tar.xz -C $P --strip-components=1
[ -d glog ] || git clone -q --depth 1 -b v0.6.0 https://github.com/google/glog.git
[ -d jsoncpp ] || git clone -q --depth 1 -b 1.9.5 https://github.com/open-source-parsers/jsoncpp.git
apt-get download libglu1-mesa-dev libgl-dev libglx-dev libc++1-14 libc++abi1-14 libunwind-14 >/dev/null
for f in *.deb; do dpkg -x $f deb_x; done
cp -rn deb_x/usr/include/GL $P/include/
cp -a deb_x/usr/lib/llvm-14/lib/*.so* $P/lib/

CLEAN="env -i HOME=$HOME PATH=/usr/bin:/bin CC=/usr/bin/gcc CXX=/usr/bin/g++"
rm -rf glog/build jsoncpp/build $TP/FM-Fusion/build
$CLEAN cmake -S glog -B glog/build -DCMAKE_INSTALL_PREFIX=$P -DBUILD_SHARED_LIBS=ON -DWITH_GFLAGS=OFF -DWITH_GTEST=OFF -DBUILD_TESTING=OFF >/dev/null
$CLEAN cmake --build glog/build -j 16 >/dev/null && $CLEAN cmake --install glog/build >/dev/null
$CLEAN cmake -S jsoncpp -B jsoncpp/build -DCMAKE_INSTALL_PREFIX=$P -DBUILD_SHARED_LIBS=ON -DBUILD_STATIC_LIBS=OFF -DBUILD_OBJECT_LIBS=OFF -DJSONCPP_WITH_TESTS=OFF >/dev/null
$CLEAN cmake --build jsoncpp/build -j 16 >/dev/null && $CLEAN cmake --install jsoncpp/build >/dev/null
cd $TP/FM-Fusion
$CLEAN cmake -S . -B build -DCMAKE_PREFIX_PATH=$P -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 -DCMAKE_CXX_FLAGS="-I$P/include" >/dev/null 2>&1
$CLEAN cmake --build build -j 20 2>&1 | grep -E "error|Built target"
readelf -d build/src/IntegrateInstanceMap | grep -i rpath
