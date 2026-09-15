#!/bin/bash
# build_hdl_mac.sh — build hdl_graph_slam (ROS 2 Jazzy) on the Mac inside the micromamba env `hdl_graph_slam_jazzy`.
# conda-forge has no g2o for osx-arm64, so g2o is built from source (tag 20200410_git, the API the ROS 2 port
# was written against) into the env prefix; then ndt_omp / fast_gicp / hdl_graph_slam with colcon.
#   ssh mac 'bash ~/objpose/lidar/build_hdl_mac.sh'          (idempotent; logs in ~/objpose/hdlgraphslam_ws/*.log)
set -euo pipefail
export MAMBA_ROOT_PREFIX="$HOME/micromamba"
set +u                                   # conda activate.d scripts (cctools) reference unset variables
eval "$("$HOME/micromamba/bin/micromamba" shell hook -s bash)"
micromamba activate hdl_graph_slam_jazzy
set -u
WS="$HOME/objpose/hdlgraphslam_ws"
G2O_TAG="${G2O_TAG:-20200410_git}"
J="${J:-10}"
# Apple's clang/ld (Xcode CLT): the conda cctools ld64 cannot read the macOS 26 SDK's libSystem.tbd
# ("arm64e.x1-macos: unknown architecture"). OpenMP comes from the env's llvm-openmp (libomp).
unset CC CXX LD AR RANLIB CFLAGS CXXFLAGS LDFLAGS CPPFLAGS
TOOLCHAIN=(-DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++
  -DCMAKE_OSX_SYSROOT="$(xcrun --show-sdk-path)"
  "-DOpenMP_C_FLAGS=-Xpreprocessor -fopenmp -I$CONDA_PREFIX/include" -DOpenMP_C_LIB_NAMES=omp
  "-DOpenMP_CXX_FLAGS=-Xpreprocessor -fopenmp -I$CONDA_PREFIX/include" -DOpenMP_CXX_LIB_NAMES=omp
  -DOpenMP_omp_LIBRARY="$CONDA_PREFIX/lib/libomp.dylib"
  # robostack macOS: the nodes link the *__rosidl_generator_py dylibs, whose Python symbols are resolved at load
  # time in the flat namespace -> link libpython into every executable so they are present
  "-DCMAKE_EXE_LINKER_FLAGS=-L$CONDA_PREFIX/lib -lpython3.12 -Wl,-rpath,$CONDA_PREFIX/lib"
  "-DCMAKE_SHARED_LINKER_FLAGS=-L$CONDA_PREFIX/lib -Wl,-rpath,$CONDA_PREFIX/lib")
if [ ! -f "$CONDA_PREFIX/lib/libg2o_core.dylib" ]; then
  mkdir -p "$WS/deps" && cd "$WS/deps"
  [ -d g2o ] || git clone --depth 1 --branch "$G2O_TAG" https://github.com/RainerKuemmerle/g2o.git
  # this tag adds x86 -msse* flags unconditionally on UNIX; Apple Silicon clang rejects them
  sed -i '' -E 's/^([[:space:]]*)add_compile_options\(-msse[^)]*\)/\1# arm64: & (disabled)/' g2o/CMakeLists.txt
  cmake -S g2o -B g2o/build -G Ninja "${TOOLCHAIN[@]}" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DDISABLE_SSE_AUTODETECT=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$CONDA_PREFIX" \
    -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DBUILD_WITH_MARCH_NATIVE=OFF -DG2O_BUILD_APPS=OFF -DG2O_BUILD_EXAMPLES=OFF \
    -DG2O_USE_OPENGL=OFF -DG2O_USE_CHOLMOD=ON -DG2O_USE_CSPARSE=ON -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_MACOSX_RPATH=ON -DCMAKE_INSTALL_RPATH="$CONDA_PREFIX/lib" > "$WS/g2o_cmake.log" 2>&1
  cmake --build g2o/build -j "$J" > "$WS/g2o_build.log" 2>&1
  cmake --install g2o/build > "$WS/g2o_install.log" 2>&1
  echo "g2o installed: $(ls $CONDA_PREFIX/lib | grep -c g2o) libs"
fi
cd "$WS"
touch deps/COLCON_IGNORE                 # the g2o checkout carries a package.xml; colcon must not build it
colcon build --symlink-install --parallel-workers 3 --cmake-args "${TOOLCHAIN[@]}" -DCMAKE_BUILD_TYPE=Release -DBUILD_VGICP_CUDA=OFF -DBUILD_apps=OFF \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" -DCMAKE_INSTALL_RPATH="$CONDA_PREFIX/lib" > "$WS/colcon_build.log" 2>&1 || { tail -40 "$WS/colcon_build.log"; exit 1; }
tail -4 "$WS/colcon_build.log"
set +u; source "$WS/install/setup.bash"; set -u
ros2 pkg executables hdl_graph_slam
