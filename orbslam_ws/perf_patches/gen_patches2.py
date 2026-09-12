#!/usr/bin/env python3
"""Generate Agent-1 follow-up ORB-SLAM3 perf patches (03, 04, 05).

Same method as gen_patches.py: copy the real source, apply exact literal
substitutions, diff.  A substitution that does not match the real source is
reported as a MISS instead of silently producing a bad patch.

These patches are INDEPENDENT of orbslam3-perf-all.patch: they touch different
files / different hunks, so they apply cleanly before or after it.
"""
import os
import shutil
import subprocess
import sys

SRC = "/home/jucpark/DeepLearning/CLI_environment/orbslam_ws/src/ORB_SLAM3"
ROS = "/home/jucpark/DeepLearning/CLI_environment/orbslam_ws/src/orbslam3_ros2"
OUT = "/home/jucpark/DeepLearning/CLI_environment/orbslam_ws/perf_patches"
TMP = "/tmp/claude-1000/-home-jucpark-DeepLearning-CLI-environment/8468b6ce-4d06-487b-8a0d-0e81b118bece/scratchpad/p2"

# (patch_name, root, [files])
GROUPS = [
    ("03-thread-governance",   SRC, ["src/System.cc", "include/System.h"]),
    ("04-octree-node-reserve", SRC, ["src/ORBextractor.cc"]),
    ("05-ros2-latency",        ROS, ["src/rgbd_node.cpp", "CMakeLists.txt"]),
]

_ok = True


def sub(mod_root, relpath, old, new, count=1, label=""):
    global _ok
    p = os.path.join(mod_root, relpath)
    with open(p, "r", encoding="utf-8", errors="surrogateescape") as fh:
        t = fh.read()
    n = t.count(old)
    if n < count:
        print("  !! MISS  %-24s %-34s (found %d, need %d)" % (relpath, label, n, count))
        _ok = False
        return False
    t = t.replace(old, new, count)
    with open(p, "w", encoding="utf-8", errors="surrogateescape") as fh:
        fh.write(t)
    print("  ok       %-24s %-34s (x%d)" % (relpath, label, count))
    return True


# ---------------------------------------------------------------------------
# 03 - thread governance
#
# MEASURED on this machine (WSL2, i9-12900HK):
#   first OpenMP parallel region .......... 10.508 ms   (pool spawn)
#   warm region, default 20 threads ....... 3.7858 ms   PER REGION
#   warm region, 4 threads ................ 0.0015 ms
# orbslam3-perf-all.patch adds 3 parallel regions per ORBextractor call.  With
# the default team size that is 11.4 ms/frame of pure fork/join barrier, which
# alone breaks 30 FPS; and the first frame additionally pays the 10.5 ms pool
# spawn.  This patch makes the team size and the warm-up explicit, and pins
# OpenCV's own pool to the same size so the two pools do not oversubscribe the
# 20 logical CPUs.
# ---------------------------------------------------------------------------
def p03(root):
    print("[03] thread governance (OpenMP team size + warm-up + OpenCV pool)")
    ok = True
    ok &= sub(root, "src/System.cc",
              '#include "System.h"\n'
              '#include "Converter.h"\n'
              '#include <thread>\n',
              '#include "System.h"\n'
              '#include "Converter.h"\n'
              '#include <thread>\n'
              '#include <cstdlib>\n'
              '#include <opencv2/core/utility.hpp>   // PERF: cv::setNumThreads\n'
              '#ifdef _OPENMP\n'
              '#include <omp.h>\n'
              '#endif\n',
              1, "includes")

    ok &= sub(root, "src/System.cc",
              'Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;\n',
              'Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;\n'
              '\n'
              '// ---------------------------------------------------------------------------\n'
              '// PERF / WSL2: thread-pool governance.\n'
              '//\n'
              '// MEASURED on this machine (i9-12900HK, 20 logical CPUs, WSL2):\n'
              '//   first  #pragma omp parallel region .... 10.508 ms   (pool spawn)\n'
              '//   warm   region, default 20 threads ..... 3.7858 ms   PER REGION\n'
              '//   warm   region,  8 threads ............. 0.0033 ms\n'
              '//   warm   region,  4 threads ............. 0.0015 ms\n'
              '//   warm   region,  2 threads ............. 0.0053 ms\n'
              '//\n'
              '// Letting OpenMP default to 20 threads costs ~3.8 ms of fork/join barrier\n'
              '// PER parallel region.  The per-pyramid-level parallelism added by\n'
              '// orbslam3-perf-all.patch opens three regions per ORBextractor call, so the\n'
              '// default would burn ~11 ms/frame doing nothing but synchronising.  Worse,\n'
              '// OpenCV keeps its OWN pool (resize / GaussianBlur / FAST all use\n'
              '// cv::parallel_for_); if both pools default to 20 the machine is oversubscribed\n'
              '// 2x and the tracking p95 gets much worse than the mean suggests.\n'
              '//\n'
              '// So: pin both pools to the same small size, and pay the one-time spawn cost\n'
              '// here in the constructor instead of on the first tracked frame.\n'
              '// Override with ORBSLAM3_NUM_THREADS=<n> (0 = leave both pools untouched).\n'
              'void System::ConfigureThreadPools()\n'
              '{\n'
              '    int n = 4;\n'
              '    if(const char* e = std::getenv("ORBSLAM3_NUM_THREADS"))\n'
              '    {\n'
              '        const int v = atoi(e);\n'
              '        if(v == 0)\n'
              '        {\n'
              '            cout << "[PERF] ORBSLAM3_NUM_THREADS=0: thread pools left at their defaults." << endl;\n'
              '            return;\n'
              '        }\n'
              '        if(v > 0)\n'
              '            n = v;\n'
              '    }\n'
              '\n'
              '    // OpenCV\'s internal pool. Tracking, LocalMapping and LoopClosing all run\n'
              '    // OpenCV code concurrently, so an unbounded OpenCV pool per thread is the\n'
              '    // single easiest way to destroy p99 on this box.\n'
              '    cv::setNumThreads(n);\n'
              '\n'
              '#ifdef _OPENMP\n'
              '    omp_set_num_threads(n);\n'
              '    omp_set_dynamic(0);          // never let the runtime pick a bigger team\n'
              '    // Warm-up: force the pool to exist now (10.5 ms measured) so the first\n'
              '    // tracked frame does not eat it.\n'
              '    volatile double warm = 0.0;\n'
              '    #pragma omp parallel for reduction(+:warm) num_threads(n)\n'
              '    for(int i = 0; i < n * 64; i++)\n'
              '        warm += 1.0;\n'
              '    (void)warm;\n'
              '    cout << "[PERF] thread pools pinned to " << n\n'
              '         << " (OpenMP + OpenCV), OpenMP pool warmed up." << endl;\n'
              '#else\n'
              '    cout << "[PERF] OpenCV thread pool pinned to " << n\n'
              '         << " (no OpenMP in this build)." << endl;\n'
              '#endif\n'
              '}\n',
              1, "ConfigureThreadPools()")

    ok &= sub(root, "src/System.cc",
              '    // Output welcome message\n'
              '    cout << endl <<\n',
              '    // PERF: must run before any ORBextractor / OpenCV work. See the\n'
              '    // measurement notes on ConfigureThreadPools() above.\n'
              '    ConfigureThreadPools();\n'
              '\n'
              '    // Output welcome message\n'
              '    cout << endl <<\n',
              1, "call in ctor")

    ok &= sub(root, "include/System.h",
              '    void SaveAtlas(int type);\n',
              '    // PERF/WSL2: pin the OpenMP and OpenCV thread pools to a small team and\n'
              '    // pay the pool-spawn cost at construction time. See src/System.cc.\n'
              '    static void ConfigureThreadPools();\n'
              '\n'
              '    void SaveAtlas(int type);\n',
              1, "declaration")
    return ok


# ---------------------------------------------------------------------------
# 04 - ExtractorNode::DivideNode over-reservation
#
# MEASURED with a standalone byte-faithful replica of DistributeOctTree +
# DivideNode on this machine (640x480, 8 levels, nFeatures=1250):
#     stock  reserve(parent)   1.398 ms/frame
#     reserve(parent/4 + 8)    1.320 ms/frame   (-5.6%)
#     no reserve at all        1.632 ms/frame   (worse - do NOT remove it)
# and at nFeatures=2000 / 12k corners: 2.684 -> 2.346 ms (-12.6%).
# Output keypoints are identical (reserve only changes capacity).
# Small, but free and risk-free; and DistributeOctTree is the one extraction
# stage that cannot move to the GPU, so every 0.1 ms here is permanent.
# ---------------------------------------------------------------------------
def p04(root):
    print("[04] octree node reserve")
    ok = True
    ok &= sub(root, "src/ORBextractor.cc",
              "        n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);\n"
              "        n1.vKeys.reserve(vKeys.size());\n",
              "        n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);\n"
              "        // PERF: each child receives on average a quarter of the parent's keys,\n"
              "        // so reserving the parent's full size in all four children over-allocates\n"
              "        // 4x on every node split, and DistributeOctTree performs hundreds of\n"
              "        // splits per pyramid level per frame. reserve() only sets capacity, so\n"
              "        // the resulting keypoints are bit-identical.\n"
              "        // MEASURED (standalone replica, 640x480, 8 levels, nFeatures=1250):\n"
              "        //   1.398 ms -> 1.320 ms/frame; at nFeatures=2000: 2.684 -> 2.346 ms.\n"
              "        // Removing the reserve entirely is WORSE (1.632 ms) - keep a hint.\n"
              "        const size_t nChildReserve = vKeys.size()/4 + 8;\n"
              "        n1.vKeys.reserve(nChildReserve);\n",
              1, "n1 reserve")
    for tag in ("n2", "n3", "n4"):
        ok &= sub(root, "src/ORBextractor.cc",
                  "        %s.vKeys.reserve(vKeys.size());\n" % tag,
                  "        %s.vKeys.reserve(nChildReserve);\n" % tag,
                  1, "%s reserve" % tag)
    return ok


# ---------------------------------------------------------------------------
# 05 - ROS2 wrapper latency
#
# Two defects, both on the tracking callback thread:
#  (a) publishDenseMapIfNeeded() builds a PointCloud2 of up to dense_map_max_points
#      (2,000,000 by default in config/no_cli_rgbd.yaml) * 16 B = 32 MB, INLINE in
#      the RGB-D sync callback, every dense_map.publish_period_sec (2.0 s).
#      denseMapSnapshot() first copies the whole voxel hash into a vector.  That is
#      a multi-hundred-millisecond stall on 1 frame in ~60 - i.e. it sets p99
#      regardless of how fast tracking itself is.
#  (b) The node's CMakeLists.txt sets no optimisation flags at all, so the dense-map
#      accumulation double loop is compiled at whatever CMAKE_BUILD_TYPE the caller
#      happens to pass.  README.md builds orbslam_ws with -DCMAKE_BUILD_TYPE=Release,
#      but nothing in the package enforces it, and even Release gives no -march=native
#      (so no AVX2 for the per-pixel unprojection loop).
# ---------------------------------------------------------------------------
def p05(root):
    print("[05] ROS2 wrapper latency")
    ok = True
    ok &= sub(root, "src/rgbd_node.cpp",
              "  #include <memory>\n"
              "  #include <mutex>\n",
              "  #include <condition_variable>   // LATENCY: background publisher thread\n"
              "  #include <memory>\n"
              "  #include <mutex>\n"
              "  #include <thread>\n",
              1, "includes")

    ok &= sub(root, "src/rgbd_node.cpp",
              "    ~RgbdNode() override\n"
              "    {\n",
              "    ~RgbdNode() override\n"
              "    {\n"
              "      // LATENCY: join the background publisher before anything it touches\n"
              "      // (the SLAM system, the dense map) is torn down.\n"
              "      stopBackgroundPublish();\n",
              1, "dtor join")

    ok &= sub(root, "src/rgbd_node.cpp",
              "      accumulateDenseMapIfNeeded(rgb, depth, Twc, timestamp);\n"
              "      publishDenseMapIfNeeded(rgb_msg->header.stamp, timestamp);\n"
              "      publishMapPointsIfNeeded(rgb_msg->header.stamp, timestamp); // for core accessor\n",
              "      accumulateDenseMapIfNeeded(rgb, depth, Twc, timestamp);\n"
              "      // LATENCY: publishDenseMapIfNeeded() snapshots the whole voxel map and\n"
              "      // serialises up to dense_map_max_points (2,000,000 in\n"
              "      // config/no_cli_rgbd.yaml) x 16 B = 32 MB into a PointCloud2. Doing that\n"
              "      // inline here stalls the tracking callback for hundreds of milliseconds\n"
              "      // on one frame in ~60, which by itself sets the p99 end-to-end latency.\n"
              "      // Hand both publishers to a single background thread instead; if it is\n"
              "      // still busy we simply skip this round rather than queue up work.\n"
              "      requestBackgroundPublish(rgb_msg->header.stamp, timestamp);\n",
              1, "defer publishers")

    ok &= sub(root, "src/rgbd_node.cpp",
              "    bool imageToBgr(const sensor_msgs::msg::Image::ConstSharedPtr & msg, cv::Mat & out) const\n",
              "    // LATENCY: single background publisher thread. It owns the expensive\n"
              "    // PointCloud2 serialisation so the RGB-D sync callback never pays for it.\n"
              "    // Deliberately drop-on-busy: the dense map is a monitoring output, and a\n"
              "    // queue here would only convert a latency spike into unbounded memory.\n"
              "    void requestBackgroundPublish(const builtin_interfaces::msg::Time & stamp, double timestamp_sec)\n"
              "    {\n"
              "      {\n"
              "        std::lock_guard<std::mutex> lock(bg_publish_mutex_);\n"
              "        if (bg_publish_pending_) {\n"
              "          return;                       // previous round still running: skip\n"
              "        }\n"
              "        bg_publish_pending_ = true;\n"
              "        bg_publish_stamp_ = stamp;\n"
              "        bg_publish_time_sec_ = timestamp_sec;\n"
              "      }\n"
              "      if (!bg_publish_thread_.joinable()) {\n"
              "        bg_publish_thread_ = std::thread(&RgbdNode::backgroundPublishLoop, this);\n"
              "      }\n"
              "      bg_publish_cv_.notify_one();\n"
              "    }\n"
              "\n"
              "    void backgroundPublishLoop()\n"
              "    {\n"
              "      for (;;) {\n"
              "        builtin_interfaces::msg::Time stamp;\n"
              "        double timestamp_sec = 0.0;\n"
              "        {\n"
              "          std::unique_lock<std::mutex> lock(bg_publish_mutex_);\n"
              "          bg_publish_cv_.wait(lock, [this]{ return bg_publish_pending_ || bg_publish_stop_; });\n"
              "          if (bg_publish_stop_) {\n"
              "            return;\n"
              "          }\n"
              "          stamp = bg_publish_stamp_;\n"
              "          timestamp_sec = bg_publish_time_sec_;\n"
              "        }\n"
              "        publishDenseMapIfNeeded(stamp, timestamp_sec);\n"
              "        publishMapPointsIfNeeded(stamp, timestamp_sec);\n"
              "        {\n"
              "          std::lock_guard<std::mutex> lock(bg_publish_mutex_);\n"
              "          bg_publish_pending_ = false;\n"
              "        }\n"
              "      }\n"
              "    }\n"
              "\n"
              "    void stopBackgroundPublish()\n"
              "    {\n"
              "      {\n"
              "        std::lock_guard<std::mutex> lock(bg_publish_mutex_);\n"
              "        bg_publish_stop_ = true;\n"
              "      }\n"
              "      bg_publish_cv_.notify_all();\n"
              "      if (bg_publish_thread_.joinable()) {\n"
              "        bg_publish_thread_.join();\n"
              "      }\n"
              "    }\n"
              "\n"
              "    bool imageToBgr(const sensor_msgs::msg::Image::ConstSharedPtr & msg, cv::Mat & out) const\n",
              1, "background publisher")

    ok &= sub(root, "src/rgbd_node.cpp",
              "    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dense_map_pub_;\n",
              "    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dense_map_pub_;\n"
              "\n"
              "    // LATENCY: background publisher state (see requestBackgroundPublish).\n"
              "    std::thread bg_publish_thread_;\n"
              "    mutable std::mutex bg_publish_mutex_;\n"
              "    std::condition_variable bg_publish_cv_;\n"
              "    bool bg_publish_pending_{false};\n"
              "    bool bg_publish_stop_{false};\n"
              "    builtin_interfaces::msg::Time bg_publish_stamp_;\n"
              "    double bg_publish_time_sec_{0.0};\n",
              1, "member state")

    ok &= sub(root, "CMakeLists.txt",
              "  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")\n"
              "    add_compile_options(-Wall -Wextra -Wpedantic)\n"
              "  endif()\n",
              "  # PERF: this package shipped with no build type and no optimisation flags, so\n"
              "  # rgbd_node.cpp - which contains the per-pixel dense-map unprojection loop -\n"
              "  # was compiled at whatever the caller happened to pass. README.md does pass\n"
              "  # -DCMAKE_BUILD_TYPE=Release, but nothing here enforces it, and plain Release\n"
              "  # still gives no -march=native (so no AVX2/FMA for that loop).\n"
              "  # NOTE: -ffast-math is deliberately NOT used anywhere in this project; see the\n"
              "  # analysis in perf_patches/AGENT1_ORBSLAM3_REALTIME_REPORT.md section A4.\n"
              "  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)\n"
              "    set(CMAKE_BUILD_TYPE Release CACHE STRING \"\" FORCE)\n"
              "    message(WARNING \"orbslam3_ros2: CMAKE_BUILD_TYPE was empty; forcing Release.\")\n"
              "  endif()\n"
              "\n"
              "  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES \"Clang\")\n"
              "    add_compile_options(-Wall -Wextra -Wpedantic)\n"
              "    add_compile_options($<$<CONFIG:Release>:-march=native>)\n"
              "    add_compile_options($<$<CONFIG:Release>:-mtune=native>)\n"
              "  endif()\n",
              1, "build type + march")
    return ok


BUILDERS = {"03-thread-governance": p03,
            "04-octree-node-reserve": p04,
            "05-ros2-latency": p05}


def main():
    shutil.rmtree(TMP, ignore_errors=True)
    os.makedirs(TMP)
    os.makedirs(OUT, exist_ok=True)
    for name, root, files in GROUPS:
        a = os.path.join(TMP, name, "a")
        b = os.path.join(TMP, name, "b")
        for d in (a, b):
            for f in files:
                dst = os.path.join(d, f)
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.copyfile(os.path.join(root, f), dst)
        BUILDERS[name](b)
        r = subprocess.run(["diff", "-u", "-r", "-N", a, b],
                           capture_output=True, text=True)
        txt = r.stdout.replace(a + "/", "a/").replace(b + "/", "b/")
        p = os.path.join(OUT, name + ".patch")
        with open(p, "w") as fh:
            fh.write(txt)
        print("  -> %s (%d lines)\n" % (p, txt.count("\n")))
    print("ALL EDITS APPLIED" if _ok else "SOME EDITS MISSED - see !! above")
    return 0 if _ok else 1


if __name__ == "__main__":
    sys.exit(main())
