#!/usr/bin/env python3
"""Generate verified ORB-SLAM3 perf patches by editing a copy and diffing.

Every edit is expressed as an exact literal (old -> new) replacement so the
generated unified diff is guaranteed to match the real source.
"""
import os
import shutil
import subprocess
import sys

SRC = "/home/jucpark/DeepLearning/CLI_environment/orbslam_ws/src/ORB_SLAM3"
ORIG = "/tmp/orb_a"
MOD = "/tmp/orb_b"
OUT = "/home/jucpark/DeepLearning/CLI_environment/orbslam_ws/perf_patches"

FILES = ["src/Frame.cc", "src/ORBmatcher.cc", "src/Tracking.cc",
         "src/ORBextractor.cc", "src/MapPoint.cc",
         "include/ORBmatcher.h", "include/ORBextractor.h",
         "include/MapPoint.h", "CMakeLists.txt"]


def prep():
    for d in (ORIG, MOD):
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
    for f in FILES:
        for d in (ORIG, MOD):
            dst = os.path.join(d, f)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(os.path.join(SRC, f), dst)


def sub(relpath, old, new, count=1, label=""):
    p = os.path.join(MOD, relpath)
    with open(p, "r", encoding="utf-8", errors="surrogateescape") as fh:
        t = fh.read()
    n = t.count(old)
    if n < count:
        print("  !! MISS  %-22s %-28s (found %d, need %d)" % (relpath, label, n, count))
        return False
    t = t.replace(old, new, count)
    with open(p, "w", encoding="utf-8", errors="surrogateescape") as fh:
        fh.write(t)
    print("  ok       %-22s %-28s (x%d)" % (relpath, label, count))
    return True


# ---------------------------------------------------------------------------
# Patch group 01 - hot path allocations (Frame::GetFeaturesInArea)
# ---------------------------------------------------------------------------
def p01():
    print("[01] hot-path allocations")
    ok = True
    ok &= sub("src/Frame.cc",
              "    vector<size_t> vIndices;\n    vIndices.reserve(N);\n",
              "    vector<size_t> vIndices;\n"
              "    // PERF: called once per visible local MapPoint per frame (~1-3k calls).\n"
              "    // reserve(N) heap-allocated N*8 bytes on every call, although a search\n"
              "    // window realistically returns O(10) indices.\n"
              "    vIndices.reserve(32);\n",
              1, "reserve(N)->32")
    ok &= sub("src/Frame.cc",
              "            const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];",
              "            // PERF: was a by-value copy of the grid cell (the '&' was missing),\n"
              "            // i.e. one heap allocation per visited cell, ~9-25 cells per call.\n"
              "            const vector<size_t> &vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];",
              1, "vCell by-ref")
    return ok


# ---------------------------------------------------------------------------
# Patch group 02 - hardware popcount Hamming distance
# ---------------------------------------------------------------------------
def p02():
    print("[02] DescriptorDistance hardware POPCNT")
    ok = True
    ok &= sub("include/ORBmatcher.h",
              "        // Computes the Hamming distance between two ORB descriptors\n"
              "        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);\n",
              "        // Computes the Hamming distance between two ORB descriptors\n"
              "        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);\n"
              "\n"
              "        // PERF: raw-pointer overload for hot inner loops. Avoids constructing a\n"
              "        // cv::Mat header (and its atomic refcount inc/dec) per comparison.\n"
              "        // Both pointers must address 32 contiguous bytes (one ORB descriptor).\n"
              "        static int DescriptorDistance(const unsigned char *pa, const unsigned char *pb);\n",
              1, "add ptr overload")

    old_body = """// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist=0;

        for(int i=0; i<8; i++, pa++, pb++)
        {
            unsigned  int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }
"""
    new_body = """// PERF: the original used the SWAR bit-hack popcount from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
// The target CPU has hardware POPCNT (already selected by -march=native), so
// __builtin_popcountll lowers to 4 POPCNT instructions instead of ~40 ALU ops.
// The result is bit-identical. This is the hottest function in the system:
// it is reached from 20 call sites and runs millions of times per second.
    int ORBmatcher::DescriptorDistance(const unsigned char *pa, const unsigned char *pb)
    {
        // A 32-byte ORB descriptor read as 4 x uint64. memcpy is the portable
        // spelling of a possibly-unaligned load; gcc/clang lower it to movq.
        uint64_t va[4], vb[4];
        memcpy(va, pa, 32);
        memcpy(vb, pb, 32);
        return __builtin_popcountll(va[0] ^ vb[0])
             + __builtin_popcountll(va[1] ^ vb[1])
             + __builtin_popcountll(va[2] ^ vb[2])
             + __builtin_popcountll(va[3] ^ vb[3]);
    }

    int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        return DescriptorDistance(a.ptr<unsigned char>(), b.ptr<unsigned char>());
    }
"""
    ok &= sub("src/ORBmatcher.cc", old_body, new_body, 1, "popcnt body")
    ok &= sub("src/ORBmatcher.cc",
              '#include "ORBmatcher.h"\n',
              '#include "ORBmatcher.h"\n'
              "\n"
              "#include <cstring>   // PERF: memcpy in DescriptorDistance\n"
              "#include <cstdint>\n",
              1, "add cstring/cstdint")
    return ok


# ---------------------------------------------------------------------------
# Patch group 03 - avoid cv::Mat header churn in the two hottest match loops
# ---------------------------------------------------------------------------
def p03():
    print("[03] matcher inner loops: raw descriptor pointers")
    ok = True
    # SearchByProjection(Frame&, vector<MapPoint*>, ...) - local map tracking
    ok &= sub("src/ORBmatcher.cc",
              """                        const cv::Mat &d = F.mDescriptors.row(idx);

                        const int dist = DescriptorDistance(MPdescriptor,d);
""",
              """                        // PERF: Mat::row() builds a cv::Mat header per comparison,
                        // including an atomic refcount inc/dec. Use the raw row.
                        const int dist = DescriptorDistance(pMPdesc, F.mDescriptors.ptr<unsigned char>(idx));
""",
              1, "SbP local-map row()")
    ok &= sub("src/ORBmatcher.cc",
              """                if(!vIndices.empty()){
                    const cv::Mat MPdescriptor = pMP->GetDescriptor();
""",
              """                if(!vIndices.empty()){
                    // PERF: MapPoint::GetDescriptor() returns mDescriptor.clone(),
                    // i.e. a heap allocation per visible map point per frame. Copy
                    // the 32 bytes into a stack buffer under the same lock instead.
                    unsigned char aMPdesc[32];
                    pMP->CopyDescriptorTo(aMPdesc);
                    const unsigned char *pMPdesc = aMPdesc;
""",
              1, "SbP local-map GetDescriptor")
    return ok


# ---------------------------------------------------------------------------
# Patch group 04 - MapPoint accessors that do not allocate
# ---------------------------------------------------------------------------
def p04():
    print("[04] MapPoint non-allocating accessors")
    ok = True
    ok &= sub("include/MapPoint.h",
              "    std::map<KeyFrame*,std::tuple<int,int>> GetObservations();\n"
              "    int Observations();\n",
              "    std::map<KeyFrame*,std::tuple<int,int>> GetObservations();\n"
              "\n"
              "    // PERF: Tracking::UpdateLocalKeyFrames() only needs to tally which\n"
              "    // KeyFrames observe this point. GetObservations() returns the whole\n"
              "    // std::map BY VALUE - a full red-black-tree deep copy, one node\n"
              "    // allocation per observation, for every matched map point every\n"
              "    // frame. This accumulates in place under the same lock, allocating\n"
              "    // nothing. No other lock is taken inside, so it cannot deadlock.\n"
              "    void AccumulateObservingKeyFrames(std::map<KeyFrame*,int> &counter);\n"
              "\n"
              "    // PERF: non-allocating variant of GetDescriptor() for match loops.\n"
              "    // Copies the 32 descriptor bytes into caller-provided storage.\n"
              "    void CopyDescriptorTo(unsigned char *dst32);\n"
              "\n"
              "    int Observations();\n",
              1, "decl accessors")
    ok &= sub("src/MapPoint.cc",
              """std::map<KeyFrame*, std::tuple<int,int>>  MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}
""",
              """std::map<KeyFrame*, std::tuple<int,int>>  MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

void MapPoint::AccumulateObservingKeyFrames(std::map<KeyFrame*,int> &counter)
{
    unique_lock<mutex> lock(mMutexFeatures);
    for(std::map<KeyFrame*,std::tuple<int,int>>::const_iterator it=mObservations.begin(),
            itend=mObservations.end(); it!=itend; it++)
        counter[it->first]++;
}
""",
              1, "AccumulateObservingKeyFrames")
    ok &= sub("src/MapPoint.cc",
              """cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}
""",
              """cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

void MapPoint::CopyDescriptorTo(unsigned char *dst32)
{
    unique_lock<mutex> lock(mMutexFeatures);
    // mDescriptor is always 1x32 CV_8U (see ComputeDistinctiveDescriptors).
    memcpy(dst32, mDescriptor.ptr<unsigned char>(), 32);
}
""",
              1, "CopyDescriptorTo")
    ok &= sub("src/MapPoint.cc",
              '#include "MapPoint.h"\n',
              '#include "MapPoint.h"\n\n#include <cstring>   // PERF: memcpy in CopyDescriptorTo\n',
              1, "add cstring")

    # use it in UpdateLocalKeyFrames (both branches)
    ok &= sub("src/Tracking.cc",
              """                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
""",
              """                    // PERF: was a full std::map deep copy per map point per frame.
                    pMP->AccumulateObservingKeyFrames(keyframeCounter);
""",
              2, "UpdateLocalKeyFrames x2")
    return ok


# ---------------------------------------------------------------------------
# Patch group 05 - skip viewer-only work when running headless
# ---------------------------------------------------------------------------
def p05():
    print("[05] headless: skip viewer-only work")
    ok = True
    ok &= sub("src/Tracking.cc",
              """        // Update drawer
        mpFrameDrawer->Update(this);
        if(mCurrentFrame.isSet())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());
""",
              """        // PERF: FrameDrawer::Update() deep-copies the entire Frame (descriptor
        // clone + all 64*48 grid cell vectors + mmProjectPoints/mmMatchedInImage
        // maps) plus the whole local map point vector, every single frame.
        // Nothing reads any of it when the Pangolin viewer is disabled, which is
        // this project's configuration (config/no_cli_rgbd.yaml has
        // visualization.enabled=false). mpViewer stays NULL unless System was
        // constructed with bUseViewer=true (System.cc:229-236).
        if(mpViewer)
        {
            // Update drawer
            mpFrameDrawer->Update(this);
            if(mCurrentFrame.isSet())
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.GetPose());
        }
""",
              1, "guard FrameDrawer::Update")
    ok &= sub("src/Tracking.cc",
              """        if(pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
""",
              """        // PERF: mmProjectPoints' only consumer is FrameDrawer (FrameDrawer.cc:89
        // and :393). One red-black-tree insert per visible local map point per
        // frame, then deep-copied again by every Frame copy. Waste when headless.
        if(mpViewer && pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
""",
              1, "guard mmProjectPoints")
    return ok


# ---------------------------------------------------------------------------
# Patch group 06 - parallel ORB extraction across pyramid levels
# ---------------------------------------------------------------------------
def p06():
    print("[06] ORBextractor: parallel pyramid levels")
    ok = True
    ok &= sub("include/ORBextractor.h",
              "    std::vector<cv::Mat> mvImagePyramid;\n",
              "    std::vector<cv::Mat> mvImagePyramid;\n"
              "\n"
              "    // PERF: persistent per-level scratch buffers for the pre-descriptor\n"
              "    // Gaussian blur. The original allocated a fresh clone() per level per\n"
              "    // frame. copyTo() into a buffer of identical size/type reuses the\n"
              "    // allocation and, exactly like clone(), yields a standalone Mat - so\n"
              "    // BORDER_REFLECT_101 stays ROI-isolated and descriptors stay identical.\n"
              "    std::vector<cv::Mat> mvBlurBuffer;\n",
              1, "mvBlurBuffer member")
    ok &= sub("src/ORBextractor.cc",
              "        mvImagePyramid.resize(nlevels);\n",
              "        mvImagePyramid.resize(nlevels);\n        mvBlurBuffer.resize(nlevels);\n",
              1, "resize mvBlurBuffer")

    # parallel FAST + octree distribution
    ok &= sub("src/ORBextractor.cc",
              """        allKeypoints.resize(nlevels);

        const float W = 35;

        for (int level = 0; level < nlevels; ++level)
        {
            const int minBorderX = EDGE_THRESHOLD-3;
""",
              """        allKeypoints.resize(nlevels);

        const float W = 35;

        // PERF: each iteration reads only mvImagePyramid[level] and writes only
        // allKeypoints[level]; DistributeOctTree() mutates no member state. The
        // levels are therefore fully independent. FAST + octree distribution
        // dominate extraction cost, so this is the best parallelization point
        // in the extractor. allKeypoints is pre-sized above, so no reallocation
        // races are possible.
#if defined(_OPENMP) && defined(ORBEXTRACTOR_OMP_THREADS)
#pragma omp parallel for schedule(dynamic) num_threads(ORBEXTRACTOR_OMP_THREADS)
#endif
        for (int level = 0; level < nlevels; ++level)
        {
            const int minBorderX = EDGE_THRESHOLD-3;
""",
              1, "omp ComputeKeyPointsOctTree")

    ok &= sub("src/ORBextractor.cc",
              """        // compute orientations
        for (int level = 0; level < nlevels; ++level)
            computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
""",
              """        // compute orientations
        // PERF: computeOrientation() is a free function reading the image and
        // writing only allKeypoints[level] -> independent per level.
#if defined(_OPENMP) && defined(ORBEXTRACTOR_OMP_THREADS)
#pragma omp parallel for schedule(dynamic) num_threads(ORBEXTRACTOR_OMP_THREADS)
#endif
        for (int level = 0; level < nlevels; ++level)
            computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
""",
              1, "omp computeOrientation")

    # split blur+descriptors (parallel) from the order-dependent scatter (serial)
    old_loop = """        int offset = 0;
        //Modified for speeding up stereo fisheye matching
        int monoIndex = 0, stereoIndex = nkeypoints-1;
        for (int level = 0; level < nlevels; ++level)
        {
            vector<KeyPoint>& keypoints = allKeypoints[level];
            int nkeypointsLevel = (int)keypoints.size();

            if(nkeypointsLevel==0)
                continue;

            // preprocess the resized image
            Mat workingMat = mvImagePyramid[level].clone();
            GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

            // Compute the descriptors
            //Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
            Mat desc = cv::Mat(nkeypointsLevel, 32, CV_8U);
            computeDescriptors(workingMat, keypoints, desc, pattern);

            offset += nkeypointsLevel;


            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
"""
    new_loop = """        int offset = 0;
        //Modified for speeding up stereo fisheye matching
        int monoIndex = 0, stereoIndex = nkeypoints-1;

        // PERF pass 1 (parallel): the per-level Gaussian blur and descriptor
        // computation are independent. Only the scatter in pass 2 carries state
        // across levels (offset / monoIndex / stereoIndex), so that stays serial.
        vector<Mat> vLevelDesc(nlevels);
#if defined(_OPENMP) && defined(ORBEXTRACTOR_OMP_THREADS)
#pragma omp parallel for schedule(dynamic) num_threads(ORBEXTRACTOR_OMP_THREADS)
#endif
        for (int level = 0; level < nlevels; ++level)
        {
            const int nkeypointsLevel = (int)allKeypoints[level].size();
            if(nkeypointsLevel==0)
                continue;

            // preprocess the resized image.
            // copyTo() into the persistent buffer replaces clone(): it reuses the
            // allocation across frames and still yields a standalone Mat, so
            // BORDER_REFLECT_101 remains ROI-isolated exactly as before.
            mvImagePyramid[level].copyTo(mvBlurBuffer[level]);
            GaussianBlur(mvBlurBuffer[level], mvBlurBuffer[level], Size(7, 7), 2, 2, BORDER_REFLECT_101);

            vLevelDesc[level] = cv::Mat(nkeypointsLevel, 32, CV_8U);
            computeDescriptors(mvBlurBuffer[level], allKeypoints[level], vLevelDesc[level], pattern);
        }

        // PERF pass 2 (serial): keypoint rescaling and scatter into the output.
        for (int level = 0; level < nlevels; ++level)
        {
            vector<KeyPoint>& keypoints = allKeypoints[level];
            int nkeypointsLevel = (int)keypoints.size();

            if(nkeypointsLevel==0)
                continue;

            Mat &desc = vLevelDesc[level];

            offset += nkeypointsLevel;


            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
"""
    ok &= sub("src/ORBextractor.cc", old_loop, new_loop, 1, "split desc/scatter")
    return ok


# ---------------------------------------------------------------------------
# Patch group 07 - CMake: OpenMP, profiling switch, unsafe-flag documentation
# ---------------------------------------------------------------------------
def p07():
    print("[07] CMakeLists: OpenMP + profiling switches")
    ok = True
    ok &= sub("CMakeLists.txt",
              'set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")\n',
              'set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")\n'
              "\n"
              "# ---------------------------------------------------------------------------\n"
              "# Performance / profiling options\n"
              "# ---------------------------------------------------------------------------\n"
              "# Flags deliberately NOT enabled, and why:\n"
              "#  * -ffast-math / -funsafe-math-optimizations: UNSAFE for this codebase. It\n"
              "#    implies -ffinite-math-only, letting the compiler assume no NaN/Inf.\n"
              "#    ORB-SLAM3 and g2o gate outlier rejection on chi2 and NaN comparisons,\n"
              "#    so this can silently turn outlier rejection into a no-op. Do not use.\n"
              "#  * -mavx512f and friends: the target CPU (i9-12900HK, Alder Lake) has NO\n"
              "#    AVX-512 - it is fused off. Forcing it yields SIGILL. -march=native\n"
              "#    already selects AVX2 + FMA + POPCNT, which is what this code benefits\n"
              "#    from (see ORBmatcher::DescriptorDistance).\n"
              "#  * -flto: safe, but ORB_SLAM3 is a shared library consumed through a\n"
              "#    prebuilt .so path, so cross-TU inlining stops at the library boundary.\n"
              "#    Little upside for a long rebuild; enable only if measured to help.\n"
              "\n"
              "# Per-stage tracking timers. The #ifdef REGISTER_TIMES instrumentation\n"
              "# already present in Tracking.cc / Frame.cc was never switched on.\n"
              'option(ORBSLAM3_REGISTER_TIMES "Enable stock per-stage tracking timers" OFF)\n'
              "if(ORBSLAM3_REGISTER_TIMES)\n"
              "  add_definitions(-DREGISTER_TIMES)\n"
              '  message(STATUS "REGISTER_TIMES enabled")\n'
              "endif()\n"
              "\n"
              "# Fine-grained per-frame profiler (include/StageTimer.h). Prints a per-frame\n"
              "# breakdown plus p50/p95/max, which is what a hard 33.33 ms budget needs.\n"
              'option(ORBSLAM3_PROFILE "Enable per-frame stage profiler" OFF)\n'
              "if(ORBSLAM3_PROFILE)\n"
              "  add_definitions(-DORBSLAM3_PROFILE)\n"
              '  message(STATUS "ORBSLAM3_PROFILE enabled (per-frame stage breakdown)")\n'
              "endif()\n"
              "\n"
              "# Parallel ORB extraction across pyramid levels. Keep this well below the\n"
              "# core count: LocalMapping and LoopClosing need cores too, and oversubscribing\n"
              "# makes tracking p95 worse even when the mean improves.\n"
              'set(ORBEXTRACTOR_OMP_THREADS "4" CACHE STRING\n'
              '    "Threads for per-pyramid-level ORB extraction (unset/1 = serial)")\n'
              "find_package(OpenMP)\n"
              "if(OpenMP_CXX_FOUND)\n"
              "  add_definitions(-DORBEXTRACTOR_OMP_THREADS=${ORBEXTRACTOR_OMP_THREADS})\n"
              '  message(STATUS "OpenMP found; ORB extraction threads = ${ORBEXTRACTOR_OMP_THREADS}")\n'
              "else()\n"
              '  message(STATUS "OpenMP NOT found; ORB extraction stays serial")\n'
              "endif()\n",
              1, "perf options block")
    ok &= sub("CMakeLists.txt",
              "DBoW2\ng2o\n-lboost_serialization\n-lcrypto\n)\n",
              "DBoW2\ng2o\n-lboost_serialization\n-lcrypto\n)\n"
              "\n"
              "if(OpenMP_CXX_FOUND)\n"
              "  target_link_libraries(${ORB_SLAM3_TARGET} OpenMP::OpenMP_CXX)\n"
              "endif()\n",
              1, "link OpenMP")
    return ok


def main():
    prep()
    allok = True
    for fn in (p01, p02, p03, p04, p05, p06, p07):
        allok &= bool(fn())
    os.makedirs(OUT, exist_ok=True)
    combined = os.path.join(OUT, "orbslam3-perf-all.patch")
    with open(combined, "w") as out:
        r = subprocess.run(["diff", "-u", "-r", "-N", ORIG, MOD],
                           capture_output=True, text=True)
        txt = r.stdout.replace(ORIG + "/", "a/").replace(MOD + "/", "b/")
        out.write(txt)
    print("\ncombined patch -> %s (%d lines)" % (combined, txt.count("\n")))
    print("ALL EDITS APPLIED" if allok else "SOME EDITS MISSED - see !! above")
    return 0 if allok else 1


if __name__ == "__main__":
    sys.exit(main())
