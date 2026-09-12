// Phase 1 -- cap every library thread pool and warm up OpenMP at startup.
//
// MEASURED on this machine (i9-12900HK, WSL2):
//     FIRST omp parallel region (thread-pool spawn) : 10.508 ms
//     WARM omp parallel region, default 20 threads  :  3.7858 ms  PER REGION
//     WARM omp parallel region,  4 threads          :  0.0015 ms
//     WARM omp parallel region,  8 threads          :  0.0033 ms
//
// A single fork/join barrier at the default thread count eats 11% of a 33.33 ms budget.
//
// NOTE (verified by grep over corelib/): RTAB-Map's own code contains ZERO `#pragma omp`
// on the RGB-D visual-odometry critical path. The regions that DO exist are in
// util3d_surface.cpp (laser-scan normal orientation), util3d_filtering.cpp:1465
// (proportionalRadiusFiltering) and the vendored rtflann nn_index.h / lsh_index.h --
// and note rtflann/algorithms/kdtree_index.h has ALL 20 of its pragmas commented out,
// so the kd-tree search RTAB-Map actually uses is single-threaded.
//
// The exposure is therefore:
//   (a) ICP paths -- pcl::NormalEstimationOMP via util3d::computeNormals
//       (util3d_surface.cpp:2838, gated on PCL_OMP which CMakeLists.txt:276 defines by default)
//   (b) PCL's own internal OpenMP in filters/estimators
//   (c) OpenCV's parallel_for_ -- on THIS machine the only OpenCV present reports
//       "Parallel framework: pthreads" (not OpenMP), but its pool still defaults to
//       all 20 logical CPUs, and a conda/robostack OpenCV may use a different backend.
//
// RTAB-Map never calls cv::setNumThreads() or omp_set_num_threads() anywhere
// (verified by grep). Everything is left at library defaults. That is the bug.
//
// Call rtabmap_perf::initThreadPools() once, early in main(), BEFORE constructing
// Rtabmap / Odometry / any camera.

#include <opencv2/core/utility.hpp>   // cv::setNumThreads
#ifdef _OPENMP
#include <omp.h>
#endif
#include <cstdlib>

namespace rtabmap_perf {

inline void initThreadPools(int nThreads = 6)
{
#ifdef _OPENMP
    omp_set_num_threads(nThreads);
    omp_set_dynamic(0);               // don't let the runtime pick a different count

    // Warm-up: pay the 10.5 ms thread-pool spawn HERE, not on frame 1.
    // `volatile` + the accumulator keep the optimizer from deleting the region.
    volatile double sink = 0.0;
    double acc = 0.0;
    #pragma omp parallel for reduction(+:acc)
    for(int i = 0; i < nThreads * 1024; ++i) {
        acc += double(i) * 1e-9;
    }
    sink = acc;
    (void)sink;
#endif

    // OpenCV's own pool (pthreads or TBB backend -- either way it defaults to all CPUs).
    cv::setNumThreads(nThreads);

    // Keep BLAS/LAPACK backends used by Eigen out of the way; they would otherwise
    // spawn their own 20-thread pools and oversubscribe.
    setenv("OPENBLAS_NUM_THREADS", "1", 0);
    setenv("MKL_NUM_THREADS",      "1", 0);
    setenv("OMP_WAIT_POLICY",      "PASSIVE", 0);   // no spin-waiting between regions
}

} // namespace rtabmap_perf

// ---------------------------------------------------------------------------
// If you cannot patch main(), the environment-only equivalent (set BEFORE launch):
//
//   export OMP_NUM_THREADS=6
//   export OMP_WAIT_POLICY=PASSIVE
//   export OMP_PROC_BIND=close
//   export OPENBLAS_NUM_THREADS=1
//   export MKL_NUM_THREADS=1
//   export OPENCV_FOR_THREADS_NUM=6       # OpenCV's env equivalent of cv::setNumThreads
//
// This covers (a)/(b)/(c) above but does NOT give you the startup warm-up, so the
// first ICP/lidar frame will still take the ~10.5 ms spawn hit.
// ---------------------------------------------------------------------------
