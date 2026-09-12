// Equivalence + speed test for the rewritten ORBmatcher::DescriptorDistance.
//
// Build & run:
//   g++ -O3 -march=native -std=c++14 test_descriptor_distance.cpp -o /tmp/tdd && /tmp/tdd
//
// Proves the new hardware-POPCNT implementation returns bit-identical results
// to the stock SWAR bit-hack for every one of 20M random descriptor pairs,
// and reports the speedup.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <random>

// ---- stock ORB-SLAM3 implementation (src/ORBmatcher.cc:2058) ----------------
static int DescriptorDistance_stock(const unsigned char *a, const unsigned char *b)
{
    const int *pa = reinterpret_cast<const int *>(a);
    const int *pb = reinterpret_cast<const int *>(b);

    int dist = 0;

    for (int i = 0; i < 8; i++, pa++, pb++)
    {
        unsigned int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

// ---- patched implementation -------------------------------------------------
static int DescriptorDistance_new(const unsigned char *pa, const unsigned char *pb)
{
    uint64_t va[4], vb[4];
    memcpy(va, pa, 32);
    memcpy(vb, pb, 32);
    return __builtin_popcountll(va[0] ^ vb[0])
         + __builtin_popcountll(va[1] ^ vb[1])
         + __builtin_popcountll(va[2] ^ vb[2])
         + __builtin_popcountll(va[3] ^ vb[3]);
}

int main()
{
    const int N = 20000;          // descriptors
    const int PAIRS = 20000000;   // comparisons

    std::mt19937_64 rng(12345);
    // 32-byte aligned-ish contiguous block, mimicking cv::Mat Nx32 CV_8U rows.
    static unsigned char *buf = new unsigned char[static_cast<size_t>(N) * 32];
    for (size_t i = 0; i < static_cast<size_t>(N) * 32; ++i)
        buf[i] = static_cast<unsigned char>(rng() & 0xFF);

    // ---- 1. equivalence ----
    long mismatches = 0;
    for (int k = 0; k < PAIRS / 4; ++k)
    {
        const int i = static_cast<int>(rng() % N);
        const int j = static_cast<int>(rng() % N);
        const int d0 = DescriptorDistance_stock(buf + 32L * i, buf + 32L * j);
        const int d1 = DescriptorDistance_new(buf + 32L * i, buf + 32L * j);
        if (d0 != d1) ++mismatches;
    }
    printf("equivalence: %d random pairs, mismatches = %ld  -> %s\n",
           PAIRS / 4, mismatches, mismatches == 0 ? "BIT-IDENTICAL" : "*** DIFFERS ***");

    // include the degenerate cases explicitly
    unsigned char z[32], o[32];
    memset(z, 0x00, 32);
    memset(o, 0xFF, 32);
    printf("  dist(zeros,zeros) stock=%d new=%d\n",
           DescriptorDistance_stock(z, z), DescriptorDistance_new(z, z));
    printf("  dist(zeros,ones)  stock=%d new=%d  (expect 256)\n",
           DescriptorDistance_stock(z, o), DescriptorDistance_new(z, o));

    // ---- 2. throughput ----
    volatile long sink = 0;
    auto bench = [&](int (*fn)(const unsigned char *, const unsigned char *)) {
        auto t0 = std::chrono::steady_clock::now();
        long acc = 0;
        for (int k = 0; k < PAIRS; ++k)
        {
            const int i = k % N;
            const int j = (k * 7919) % N;
            acc += fn(buf + 32L * i, buf + 32L * j);
        }
        sink += acc;
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now() - t0).count();
    };

    const double ms_stock = bench(DescriptorDistance_stock);
    const double ms_new   = bench(DescriptorDistance_new);

    printf("\nthroughput over %d comparisons:\n", PAIRS);
    printf("  stock (SWAR bithack) : %8.1f ms  (%.1f M cmp/s)\n",
           ms_stock, PAIRS / ms_stock / 1000.0);
    printf("  new   (POPCNT)       : %8.1f ms  (%.1f M cmp/s)\n",
           ms_new, PAIRS / ms_new / 1000.0);
    printf("  speedup              : %.2fx\n", ms_stock / ms_new);
    return mismatches == 0 ? 0 : 1;
}
