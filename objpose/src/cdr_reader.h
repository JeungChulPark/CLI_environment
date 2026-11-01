// cdr_reader.h — minimal ROS 2 CDR reader, shared by raw_session.h and gyro_aid.h.
// Split out of raw_session.h so gyro_aid.h can also be used by programs that do not include raw_session.h
// (orbslam_ws/mac_harness/live_orbslam.cc, whose live_common.h defines helpers of the same names).
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// Minimal CDR reader (little/big endian encapsulation; alignment relative to the 4-byte header).
struct CdrReader {
  const std::vector<uint8_t>& b;
  size_t o = 4;
  bool le;
  explicit CdrReader(const std::vector<uint8_t>& blob) : b(blob) { le = blob.size() > 1 && blob[1] == 1; }
  void align(size_t n) { size_t r = (o - 4) % n; if (r) o += n - r; }
  template <typename T> T get() {
    align(sizeof(T));
    T v;
    std::memcpy(&v, b.data() + o, sizeof(T));
    if (!le) {
      uint8_t tmp[sizeof(T)];
      std::memcpy(tmp, &v, sizeof(T));
      std::reverse(tmp, tmp + sizeof(T));
      std::memcpy(&v, tmp, sizeof(T));
    }
    o += sizeof(T);
    return v;
  }
  std::string str() {
    uint32_t n = get<uint32_t>();
    std::string s(reinterpret_cast<const char*>(b.data() + o), n ? n - 1 : 0);
    o += n;
    return s;
  }
};
