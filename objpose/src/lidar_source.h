// lidar_source.h — VLP-16 scans into SensorData, so RTAB-Map can close loops with ICP instead of
// with the camera (--lidar DIR, and Reg/Strategy 1=Icp or 2=VisIcp).
//
// Why: with an accurate odometry (KISS-ICP on the same lidar, sigma ~1 cm per 1 Hz link) the mapper's
// VISUAL loop constraints are the noisier party — PnP over ~300 GFTT/BRIEF features whose 3D comes
// from stereo-IR depth. Folding them into the graph measurably degrades the chain they correct
// (measured on 260901: revisit consistency 4.88 -> 5.48 cm). An ICP constraint between two revisited
// scans is built from the same sensor as the odometry, so it cannot be worse than it.
//
// Loop DETECTION stays appearance-based either way — Rtabmap's Bayes filter over the visual bag of
// words is what proposes candidates. Reg/Strategy only decides how the proposed link's transform is
// computed and verified. So the camera is still needed; the lidar only replaces the geometry step.
//
// FRAMES. The rig json gives T_lidar_cam (p_lidar = T_lidar_cam * p_cam_optical), while
// LaserScan::localTransform must be T_base_lidar, base being RTAB-Map's x-forward frame. With
// O = CameraModel::opticalRotation() = T_base_optical:
//     T_base_lidar = O * T_lidar_cam^-1
// Getting this wrong puts the scan in the wrong place relative to the camera and every ICP link is
// then biased by that constant — which looks like "ICP is worse", not like a bug. report() prints
// the transform and the scan extent so it can be checked against the rig geometry (the lidar sits
// 0.59 m above ground, the SLAM camera 1.04 m, per rt/260901/X_lidar_sam_260901.json).
//
// CLOCK. time.lidar_minus_cam_s: an event stamped t on the camera clock is stamped t + tau on the
// lidar clock (260901: -0.185 ms). The velodyne header stamp is the END of the sweep (the per-point
// `time` field runs -0.0995..0 s), which is also what KISS-ICP deskews to, so stamp and geometry
// agree and no extra shift is needed. Scans arrive at 10 Hz against 30 Hz frames, so at most every
// third frame carries a fresh one; at() returns the nearest within max_gap and reports the gap.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <sqlite3.h>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>

#include "raw_session.h"
#include "rtab_aux.h"   // rt_from_eigen(), and the same rig-json conventions

struct LidarSource {
  struct Scan {
    long long t_ns;
    std::vector<float> xyz;      // 3 floats per point, lidar frame
  };
  std::vector<Scan> scans;
  std::string db3, topic;
  long long tau_ns = 0;
  double rate_hz = 0;
  float range_min = 0.5f, range_max = 30.0f;     // same window KISS-ICP used (kiss_gt.py)
  long long max_gap_ns = 60000000;               // 60 ms: a 10 Hz scan is at most 50 ms from a frame
  rtabmap::Transform local;                      // T_base_lidar
  long long found = 0, missing = 0, max_gap_seen_ns = 0;
  // The rig json's vertical lever arm is NOT measured: calib_rig.py regularises it to zero because a
  // planar drive cannot observe it ("lever arm along the (vertical) rotation axis is unobservable").
  // On 260901 it comes out as +0.43 m while the rig really has the lidar 0.45 m BELOW the camera
  // (0.59 m vs 1.04 m above ground). Set lever_z to override it. For yaw-only motion the error is
  // harmless — conjugating a relative pose by a pure z offset changes it by (R-I)*(0,0,dz), which is
  // zero when R is a rotation about z — but that is an argument, so --lidar-z exists to test it.
  bool lever_z_set = false;
  float lever_z = 0.f;

  static std::string one_db3(const std::string& dir) {
    DIR* d = opendir(dir.c_str());
    if (!d) throw std::runtime_error("cannot open lidar folder " + dir);
    std::string out;
    while (dirent* e = readdir(d)) {
      std::string n = e->d_name;
      if (n.size() > 4 && n.substr(n.size() - 4) == ".db3") out = dir + "/" + n;
    }
    closedir(d);
    if (out.empty()) throw std::runtime_error("no .db3 in lidar folder " + dir);
    return out;
  }

  // sensor_msgs/PointCloud2 -> the x,y,z of every finite point inside [range_min, range_max]
  bool parse(const std::vector<uint8_t>& blob, Scan& out) const {
    CdrReader r(blob);
    int32_t sec = r.get<int32_t>();
    uint32_t nsec = r.get<uint32_t>();
    r.str();                                       // frame_id
    uint32_t height = r.get<uint32_t>(), width = r.get<uint32_t>();
    uint32_t nf = r.get<uint32_t>();
    int off_x = -1, off_y = -1, off_z = -1;
    for (uint32_t i = 0; i < nf; ++i) {
      std::string name = r.str();
      uint32_t off = r.get<uint32_t>();
      uint8_t dt = r.get<uint8_t>();
      r.get<uint32_t>();                           // count
      if (dt != 7) continue;                       // 7 = FLOAT32; x/y/z are float on this driver
      if (name == "x") off_x = int(off);
      else if (name == "y") off_y = int(off);
      else if (name == "z") off_z = int(off);
    }
    r.get<uint8_t>();                              // is_bigendian
    uint32_t point_step = r.get<uint32_t>();
    r.get<uint32_t>();                             // row_step
    uint32_t nbytes = r.get<uint32_t>();
    if (off_x < 0 || off_y < 0 || off_z < 0 || point_step < 12) return false;
    const uint8_t* p = blob.data() + r.o;
    const size_t n = std::min<size_t>(size_t(height) * width, nbytes / point_step);
    out.t_ns = (long long)sec * 1000000000LL + nsec;
    out.xyz.clear();
    out.xyz.reserve(n * 3);
    for (size_t i = 0; i < n; ++i) {
      const uint8_t* q = p + i * point_step;
      float x, y, z;
      std::memcpy(&x, q + off_x, 4);
      std::memcpy(&y, q + off_y, 4);
      std::memcpy(&z, q + off_z, 4);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
      const float d = std::sqrt(x * x + y * y + z * z);
      if (d < range_min || d > range_max) continue;
      out.xyz.push_back(x);
      out.xyz.push_back(y);
      out.xyz.push_back(z);
    }
    return !out.xyz.empty();
  }

  void load(const std::string& lidar_dir, const std::string& rig_json, const std::string& topic_in) {
    std::ifstream in(rig_json);
    if (!in) throw std::runtime_error("cannot read rig json " + rig_json);
    nlohmann::json rig = nlohmann::json::parse(in);
    Eigen::Matrix4d L = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c) L(r, c) = rig.at("T_lidar_cam").at(r).at(c).get<double>();
    tau_ns = (long long)std::llround(rig.at("time").at("lidar_minus_cam_s").get<double>() * 1e9);
    const rtabmap::Transform T_lidar_cam = rt_from_eigen(L);
    local = rtabmap::CameraModel::opticalRotation() * T_lidar_cam.inverse();
    if (lever_z_set) {
      Eigen::Matrix4f M = local.toEigen4f();
      M(2, 3) = lever_z;
      local = rtabmap::Transform(M(0,0),M(0,1),M(0,2),M(0,3), M(1,0),M(1,1),M(1,2),M(1,3),
                                 M(2,0),M(2,1),M(2,2),M(2,3));
    }

    topic = topic_in;
    db3 = one_db3(lidar_dir);
    sqlite3* db = nullptr;
    std::string uri = "file:" + db3 + "?immutable=1";
    if (sqlite3_open_v2(uri.c_str(), &db, SQLITE_OPEN_READONLY | SQLITE_OPEN_URI, nullptr) != SQLITE_OK)
      throw std::runtime_error("cannot open " + db3);
    sqlite3_stmt* st;
    sqlite3_prepare_v2(db,
                       "SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY m.timestamp",
                       -1, &st, nullptr);
    sqlite3_bind_text(st, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
    int rc;
    while ((rc = sqlite3_step(st)) == SQLITE_ROW) {
      const uint8_t* p = static_cast<const uint8_t*>(sqlite3_column_blob(st, 0));
      std::vector<uint8_t> blob(p, p + sqlite3_column_bytes(st, 0));
      Scan s;
      if (!parse(blob, s)) continue;
      if (!scans.empty() && s.t_ns <= scans.back().t_ns) continue;   // the 260901 bags have no dupes
      scans.push_back(std::move(s));
    }
    std::string err = rc == SQLITE_DONE ? "" : sqlite3_errmsg(db);
    sqlite3_finalize(st);
    sqlite3_close(db);
    if (!err.empty()) throw std::runtime_error("reading " + db3 + ": " + err);
    if (scans.size() < 10) throw std::runtime_error("lidar topic " + topic + " has no data in " + db3);
    rate_hz = double(scans.size() - 1) / (double(scans.back().t_ns - scans.front().t_ns) * 1e-9);
  }

  // Nearest scan to a camera stamp. Empty when there is none close enough (RTAB-Map then simply
  // creates a node without a scan, and that node can only take a visual link).
  rtabmap::LaserScan at(long long t_cam_ns) {
    if (scans.empty()) { ++missing; return rtabmap::LaserScan(); }
    const long long t = t_cam_ns + tau_ns;
    size_t i = size_t(std::lower_bound(scans.begin(), scans.end(), t,
                                       [](const Scan& s, long long v) { return s.t_ns < v; }) - scans.begin());
    if (i == scans.size()) i = scans.size() - 1;
    else if (i > 0 && (t - scans[i - 1].t_ns) < (scans[i].t_ns - t)) --i;
    const long long gap = std::llabs(t - scans[i].t_ns);
    if (gap > max_gap_ns) { ++missing; return rtabmap::LaserScan(); }
    ++found;
    max_gap_seen_ns = std::max(max_gap_seen_ns, gap);
    const Scan& s = scans[i];
    cv::Mat m(1, int(s.xyz.size() / 3), CV_32FC3);
    std::memcpy(m.data, s.xyz.data(), s.xyz.size() * sizeof(float));
    return rtabmap::LaserScan(m, 0, range_max, rtabmap::LaserScan::kXYZ, local);
  }

  // Self-check: the scan extent in the base frame. z should straddle the floor at about -0.59 m
  // (the lidar's height on this rig) and the cloud should reach out to range_max in x and y.
  std::string report() const {
    if (scans.empty()) return "lidar: no scans";
    const Scan& s = scans[scans.size() / 2];
    Eigen::Matrix4f T = local.toEigen4f();
    float lo[3] = {1e9f, 1e9f, 1e9f}, hi[3] = {-1e9f, -1e9f, -1e9f};
    for (size_t i = 0; i + 2 < s.xyz.size(); i += 3) {
      Eigen::Vector4f v(s.xyz[i], s.xyz[i + 1], s.xyz[i + 2], 1.f);
      Eigen::Vector4f b = T * v;
      for (int k = 0; k < 3; ++k) { lo[k] = std::min(lo[k], b[k]); hi[k] = std::max(hi[k], b[k]); }
    }
    char buf[300];
    snprintf(buf, sizeof(buf),
             "mid scan %zu points -> base frame x %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f m "
             "(z low should be about -0.59, the lidar height on this rig)",
             s.xyz.size() / 3, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]);
    return buf;
  }
};
