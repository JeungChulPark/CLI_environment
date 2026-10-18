// rtab_aux.h — external odometry for rtab_stream. The IMU path lives in imu_source.h.
//
//   ExtOdom   --ext-odom TUM   an external odometry trajectory (here KISS-ICP on the VLP-16) replacing the
//                              visual odometry entirely: rtab_stream stops calling rtabmap::Odometry and
//                              passes these poses straight to Rtabmap::process(data, odom). The camera is
//                              still what the mapper sees — only the odometry source changes.
//
// Rig json keys used (objpose/rt/260901/bigeight/calib_rig.py):
//   T_lidar_cam [4x4]         camera(optical) -> lidar
//   time.lidar_minus_cam_s    an event stamped t on the camera clock is stamped t + tau on the lidar clock
#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <dirent.h>
#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Transform.h>

#include "raw_session.h"

// ---------------------------------------------------------------------------------------------
static inline rtabmap::Transform rt_from_eigen(const Eigen::Matrix4d& M) {
  return rtabmap::Transform((float)M(0, 0), (float)M(0, 1), (float)M(0, 2), (float)M(0, 3),
                            (float)M(1, 0), (float)M(1, 1), (float)M(1, 2), (float)M(1, 3),
                            (float)M(2, 0), (float)M(2, 1), (float)M(2, 2), (float)M(2, 3));
}

static inline std::string rt_find_db3(const std::string& dir) {
  DIR* d = opendir(dir.c_str());
  if (!d) throw std::runtime_error("cannot open IMU folder " + dir);
  std::string out;
  while (dirent* e = readdir(d)) {
    std::string n = e->d_name;
    if (n.size() > 4 && n.substr(n.size() - 4) == ".db3") out = dir + "/" + n;
  }
  closedir(d);
  if (out.empty()) throw std::runtime_error("no .db3 in IMU folder " + dir);
  return out;
}

// ---------------------------------------------------------------------------------------------
struct ExtOdom {
  std::vector<long long> t_ns;                 // on the CAMERA clock already
  std::vector<rtabmap::Transform> T;           // T_odom_base, first pose = identity
  long long served = 0, missed = 0;
  std::string path;

  // TUM lines are "t tx ty tz qx qy qz qw" with t on the LIDAR clock and the pose T_world_lidar.
  //   T_world_base = T_world_lidar * T_lidar_cam * T_base_optical^-1
  // then everything is re-anchored on the first pose so the stream starts at identity, as
  // rtabmap::Odometry would.
  void load(const std::string& tum, const std::string& rig_json) {
    path = tum;
    std::ifstream rj(rig_json);
    if (!rj) throw std::runtime_error("cannot read rig json " + rig_json);
    nlohmann::json rig = nlohmann::json::parse(rj);
    Eigen::Matrix4d L = Eigen::Matrix4d::Identity();
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c) L(r, c) = rig.at("T_lidar_cam").at(r).at(c).get<double>();
    const long long tau = (long long)std::llround(rig.at("time").at("lidar_minus_cam_s").get<double>() * 1e9);
    const rtabmap::Transform T_lidar_cam = rt_from_eigen(L);
    const rtabmap::Transform O = rtabmap::CameraModel::opticalRotation();

    std::ifstream in(tum);
    if (!in) throw std::runtime_error("cannot read external odometry " + tum);
    std::string line;
    std::vector<rtabmap::Transform> raw;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') continue;
      double t, x, y, z, qx, qy, qz, qw;
      if (sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &t, &x, &y, &z, &qx, &qy, &qz, &qw) != 8) continue;
      // stamp back onto the camera clock
      t_ns.push_back((long long)std::llround(t * 1e9) - tau);
      raw.push_back(rtabmap::Transform((float)x, (float)y, (float)z, (float)qx, (float)qy, (float)qz, (float)qw) *
                    T_lidar_cam * O.inverse());
    }
    if (raw.size() < 10) throw std::runtime_error("external odometry " + tum + " has too few poses");
    const rtabmap::Transform inv0 = raw.front().inverse();
    T.reserve(raw.size());
    for (const auto& t : raw) T.push_back(inv0 * t);
  }

  // Linear position / slerp orientation between the two bracketing poses.
  bool at(long long t_cam_ns, rtabmap::Transform& out) {
    if (t_cam_ns < t_ns.front() || t_cam_ns > t_ns.back()) { ++missed; return false; }
    size_t k = size_t(std::upper_bound(t_ns.begin(), t_ns.end(), t_cam_ns) - t_ns.begin());
    if (k == 0) { out = T.front(); ++served; return true; }
    if (k >= t_ns.size()) { out = T.back(); ++served; return true; }
    const double u = double(t_cam_ns - t_ns[k - 1]) / double(t_ns[k] - t_ns[k - 1]);
    const rtabmap::Transform &A = T[k - 1], &B = T[k];
    Eigen::Quaternionf qa = A.getQuaternionf(), qb = B.getQuaternionf();
    if (qa.dot(qb) < 0) qb.coeffs() *= -1;
    Eigen::Quaternionf qi = qa.slerp((float)u, qb);
    qi.normalize();
    out = rtabmap::Transform(A.x() + (B.x() - A.x()) * (float)u,
                             A.y() + (B.y() - A.y()) * (float)u,
                             A.z() + (B.z() - A.z()) * (float)u,
                             qi.x(), qi.y(), qi.z(), qi.w());
    ++served;
    return true;
  }
};
