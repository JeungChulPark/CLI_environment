// gyro_aid.h — gyro-only orientation for slam_stream's --gyro option (ORB-SLAM3 "GYRO AID", GyroEdges.h).
//
// Reads sensor_msgs/Imu from a rosbag2 sqlite file (the converted session's imu/ folder, /xsens/imu/data),
// removes the gyro bias, integrates the angular rate (midpoint rule) and returns, for a camera stamp, the
// camera orientation in the gyro-integration frame:  R_gc(t) = R_gi(t + tau) * R_cam_imu^T.
// ORB-SLAM3 only ever uses R_gc(t_i)^T * R_gc(t_j), so neither the start value nor gravity matters.
//
// Rig json (objpose/rt/260901/bigeight/calib_rig.py):
//   R_cam_imu [3x3]            rotation taking IMU-frame vectors to the camera frame (w_cam = R_cam_imu * w_imu)
//   time.imu_minus_cam_s       an event stamped t on the camera clock is stamped t + tau on the IMU clock
//   gyro_bias_rad_s [3]        fallback bias
// Bias (the part that matters: 3e-4 rad/s of bias error is 1 deg/min of heading drift):
//   start value   rig json `gyro_bias_rad_s`
//   online        visual low-rate update. Whenever the camera has hardly TURNED for `zupt_window_s` (standing or
//                 driving straight; judged from the SLAM poses themselves) the bias is
//                 (gyro rotation - visual rotation) / T over that window. Without turning, the camera keeps seeing
//                 the same map points, so the visual rotation over 3 s is good to a few 0.01 deg and free of the
//                 drift that builds up while turning; subtracting it also removes the real slow wobble that a plain
//                 gyro average would absorb (260901: +-4e-4 rad/s between 5 s averages, 1.6e-4 after subtraction).
//                 Running mean over the last 20 windows. No update while turning: drift and bias are not separable.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

#include <dirent.h>
#include <nlohmann/json.hpp>
#include <sqlite3.h>

#include "raw_session.h"

struct GyroAid {
  std::vector<long long> t_ns;             // IMU header stamps
  std::vector<Eigen::Vector3d> w_cam;      // RAW gyro rate rotated into the camera frame (rad/s), bias NOT removed
  Eigen::Matrix3d R_ci = Eigen::Matrix3d::Identity();
  Eigen::Vector3d bias_cam = Eigen::Vector3d::Zero();   // current bias estimate, camera frame
  long long tau_ns = 0;
  std::string db3, topic;
  double rate_hz = 0;
  // zero-rate update
  double zupt_window_s = 3.0, zupt_max_rot_rad = 0.035;   // window length, max turn inside it (2 deg)
  int zupt_prior_weight = 2, zupt_memory = 20;             // start value counts as 2 windows; running mean over <= 20 windows (~60 s)
  int zupt_updates = 0;
  bool zupt = true;

  // integration state (camera frame), advanced frame by frame
  bool started = false;
  int epoch = 0;                           // bumped whenever the integration restarts (IMU gap): no gyro tie across epochs
  long long t_last = 0;                    // IMU-clock time the integration has reached
  Eigen::Matrix3d R_gc = Eigen::Matrix3d::Identity();      // bias-corrected: handed to ORB-SLAM3
  Eigen::Matrix3d R_raw = Eigen::Matrix3d::Identity();     // no bias removed: for the zero-rate update
  struct Obs { long long t; Eigen::Matrix3d R_wc; Eigen::Matrix3d R_raw; };
  std::deque<Obs> win;

  static std::string find_db3(const std::string& dir) {
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

  void load(const std::string& imu_dir, const std::string& rig_json, const std::string& topic_in) {
    std::ifstream in(rig_json);
    if (!in) throw std::runtime_error("cannot read rig json " + rig_json);
    nlohmann::json rig = nlohmann::json::parse(in);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) R_ci(r, c) = rig.at("R_cam_imu").at(r).at(c).get<double>();
    tau_ns = (long long)std::llround(rig.at("time").at("imu_minus_cam_s").get<double>() * 1e9);
    Eigen::Vector3d rig_bias = Eigen::Vector3d::Zero();
    if (rig.contains("gyro_bias_rad_s"))
      for (int k = 0; k < 3; ++k) rig_bias[k] = rig["gyro_bias_rad_s"].at(k).get<double>();
    bias_cam = R_ci * rig_bias;

    topic = topic_in;
    db3 = find_db3(imu_dir);
    sqlite3* db = nullptr;
    // immutable: no locking / journal checks, so a read-only bag on NFS opened by several processes at once
    // never returns an empty result (seen 2026-09-18 with two runs sharing one IMU bag)
    std::string uri = "file:" + db3 + "?immutable=1";
    if (sqlite3_open_v2(uri.c_str(), &db, SQLITE_OPEN_READONLY | SQLITE_OPEN_URI, nullptr) != SQLITE_OK)
      throw std::runtime_error("cannot open " + db3);
    sqlite3_stmt* st;
    sqlite3_prepare_v2(db, "SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY m.timestamp",
                       -1, &st, nullptr);
    sqlite3_bind_text(st, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
    int rc;
    while ((rc = sqlite3_step(st)) == SQLITE_ROW) {
      const uint8_t* p = static_cast<const uint8_t*>(sqlite3_column_blob(st, 0));
      int n = sqlite3_column_bytes(st, 0);
      std::vector<uint8_t> blob(p, p + n);
      CdrReader r(blob);
      int32_t sec = r.get<int32_t>();
      uint32_t nsec = r.get<uint32_t>();
      r.str();                                   // frame_id
      for (int k = 0; k < 4 + 9; ++k) r.get<double>();   // orientation + covariance
      Eigen::Vector3d g;
      for (int k = 0; k < 3; ++k) g[k] = r.get<double>();
      long long t = (long long)sec * 1000000000LL + nsec;
      if (!t_ns.empty() && t <= t_ns.back()) continue;   // keep stamps strictly increasing
      t_ns.push_back(t);
      w_cam.push_back(R_ci * g);
    }
    std::string err = rc == SQLITE_DONE ? "" : sqlite3_errmsg(db);
    sqlite3_finalize(st);
    sqlite3_close(db);
    if (!err.empty()) throw std::runtime_error("reading " + db3 + ": " + err);
    if (t_ns.size() < 10) throw std::runtime_error("IMU topic " + topic + " has no data in " + db3);
    rate_hz = double(t_ns.size() - 1) / (double(t_ns.back() - t_ns.front()) * 1e-9);
  }

  static Eigen::Matrix3d exp_so3(const Eigen::Vector3d& v) {
    double a = v.norm();
    if (a < 1e-12) return Eigen::Matrix3d::Identity();
    return Eigen::AngleAxisd(a, v / a).toRotationMatrix();
  }
  static Eigen::Vector3d log_so3(const Eigen::Matrix3d& R) {
    Eigen::AngleAxisd aa(R);
    return aa.angle() * aa.axis();
  }

  Eigen::Vector3d rate_at(long long t) const {   // linear interpolation of the raw camera-frame rate
    size_t k = size_t(std::upper_bound(t_ns.begin(), t_ns.end(), t) - t_ns.begin());
    if (k == 0) return w_cam.front();
    if (k >= t_ns.size()) return w_cam.back();
    double u = double(t - t_ns[k - 1]) / double(t_ns[k] - t_ns[k - 1]);
    return w_cam[k - 1] + (w_cam[k] - w_cam[k - 1]) * u;
  }

  // Integrate the gyro up to a camera stamp (trapezoid between IMU samples, exact at the two ends) and return
  // the bias-corrected camera orientation R_(g <- cam). false outside the IMU stream or across an IMU gap.
  bool advance(long long t_cam_ns, Eigen::Matrix3f& out) {
    long long t = t_cam_ns + tau_ns;
    if (t < t_ns.front() || t > t_ns.back()) { started = false; return false; }
    if (!started || t <= t_last || double(t - t_last) * 1e-9 > 1.0) {   // (re)start: only differences matter
      started = true;
      ++epoch;
      t_last = t;
      win.clear();
      out = R_gc.cast<float>();
      return true;
    }
    size_t k = size_t(std::upper_bound(t_ns.begin(), t_ns.end(), t_last) - t_ns.begin());
    long long ta = t_last;
    Eigen::Vector3d wa = rate_at(ta);
    while (ta < t) {
      long long tb = (k < t_ns.size() && t_ns[k] < t) ? t_ns[k] : t;
      Eigen::Vector3d wb = (tb == t) ? rate_at(t) : w_cam[k];
      double dt = double(tb - ta) * 1e-9;
      if (dt > 0.1) { started = false; return false; }   // IMU gap
      Eigen::Vector3d wm = 0.5 * (wa + wb);
      R_raw = R_raw * exp_so3(wm * dt);
      R_gc = R_gc * exp_so3((wm - bias_cam) * dt);
      ta = tb; wa = wb;
      if (tb != t) ++k;
    }
    t_last = t;
    // re-orthonormalise now and then (products of thousands of rotations)
    Eigen::Quaterniond q1(R_gc), q2(R_raw);
    R_gc = q1.normalized().toRotationMatrix();
    R_raw = q2.normalized().toRotationMatrix();
    out = R_gc.cast<float>();
    return true;
  }

  // Feed the SLAM result of the frame just advanced to. Returns true when the bias was updated.
  bool observe(long long t_cam_ns, bool ok, const Eigen::Matrix3f& R_wc, bool map_changed) {
    if (!zupt) return false;
    if (!ok || map_changed || !started) { win.clear(); return false; }
    win.push_back({t_cam_ns, R_wc.cast<double>(), R_raw});
    while (win.size() > 2 && double(win.back().t - win[1].t) * 1e-9 >= zupt_window_s) win.pop_front();
    double T = double(win.back().t - win.front().t) * 1e-9;
    if (T < zupt_window_s) return false;
    const Obs& a = win.front();
    const Obs& b = win.back();
    // hardly turning over the whole window (standing OR driving straight): every orientation close to the first
    for (const Obs& o : win)
      if (log_so3(a.R_wc.transpose() * o.R_wc).norm() > zupt_max_rot_rad) { win.pop_front(); return false; }
    Eigen::Vector3d gyro_rot = log_so3(a.R_raw.transpose() * b.R_raw);   // R_(ca <- cb), raw gyro
    Eigen::Vector3d vis_rot = log_so3(a.R_wc.transpose() * b.R_wc);      // R_(ca <- cb), SLAM
    Eigen::Vector3d meas = (gyro_rot - vis_rot) / T;
    win.clear();                                                          // next update needs a fresh window
    if ((meas - bias_cam).norm() > 0.01) return false;                    // implausible: ignore
    ++zupt_updates;
    bias_cam += (meas - bias_cam) / double(std::min(zupt_updates + zupt_prior_weight, zupt_memory));
    return true;
  }
};
