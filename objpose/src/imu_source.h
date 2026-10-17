// imu_source.h — per-frame rtabmap::IMU for rtab_stream's --gyro option (RTAB-SLAM + IMU).
//
// Reads sensor_msgs/Imu from a rosbag2 sqlite file (the converted session's imu/ folder,
// /xsens/imu/data) and answers, for a camera stamp, the rtabmap::IMU belonging to that frame.
// RTAB-Map then uses it in two independent places, both of them STOCK library behaviour — unlike
// the ORB-SLAM3 gyro aid in gyro_aid.h, which needed a patched SetFrameGyro():
//   * OdometryF2M's local bundle adjustment takes the orientation as a per-node constraint
//     (OdometryF2M::bundleIMUOrientations_), while OdomF2M/BundleAdjustment != 0 (default 1 = g2o);
//   * Memory attaches a Link::kGravity self-reference (From == To) to each new node, which graph
//     optimisation honours while Optimizer/GravitySigma > 0 (g2o or GTSAM strategies only).
// The second one is the point of this module: gyro-only orientation is already good here (the rig
// json's gyro_vs_kiss residual is 1.25 deg median over 315 s), so a mere orientation prior has
// little left to win. What is new is that loop closure can no longer tilt the map away from
// gravity.
//
// Orientation comes from the DEVICE, not from integrating here. The MTi reports a unit quaternion
// on every sample (measured: |q| = 1.000000 across 260901_cbnu_bigeightcircle), i.e. its own AHRS
// already fuses rate and gravity. rtabmap::IMUFilter (Madgwick / complementary) exists and its
// create() is public, but RTAB-Map never runs it on this path — only SensorCaptureThread, Camera
// and IMUThread do, and rtab_stream builds SensorData itself — so using it would mean running our
// own second, worse copy of what the device already did. It stays the fallback for the day the
// device estimate is found wanting.
//
// FRAMES — the one line that can silently ruin a run. The rig json gives R_cam_imu in the camera
// OPTICAL frame (w_cam = R_cam_imu * w_imu), while IMU::localTransform must be T_base_imu, base
// being RTAB-Map's x-forward frame. With O = CameraModel::opticalRotation() = T_base_optical:
//     T_base_imu = O * T_optical_imu
// Getting this wrong does not fail loudly: it points gravity sideways, and the gravity constraint
// then pulls hard the wrong way, which looks like "IMU made it worse" rather than like a bug. So
// gravity_report() measures gravity over this bag's stillest samples and prints it in both frames;
// read it before trusting a run. In the base frame it must read about (0, 0, +9.8): an
// accelerometer at rest measures the REACTION to gravity, not gravity, and base is z-up — so the
// sign is positive even though the same vector reads -9.78 on the device's own z, which points
// down. (Measured on 260901: imu (0.523, -0.051, -9.781) -> base (0.533, -0.041, +9.781).)
// The IMU lever arm is left at zero — the rig json does not measure it, and neither orientation
// nor gravity depends on it.
//
// CLOCK. The rig json defines time.imu_minus_cam_s as "an event stamped t on the camera clock is
// stamped t + tau on the IMU clock", so t_imu = t_cam + tau (260901: -2.740 ms). At the measured
// 200 Hz the nearest sample is within 2.5 ms, which at this dataset's fastest rotation
// (0.374 rad/s) is 0.05 deg, so at() takes the nearest sample rather than interpolating, and
// reports the gap it had to accept.
//
// COVARIANCES. The xsens driver leaves all three covariance matrices zero in the bag, and
// RTAB-Map reads an empty/zero covariance as "this field is not set", so they must be supplied
// here: the rate from the rig json's gyro_static_std_rad_s, the orientation from its
// gyro_vs_kiss.err_deg_median, and the accelerometer from a still window of this bag.
//
// The reported angular velocity is NOT bias-corrected. The rig json's gyro_bias_rad_s is
// 1.4e-3 rad/s at most, against a typical rate of 8.7e-2 rad/s here, and RTAB-Map uses the rate
// only for prediction/deskewing — the orientation it constrains poses with comes from the device,
// which does its own bias handling. (gyro_aid.h must subtract the bias because it integrates.)
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <dirent.h>
#include <nlohmann/json.hpp>
#include <opencv2/core/core.hpp>

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/Transform.h>

#include "raw_session.h"

struct ImuSource {
  // Samples as recorded: IMU clock, IMU frame.
  std::vector<long long> t_ns;
  std::vector<Eigen::Quaterniond> q;    // device orientation, qx qy qz qw on the wire
  std::vector<Eigen::Vector3d> w;       // angular velocity [rad/s], bias NOT removed
  std::vector<Eigen::Vector3d> a;       // linear acceleration [m/s^2], gravity included

  std::string db3, topic, rig;
  long long tau_ns = 0;                 // t_imu = t_cam + tau_ns
  double rate_hz = 0;
  rtabmap::Transform local;             // T_base_imu, handed to every rtabmap::IMU
  Eigen::Matrix3d R_cam_imu = Eigen::Matrix3d::Identity();
  cv::Mat cov_q, cov_w, cov_a;          // 3x3 CV_64F, never empty (see note above)

  // Diagnostic: rotate the reported orientation about the reference frame's vertical axis before
  // handing it to RTAB-Map. Whatever the device's reference is (ENU or NED), its vertical is z,
  // so a rotation about z leaves the gravity direction in the body frame EXACTLY unchanged and
  // moves only heading. A constraint that uses gravity alone therefore cannot notice this, and a
  // run with it set must come out bit-identical; any change proves the constraint also pins yaw.
  double yaw_offset_rad = 0;

  // 4 samples at 200 Hz. Wider than any gap this recording has; a real hole should be reported,
  // not papered over with a stale orientation.
  long long max_gap_ns = 20000000;
  long long found = 0, missing = 0, max_gap_seen_ns = 0;

  // ---------------------------------------------------------------- loading
  static std::string one_db3(const std::string& dir) {
    std::vector<std::string> cands;
    if (DIR* d = opendir(dir.c_str())) {
      while (dirent* e = readdir(d)) {
        std::string n = e->d_name;
        if (n.size() > 4 && n.compare(n.size() - 4, 4, ".db3") == 0) cands.push_back(n);
      }
      closedir(d);
    }
    std::sort(cands.begin(), cands.end());
    if (cands.size() != 1) throw std::runtime_error("expected exactly one .db3 in " + dir);
    return dir + "/" + cands[0];
  }

  // sensor_msgs/Imu: header{stamp{sec,nsec}, frame_id}, orientation[4], orientation_cov[9],
  // angular_velocity[3], angular_velocity_cov[9], linear_acceleration[3], linear_acceleration_cov[9].
  // CdrReader::get<double>() aligns to 8 bytes on its own, which is what the padding after the
  // frame_id string needs.
  static bool parse(const std::vector<uint8_t>& blob, long long& t, Eigen::Quaterniond& qq,
                    Eigen::Vector3d& ww, Eigen::Vector3d& aa) {
    if (blob.size() < 300) return false;
    CdrReader r(blob);
    int32_t sec = r.get<int32_t>();
    uint32_t nsec = r.get<uint32_t>();
    r.str();                                                // frame_id
    const double qx = r.get<double>(), qy = r.get<double>(), qz = r.get<double>(), qw = r.get<double>();
    for (int i = 0; i < 9; ++i) r.get<double>();            // orientation_covariance
    ww.x() = r.get<double>(); ww.y() = r.get<double>(); ww.z() = r.get<double>();
    for (int i = 0; i < 9; ++i) r.get<double>();            // angular_velocity_covariance
    aa.x() = r.get<double>(); aa.y() = r.get<double>(); aa.z() = r.get<double>();
    t = (long long)sec * 1000000000LL + nsec;
    qq = Eigen::Quaterniond(qw, qx, qy, qz);                // Eigen takes w first
    return true;
  }

  void load(const std::string& imu_dir, const std::string& rig_json, const std::string& topic_in) {
    topic = topic_in;
    rig = rig_json;
    db3 = one_db3(imu_dir);
    if (!has_topic(db3, topic.c_str())) throw std::runtime_error(db3 + " has no topic " + topic);

    auto index = header_stamps(db3, topic.c_str());         // works for any Header-first message
    Bag bag(db3);
    Blob b;
    t_ns.reserve(index.size());
    q.reserve(index.size());
    w.reserve(index.size());
    a.reserve(index.size());
    for (const auto& it : index) {
      if (!bag.by_id(it.first, b)) continue;
      long long t;
      Eigen::Quaterniond qq;
      Eigen::Vector3d ww, aa;
      if (!parse(b.d, t, qq, ww, aa)) continue;
      t_ns.push_back(t);
      q.push_back(qq);
      w.push_back(ww);
      a.push_back(aa);
    }
    if (t_ns.size() < 2) throw std::runtime_error("no usable IMU samples in " + db3);
    rate_hz = double(t_ns.size() - 1) / (double(t_ns.back() - t_ns.front()) * 1e-9);

    nlohmann::json j;
    { std::ifstream f(rig_json);
      if (!f) throw std::runtime_error("cannot read rig " + rig_json);
      f >> j; }
    const auto& R = j.at("R_cam_imu");
    for (int i = 0; i < 3; ++i)
      for (int k = 0; k < 3; ++k) R_cam_imu(i, k) = R[i][k].get<double>();
    tau_ns = (long long)llround(j.at("time").at("imu_minus_cam_s").get<double>() * 1e9);

    // T_base_imu = O * T_optical_imu, O = T_base_optical (see the FRAMES note).
    Eigen::Matrix4d T_oi = Eigen::Matrix4d::Identity();
    T_oi.block<3, 3>(0, 0) = R_cam_imu;                     // lever arm deliberately zero
    local = rtabmap::CameraModel::opticalRotation() * rtabmap::Transform::fromEigen4d(T_oi);

    cov_w = diag3(sq(vec3(j, "gyro_static_std_rad_s", 0.006)));
    const double sig_q = j.contains("gyro_vs_kiss")
                             ? j["gyro_vs_kiss"].value("err_deg_median", 1.25) * M_PI / 180.0
                             : 1.25 * M_PI / 180.0;
    cov_q = diag3(Eigen::Vector3d::Constant(sig_q * sig_q));
    cov_a = diag3(sq(still_accel_std()));
  }

  // ---------------------------------------------------------------- per frame
  rtabmap::IMU at(long long t_cam_ns) {
    if (t_ns.empty()) { ++missing; return rtabmap::IMU(); }
    const long long t = t_cam_ns + tau_ns;
    size_t i = std::lower_bound(t_ns.begin(), t_ns.end(), t) - t_ns.begin();
    if (i == t_ns.size()) i = t_ns.size() - 1;
    else if (i > 0 && (t - t_ns[i - 1]) < (t_ns[i] - t)) --i;
    const long long gap = std::llabs(t - t_ns[i]);
    if (gap > max_gap_ns) { ++missing; return rtabmap::IMU(); }
    ++found;
    max_gap_seen_ns = std::max(max_gap_seen_ns, gap);
    Eigen::Quaterniond qi = q[i];
    if (yaw_offset_rad != 0.0)
      qi = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_offset_rad, Eigen::Vector3d::UnitZ())) * qi;
    return rtabmap::IMU(cv::Vec4d(qi.x(), qi.y(), qi.z(), qi.w()), cov_q,
                        cv::Vec3d(w[i].x(), w[i].y(), w[i].z()), cov_w,
                        cv::Vec3d(a[i].x(), a[i].y(), a[i].z()), cov_a, local);
  }

  // ---------------------------------------------------------------- self-check
  // Mean acceleration over the stillest tenth of the recording, in the IMU frame and in the base
  // frame. The base-frame value must read about (0, 0, +9.8) — reaction to gravity, base is z-up —
  // which tests R_cam_imu, the optical->base step and the payload parsing all at once.
  std::string gravity_report() const {
    std::vector<size_t> idx(w.size());
    std::iota(idx.begin(), idx.end(), 0);
    const size_t n = std::max<size_t>(1, idx.size() / 10);
    std::partial_sort(idx.begin(), idx.begin() + n, idx.end(),
                      [&](size_t x, size_t y) { return w[x].norm() < w[y].norm(); });
    Eigen::Vector3d m = Eigen::Vector3d::Zero();
    for (size_t k = 0; k < n; ++k) m += a[idx[k]];
    m /= double(n);
    const Eigen::Matrix3d R = local.toEigen4d().block<3, 3>(0, 0);
    const Eigen::Vector3d mb = R * m;
    char buf[320];
    snprintf(buf, sizeof(buf),
             "gravity over %zu stillest samples: imu frame (%.3f %.3f %.3f) |%.3f| -> base frame "
             "(%.3f %.3f %.3f); base must be about (0 0 +9.8)",
             n, m.x(), m.y(), m.z(), m.norm(), mb.x(), mb.y(), mb.z());
    return buf;
  }

 private:
  static cv::Mat diag3(const Eigen::Vector3d& v) {
    cv::Mat m = cv::Mat::zeros(3, 3, CV_64FC1);
    for (int i = 0; i < 3; ++i) m.at<double>(i, i) = v[i];
    return m;
  }
  static Eigen::Vector3d sq(const Eigen::Vector3d& v) { return v.cwiseProduct(v); }
  static Eigen::Vector3d vec3(const nlohmann::json& j, const char* key, double fallback) {
    Eigen::Vector3d v = Eigen::Vector3d::Constant(fallback);
    if (j.contains(key) && j[key].size() == 3)
      for (int i = 0; i < 3; ++i) v[i] = j[key][i].get<double>();
    return v;
  }
  // Standard deviation per axis over the stillest tenth: the accelerometer noise the rig json
  // does not record.
  Eigen::Vector3d still_accel_std() const {
    std::vector<size_t> idx(w.size());
    std::iota(idx.begin(), idx.end(), 0);
    const size_t n = std::max<size_t>(2, idx.size() / 10);
    std::partial_sort(idx.begin(), idx.begin() + n, idx.end(),
                      [&](size_t x, size_t y) { return w[x].norm() < w[y].norm(); });
    Eigen::Vector3d m = Eigen::Vector3d::Zero();
    for (size_t k = 0; k < n; ++k) m += a[idx[k]];
    m /= double(n);
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    for (size_t k = 0; k < n; ++k) v += (a[idx[k]] - m).cwiseProduct(a[idx[k]] - m);
    return (v / double(n - 1)).cwiseSqrt();
  }
};
