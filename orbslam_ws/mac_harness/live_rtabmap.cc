// Live RTAB-Map RGB-D on macOS, watched in the browser (see live_common.h).
//
//   live_rtabmap BAG.db3 OUTDIR [--param Key=Value ...] [common flags]
//
// Laid out like the ROS graph the 30 FPS study profiled: visual odometry (Odometry::process) runs
// on every frame and is what the harness times; the RTAB-Map core (memory, loop closure, graph
// optimisation) runs on its own thread and takes the latest odometry output at
// Rtabmap/DetectionRate, as the rtabmap node does. Parameters are RTAB-Map's defaults, as in that
// study (F2M odometry, GFTT/BRIEF), unless overridden with --param.
//
// The map shown is the RGB point cloud of every graph node, placed at its optimised pose, so loop
// closures visibly pull it into place. OUTDIR also receives rtabmap.db.
//
// Keyframe anchoring, in the same files and columns live_orbslam writes (read with
// scripts/keyframe_anchor.py). RTAB-Map's keyframes are its graph nodes:
//   FrameKeyframeRef.csv      per frame: the newest node the mapper had made by then, T_node_cam from
//                             odometry, and the live T_w_cam (ref_after_frame=1: frames tracked before
//                             the first node existed, anchored to it afterwards)
//   KeyFramePosesFinal.csv    per node: optimised T_w_node when the run ended (culled=1: node no longer
//                             in the graph, placed from its nearest surviving node through odometry)
//   KeyFramePoseSnapshots.csv every node's T_w_node after each map update (1 Hz)
//   MapEvents.csv             loop closures / proximity detections (map_corrected), odometry lost
//   CameraTrajectory.txt      final: T_w_node(final) * T_node_cam, as ORB-SLAM3's file is final
//   CameraTrajectoryLive.txt  live: the pose shown while tracking
// Poses are camera x right / y down / z forward; world = the first camera's optical frame.

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>

#include <Eigen/Geometry>

#include "live_common.h"

using rtabmap::Transform;

static void put_tf(std::ostream& o, const Transform& T, char sep = ',') {
  const Eigen::Quaternionf q = T.getQuaternionf();
  o << T.x() << sep << T.y() << sep << T.z() << sep << q.x() << sep << q.y() << sep << q.z() << sep << q.w();
}

class RtabBackend : public SlamBackend {
 public:
  RtabBackend(const BagCamera& cam, const rtabmap::ParametersMap& params, const std::string& out) : params_(params) {
    const Intr& ic = cam.color_intrinsics();
    // RTAB-Map works in a base frame (x forward, z up); the camera sits on it via the optical rotation
    model_ = rtabmap::CameraModel(ic.fx, ic.fy, ic.cx, ic.cy, O_, 0, cv::Size(ic.w, ic.h));
    odom_.reset(rtabmap::Odometry::create(params_));
    rtab_.init(params_, out + "/rtabmap.db");
    float rate = rtabmap::Parameters::defaultRtabmapDetectionRate();
    rtabmap::Parameters::parse(params_, rtabmap::Parameters::kRtabmapDetectionRate(), rate);
    period_ = rate > 0 ? 1.0 / rate : 0.0;
    snapshots_.open(out + "/KeyFramePoseSnapshots.csv");
    snapshots_ << "snapshot,frame,stamp,kf_id,culled,w_kf_tx,w_kf_ty,w_kf_tz,w_kf_qx,w_kf_qy,w_kf_qz,w_kf_qw\n"
               << std::fixed << std::setprecision(6);
    mapper_ = std::thread([this] { map_loop(); });
  }

  TrackResult track(const Frame& f) override {
    data_ = rtabmap::SensorData(f.bgr, f.depth, model_, ++id_, f.stamp);
    info_ = rtabmap::OdometryInfo();
    odom_pose_ = odom_->process(data_, &info_);
    cur_idx_ = f.idx;
    cur_stamp_ = f.stamp;
    TrackResult r;
    if (odom_pose_.isNull()) {
      r.state = 4;  // odometry lost; RTAB-Map's default Odom/ResetCountdown 0 waits for a reset
      lost_++;
      return r;
    }
    r.state = 2;
    r.has_pose = true;
    r.Twc = to_view(correction() * odom_pose_ * O_);
    return r;
  }

  void observations(std::vector<cv::Point2f>& kp, std::vector<float>& xyz) override {
    RefRow row;
    row.frame = cur_idx_;
    row.stamp = cur_stamp_;
    row.T_odom_base = odom_pose_;
    last_frame_ = cur_idx_;
    last_stamp_ = cur_stamp_;
    if (odom_pose_.isNull()) {
      if (!was_lost_) add_event(cur_idx_, cur_stamp_, "odom_lost");
      was_lost_ = true;
      refs_.push_back(row);
      return;
    }
    was_lost_ = false;
    Transform corr;
    {  // hand the frame to the mapper (latest wins, as a 1 Hz rtabmap node would see it) and read its newest node
      std::lock_guard<std::mutex> l(m_);
      pending_ = std::make_shared<rtabmap::SensorData>(data_);
      pending_pose_ = odom_pose_;
      pending_frame_ = cur_idx_;
      pending_stamp_ = cur_stamp_;
      corr = correction_;
      if (latest_node_ > 0) {
        row.node = latest_node_;
        row.T_node_cam = cam_of(nodes_.at(latest_node_).T_odom_base).inverse() * cam_of(odom_pose_);
      }
    }
    cv_.notify_one();
    row.live = view_cam(corr * odom_pose_);
    refs_.push_back(row);

    features_sum_ += info_.features;
    inliers_sum_ += info_.reg.inliers;
    odom_frames_++;
    const Eigen::Matrix4f M = (O_.inverse() * corr).toEigen4f();
    for (int id : info_.reg.inliersIDs) {
      const auto w = info_.words.find(id);
      const auto p = info_.localMap.find(id);
      if (w == info_.words.end() || p == info_.localMap.end()) continue;
      kp.push_back(w->second.pt);
      const Eigen::Vector4f v = M * Eigen::Vector4f(p->second.x, p->second.y, p->second.z, 1);
      xyz.insert(xyz.end(), {v.x(), v.y(), v.z()});
    }
  }

  void map(std::vector<float>& xyz, std::vector<uint8_t>& rgb) override {
    std::lock_guard<std::mutex> l(m_);
    xyz = map_xyz_;
    rgb = map_rgb_;
  }

  void finish(const std::string& out) override {
    {
      std::lock_guard<std::mutex> l(m_);
      stop_ = true;
    }
    cv_.notify_all();
    mapper_.join();
    const std::map<int, Transform> poses = rtab_.getLocalOptimizedPoses();
    snapshot(poses);
    rtab_.close(true);

    // final node poses (camera frame, viewer world); nodes gone from the graph ride on the nearest survivor
    std::map<int, Transform> final_node;
    std::map<int, int> culled;
    for (const auto& n : nodes_) {
      const auto p = poses.find(n.first);
      if (p != poses.end()) {
        final_node[n.first] = view_cam(p->second);
        continue;
      }
      int best = 0;
      for (const auto& s : nodes_)
        if (poses.count(s.first) && (!best || std::abs(s.second.frame - n.second.frame) < std::abs(nodes_.at(best).frame - n.second.frame)))
          best = s.first;
      if (!best) continue;
      // T_w_node = T_w_best * (T_odom_best_cam^-1 * T_odom_node_cam)
      final_node[n.first] = view_cam(poses.at(best)) * (cam_of(nodes_.at(best).T_odom_base).inverse() * cam_of(n.second.T_odom_base));
      culled[n.first] = 1;
    }
    std::ofstream kf(out + "/KeyFramePosesFinal.csv");
    kf << "kf_id,stamp,culled,w_kf_tx,w_kf_ty,w_kf_tz,w_kf_qx,w_kf_qy,w_kf_qz,w_kf_qw\n" << std::fixed << std::setprecision(6);
    for (const auto& n : final_node) {
      kf << n.first << ',' << nodes_.at(n.first).stamp << ',' << culled.count(n.first) << ',';
      put_tf(kf, n.second);
      kf << '\n';
    }

    // frames tracked before the first node existed are anchored to it after the fact
    const int first = nodes_.empty() ? 0 : nodes_.begin()->first;
    std::ofstream fr(out + "/FrameKeyframeRef.csv");
    std::ofstream tf(out + "/CameraTrajectory.txt"), tl(out + "/CameraTrajectoryLive.txt");
    fr << "frame,stamp,state,lost,kf_id,kf_cam_tx,kf_cam_ty,kf_cam_tz,kf_cam_qx,kf_cam_qy,kf_cam_qz,kf_cam_qw,"
          "live_w_cam_tx,live_w_cam_ty,live_w_cam_tz,live_w_cam_qx,live_w_cam_qy,live_w_cam_qz,live_w_cam_qw,ref_after_frame\n";
    for (auto* s : {&fr, &tf, &tl}) *s << std::fixed << std::setprecision(6);
    for (auto& r : refs_) {
      const bool lost = r.T_odom_base.isNull();
      bool after = false;
      if (!lost && r.node == 0 && first) {
        r.node = first;
        r.T_node_cam = cam_of(nodes_.at(first).T_odom_base).inverse() * cam_of(r.T_odom_base);
        after = true;
      }
      fr << r.frame << ',' << r.stamp << ',' << (lost ? 4 : 2) << ',' << (lost ? 1 : 0) << ',';
      if (!lost && r.node) { fr << r.node << ','; put_tf(fr, r.T_node_cam); } else fr << ",,,,,,,";
      fr << ',';
      if (!lost) put_tf(fr, r.live); else fr << ",,,,,,";
      fr << ',' << (after ? 1 : 0) << '\n';
      if (lost) continue;
      tl << r.stamp << ' ';
      put_tf(tl, r.live, ' ');
      tl << '\n';
      const auto n = final_node.find(r.node);
      if (n == final_node.end()) continue;
      tf << r.stamp << ' ';
      put_tf(tf, n->second * r.T_node_cam, ' ');
      tf << '\n';
    }

    std::ofstream ev(out + "/MapEvents.csv");
    ev << "frame,stamp,event\n" << std::fixed << std::setprecision(6);
    std::lock_guard<std::mutex> l(m_);
    for (const auto& e : events_) ev << e.frame << ',' << e.stamp << ',' << e.what << '\n';
  }

  std::string summary() override {
    std::vector<double> v = map_ms_;
    std::sort(v.begin(), v.end());
    double mean = 0;
    for (double x : v) mean += x;
    long corrected = std::count_if(events_.begin(), events_.end(), [](const Event& e) { return e.what == "map_corrected"; });
    std::ostringstream o;
    o << std::fixed << std::setprecision(2) << "odometry_lost_frames " << lost_
      << "\nodometry_features_mean " << features_sum_ / std::max(1L, odom_frames_)
      << "\nodometry_inliers_mean " << inliers_sum_ / std::max(1L, odom_frames_)
      << "\nrtabmap_updates " << v.size() << "\nrtabmap_update_ms mean " << (v.empty() ? 0 : mean / v.size())
      << " p99 " << (v.empty() ? 0 : v[(size_t)(0.99 * (v.size() - 1))]) << " max " << (v.empty() ? 0 : v.back())
      << "\ngraph_nodes " << nodes_in_graph_ << "\nloop_closures " << loops_ << "\nmap_corrections " << corrected
      << "\nkeyframes_referenced " << nodes_.size() << "\nkeyframe_snapshots " << snapshot_n_
      << "\nmap_cloud_points " << map_xyz_.size() / 3 << "\n";
    return o.str();
  }

 private:
  struct Node { int frame; double stamp; Transform T_odom_base; };
  struct RefRow { int frame = 0; double stamp = 0; Transform T_odom_base; int node = 0; Transform T_node_cam; Transform live; };
  struct Event { int frame; double stamp; std::string what; };

  Transform correction() {
    std::lock_guard<std::mutex> l(m_);
    return correction_;
  }
  Transform cam_of(const Transform& T_base) const { return T_base * O_; }
  // map-frame base pose -> camera pose in the viewer world (the first camera's optical frame)
  Transform view_cam(const Transform& T_base) const { return O_.inverse() * T_base * O_; }
  Eigen::Matrix4f to_view(const Transform& T) const { return (O_.inverse() * T).toEigen4f(); }
  void add_event(int frame, double stamp, const char* what) {
    std::lock_guard<std::mutex> l(m_);
    events_.push_back({frame, stamp, what});
  }

  void snapshot(const std::map<int, Transform>& poses) {
    const int frame = last_frame_;
    const double stamp = last_stamp_;
    for (const auto& p : poses) {
      if (p.first <= 0) continue;
      snapshots_ << snapshot_n_ << ',' << frame << ',' << stamp << ',' << p.first << ",0,";
      put_tf(snapshots_, view_cam(p.second));
      snapshots_ << '\n';
    }
    snapshots_.flush();
    snapshot_n_++;
  }

  void map_loop() {
    pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, 0);
    std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;  // per node, in the node's base frame
    auto next = Clock::now();
    while (true) {
      std::shared_ptr<rtabmap::SensorData> d;
      Transform pose;
      int frame;
      double stamp;
      {
        std::unique_lock<std::mutex> l(m_);
        cv_.wait(l, [&] { return stop_ || pending_; });
        cv_.wait_until(l, next, [&] { return stop_; });
        if (stop_) return;
        d = std::move(pending_);
        pose = pending_pose_;
        frame = pending_frame_;
        stamp = pending_stamp_;
      }
      next = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(period_));

      auto t0 = Clock::now();
      rtab_.process(*d, pose);
      map_ms_.push_back(ms_between(t0, Clock::now()));
      const auto& st = rtab_.getStatistics();
      if (st.loopClosureId() > 0) loops_++;
      const bool corrected = st.loopClosureId() > 0 || st.proximityDetectionId() > 0;

      const int last = rtab_.getLastLocationId();
      const auto& poses = rtab_.getLocalOptimizedPoses();
      bool new_node = false;
      if (last > 0 && !clouds.count(last) && poses.count(last)) {
        auto c = rtabmap::util3d::removeNaNFromPointCloud(rtabmap::util3d::cloudRGBFromSensorData(*d, 4, 4.0f, 0.3f));
        clouds[last] = c->empty() ? c : rtabmap::util3d::voxelize(c, 0.03f);
        new_node = true;
      }
      snapshot(poses);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr all(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (const auto& np : poses) {
        const auto c = clouds.find(np.first);
        if (c != clouds.end() && !c->second->empty()) *all += *rtabmap::util3d::transformPointCloud(c->second, np.second);
      }
      if (!all->empty()) all = rtabmap::util3d::voxelize(all, 0.04f);
      std::vector<float> xyz;
      std::vector<uint8_t> rgb;
      xyz.reserve(all->size() * 3);
      rgb.reserve(all->size() * 3);
      const Eigen::Matrix4f V = O_.inverse().toEigen4f();
      for (const auto& p : all->points) {
        const Eigen::Vector4f v = V * Eigen::Vector4f(p.x, p.y, p.z, 1);
        xyz.insert(xyz.end(), {v.x(), v.y(), v.z()});
        rgb.insert(rgb.end(), {p.r, p.g, p.b});
      }
      std::lock_guard<std::mutex> l(m_);
      if (new_node) {
        nodes_[last] = {frame, stamp, pose};
        latest_node_ = last;
      }
      if (corrected) events_.push_back({last_frame_.load(), last_stamp_.load(), "map_corrected"});
      correction_ = rtab_.getMapCorrection();
      nodes_in_graph_ = poses.size();
      map_xyz_.swap(xyz);
      map_rgb_.swap(rgb);
    }
  }

  const Transform O_ = rtabmap::CameraModel::opticalRotation();
  rtabmap::ParametersMap params_;
  rtabmap::CameraModel model_;
  std::unique_ptr<rtabmap::Odometry> odom_;
  rtabmap::Rtabmap rtab_;
  double period_ = 1.0;

  // tracker thread
  int id_ = 0, cur_idx_ = 0;
  double cur_stamp_ = 0;
  rtabmap::SensorData data_;
  rtabmap::OdometryInfo info_;
  Transform odom_pose_;
  long lost_ = 0, odom_frames_ = 0;
  double features_sum_ = 0, inliers_sum_ = 0;
  bool was_lost_ = false;
  std::vector<RefRow> refs_;

  // shared with the mapper (under m_ unless atomic)
  std::thread mapper_;
  std::mutex m_;
  std::condition_variable cv_;
  bool stop_ = false;
  std::shared_ptr<rtabmap::SensorData> pending_;
  Transform pending_pose_;
  int pending_frame_ = 0;
  double pending_stamp_ = 0;
  Transform correction_ = Transform::getIdentity();
  std::map<int, Node> nodes_;  // nodes the mapper created, with the odometry pose they were made at
  int latest_node_ = 0;
  std::vector<Event> events_;
  std::vector<float> map_xyz_;
  std::vector<uint8_t> map_rgb_;
  std::vector<double> map_ms_;
  size_t nodes_in_graph_ = 0;
  long loops_ = 0;
  std::atomic<int> last_frame_{-1};
  std::atomic<double> last_stamp_{0};
  std::ofstream snapshots_;  // mapper thread, then finish() once it has stopped
  long snapshot_n_ = 0;
};

int main(int argc, char** argv) {
  LiveOptions opt = parse_live_options(argc, argv);
  rtabmap::ParametersMap params;
  std::vector<std::string> pos;
  std::string overrides;
  for (size_t i = 0; i < opt.rest.size(); i++) {
    if (opt.rest[i] == "--param" && i + 1 < opt.rest.size()) {
      const std::string kv = opt.rest[++i];
      const size_t eq = kv.find('=');
      if (eq == std::string::npos) { std::cerr << "--param expects Key=Value, got " << kv << "\n"; return 1; }
      params[kv.substr(0, eq)] = kv.substr(eq + 1);
      overrides += (overrides.empty() ? " " : ", ") + kv;
    } else {
      pos.push_back(opt.rest[i]);
    }
  }
  if (pos.size() != 2) {
    std::cerr << "usage: live_rtabmap BAG.db3 OUTDIR [--param Key=Value ...] " << LIVE_COMMON_FLAGS << "\n";
    return 1;
  }
  ULogger::setType(ULogger::kTypeConsole);
  ULogger::setLevel(ULogger::kWarning);
  const std::string bag = pos[0], out = pos[1];
  return run_live(opt, bag, out, "RTAB-Map", std::string("RTAB-Map ") + RTABMAP_VERSION + " defaults" + overrides,
                  [&](const BagCamera& cam) { return std::unique_ptr<SlamBackend>(new RtabBackend(cam, params, out)); });
}
