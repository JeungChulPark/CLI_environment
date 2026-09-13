// Live ORB-SLAM3 RGB-D on macOS, watched in the browser (see live_common.h).
//
//   live_orbslam VOCAB SETTINGS BAG.db3 OUTDIR [common flags]
//
// By default tracking starts once a browser has connected, and the viewer stays up after the bag
// ends until Ctrl+C. OUTDIR receives the files rt_harness writes plus the stream recording, and the
// keyframe anchoring needed to attach other estimates (e.g. SAM-6D object poses) to the map:
//
//   FrameKeyframeRef.csv      per frame: reference keyframe id, T_kf_cam, and the live T_w_cam
//   KeyFramePosesFinal.csv    per referenced keyframe: final T_w_kf (culled ones resolved via the tree)
//   KeyFramePoseSnapshots.csv T_w_kf of every referenced keyframe, once a second while running
//   MapEvents.csv             frames where the map was corrected (loop closure, merge, global BA) or restarted
//
// Poses are tx ty tz qx qy qz qw, camera frame x right / y down / z forward, world = SaveTrajectoryTUM's.
// T_w_kf(final) * T_kf_cam reproduces CameraTrajectory.txt; T_w_kf(snapshot at time t) * T_kf_cam
// is what an online system knew at t. See scripts/keyframe_anchor.py.

#include <System.h>

#include "live_common.h"

static void put_se3(std::ostream& o, const Sophus::SE3f& T) {
  const Eigen::Vector3f t = T.translation();
  const Eigen::Quaternionf q = T.unit_quaternion();
  o << t.x() << ',' << t.y() << ',' << t.z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
}

class OrbBackend : public SlamBackend {
 public:
  OrbBackend(const std::string& vocab, const std::string& settings, const std::string& out)
      : slam_(vocab, settings, ORB_SLAM3::System::RGBD, false) {
    snapshots_.open(out + "/KeyFramePoseSnapshots.csv");
    snapshots_ << "snapshot,frame,stamp,kf_id,culled,w_kf_tx,w_kf_ty,w_kf_tz,w_kf_qx,w_kf_qy,w_kf_qz,w_kf_qw\n"
               << std::fixed << std::setprecision(6);
  }

  TrackResult track(const Frame& f) override {
    const Sophus::SE3f Tcw = slam_.TrackRGBD(f.bgr, f.depth, f.stamp);
    TrackResult r;
    r.state = slam_.GetTrackingState();
    r.has_pose = !Tcw.matrix().isZero(0);
    if (r.has_pose) r.Twc = Tcw.inverse().matrix();
    cur_ = {f.idx, f.stamp, r.state, r.has_pose, Tcw};
    return r;
  }

  void track_time(double ms) override {
#ifdef REGISTER_TIMES
    slam_.InsertTrackTime(ms);
#endif
  }

  void observations(std::vector<cv::Point2f>& kp, std::vector<float>& xyz) override {
    const auto kps = slam_.GetTrackedKeyPointsUn();
    const auto mps = slam_.GetTrackedMapPoints();
    for (size_t i = 0; i < std::min(kps.size(), mps.size()); i++) {
      if (!mps[i] || mps[i]->isBad()) continue;
      kp.push_back(kps[i].pt);
      const Eigen::Vector3f p = mps[i]->GetWorldPos();
      xyz.insert(xyz.end(), {p.x(), p.y(), p.z()});
    }

    // keyframe anchoring for this frame (tracker thread, right after TrackRGBD)
    RefRow row;
    row.frame = cur_;
    row.has_ref = slam_.GetLastFrameReference(row.kf_id, row.Tkf_c);
    refs_.push_back(row);
    last_frame_ = cur_.idx;
    last_stamp_ = cur_.stamp;
    if (slam_.MapChanged()) events_.push_back({cur_.idx, cur_.stamp, "map_corrected"});
    // back to NO_IMAGES / NOT_INITIALIZED after tracking: ORB-SLAM3 started a new map
    if (cur_.state <= 1 && tracked_before_) events_.push_back({cur_.idx, cur_.stamp, "new_map"});
    tracked_before_ = cur_.state == 2 ? true : (cur_.state <= 1 ? false : tracked_before_);
  }

  void map(std::vector<float>& xyz, std::vector<uint8_t>& rgb) override {
    xyz.clear();
    rgb.clear();
    for (auto& p : slam_.GetAllMapPointsWorld()) xyz.insert(xyz.end(), {p.x(), p.y(), p.z()});
    snapshot();
  }

  void finish(const std::string& out) override {
    slam_.Shutdown();
    slam_.SaveTrajectoryTUM(out + "/CameraTrajectory.txt");
    slam_.SaveKeyFrameTrajectoryTUM(out + "/KeyFrameTrajectory.txt");

    std::ofstream fr(out + "/FrameKeyframeRef.csv");
    fr << "frame,stamp,state,lost,kf_id,kf_cam_tx,kf_cam_ty,kf_cam_tz,kf_cam_qx,kf_cam_qy,kf_cam_qz,kf_cam_qw,"
          "live_w_cam_tx,live_w_cam_ty,live_w_cam_tz,live_w_cam_qx,live_w_cam_qy,live_w_cam_qz,live_w_cam_qw\n"
       << std::fixed << std::setprecision(6);
    for (const auto& r : refs_) {
      fr << r.frame.idx << ',' << r.frame.stamp << ',' << r.frame.state << ',' << (r.has_ref ? 0 : 1) << ',';
      if (r.kf_id != kNoKF) { fr << r.kf_id << ','; put_se3(fr, r.Tkf_c); } else fr << ",,,,,,,";
      fr << ',';
      if (r.frame.has_pose) put_se3(fr, r.frame.Tcw.inverse()); else fr << ",,,,,,";
      fr << '\n';
    }

    std::ofstream kf(out + "/KeyFramePosesFinal.csv");
    kf << "kf_id,stamp,culled,w_kf_tx,w_kf_ty,w_kf_tz,w_kf_qx,w_kf_qy,w_kf_qz,w_kf_qw\n" << std::fixed << std::setprecision(6);
    for (unsigned long id : slam_.GetReferencedKeyFrameIds()) {
      Sophus::SE3f T;
      double stamp;
      bool culled;
      if (!slam_.GetKeyFramePoseWorld(id, T, stamp, culled)) continue;
      kf << id << ',' << stamp << ',' << (culled ? 1 : 0) << ',';
      put_se3(kf, T);
      kf << '\n';
    }

    std::ofstream ev(out + "/MapEvents.csv");
    ev << "frame,stamp,event\n" << std::fixed << std::setprecision(6);
    for (const auto& e : events_) ev << e.frame << ',' << e.stamp << ',' << e.what << '\n';
  }

  std::string summary() override {
    long corrected = std::count_if(events_.begin(), events_.end(), [](const Event& e) { return e.what == "map_corrected"; });
    std::ostringstream o;
    o << "keyframes_referenced " << slam_.GetReferencedKeyFrameIds().size() << "\nmap_corrections " << corrected
      << "\nnew_maps " << events_.size() - corrected << "\nkeyframe_snapshots " << snapshot_n_ << "\n";
    return o.str();
  }

 private:
  static constexpr unsigned long kNoKF = ~0UL;
  struct FrameInfo { int idx = 0; double stamp = 0; int state = 0; bool has_pose = false; Sophus::SE3f Tcw; };
  struct RefRow { FrameInfo frame; bool has_ref = false; unsigned long kf_id = kNoKF; Sophus::SE3f Tkf_c; };
  struct Event { int frame; double stamp; std::string what; };

  // publisher thread: the current pose of every keyframe referenced so far
  void snapshot() {
    const int frame = last_frame_;
    const double stamp = last_stamp_;
    for (unsigned long id : slam_.GetReferencedKeyFrameIds()) {
      Sophus::SE3f T;
      double kst;
      bool culled;
      if (!slam_.GetKeyFramePoseWorld(id, T, kst, culled)) continue;
      snapshots_ << snapshot_n_ << ',' << frame << ',' << stamp << ',' << id << ',' << (culled ? 1 : 0) << ',';
      put_se3(snapshots_, T);
      snapshots_ << '\n';
    }
    snapshots_.flush();
    snapshot_n_++;
  }

  ORB_SLAM3::System slam_;
  FrameInfo cur_;
  std::vector<RefRow> refs_;
  std::vector<Event> events_;
  bool tracked_before_ = false;
  std::atomic<int> last_frame_{-1};
  std::atomic<double> last_stamp_{0};
  std::ofstream snapshots_;
  long snapshot_n_ = 0;
};

int main(int argc, char** argv) {
  const LiveOptions opt = parse_live_options(argc, argv);
  if (opt.rest.size() != 4) {
    std::cerr << "usage: live_orbslam VOCAB SETTINGS BAG.db3 OUTDIR " << LIVE_COMMON_FLAGS << "\n";
    return 1;
  }
  const std::string vocab = opt.rest[0], settings = opt.rest[1], bag = opt.rest[2], out = opt.rest[3];
  return run_live(opt, bag, out, "ORB-SLAM3", settings.substr(settings.find_last_of('/') + 1), [&](const BagCamera&) {
    return std::unique_ptr<SlamBackend>(new OrbBackend(vocab, settings, out));
  });
}
