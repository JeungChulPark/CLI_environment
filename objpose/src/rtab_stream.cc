// rtab_stream — RTAB-Map RGB-D backend with the same CLI and live protocol as slam_stream.
//
// Layout follows orbslam_ws/mac_harness/live_rtabmap.cc (the validated Mac run): visual
// odometry (rtabmap::Odometry, RTAB-Map defaults = F2M, GFTT/BRIEF) runs on every frame in the
// tracking thread; the RTAB-Map core (memory, loop closure, graph optimisation) runs on its own
// thread and takes the latest odometry output at Rtabmap/DetectionRate (default 1 Hz), exactly
// as the rtabmap ROS node does. Odometry covariance is left at Rtabmap::process's default, as in
// the harness.
//
// Frames and poses (same convention as ORB-SLAM3 / slam_stream):
//   RTAB-Map works in a base frame (x forward, y left, z up) with the camera mounted on it through
//   O = CameraModel::opticalRotation() (T_base_optical). Every pose leaving this program is
//     T_w_cam = O^-1 * T_map_base * O
//   i.e. camera optical axes (x right, y down, z forward) and world = the first camera's optical
//   frame (odometry starts at identity, so the first T_w_cam is identity).
//   Live pose  : T_map_base = map_correction * T_odom_base   (RTAB-Map's map->odom chain)
//   Node pose  : T_map_base = getLocalOptimizedPoses()[id]
//
// Keyframe equivalents: RTAB-Map graph nodes (signatures). ref_kf = newest node the mapper has
// added by the time this frame is tracked.

#include <atomic>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <signal.h>

#include "raw_session.h"

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Statistics.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/utilite/ULogger.h>

using rtabmap::Transform;
using Clock = std::chrono::steady_clock;

static double ms_since(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

static std::string fmt6(double v) {
  char b[32];
  snprintf(b, sizeof(b), "%.6f", v);
  std::string s(b);
  if (s == "-0.000000") s = "0.000000";
  return s;
}

struct Args {
  std::string session, mode = "offline", out, connect, time_source = "frame";
  int features = 0;
  long long offset_ns = 0;
  double rate = 1.0, pace = 0.0;
  int max_frames = 0;
  std::vector<std::pair<std::string, std::string>> params;
};

static void usage() {
  std::cerr << "rtab_stream --session DIR [--features N] [--offset-ns NS] [--time-source frame|assoc]\n"
               "            [--max-frames N] [--param Key=Value ...] [--settings X --vocab X (ignored)]\n"
               "            --mode offline --out TUM.txt [--pace R] | --mode live --connect HOST:PORT [--rate R]\n";
}

// ---------------------------------------------------------------- backend
class RtabBackend {
 public:
  RtabBackend(const Calib& c, const rtabmap::ParametersMap& params, const std::string& db, bool async)
      : params_(params), async_(async) {
    model_ = rtabmap::CameraModel(c.fxc, c.fyc, c.ppxc, c.ppyc, O_, 0, cv::Size(c.W, c.H));
    odom_.reset(rtabmap::Odometry::create(params_));
    rtab_.init(params_, db);
    float rate = rtabmap::Parameters::defaultRtabmapDetectionRate();
    rtabmap::Parameters::parse(params_, rtabmap::Parameters::kRtabmapDetectionRate(), rate);
    period_s_ = rate > 0 ? 1.0 / rate : 0.0;
    if (async_) mapper_ = std::thread([this] { map_loop(); });
  }
  ~RtabBackend() { stop(); }

  struct Track {
    bool ok = false;
    Transform T_odom_base;     // null when lost
    Transform T_w_cam;         // optical, map frame
    double odom_ms = 0, map_ms = 0;
    bool map_changed = false;
    int ref_kf = 0, kf_map_id = -1, map_id = -1;
    Transform T_w_kf;          // optical
    int features = 0, inliers = 0;
    int map_updates = 0;       // mapper updates completed since last frame
  };

  // Tracking thread. color/depth must be fresh buffers (SensorData shares them with the mapper).
  Track track(const cv::Mat& bgr, const cv::Mat& depth, double stamp_s, long long data_ns) {
    Track r;
    rtabmap::SensorData data(bgr, depth, model_, ++id_, stamp_s);
    rtabmap::OdometryInfo info;
    auto t0 = Clock::now();
    Transform odom = odom_->process(data, &info);
    r.odom_ms = ms_since(t0);
    r.features = info.features;
    r.inliers = info.reg.inliers;
    if (!odom.isNull()) {
      if (async_) {
        std::lock_guard<std::mutex> l(m_);
        pending_ = std::make_shared<rtabmap::SensorData>(data);
        pending_pose_ = odom;
        cv_.notify_one();
      } else if (last_sync_ns_ < 0 || double(data_ns - last_sync_ns_) * 1e-9 >= period_s_) {
        // offline, unpaced: feed the mapper on the data clock so it sees the same 1 Hz cadence
        last_sync_ns_ = data_ns;
        map_update(data, odom);
      }
    }
    std::lock_guard<std::mutex> l(m_);
    r.map_ms = map_ms_accum_;
    r.map_updates = map_updates_since_;
    map_ms_accum_ = 0;
    map_updates_since_ = 0;
    r.map_changed = changed_version_ != changed_seen_;
    changed_seen_ = changed_version_;
    r.map_id = map_id_;
    r.T_odom_base = odom;
    if (!odom.isNull()) {
      r.ok = true;
      r.T_w_cam = view(correction_ * odom);
    }
    if (latest_node_ > 0) {
      auto it = optimized_.find(latest_node_);
      if (it != optimized_.end()) {
        r.ref_kf = latest_node_;
        r.T_w_kf = view(it->second);
        r.kf_map_id = node_map_.count(latest_node_) ? node_map_.at(latest_node_) : map_id_;
      }
    }
    return r;
  }

  // kf_update body pieces from the latest optimised graph (current map only).
  int kf_rows(std::string& kfs, int& map_id) {
    std::lock_guard<std::mutex> l(m_);
    map_id = map_id_;
    int n = 0;
    kfs.reserve(optimized_.size() * 140);
    for (const auto& p : optimized_) {
      if (p.first <= 0) continue;  // negative ids are landmarks
      auto mid = node_map_.find(p.first);
      if (mid != node_map_.end() && mid->second != map_id_) continue;
      Eigen::Matrix4f T = view(p.second).toEigen4f();
      if (n) kfs += ",";
      kfs += "[" + std::to_string(p.first);
      for (int rr = 0; rr < 3; ++rr)
        for (int cc = 0; cc < 4; ++cc) kfs += "," + fmt6(T(rr, cc));
      kfs += "]";
      ++n;
    }
    return n;
  }

  struct NodeInfo { Transform T_odom_base; };
  // Final optimised poses (base frame) + odometry pose each node was created at.
  void finish(std::map<int, Transform>& final_poses, std::map<int, Transform>& node_odom) {
    stop();
    std::lock_guard<std::mutex> l(m_);
    final_poses = rtab_.getLocalOptimizedPoses();
    if (final_poses.empty()) final_poses = optimized_;  // last update added no node
    node_odom = node_odom_;
  }
  void close(bool save) { rtab_.close(save); }

  Transform view(const Transform& T_base) const { return O_.inverse() * T_base * O_; }
  Transform cam_of(const Transform& T_base) const { return T_base * O_; }
  int loops() const { return loops_; }
  int proximities() const { return proximities_; }
  int empty_updates() const { return empty_updates_; }
  size_t nodes() {
    std::lock_guard<std::mutex> l(m_);
    return optimized_.size();
  }
  std::vector<double> map_ms_all() {
    std::lock_guard<std::mutex> l(m_);
    return map_ms_all_;
  }
  int latest_node() {
    std::lock_guard<std::mutex> l(m_);
    return latest_node_;
  }

 private:
  void stop() {
    if (!async_ || !mapper_.joinable()) return;
    {
      std::lock_guard<std::mutex> l(m_);
      stop_ = true;
    }
    cv_.notify_all();
    mapper_.join();
  }

  void map_update(const rtabmap::SensorData& d, const Transform& odom) {
    auto t0 = Clock::now();
    rtab_.process(d, odom);
    double ms = ms_since(t0);
    const rtabmap::Statistics& st = rtab_.getStatistics();
    const bool loop = st.loopClosureId() > 0, prox = st.proximityDetectionId() > 0;
    int last = rtab_.getLastLocationId();
    std::map<int, Transform> poses = rtab_.getLocalOptimizedPoses();
    const rtabmap::Memory* mem = rtab_.getMemory();
    int map_id = (mem && last > 0) ? mem->getMapId(last) : -1;
    std::map<int, int> node_map;
    for (const auto& p : poses)
      if (p.first > 0 && mem) node_map[p.first] = mem->getMapId(p.first);
    std::lock_guard<std::mutex> l(m_);
    if (loop) ++loops_;
    if (prox) ++proximities_;
    if (loop || prox) ++changed_version_;
    if (last > 0 && poses.count(last) && !node_odom_.count(last)) {
      node_odom_[last] = odom;  // node made from this odometry pose
      latest_node_ = last;
    }
    // Rtabmap::process() can leave getLocalOptimizedPoses() empty on an update that adds no node
    // (e.g. rehearsal while barely moving). Keep the last graph and correction rather than
    // reporting an empty map.
    if (!poses.empty()) {
      optimized_.swap(poses);
      node_map_.swap(node_map);
      if (map_id >= 0) map_id_ = map_id;
      correction_ = rtab_.getMapCorrection();
    } else {
      ++empty_updates_;
    }
    map_ms_accum_ += ms;
    ++map_updates_since_;
    map_ms_all_.push_back(ms);
  }

  void map_loop() {
    auto next = Clock::now();
    while (true) {
      std::shared_ptr<rtabmap::SensorData> d;
      Transform pose;
      {
        std::unique_lock<std::mutex> l(m_);
        cv_.wait(l, [&] { return stop_ || pending_; });
        cv_.wait_until(l, next, [&] { return stop_; });
        if (stop_) return;
        d = std::move(pending_);
        pose = pending_pose_;
      }
      next = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(period_s_));
      map_update(*d, pose);
    }
  }

  const Transform O_ = rtabmap::CameraModel::opticalRotation();
  rtabmap::ParametersMap params_;
  bool async_;
  rtabmap::CameraModel model_;
  std::unique_ptr<rtabmap::Odometry> odom_;
  rtabmap::Rtabmap rtab_;
  double period_s_ = 1.0;
  int id_ = 0;
  long long last_sync_ns_ = -1;

  std::thread mapper_;
  std::mutex m_;
  std::condition_variable cv_;
  bool stop_ = false;
  std::shared_ptr<rtabmap::SensorData> pending_;
  Transform pending_pose_;
  Transform correction_ = Transform::getIdentity();
  std::map<int, Transform> optimized_;
  std::map<int, int> node_map_;
  std::map<int, Transform> node_odom_;
  int latest_node_ = 0, map_id_ = -1;
  long changed_version_ = 0, changed_seen_ = 0;
  double map_ms_accum_ = 0;
  int map_updates_since_ = 0;
  std::vector<double> map_ms_all_;
  std::atomic<int> loops_{0}, proximities_{0}, empty_updates_{0};
};

static std::string tf16(const Transform& T) {
  Eigen::Matrix4f M = T.toEigen4f();
  std::string s = "[";
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) s += (r || c ? "," : "") + fmt6(M(r, c));
  return s + "]";
}

static void tum_line(std::ostream& o, long long t_ns, const Transform& T) {
  Eigen::Quaternionf q = T.getQuaternionf();
  char buf[256];
  snprintf(buf, sizeof(buf), "%.9f %.6f %.6f %.6f %.7f %.7f %.7f %.7f\n", double(t_ns) * 1e-9, T.x(), T.y(), T.z(),
           q.x(), q.y(), q.z(), q.w());
  o << buf;
}

int main(int argc, char** argv) {
  signal(SIGPIPE, SIG_IGN);
  Args a;
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    auto val = [&]() -> std::string {
      if (i + 1 >= argc) { usage(); exit(2); }
      return argv[++i];
    };
    if (k == "--session") a.session = val();
    else if (k == "--settings" || k == "--vocab") val();  // ORB-SLAM3 only
    else if (k == "--mode") a.mode = val();
    else if (k == "--out") a.out = val();
    else if (k == "--connect") a.connect = val();
    else if (k == "--features") a.features = std::stoi(val());
    else if (k == "--offset-ns") a.offset_ns = std::stoll(val());
    else if (k == "--time-source") a.time_source = val();
    else if (k == "--rate") a.rate = std::stod(val());
    else if (k == "--pace") a.pace = std::stod(val());
    else if (k == "--max-frames") a.max_frames = std::stoi(val());
    else if (k == "--param") {
      std::string kv = val();
      auto eq = kv.find('=');
      if (eq == std::string::npos) { usage(); return 2; }
      a.params.push_back({kv.substr(0, eq), kv.substr(eq + 1)});
    } else { usage(); return 2; }
  }
  if (a.session.empty() || (a.mode == "offline" && a.out.empty()) || (a.mode == "live" && a.connect.empty()) ||
      (a.mode != "offline" && a.mode != "live")) {
    usage();
    return 2;
  }

  ULogger::setType(ULogger::kTypeConsole);
  ULogger::setLevel(ULogger::kError);  // g2o "no csparse, using PCG" warns on every update

  Session s = load_session(a.session, a.offset_ns, a.time_source);
  if (a.max_frames > 0 && int(s.frames.size()) > a.max_frames) s.frames.resize(a.max_frames);
  const Calib& c = s.calib;
  fprintf(stderr, "[rtab_stream] %s | %zu frames | time-source=%s domain=%s offset_ns=%lld non_monotonic=%lld\n",
          s.db3.c_str(), s.frames.size(), s.time_source.c_str(), s.time_domain.c_str(), a.offset_ns, s.non_monotonic);
  Aligner aligner(c);

  // Parameters: RTAB-Map defaults (as the harness), --features -> Vis/MaxFeatures, then --param.
  rtabmap::ParametersMap params;
  if (a.features > 0) params[rtabmap::Parameters::kVisMaxFeatures()] = std::to_string(a.features);
  for (auto& kv : a.params) params[kv.first] = kv.second;
  rtabmap::ParametersMap effective = rtabmap::Parameters::getDefaultParameters();
  for (auto& kv : params) effective[kv.first] = kv.second;
  static const char* SHOW[] = {"Odom/Strategy", "Vis/FeatureType", "Vis/MaxFeatures", "Vis/CorType", "Vis/MinInliers",
                               "Kp/DetectorStrategy", "Kp/MaxFeatures", "Reg/Strategy", "Rtabmap/DetectionRate",
                               "Rtabmap/TimeThr", "RGBD/LinearUpdate", "RGBD/AngularUpdate", "RGBD/ProximityBySpace",
                               "Optimizer/Strategy", "Mem/STMSize", "Mem/IncrementalMemory", "Odom/ResetCountdown",
                               "OdomF2M/MaxSize"};
  json shown = json::object();
  for (const char* k : SHOW)
    if (effective.count(k)) shown[k] = effective[k];
  fprintf(stderr, "[rtab_stream] RTAB-Map %s params: %s | overrides:", RTABMAP_VERSION, shown.dump().c_str());
  for (auto& kv : params) fprintf(stderr, " %s=%s", kv.first.c_str(), kv.second.c_str());
  fprintf(stderr, "\n");

  const bool async = !(a.mode == "offline" && a.pace <= 0);
  std::string db = a.mode == "offline" ? a.out + ".rtabmap.db" : "";
  if (!db.empty()) unlink(db.c_str());
  RtabBackend be(c, params, db, async);

  Bag bag(s.db3);
  Blob cb, dbl;
  struct FrameOut {
    long long t_ns; bool ok; Transform T_odom_base, live; int node; double odom_ms, map_ms; bool map_changed; int map_id;
    int features, inliers;
  };
  std::vector<FrameOut> frames_out;
  std::vector<double> odom_ms, map_ms_frames, track_ms;
  int processed = 0, ok_count = 0, dropped = 0;

  auto process = [&](const FrameRef& f, RtabBackend::Track& r, double& prep_ms) -> bool {
    auto t0 = Clock::now();
    if (!bag.by_id(f.color_id, cb) || !bag.by_id(f.depth_id, dbl)) return false;
    ImageView ci = read_image_msg(cb.d), di = read_image_msg(dbl.d);
    if (int(ci.w) != c.W || int(ci.h) != c.H || int(di.w) != c.W || int(di.h) != c.H ||
        ci.len != ci.w * ci.h * 3 || di.len != di.w * di.h * 2)
      throw std::runtime_error("unexpected image geometry/size");
    cv::Mat color;
    if (ci.enc == "bgr8") {
      color = cv::Mat(c.H, c.W, CV_8UC3, (void*)ci.data).clone();
    } else if (ci.enc == "rgb8") {
      cv::Mat rgb(c.H, c.W, CV_8UC3, (void*)ci.data);
      color.create(c.H, c.W, CV_8UC3);
      for (int v = 0; v < c.H; ++v)
        for (int u = 0; u < c.W; ++u) {
          const auto& p = rgb.at<cv::Vec3b>(v, u);
          color.at<cv::Vec3b>(v, u) = cv::Vec3b(p[2], p[1], p[0]);
        }
    } else {
      throw std::runtime_error("unsupported colour encoding " + ci.enc);
    }
    if (di.enc != "mono16" && di.enc != "16UC1") throw std::runtime_error("unsupported depth encoding " + di.enc);
    std::vector<uint16_t> raw(c.W * c.H);
    std::memcpy(raw.data(), di.data, raw.size() * 2);
    cv::Mat depth;  // fresh buffer per frame: the mapper keeps a reference to it
    depth_to_color(s, aligner, raw.data(), depth);
    prep_ms = ms_since(t0);
    r = be.track(color, depth, double(f.t_ns) * 1e-9, f.t_ns);
    return true;
  };

  auto record = [&](const FrameRef& f, const RtabBackend::Track& r) {
    ++processed;
    if (r.ok) ++ok_count;
    odom_ms.push_back(r.odom_ms);
    track_ms.push_back(r.odom_ms + r.map_ms);
    if (r.map_updates) map_ms_frames.push_back(r.map_ms);
    frames_out.push_back({f.t_ns, r.ok, r.T_odom_base, r.T_w_cam, r.ref_kf, r.odom_ms, r.map_ms, r.map_changed,
                          r.map_id, r.features, r.inliers});
  };

  if (a.mode == "offline") {
    auto wall0 = Clock::now();
    std::ofstream live(a.out);
    for (size_t i = 0; i < s.frames.size(); ++i) {
      if (a.pace > 0) {
        double due_s = double(s.frames[i].t_ns - s.frames.front().t_ns) * 1e-9 / a.pace;
        double el = std::chrono::duration<double>(Clock::now() - wall0).count();
        if (due_s > el) std::this_thread::sleep_for(std::chrono::duration<double>(due_s - el));
      }
      RtabBackend::Track r;
      double prep = 0;
      if (!process(s.frames[i], r, prep)) continue;
      record(s.frames[i], r);
      if (r.ok) tum_line(live, s.frames[i].t_ns, r.T_w_cam);
      if (processed % 1000 == 0)
        fprintf(stderr, "[rtab_stream] %d/%zu ok=%d loops=%d nodes=%zu\n", processed, s.frames.size(), ok_count,
                be.loops(), be.nodes());
    }
    double wall = std::chrono::duration<double>(Clock::now() - wall0).count();

    // Final optimised trajectory: T_w_node(final) * T_node_cam(odometry), like CameraTrajectory.txt
    // of the harness / ORB-SLAM3's keyframe-anchored file. Frames tracked before the first node are
    // anchored to it afterwards; nodes no longer in the graph ride on the nearest surviving node.
    std::map<int, Transform> finals, node_odom;
    be.finish(finals, node_odom);
    std::map<int, long long> node_t;
    for (auto& fo : frames_out)
      if (fo.node && !node_t.count(fo.node)) node_t[fo.node] = fo.t_ns;
    auto node_final_cam = [&](int id, Transform& out) -> bool {
      auto it = finals.find(id);
      if (it != finals.end()) { out = be.view(it->second); return true; }
      int best = 0;
      for (auto& n : node_odom)
        if (finals.count(n.first) && (!best || std::llabs(node_t[n.first] - node_t[id]) < std::llabs(node_t[best] - node_t[id])))
          best = n.first;
      if (!best || !node_odom.count(id)) return false;
      out = be.view(finals.at(best)) * (be.cam_of(node_odom.at(best)).inverse() * be.cam_of(node_odom.at(id)));
      return true;
    };
    int first_node = node_odom.empty() ? 0 : node_odom.begin()->first;
    std::ofstream opt(a.out + ".optimized.txt");
    std::ofstream fcsv(a.out + ".frames.csv");
    fcsv << "t_ns,state,ok,map_id,track_ms,odom_ms,map_ms,map_changed,ref_kf,features,inliers,tx,ty,tz,qx,qy,qz,qw\n";
    int n_opt = 0;
    for (auto& fo : frames_out) {
      char buf[400];
      snprintf(buf, sizeof(buf), "%lld,%s,%d,%d,%.3f,%.3f,%.3f,%d,%d,%d,%d", fo.t_ns, fo.ok ? "OK" : "LOST", int(fo.ok),
               fo.map_id, fo.odom_ms + fo.map_ms, fo.odom_ms, fo.map_ms, int(fo.map_changed), fo.node, fo.features,
               fo.inliers);
      fcsv << buf;
      if (fo.ok) {
        Eigen::Quaternionf q = fo.live.getQuaternionf();
        snprintf(buf, sizeof(buf), ",%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f\n", fo.live.x(), fo.live.y(), fo.live.z(), q.x(),
                 q.y(), q.z(), q.w());
        fcsv << buf;
      } else {
        fcsv << ",,,,,,,\n";
      }
      if (!fo.ok) continue;
      int node = fo.node ? fo.node : first_node;
      Transform Tn;
      if (!node || !node_odom.count(node) || !node_final_cam(node, Tn)) continue;
      Transform T_node_cam = be.cam_of(node_odom.at(node)).inverse() * be.cam_of(fo.T_odom_base);
      tum_line(opt, fo.t_ns, Tn * T_node_cam);
      ++n_opt;
    }
    std::vector<double> mm = be.map_ms_all();
    json side = {
        {"backend", "rtabmap"}, {"rtabmap_version", RTABMAP_VERSION}, {"session", s.dir}, {"db3", s.db3},
        {"features", a.features}, {"params_effective", shown}, {"offset_ns", a.offset_ns},
        {"time_source", s.time_source}, {"time_domain", s.time_domain}, {"non_monotonic", s.non_monotonic},
        {"pace", a.pace}, {"mapper", async ? "async thread at Rtabmap/DetectionRate (wall clock)" : "synchronous at Rtabmap/DetectionRate (data clock)"},
        {"frames", s.frames.size()}, {"processed", processed}, {"ok_frames", ok_count},
        {"ok_ratio", processed ? double(ok_count) / processed : 0.0},
        {"odom_ms", {{"mean", odom_ms.empty() ? 0 : std::accumulate(odom_ms.begin(), odom_ms.end(), 0.0) / odom_ms.size()},
                     {"p50", pct(odom_ms, 50)}, {"p95", pct(odom_ms, 95)}, {"p99", pct(odom_ms, 99)},
                     {"over_33ms", std::count_if(odom_ms.begin(), odom_ms.end(), [](double x) { return x > 33.333; })}}},
        {"map_update_ms", {{"n", mm.size()}, {"mean", mm.empty() ? 0 : std::accumulate(mm.begin(), mm.end(), 0.0) / mm.size()},
                           {"p99", pct(mm, 99)}, {"max", mm.empty() ? 0 : *std::max_element(mm.begin(), mm.end())}}},
        {"loop_closures", be.loops()}, {"proximity_detections", be.proximities()}, {"empty_graph_updates", be.empty_updates()}, {"graph_nodes_final", finals.size()},
        {"optimized_frames", n_opt}, {"wall_s", wall},
        {"trajectory", "TUM T_world_cam (optical, world = first camera): live = O^-1*correction*odom*O per frame; "
                       ".optimized.txt = final node pose * odometry T_node_cam"}};
    std::ofstream(a.out + ".json") << side.dump(1) << "\n";
    be.close(true);
    fprintf(stderr, "[rtab_stream] done: %d processed, %d ok, loops %d, nodes %zu, wall %.1f s\n", processed, ok_count,
            be.loops(), finals.size(), wall);
    return 0;
  }

  // ---------------------------------------------------------------- live
  Line line;
  if (!line.connect_to(a.connect)) {
    fprintf(stderr, "[rtab_stream] cannot connect to %s\n", a.connect.c_str());
    return 1;
  }
  line.send_json({{"type", "hello"}, {"source", "rtabmap"}, {"features", a.features}, {"session", s.dir},
                  {"first_ns", s.frames.front().t_ns}, {"last_ns", s.frames.back().t_ns},
                  {"n_frames", s.frames.size()}, {"time_source", s.time_source}, {"time_domain", s.time_domain},
                  {"kf_updates", true}, {"rtabmap_version", RTABMAP_VERSION}, {"params", shown}});
  json start;
  do {
    if (!line.read_json(start)) {
      fprintf(stderr, "[rtab_stream] server closed before start\n");
      return 1;
    }
  } while (start.value("type", "") != "start");
  long long t0_ns = start.value("t0_ns", s.frames.front().t_ns);
  double rate = start.value("rate", a.rate);
  fprintf(stderr, "[rtab_stream] start t0_ns=%lld rate=%.2f\n", t0_ns, rate);

  std::vector<double> kf_build_ms;
  std::vector<size_t> kf_bytes;
  long long last_t_ns = t0_ns;
  auto send_kf_update = [&](const char* reason) -> bool {
    auto tb = Clock::now();
    std::string kfs;
    int map_id = -1;
    int n = be.kf_rows(kfs, map_id);
    double build = ms_since(tb);
    std::string body = "{\"type\":\"kf_update\",\"t_ns\":" + std::to_string(last_t_ns) + ",\"map_id\":" +
                       std::to_string(map_id) + ",\"reason\":\"" + reason + "\",\"n\":" + std::to_string(n) +
                       ",\"build_ms\":" + fmt6(build) + ",\"kfs\":[" + kfs + "]}";
    kf_build_ms.push_back(build);
    kf_bytes.push_back(body.size() + 1);
    return line.send_raw(body);
  };
  const double kf_period_s = 1.0;
  auto last_periodic = Clock::now();
  std::vector<Clock::time_point> kf_followups;
  auto service_kf_updates = [&]() -> bool {
    auto now = Clock::now();
    bool ok = true, sent = false;
    for (auto it = kf_followups.begin(); it != kf_followups.end();) {
      if (*it <= now) {
        if (!sent) {
          ok = ok && send_kf_update("map_changed");
          sent = true;
          last_periodic = now;
        }
        it = kf_followups.erase(it);
      } else {
        ++it;
      }
    }
    if (ok && std::chrono::duration<double>(now - last_periodic).count() >= kf_period_s) {
      ok = send_kf_update("periodic");
      last_periodic = now;
    }
    return ok;
  };

  size_t idx = 0;
  while (idx < s.frames.size() && s.frames[idx].t_ns < t0_ns) ++idx;
  auto wall0 = Clock::now();
  last_periodic = wall0;
  int map_changed_frames = 0;
  bool alive = true;
  while (alive && idx < s.frames.size()) {
    if (!service_kf_updates()) { alive = false; break; }
    double elapsed = std::chrono::duration<double>(Clock::now() - wall0).count();
    double due_s = double(s.frames[idx].t_ns - t0_ns) * 1e-9 / rate;
    if (due_s > elapsed) {
      std::this_thread::sleep_for(std::chrono::duration<double>(std::min(due_s - elapsed, 0.005)));
      continue;
    }
    size_t j = idx;
    while (j + 1 < s.frames.size() && double(s.frames[j + 1].t_ns - t0_ns) * 1e-9 / rate <= elapsed) ++j;
    dropped += int(j - idx);
    RtabBackend::Track r;
    double prep = 0;
    if (process(s.frames[j], r, prep)) {
      record(s.frames[j], r);
      last_t_ns = s.frames[j].t_ns;
      if (r.map_changed) ++map_changed_frames;
      std::string body = "{\"type\":\"pose\",\"t_ns\":" + std::to_string(s.frames[j].t_ns) + ",\"state\":\"" +
                         (r.ok ? "OK" : "LOST") + "\",\"T_wc\":" + (r.ok ? tf16(r.T_w_cam) : std::string("null")) +
                         ",\"track_ms\":" + fmt6(r.odom_ms + r.map_ms) + ",\"odom_ms\":" + fmt6(r.odom_ms) +
                         ",\"map_ms\":" + fmt6(r.map_ms) + ",\"prep_ms\":" + fmt6(prep) +
                         ",\"frame_idx\":" + std::to_string(j) + ",\"dropped\":" + std::to_string(dropped) +
                         ",\"map_changed\":" + (r.map_changed ? "true" : "false") + ",\"map_id\":" +
                         std::to_string(r.map_id) + ",\"features\":" + std::to_string(r.features) +
                         ",\"inliers\":" + std::to_string(r.inliers);
      if (r.ref_kf > 0)
        body += ",\"ref_kf\":" + std::to_string(r.ref_kf) + ",\"kf_map_id\":" + std::to_string(r.kf_map_id) +
                ",\"T_w_kf\":" + tf16(r.T_w_kf) + "}";
      else
        body += ",\"ref_kf\":null,\"kf_map_id\":null,\"T_w_kf\":null}";
      alive = line.send_raw(body);
      if (alive && r.map_changed) {
        alive = send_kf_update("map_changed");
        auto now = Clock::now();
        kf_followups.push_back(now + std::chrono::milliseconds(500));
        kf_followups.push_back(now + std::chrono::milliseconds(3000));
        last_periodic = now;
      }
    }
    idx = j + 1;
  }
  if (alive) alive = send_kf_update("end");
  if (alive) line.send_json({{"type", "end"}, {"processed", processed}, {"dropped", dropped}});
  std::vector<double> mm = be.map_ms_all();
  fprintf(stderr,
          "[rtab_stream] live done: processed %d ok %d dropped %d | odom mean %.2f p99 %.2f ms | map updates %zu mean %.1f "
          "p99 %.1f max %.1f ms | loops %d proximity %d map_changed frames %d nodes %zu empty-graph updates %d%s\n",
          processed, ok_count, dropped,
          odom_ms.empty() ? 0 : std::accumulate(odom_ms.begin(), odom_ms.end(), 0.0) / odom_ms.size(), pct(odom_ms, 99),
          mm.size(), mm.empty() ? 0 : std::accumulate(mm.begin(), mm.end(), 0.0) / mm.size(), pct(mm, 99),
          mm.empty() ? 0 : *std::max_element(mm.begin(), mm.end()), be.loops(), be.proximities(), map_changed_frames,
          be.nodes(), be.empty_updates(), alive ? "" : " (server disconnected)");
  if (!kf_build_ms.empty())
    fprintf(stderr, "[rtab_stream] kf_update: %zu sent, build mean %.2f max %.2f ms, size mean %.0f max %zu bytes\n",
            kf_build_ms.size(), std::accumulate(kf_build_ms.begin(), kf_build_ms.end(), 0.0) / kf_build_ms.size(),
            *std::max_element(kf_build_ms.begin(), kf_build_ms.end()),
            std::accumulate(kf_bytes.begin(), kf_bytes.end(), 0.0) / kf_bytes.size(),
            *std::max_element(kf_bytes.begin(), kf_bytes.end()));
  std::map<int, Transform> finals, node_odom;
  be.finish(finals, node_odom);
  be.close(false);
  return 0;
}
