// OpenLORIS-Scene runner for ORB-SLAM3 RGB-D (macOS, no ROS). Builds against any ORB-SLAM3 v1.0 tree, so the
// same program drives the upstream release and our modified tree (see CMakeLists.txt, build.sh).
//
//   openloris_rgbd VOCAB SETTINGS SEQ_DIR OUT_DIR [--realtime 1] [--rate 1.0] [--keep-last 5]
//                  [--stride 1] [--set Key=Value ...] [--gyro RIG.json]   (--gyro only in the WITH_GYRO build)
//
// SEQ_DIR is one extracted package sequence (color.txt, aligned_depth.txt, color/, aligned_depth/,
// d400_gyroscope.txt). Colour and aligned depth are paired by nearest stamp (< 5 ms apart).
//
// --realtime 1 (default): a pacer thread releases frame i at t0 + (stamp_i - stamp_0) / rate into a KEEP_LAST
//   drop-oldest slot, exactly like the live camera path; frames the tracker cannot keep up with are dropped
//   and counted. --realtime 0 tracks every frame back to back (the usual offline benchmark protocol).
//
// --stride k keeps every k-th colour/depth pair: k times the motion between frames, as if the robot moved k times
//   faster (without the extra motion blur). Combine with --rate k to keep the 30 Hz frame rate. The gyro still
//   integrates every IMU sample between the kept frames, as it would on a faster robot.
//
// Outputs in OUT_DIR:
//   live_tum.txt        the pose ORB-SLAM3 returned for each frame at that moment (only frames with state OK);
//                       what an online consumer sees. Frame d400_color, T_world_camera.
//   final_tum.txt       SaveTrajectoryTUM after shutdown (keyframe-anchored, includes later loop corrections)
//   frames.csv          per frame: stamp, track_ms, state, dropped
//   summary.json        counts, timing percentiles, maps, gyro statistics
#include <System.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pthread/qos.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef WITH_GYRO
#include "gyro_aid.h"   // objpose/src: the same integrator + online bias update the 260901 experiments used
#endif

using Clock = std::chrono::steady_clock;

struct Entry { double t; std::string path; };

static std::vector<Entry> read_list(const std::string& file) {
  std::vector<Entry> v;
  std::ifstream in(file);
  if (!in) throw std::runtime_error("cannot read " + file);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    Entry e;
    ss >> e.t >> e.path;
    if (!e.path.empty()) v.push_back(e);
  }
  // not always in time order: office1-7/aligned_depth.txt starts with two frames from 0.6 s later
  std::sort(v.begin(), v.end(), [](const Entry& a, const Entry& b) { return a.t < b.t; });
  return v;
}

// SETTINGS with Key=Value overrides applied: replace an existing "Key:" line or append it. The first line
// must stay "%YAML:1.0" or cv::FileStorage refuses the file.
static std::string derived_settings(const std::string& base, const std::vector<std::string>& sets, const std::string& out) {
  std::ifstream in(base);
  if (!in) throw std::runtime_error("cannot read " + base);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) lines.push_back(line);
  for (const std::string& kv : sets) {
    size_t eq = kv.find('=');
    if (eq == std::string::npos) throw std::runtime_error("--set needs Key=Value: " + kv);
    std::string key = kv.substr(0, eq), val = kv.substr(eq + 1);
    bool done = false;
    for (std::string& l : lines)
      if (l.rfind(key + ":", 0) == 0) { l = key + ": " + val; done = true; }
    if (!done) lines.push_back(key + ": " + val);
  }
  std::string path = out + "/settings.yaml";
  std::ofstream o(path);
  for (const std::string& l : lines) o << l << "\n";
  return path;
}

struct Frame { int idx; double t; cv::Mat rgb, depth; Clock::time_point released; };

int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "usage: openloris_rgbd VOCAB SETTINGS SEQ_DIR OUT_DIR [--realtime 1] [--rate 1.0] [--keep-last 5]\n"
                 "                     [--stride 1] [--set Key=Value ...] [--gyro RIG.json]\n";
    return 2;
  }
  std::string vocab = argv[1], settings = argv[2], seq = argv[3], out = argv[4], gyro_rig;
  bool realtime = true;
  double rate = 1.0;
  size_t keep_last = 5, stride = 1;
  std::vector<std::string> sets;
  for (int i = 5; i < argc; ++i) {
    std::string k = argv[i];
    auto val = [&]() -> std::string { if (i + 1 >= argc) throw std::runtime_error(k + " needs a value"); return argv[++i]; };
    if (k == "--realtime") realtime = std::stoi(val()) != 0;
    else if (k == "--rate") rate = std::stod(val());
    else if (k == "--keep-last") keep_last = std::stoul(val());
    else if (k == "--stride") stride = std::max<size_t>(1, std::stoul(val()));
    else if (k == "--set") sets.push_back(val());
    else if (k == "--gyro") gyro_rig = val();
    else { std::cerr << "unknown option " << k << "\n"; return 2; }
  }
#ifndef WITH_GYRO
  if (!gyro_rig.empty()) { std::cerr << "--gyro needs the WITH_GYRO build (modified tree)\n"; return 2; }
#endif
  if (system(("mkdir -p '" + out + "'").c_str()) != 0) return 1;

  // ---- pair colour with aligned depth ----
  std::vector<Entry> color = read_list(seq + "/color.txt"), depth = read_list(seq + "/aligned_depth.txt");
  struct Pair { double t; std::string c, d; };
  std::vector<Pair> pairs;
  size_t j = 0;
  long no_depth = 0;
  for (const Entry& c : color) {
    while (j + 1 < depth.size() && std::fabs(depth[j + 1].t - c.t) <= std::fabs(depth[j].t - c.t)) ++j;
    if (depth.empty() || std::fabs(depth[j].t - c.t) > 0.005) { ++no_depth; continue; }
    pairs.push_back({c.t, seq + "/" + c.path, seq + "/" + depth[j].path});
  }
  if (stride > 1) {
    std::vector<Pair> kept;
    for (size_t i = 0; i < pairs.size(); i += stride) kept.push_back(pairs[i]);
    pairs.swap(kept);
  }
  std::cerr << "[openloris] " << seq << ": " << pairs.size() << " colour/depth pairs (stride " << stride << ", " << no_depth
            << " colour frames without depth), " << (pairs.empty() ? 0 : pairs.back().t - pairs.front().t) << " s\n";
  if (pairs.empty()) return 1;

#ifdef WITH_GYRO
  GyroAid gyro;
  bool use_gyro = !gyro_rig.empty();
  if (use_gyro) {
    std::ifstream rin(gyro_rig);
    if (!rin) throw std::runtime_error("cannot read " + gyro_rig);
    nlohmann::json rig = nlohmann::json::parse(rin);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) gyro.R_ci(r, c) = rig.at("R_cam_imu").at(r).at(c).get<double>();
    gyro.tau_ns = (long long)std::llround(rig.at("time").at("imu_minus_cam_s").get<double>() * 1e9);
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    if (rig.contains("gyro_bias_rad_s"))
      for (int k = 0; k < 3; ++k) b[k] = rig["gyro_bias_rad_s"].at(k).get<double>();
    gyro.bias_cam = gyro.R_ci * b;
    std::ifstream gin(seq + "/d400_gyroscope.txt");
    std::string line;
    while (std::getline(gin, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream ss(line);
      long double t;
      Eigen::Vector3d w;
      ss >> t >> w[0] >> w[1] >> w[2];
      long long tn = (long long)std::llround(t * 1e9L);
      if (!gyro.t_ns.empty() && tn <= gyro.t_ns.back()) continue;
      gyro.t_ns.push_back(tn);
      gyro.w_cam.push_back(gyro.R_ci * w);
    }
    if (gyro.t_ns.size() < 10) throw std::runtime_error("no gyro samples in " + seq + "/d400_gyroscope.txt");
    gyro.rate_hz = double(gyro.t_ns.size() - 1) / (double(gyro.t_ns.back() - gyro.t_ns.front()) * 1e-9);
    gyro.db3 = seq + "/d400_gyroscope.txt";
    sets.push_back("Gyro.Use=1");
    std::cerr << "[openloris] gyro aid: " << gyro.t_ns.size() << " samples, " << gyro.rate_hz << " Hz\n";
  }
#endif

  std::string settings_used = derived_settings(settings, sets, out);
  ORB_SLAM3::System slam(vocab, settings_used, ORB_SLAM3::System::RGBD, false);

  auto load = [&](size_t i) {
    Frame f;
    f.idx = int(i);
    f.t = pairs[i].t;
    f.rgb = cv::imread(pairs[i].c, cv::IMREAD_UNCHANGED);
    f.depth = cv::imread(pairs[i].d, cv::IMREAD_UNCHANGED);
    if (f.rgb.empty() || f.depth.empty()) throw std::runtime_error("cannot read frame " + pairs[i].c);
    return f;
  };

  // ---- source: pacer (real time) or plain sequential feed ----
  std::deque<Frame> slot;
  std::mutex m;
  std::condition_variable cv;
  bool src_done = false;
  std::vector<int> dropped;
  std::thread pacer;
  if (realtime) {
    pacer = std::thread([&] {
      pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
      Frame next = load(0);
      auto t0 = Clock::now() + std::chrono::milliseconds(300);
      for (size_t i = 0; i < pairs.size(); ++i) {
        auto due = t0 + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>((pairs[i].t - pairs[0].t) / rate));
        std::this_thread::sleep_until(due);
        next.released = Clock::now();
        {
          std::lock_guard<std::mutex> l(m);
          if (slot.size() >= keep_last) { dropped.push_back(slot.front().idx); slot.pop_front(); }
          slot.push_back(std::move(next));
        }
        cv.notify_one();
        if (i + 1 < pairs.size()) next = load(i + 1);   // decode ahead while waiting for the next due time
      }
      std::lock_guard<std::mutex> l(m);
      src_done = true;
      cv.notify_all();
    });
  }

  struct Row { int idx; double t, track_ms, wait_ms; int state; };
  std::vector<Row> rows;
  std::ofstream live(out + "/live_tum.txt");
  live << std::fixed << std::setprecision(9);
  long map_changes = 0;
#ifdef WITH_GYRO
  long gyro_frames = 0, gyro_missing = 0;
#endif
  pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
  auto wall0 = Clock::now();
  for (size_t n = 0;; ++n) {
    Frame f;
    if (realtime) {
      std::unique_lock<std::mutex> l(m);
      cv.wait(l, [&] { return !slot.empty() || src_done; });
      if (slot.empty()) break;
      f = std::move(slot.front());
      slot.pop_front();
    } else {
      if (n >= pairs.size()) break;
      f = load(n);
      f.released = Clock::now();
    }
    long long t_ns = (long long)std::llround(f.t * 1e9);
#ifdef WITH_GYRO
    bool gyro_now = false;
    if (use_gyro) {
      Eigen::Matrix3f R_gc;
      gyro_now = gyro.advance(t_ns, R_gc);
      if (gyro_now) { slam.SetFrameGyro(R_gc, gyro.epoch); ++gyro_frames; } else ++gyro_missing;
    }
#endif
    auto a = Clock::now();
    Sophus::SE3f Tcw = slam.TrackRGBD(f.rgb, f.depth, f.t);
    auto b = Clock::now();
    int state = slam.GetTrackingState();
    bool changed = slam.MapChanged();
    if (changed) ++map_changes;
    rows.push_back({f.idx, f.t, std::chrono::duration<double, std::milli>(b - a).count(),
                    std::chrono::duration<double, std::milli>(a - f.released).count(), state});
    if (state == ORB_SLAM3::Tracking::OK) {
      Sophus::SE3f Twc = Tcw.inverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f p = Twc.translation();
      live << f.t << " " << p.x() << " " << p.y() << " " << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
           << q.w() << "\n";
    }
#ifdef WITH_GYRO
    if (use_gyro && gyro_now) gyro.observe(t_ns, state == ORB_SLAM3::Tracking::OK, Tcw.inverse().rotationMatrix(), changed);
#endif
  }
  double wall_s = std::chrono::duration<double>(Clock::now() - wall0).count();
  if (pacer.joinable()) pacer.join();
  live.close();
  slam.Shutdown();
  slam.SaveTrajectoryTUM(out + "/final_tum.txt");

  // ---- per-frame log and summary ----
  std::vector<bool> was_dropped(pairs.size(), false);
  for (int d : dropped) was_dropped[d] = true;
  {
    std::ofstream f(out + "/frames.csv");
    f << "frame,stamp,track_ms,queue_wait_ms,state\n" << std::fixed << std::setprecision(6);
    for (const Row& r : rows) f << r.idx << "," << r.t << "," << r.track_ms << "," << r.wait_ms << "," << r.state << "\n";
  }
  std::vector<double> v;
  for (const Row& r : rows) v.push_back(r.track_ms);
  std::sort(v.begin(), v.end());
  auto pct = [&](double p) { return v.empty() ? 0.0 : v[std::min(v.size() - 1, (size_t)std::ceil(p / 100 * v.size()) - 1)]; };
  double mean = 0;
  for (double x : v) mean += x;
  mean /= std::max<size_t>(1, v.size());
  long ok = std::count_if(rows.begin(), rows.end(), [](const Row& r) { return r.state == ORB_SLAM3::Tracking::OK; });
  long over = std::count_if(v.begin(), v.end(), [](double x) { return x > 1000.0 / 30.0; });

  std::ofstream s(out + "/summary.json");
  s << std::fixed << std::setprecision(3) << "{\n"
    << "  \"sequence\": \"" << seq << "\",\n  \"realtime\": " << (realtime ? "true" : "false") << ",\n  \"rate\": " << rate << ",\n  \"stride\": " << stride << ",\n"
    << "  \"pairs\": " << pairs.size() << ",\n  \"tracked\": " << rows.size() << ",\n  \"dropped\": " << dropped.size() << ",\n"
    << "  \"state_ok\": " << ok << ",\n  \"ok_ratio_of_all\": " << double(ok) / pairs.size() << ",\n"
    << "  \"map_changes\": " << map_changes << ",\n"
    << "  \"wall_s\": " << wall_s << ",\n"
    << "  \"track_ms\": {\"mean\": " << mean << ", \"p50\": " << pct(50) << ", \"p95\": " << pct(95) << ", \"p99\": " << pct(99)
    << ", \"max\": " << (v.empty() ? 0 : v.back()) << ", \"over_33ms\": " << over << "},\n"
#ifdef WITH_GYRO
    << "  \"gyro\": " << (use_gyro ? "true" : "false") << ",\n  \"gyro_frames\": " << gyro_frames << ",\n  \"gyro_missing\": " << gyro_missing
    << ",\n  \"gyro_bias_updates\": " << gyro.zupt_updates << ",\n"
#else
    << "  \"gyro\": false,\n"
#endif
    << "  \"settings\": \"" << settings_used << "\"\n}\n";
  std::cerr << "[openloris] done: " << rows.size() << "/" << pairs.size() << " tracked, " << dropped.size() << " dropped, "
            << ok << " OK, track mean " << mean << " ms p99 " << pct(99) << " ms, " << wall_s << " s\n";
  return 0;
}
