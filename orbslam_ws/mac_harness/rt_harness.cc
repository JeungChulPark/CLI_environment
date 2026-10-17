// Real-time replay harness for ORB-SLAM3 RGB-D on macOS (no ROS).
//
// Reads the librealsense-SDK rosbag2 (.db3) directly with sqlite3, pairs colour and
// depth exactly as orbslam_ws/scripts/rs_bag_bridge.py does (hold colour until a newer
// depth exists, pick nearest, 20 ms skew limit), aligns depth into the colour frame,
// and releases pairs at a fixed 30 Hz into a KEEP_LAST(5) drop-oldest queue — the
// same shape as the bridge's BEST_EFFORT publisher feeding rgbd_node. Stamps are
// t0 + n/30 (monotonic by construction).
//
// Per frame it records TrackRGBD wall time (steady_clock, identical to rgbd_node.cpp),
// tracking state, lost flag, queue wait and release->done latency, plus dropped frames.

#include <sqlite3.h>
#include <pthread/qos.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cstdio>
#include <System.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <algorithm>
#include <cmath>

using Clock = std::chrono::steady_clock;

static const int T_DEPTH_CI = 3, T_COLOR_CI = 5, T_DEPTH_UNITS = 30,
                 T_DEPTH_TF = 110, T_DEPTH_IMG = 108, T_COLOR_IMG = 111, T_COLOR_TF = 113;

static std::map<std::string, std::string> parse_kv(const std::string& s) {
  std::map<std::string, std::string> m;
  std::stringstream ss(s);
  std::string part;
  while (std::getline(ss, part, ';')) {
    auto p = part.find('=');
    if (p != std::string::npos) m[part.substr(0, p)] = part.substr(p + 1);
  }
  return m;
}
static std::vector<double> csv(const std::string& s) {
  std::vector<double> v; std::stringstream ss(s); std::string x;
  while (std::getline(ss, x, ',')) v.push_back(std::stod(x));
  return v;
}

struct Intr { int w, h; double fx, fy, cx, cy; };
static Intr intr(const std::string& s) {
  auto m = parse_kv(s);
  return {(int)std::stod(m["width"]), (int)std::stod(m["height"]), std::stod(m["fx"]),
          std::stod(m["fy"]), std::stod(m["ppx"]), std::stod(m["ppy"])};
}

// std_msgs/String CDR: 4 encap + uint32 len + chars
static std::string cdr_string(const unsigned char* d, int n) {
  uint32_t len; memcpy(&len, d + 4, 4);
  return std::string((const char*)d + 8, std::max(0, (int)len - 1));
}

struct Img { int64_t ts; int h, w; std::vector<uint8_t> data; };
// sensor_msgs/Image CDR (little endian), offsets relative to after the 4-byte encap
static Img cdr_image(const unsigned char* d, int n, int64_t ts) {
  const unsigned char* b = d + 4; size_t o = 8;  // stamp
  auto u32 = [&](void) { o = (o + 3) & ~size_t(3); uint32_t v; memcpy(&v, b + o, 4); o += 4; return v; };
  uint32_t fl = u32(); o += fl;           // frame_id
  uint32_t h = u32(), w = u32();
  uint32_t el = u32(); o += el;           // encoding
  o += 1;                                 // is_bigendian
  u32();                                  // step
  uint32_t dl = u32();
  Img im; im.ts = ts; im.h = h; im.w = w; im.data.assign(b + o, b + o + dl);
  return im;
}

struct Pair { int64_t ts; cv::Mat rgb; cv::Mat depth; };

template <class T> struct BQueue {  // bounded blocking queue
  std::deque<T> q; std::mutex m; std::condition_variable cv; size_t cap; bool done = false;
  explicit BQueue(size_t c) : cap(c) {}
  void push(T v) { std::unique_lock<std::mutex> l(m); cv.wait(l, [&] { return q.size() < cap; });
                   q.push_back(std::move(v)); cv.notify_all(); }
  bool pop(T& v) { std::unique_lock<std::mutex> l(m); cv.wait(l, [&] { return !q.empty() || done; });
                   if (q.empty()) return false; v = std::move(q.front()); q.pop_front(); cv.notify_all(); return true; }
  void finish() { std::lock_guard<std::mutex> l(m); done = true; cv.notify_all(); }
};

struct Released { int idx; double stamp; Clock::time_point t_rel; Pair p; };

int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "usage: rt_harness VOCAB SETTINGS BAG.db3 OUTDIR [rate=30] [limit=0] [queue=5]\n"
                 "       rt_harness - - BAG.db3 OUT.mp4 video   (paired colour frames only, 384x288 H.264)\n";
    return 1;
  }
  std::string vocab = argv[1], settings = argv[2], bag = argv[3], out = argv[4];
  // video mode renders exactly the frames the tracker is fed, so video frame i == trajectory row i
  const bool video_mode = argc > 5 && std::string(argv[5]) == "video";
  double rate = argc > 5 && !video_mode ? atof(argv[5]) : 30.0;
  int limit = argc > 6 ? atoi(argv[6]) : 0;
  size_t qdepth = argc > 7 ? atoi(argv[7]) : 5;

  sqlite3* db;
  if (sqlite3_open_v2(bag.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    std::cerr << "cannot open " << bag << "\n"; return 1;
  }
  std::map<int, std::string> cfg;
  {
    sqlite3_stmt* st;
    sqlite3_prepare_v2(db, "SELECT topic_id, data FROM messages WHERE topic_id IN (3,5,30,110,113) "
                           "GROUP BY topic_id", -1, &st, nullptr);
    while (sqlite3_step(st) == SQLITE_ROW)
      cfg[sqlite3_column_int(st, 0)] = cdr_string((const unsigned char*)sqlite3_column_blob(st, 1),
                                                  sqlite3_column_bytes(st, 1));
    sqlite3_finalize(st);
  }
  Intr ic = intr(cfg[T_COLOR_CI]), idp = intr(cfg[T_DEPTH_CI]);
  double units = std::stod(cfg[T_DEPTH_UNITS]);
  auto rc = csv(parse_kv(cfg[T_COLOR_TF])["rotation"]), tc = csv(parse_kv(cfg[T_COLOR_TF])["translation"]);
  // depth tf is identity (checked), so p_c = Rc p_d + tc  (ref->stream, as in rs_bag_bridge.py)
  float R[9]; for (int i = 0; i < 9; i++) R[i] = rc[i];
  float t[3] = {(float)tc[0], (float)tc[1], (float)tc[2]};
  std::cerr << "color fx=" << ic.fx << " depth fx=" << idp.fx << " units=" << units
            << " t=[" << t[0] << "," << t[1] << "," << t[2] << "]\n";

  std::vector<float> rayx(idp.w * idp.h), rayy(idp.w * idp.h);
  for (int v = 0; v < idp.h; v++)
    for (int u = 0; u < idp.w; u++) {
      rayx[v * idp.w + u] = (u - idp.cx) / idp.fx;
      rayy[v * idp.w + u] = (v - idp.cy) / idp.fy;
    }
  auto align = [&](const Img& d) {
    cv::Mat outm(ic.h, ic.w, CV_16UC1, cv::Scalar(0));
    std::vector<uint16_t> buf(ic.w * ic.h, 65535);
    const uint16_t* src = (const uint16_t*)d.data.data();
    for (int i = 0; i < idp.w * idp.h; i++) {
      if (!src[i]) continue;
      float z = src[i] * (float)units, x = rayx[i] * z, y = rayy[i] * z;
      float xc = R[0] * x + R[1] * y + R[2] * z + t[0];
      float yc = R[3] * x + R[4] * y + R[5] * z + t[1];
      float zz = R[6] * x + R[7] * y + R[8] * z + t[2];
      if (zz <= 0) continue;
      int uu = (int)std::nearbyint(xc / zz * ic.fx + ic.cx), vv = (int)std::nearbyint(yc / zz * ic.fy + ic.cy);
      if (uu < 0 || uu >= ic.w || vv < 0 || vv >= ic.h) continue;
      uint16_t zi = (uint16_t)std::nearbyint(zz / units);
      uint16_t& b = buf[vv * ic.w + uu];
      if (zi < b) b = zi;
    }
    uint16_t* o = outm.ptr<uint16_t>();
    for (int i = 0; i < ic.w * ic.h; i++) o[i] = buf[i] == 65535 ? 0 : buf[i];
    return outm;
  };

  // ---- reader thread: bag -> paired, aligned frames (ahead of the pacer) ----
  BQueue<Pair> ready(60);
  std::atomic<long> n_pair{0}, n_skew_drop{0};
  std::vector<double> align_ms;
  FILE* vpipe = nullptr;
  if (video_mode) {
    std::string cmd = "ffmpeg -y -loglevel error -f rawvideo -pix_fmt bgr24 -s 384x288 -r 30 -i - "
                      "-c:v libx264 -preset slow -crf 30 -pix_fmt yuv420p -movflags +faststart '" + out + "'";
    vpipe = popen(cmd.c_str(), "w");
    if (!vpipe) { std::cerr << "cannot start ffmpeg\n"; return 1; }
  }
  std::thread reader([&] {
    sqlite3_stmt* st;
    sqlite3_prepare_v2(db, "SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (108,111) "
                           "ORDER BY timestamp", -1, &st, nullptr);
    std::deque<Img> dbuf, cbuf;
    while (sqlite3_step(st) == SQLITE_ROW) {
      int tid = sqlite3_column_int(st, 0);
      int64_t ts = sqlite3_column_int64(st, 1);
      Img im = cdr_image((const unsigned char*)sqlite3_column_blob(st, 2), sqlite3_column_bytes(st, 2), ts);
      if (tid == T_DEPTH_IMG) { dbuf.push_back(std::move(im)); if (dbuf.size() > 8) dbuf.pop_front(); }
      else cbuf.push_back(std::move(im));
      if (dbuf.empty() || cbuf.empty() || dbuf.back().ts <= cbuf.front().ts) continue;
      Img c = std::move(cbuf.front()); cbuf.pop_front();
      const Img* best = nullptr; int64_t bd = INT64_MAX;
      for (auto& d : dbuf) { int64_t dd = std::llabs(d.ts - c.ts); if (dd < bd) { bd = dd; best = &d; } }
      if (bd > 20000000) { n_skew_drop++; continue; }
      if (video_mode) {
        cv::Mat small;
        cv::resize(cv::Mat(c.h, c.w, CV_8UC3, c.data.data()), small, cv::Size(384, 288), 0, 0, cv::INTER_AREA);
        fwrite(small.data, 1, small.total() * 3, vpipe);
        ++n_pair;
        continue;
      }
      auto ta = Clock::now();
      Pair p; p.ts = c.ts;
      p.depth = align(*best);
      cv::Mat bgr(c.h, c.w, CV_8UC3, c.data.data());
      p.rgb = bgr.clone();  // settings Camera.RGB: 1 -> ORB-SLAM3 converts as RGB; bridge sent bgr8 raw too
      align_ms.push_back(std::chrono::duration<double, std::milli>(Clock::now() - ta).count());
      ready.push(std::move(p));
      if (limit && ++n_pair >= limit) break;
      if (!limit) ++n_pair;
    }
    sqlite3_finalize(st);
    ready.finish();
  });

  if (video_mode) {
    reader.join();
    int rc = pclose(vpipe);
    std::cout << "wrote " << out << ": " << n_pair << " frames (" << n_skew_drop << " dropped), ffmpeg rc " << rc << "\n";
    sqlite3_close(db);
    return rc == 0 ? 0 : 1;
  }

  ORB_SLAM3::System slam(vocab, settings, ORB_SLAM3::System::RGBD, false);

  // prefill a second of frames so the reader's warm-up never throttles the pacer
  { std::unique_lock<std::mutex> l(ready.m); ready.cv.wait(l, [&] { return ready.q.size() >= 30 || ready.done; }); }

  // ---- pacer: fixed-rate release into a KEEP_LAST(qdepth) drop-oldest slot ----
  std::deque<Released> live; std::mutex lm; std::condition_variable lcv; bool live_done = false;
  long n_released = 0, n_queue_drop = 0, n_late_release = 0;
  double max_release_late_ms = 0;
  std::vector<int> dropped_idx;
  const double t0_stamp = 1000.0;
  auto t0 = Clock::now() + std::chrono::milliseconds(200);
  std::thread pacer([&] {
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);  // a camera driver is not App-Napped
    Pair p; int idx = 0;
    while (ready.pop(p)) {
      auto target = t0 + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(idx / rate));
      std::this_thread::sleep_until(target);
      double late = std::chrono::duration<double, std::milli>(Clock::now() - target).count();
      if (late > 5.0) n_late_release++;
      max_release_late_ms = std::max(max_release_late_ms, late);
      Released r{idx, t0_stamp + idx / 30.0, Clock::now(), std::move(p)};
      {
        std::lock_guard<std::mutex> l(lm);
        if (live.size() >= qdepth) { dropped_idx.push_back(live.front().idx); live.pop_front(); n_queue_drop++; }
        live.push_back(std::move(r));
      }
      lcv.notify_one();
      idx++; n_released++;
    }
    std::lock_guard<std::mutex> l(lm); live_done = true; lcv.notify_all();
  });

  struct Row { int idx; double stamp, track_ms, wait_ms, e2e_ms; int state, lost; };
  std::vector<Row> rows; rows.reserve(9000);
  pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
  auto wall0 = Clock::now();
  while (true) {
    Released r;
    {
      std::unique_lock<std::mutex> l(lm);
      lcv.wait(l, [&] { return !live.empty() || live_done; });
      if (live.empty()) break;
      r = std::move(live.front()); live.pop_front();
    }
    auto ts0 = Clock::now();
    Sophus::SE3f Tcw = slam.TrackRGBD(r.p.rgb, r.p.depth, r.stamp);
    auto ts1 = Clock::now();
    double ms = std::chrono::duration<double, std::milli>(ts1 - ts0).count();
    rows.push_back({r.idx, r.stamp, ms,
                    std::chrono::duration<double, std::milli>(ts0 - r.t_rel).count(),
                    std::chrono::duration<double, std::milli>(ts1 - r.t_rel).count(),
                    slam.GetTrackingState(), Tcw.matrix().isZero(0) ? 1 : 0});
#ifdef REGISTER_TIMES
    slam.InsertTrackTime(ms);
#endif
    if (rows.size() % 600 == 0)
      std::cerr << "[rt] " << rows.size() << " tracked, last " << std::fixed << std::setprecision(2) << ms
                << " ms, dropped " << n_queue_drop << "\n";
  }
  double wall_s = std::chrono::duration<double>(Clock::now() - wall0).count();
  reader.join(); pacer.join();
  slam.Shutdown();
  slam.SaveTrajectoryTUM(out + "/CameraTrajectory.txt");
  slam.SaveKeyFrameTrajectoryTUM(out + "/KeyFrameTrajectory.txt");
  {
    // same 11-line ascii PCD layout rgbd_node writes, which build_live_viz.py skips
    auto pts = slam.GetAllMapPointsWorld();
    std::ofstream f(out + "/map_points.pcd");
    f << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
      << "COUNT 1 1 1\nWIDTH " << pts.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << pts.size()
      << "\nDATA ascii\n" << std::fixed << std::setprecision(5);
    for (auto& p : pts) f << p.x() << " " << p.y() << " " << p.z() << "\n";
  }

  {
    std::ofstream f(out + "/TrackPerFrame.csv");
    f << "frame,stamp,track_ms,state,no_pose,queue_wait_ms,release_to_done_ms\n" << std::fixed << std::setprecision(6);
    for (auto& r : rows)
      f << r.idx << "," << r.stamp << "," << r.track_ms << "," << r.state << "," << r.lost << ","
        << r.wait_ms << "," << r.e2e_ms << "\n";
  }
  std::vector<double> v; for (auto& r : rows) v.push_back(r.track_ms);
  std::sort(v.begin(), v.end());
  auto pct = [&](double p) { return v.empty() ? 0 : v[std::min(v.size() - 1, (size_t)std::ceil(p / 100 * v.size()) - 1)]; };
  double mean = 0; for (double x : v) mean += x; mean /= std::max<size_t>(1, v.size());
  long over = std::count_if(v.begin(), v.end(), [](double x) { return x > 1000.0 / 30.0; });
  long notok = std::count_if(rows.begin(), rows.end(), [](const Row& r) { return r.state != 2; });
  long nopose = std::count_if(rows.begin(), rows.end(), [](const Row& r) { return r.lost; });
  std::vector<double> e2e; for (auto& r : rows) e2e.push_back(r.e2e_ms); std::sort(e2e.begin(), e2e.end());
  double am = 0; for (double x : align_ms) am += x; am /= std::max<size_t>(1, align_ms.size());

  std::ofstream s(out + "/summary.txt");
  std::ostringstream o; o << std::fixed << std::setprecision(2);
  o << "pairs_released " << n_released << "\nskew_dropped " << n_skew_drop << "\ntracked " << rows.size()
    << "\nqueue_dropped " << n_queue_drop << "\nlate_release_gt5ms " << n_late_release
    << "\nmax_release_late_ms " << max_release_late_ms << "\nwall_s " << wall_s
    << "\neffective_hz " << rows.size() / wall_s
    << "\ntrack_ms mean " << mean << " p50 " << pct(50) << " p95 " << pct(95) << " p99 " << pct(99)
    << " p99.9 " << pct(99.9) << " max " << (v.empty() ? 0 : v.back())
    << "\nover_33.33ms " << over << "\nrelease_to_done_ms p99 " << (e2e.empty() ? 0 : e2e[(size_t)(0.99 * (e2e.size() - 1))])
    << " max " << (e2e.empty() ? 0 : e2e.back())
    << "\nstate_not_ok " << notok << "\nno_pose " << nopose << "\nalign_ms_mean " << am << "\n";
  s << o.str(); std::cout << "==== SUMMARY ====\n" << o.str();
  sqlite3_close(db);
  return 0;
}
