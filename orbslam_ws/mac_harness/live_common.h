// Shared by live_orbslam.cc and live_rtabmap.cc: a 30 Hz camera stream in, a SLAM backend tracking it,
// and the browser viewer streaming the result.
//
// The camera is simulated by BagCamera, which replays the librealsense-SDK rosbag2 (.db3) in real
// time with the pairing, depth alignment and pacing of rt_harness.cc, and hands frames to the
// tracker through a KEEP_LAST(5) drop-oldest FrameSink. The tracker only ever sees the sink, so a
// live RealSense source that pushes aligned colour + depth into it can replace BagCamera as is.
//
// While the backend tracks, a publisher thread streams each tracked frame (colour JPEG with the
// tracked features drawn, the depth image, camera pose, latency, tracking state) and, periodically,
// the map over WebSocket to web/index.html. Visualisation never blocks tracking: the tracker queues
// its result and moves on, and a slow browser drops frames instead. The stream is also recorded into
// OUTDIR, so the page can replay it after a reload or once the bag has ended.
//
// Each backend is its own executable: ORB-SLAM3 builds against Eigen 3, RTAB-Map's PCL against Eigen 5.
#pragma once

#include <sqlite3.h>
#include <pthread/qos.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include "ws_server.h"

using Clock = std::chrono::steady_clock;
static std::atomic<bool> g_stop{false};  // stop the camera and tracking (first Ctrl+C / SIGINT)
static std::atomic<bool> g_quit{false};  // leave the process too (second Ctrl+C, or SIGTERM)
static const double DEADLINE_MS = 1000.0 / 30.0;

static double ms_between(Clock::time_point a, Clock::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}
static int64_t wall_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count();
}

// ---------------------------------------------------------------- camera -> tracker hand-off

struct Frame {
  int idx = 0;
  double stamp = 0;             // seconds, monotonic
  Clock::time_point t_rel;      // when the camera released it
  int64_t rel_wall_ms = 0;      // same instant on the wall clock, for the browser's latency
  cv::Mat bgr, depth;           // CV_8UC3, CV_16UC1 aligned to colour (millimetres)
};

// KEEP_LAST(depth) drop-oldest, like the BEST_EFFORT topic a camera driver publishes on
class FrameSink {
 public:
  explicit FrameSink(size_t depth) : depth_(depth) {}
  void push(Frame f) {
    {
      std::lock_guard<std::mutex> l(m_);
      if (q_.size() >= depth_) { q_.pop_front(); dropped_++; }
      q_.push_back(std::move(f));
    }
    cv_.notify_one();
  }
  bool pop(Frame& f) {
    std::unique_lock<std::mutex> l(m_);
    cv_.wait(l, [&] { return !q_.empty() || closed_; });
    if (q_.empty()) return false;
    f = std::move(q_.front());
    q_.pop_front();
    return true;
  }
  void close() {
    { std::lock_guard<std::mutex> l(m_); closed_ = true; }
    cv_.notify_all();
  }
  long dropped() { std::lock_guard<std::mutex> l(m_); return dropped_; }

 private:
  size_t depth_;
  std::deque<Frame> q_;
  std::mutex m_;
  std::condition_variable cv_;
  bool closed_ = false;
  long dropped_ = 0;
};

// ---------------------------------------------------------------- bag camera (as rt_harness.cc)

static const int T_DEPTH_CI = 3, T_COLOR_CI = 5, T_DEPTH_UNITS = 30, T_DEPTH_IMG = 108,
                 T_COLOR_IMG = 111, T_COLOR_TF = 113;

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

class BagCamera {
 public:
  long pairs = 0, skew_dropped = 0, late_release = 0;
  double max_release_late_ms = 0, align_ms_mean = 0;

  bool open(const std::string& bag) {
    if (sqlite3_open_v2(bag.c_str(), &db_, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) return false;
    std::map<int, std::string> cfg;
    sqlite3_stmt* st;
    sqlite3_prepare_v2(db_, "SELECT topic_id, data FROM messages WHERE topic_id IN (3,5,30,110,113) "
                            "GROUP BY topic_id", -1, &st, nullptr);
    while (sqlite3_step(st) == SQLITE_ROW)
      cfg[sqlite3_column_int(st, 0)] = cdr_string((const unsigned char*)sqlite3_column_blob(st, 1),
                                                  sqlite3_column_bytes(st, 1));
    sqlite3_finalize(st);
    ic_ = intr(cfg[T_COLOR_CI]); idp_ = intr(cfg[T_DEPTH_CI]);
    units_ = std::stod(cfg[T_DEPTH_UNITS]);
    auto rc = csv(parse_kv(cfg[T_COLOR_TF])["rotation"]), tc = csv(parse_kv(cfg[T_COLOR_TF])["translation"]);
    // depth tf is identity (checked), so p_c = Rc p_d + tc  (ref->stream, as in rs_bag_bridge.py)
    for (int i = 0; i < 9; i++) R_[i] = rc[i];
    for (int i = 0; i < 3; i++) t_[i] = tc[i];
    rayx_.resize(idp_.w * idp_.h); rayy_.resize(idp_.w * idp_.h);
    for (int v = 0; v < idp_.h; v++)
      for (int u = 0; u < idp_.w; u++) {
        rayx_[v * idp_.w + u] = (u - idp_.cx) / idp_.fx;
        rayy_[v * idp_.w + u] = (v - idp_.cy) / idp_.fy;
      }
    return true;
  }
  int width() const { return ic_.w; }
  const Intr& color_intrinsics() const { return ic_; }
  int height() const { return ic_.h; }

  // reads ahead into a buffer, then releases at `rate` Hz into the sink until the bag ends
  void start(FrameSink& sink, double rate, int limit = 0) {
    reader_ = std::thread([this] { read_loop(); });
    pacer_ = std::thread([this, &sink, rate, limit] { pace_loop(sink, rate, limit); });
  }
  void join() {
    if (pacer_.joinable()) pacer_.join();
    if (reader_.joinable()) reader_.join();
    sqlite3_close(db_);
  }

 private:
  cv::Mat align(const Img& d) {
    cv::Mat out(ic_.h, ic_.w, CV_16UC1, cv::Scalar(0));
    std::vector<uint16_t> buf(ic_.w * ic_.h, 65535);
    const uint16_t* src = (const uint16_t*)d.data.data();
    for (int i = 0; i < idp_.w * idp_.h; i++) {
      if (!src[i]) continue;
      float z = src[i] * (float)units_, x = rayx_[i] * z, y = rayy_[i] * z;
      float xc = R_[0] * x + R_[1] * y + R_[2] * z + t_[0];
      float yc = R_[3] * x + R_[4] * y + R_[5] * z + t_[1];
      float zz = R_[6] * x + R_[7] * y + R_[8] * z + t_[2];
      if (zz <= 0) continue;
      int uu = (int)std::nearbyint(xc / zz * ic_.fx + ic_.cx), vv = (int)std::nearbyint(yc / zz * ic_.fy + ic_.cy);
      if (uu < 0 || uu >= ic_.w || vv < 0 || vv >= ic_.h) continue;
      uint16_t zi = (uint16_t)std::nearbyint(zz / units_);
      uint16_t& b = buf[vv * ic_.w + uu];
      if (zi < b) b = zi;
    }
    uint16_t* o = out.ptr<uint16_t>();
    for (int i = 0; i < ic_.w * ic_.h; i++) o[i] = buf[i] == 65535 ? 0 : buf[i];
    return out;
  }

  void read_loop() {
    sqlite3_stmt* st;
    sqlite3_prepare_v2(db_, "SELECT topic_id, timestamp, data FROM messages WHERE topic_id IN (108,111) "
                            "ORDER BY timestamp", -1, &st, nullptr);
    std::deque<Img> dbuf, cbuf;
    double align_sum = 0;
    while (!g_stop && sqlite3_step(st) == SQLITE_ROW) {
      int tid = sqlite3_column_int(st, 0);
      Img im = cdr_image((const unsigned char*)sqlite3_column_blob(st, 2), sqlite3_column_bytes(st, 2),
                         sqlite3_column_int64(st, 1));
      if (tid == T_DEPTH_IMG) { dbuf.push_back(std::move(im)); if (dbuf.size() > 8) dbuf.pop_front(); }
      else cbuf.push_back(std::move(im));
      if (dbuf.empty() || cbuf.empty() || dbuf.back().ts <= cbuf.front().ts) continue;
      Img c = std::move(cbuf.front()); cbuf.pop_front();
      const Img* best = nullptr; int64_t bd = INT64_MAX;
      for (auto& d : dbuf) { int64_t dd = std::llabs(d.ts - c.ts); if (dd < bd) { bd = dd; best = &d; } }
      if (bd > 20000000) { skew_dropped++; continue; }
      auto ta = Clock::now();
      Frame f;
      f.depth = align(*best);
      f.bgr = cv::Mat(c.h, c.w, CV_8UC3, c.data.data()).clone();
      align_sum += ms_between(ta, Clock::now());
      std::unique_lock<std::mutex> l(m_);
      cv_.wait(l, [&] { return ready_.size() < 60 || stop_reader_; });
      if (stop_reader_) break;
      ready_.push_back(std::move(f));
      pairs++;
      cv_.notify_all();
    }
    sqlite3_finalize(st);
    align_ms_mean = pairs ? align_sum / pairs : 0;
    std::lock_guard<std::mutex> l(m_);
    read_done_ = true;
    cv_.notify_all();
  }

  void pace_loop(FrameSink& sink, double rate, int limit) {
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);  // a camera driver is not App-Napped
    {  // a second of frames up front so the reader's warm-up never throttles the release clock
      std::unique_lock<std::mutex> l(m_);
      cv_.wait(l, [&] { return ready_.size() >= 30 || read_done_; });
    }
    auto t0 = Clock::now() + std::chrono::milliseconds(200);
    for (int idx = 0; !g_stop && (!limit || idx < limit); idx++) {
      Frame f;
      {
        std::unique_lock<std::mutex> l(m_);
        cv_.wait(l, [&] { return !ready_.empty() || read_done_; });
        if (ready_.empty()) break;
        f = std::move(ready_.front());
        ready_.pop_front();
        cv_.notify_all();
      }
      auto target = t0 + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(idx / rate));
      std::this_thread::sleep_until(target);
      double late = ms_between(target, Clock::now());
      if (late > 5.0) late_release++;
      max_release_late_ms = std::max(max_release_late_ms, late);
      f.idx = idx;
      f.stamp = 1000.0 + idx / rate;
      f.t_rel = Clock::now();
      f.rel_wall_ms = wall_ms();
      sink.push(std::move(f));
    }
    { std::lock_guard<std::mutex> l(m_); stop_reader_ = true; }
    cv_.notify_all();
    sink.close();
  }

  sqlite3* db_ = nullptr;
  Intr ic_{}, idp_{};
  double units_ = 0.001;
  float R_[9]{}, t_[3]{};
  std::vector<float> rayx_, rayy_;
  std::deque<Frame> ready_;
  std::mutex m_;
  std::condition_variable cv_;
  bool read_done_ = false, stop_reader_ = false;  // stop_reader_: the pacer is done, stop reading
  std::thread reader_, pacer_;
};

// ---------------------------------------------------------------- tracker results

// track time histogram in 0.05 ms bins: O(1) per frame, so live P99 costs the tracker nothing
struct LatencyHist {
  std::vector<long> bins = std::vector<long>(4000, 0);
  long n = 0, over = 0;
  double sum = 0, max = 0;
  void add(double ms) {
    bins[std::min<size_t>(bins.size() - 1, size_t(ms / 0.05))]++;
    n++; sum += ms; max = std::max(max, ms);
    if (ms > DEADLINE_MS) over++;
  }
  double pct(double p) const {
    long want = (long)std::ceil(p / 100 * n), acc = 0;
    for (size_t i = 0; i < bins.size(); i++)
      if ((acc += bins[i]) >= want && want > 0) return std::min((i + 1) * 0.05, max);  // bin upper edge, capped
    return 0;
  }
};

struct Snapshot {
  int idx, state;
  bool has_pose;
  Eigen::Matrix4f Twc;
  double stamp, track_ms, wait_ms, e2e_ms, hz, mean, p99, max, viz_ms;
  long n, over, drops;
  int64_t rel_wall_ms;
  cv::Mat bgr, depth;
  std::vector<cv::Point2f> kp;   // features matched to a map point in this frame
  std::vector<float> xyz;        // those map points, world frame
};

struct Row { int idx; double stamp, track_ms, wait_ms, e2e_ms; int state, lost; };

// Every published frame and map snapshot is also appended to OUTDIR, so the page can show the
// video (with map, pose and timing) after a reload or once the run is over, not only live.
class Recorder {
 public:
  bool open(const std::string& dir) {
    ff_ = fopen((dir + "/stream_frames.bin").c_str(), "w+b");
    mf_ = fopen((dir + "/stream_maps.bin").c_str(), "w+b");
    return ff_ && mf_;
  }
  void add_frame(int idx, const std::string& payload) {
    std::lock_guard<std::mutex> l(m_);
    if ((int)frames_.size() <= idx) frames_.resize(idx + 1, {-1, 0});
    frames_[idx] = {append(ff_, payload), payload.size()};
  }
  void add_map(int frame_idx, const std::string& payload) {
    std::lock_guard<std::mutex> l(m_);
    maps_.push_back({frame_idx, {append(mf_, payload), payload.size()}});
  }
  // the recorded frame at idx, or the closest earlier one if the publisher had to skip it
  bool frame(int idx, std::string& out) {
    std::lock_guard<std::mutex> l(m_);
    for (int i = std::min(idx, (int)frames_.size() - 1); i >= 0; i--)
      if (frames_[i].first >= 0) return read(ff_, frames_[i], out);
    return false;
  }
  bool map(size_t k, std::string& out) {
    std::lock_guard<std::mutex> l(m_);
    return k < maps_.size() && read(mf_, maps_[k].second, out);
  }
  std::vector<int> map_frames() {
    std::lock_guard<std::mutex> l(m_);
    std::vector<int> v;
    for (auto& m : maps_) v.push_back(m.first);
    return v;
  }

 private:
  using Span = std::pair<off_t, size_t>;
  static off_t append(FILE* f, const std::string& s) {
    fseeko(f, 0, SEEK_END);
    off_t off = ftello(f);
    fwrite(s.data(), 1, s.size(), f);
    fflush(f);
    return off;
  }
  static bool read(FILE* f, const Span& sp, std::string& out) {
    out.resize(sp.second);
    return pread(fileno(f), &out[0], sp.second, sp.first) == (ssize_t)sp.second;
  }
  std::mutex m_;
  FILE* ff_ = nullptr;
  FILE* mf_ = nullptr;
  std::vector<Span> frames_;
  std::vector<std::pair<int, Span>> maps_;
};


static void put_u32(std::string& s, uint32_t v) { s.append((const char*)&v, 4); }

// ---------------------------------------------------------------- the SLAM system being driven

struct TrackResult {
  int state = 0;         // ORB-SLAM3 convention: 1 initialising, 2 OK, 3 recently lost, 4 lost
  bool has_pose = false;
  Eigen::Matrix4f Twc = Eigen::Matrix4f::Identity();  // camera (x right, y down, z forward) -> world
};

// World frame for every backend: the first camera's optical frame, as ORB-SLAM3 uses, so the
// viewer draws both systems the same way round.
class SlamBackend {
 public:
  virtual ~SlamBackend() = default;
  // the per-frame work the 33.3 ms deadline applies to; the harness times exactly this call
  virtual TrackResult track(const Frame& f) = 0;
  virtual void track_time(double ms) {}
  // after timing: pixels of the features this frame used, and their world points (xyz triples)
  virtual void observations(std::vector<cv::Point2f>& kp, std::vector<float>& xyz) = 0;
  // current map as world points; rgb (3 bytes per point) may stay empty. Publisher thread.
  virtual void map(std::vector<float>& xyz, std::vector<uint8_t>& rgb) = 0;
  virtual double map_period_s() const { return 1.0; }
  // stop background work and write the system's own outputs into out
  virtual void finish(const std::string& out) = 0;
  virtual std::string summary() { return ""; }  // extra summary.txt lines
};

static std::string summary_text(const LatencyHist& h, const BagCamera& cam, long drops, double wall_s,
                                double viz_mean, double jpeg_mean, const std::vector<Row>& rows) {
  long notok = std::count_if(rows.begin(), rows.end(), [](const Row& r) { return r.state != 2; });
  long nopose = std::count_if(rows.begin(), rows.end(), [](const Row& r) { return r.lost; });
  std::vector<double> e2e;
  for (auto& r : rows) e2e.push_back(r.e2e_ms);
  std::sort(e2e.begin(), e2e.end());
  std::ostringstream o;
  o << std::fixed << std::setprecision(2);
  o << "pairs_released " << cam.pairs << "\nskew_dropped " << cam.skew_dropped << "\ntracked " << rows.size()
    << "\nqueue_dropped " << drops << "\nlate_release_gt5ms " << cam.late_release
    << "\nmax_release_late_ms " << cam.max_release_late_ms << "\nwall_s " << wall_s
    << "\neffective_hz " << rows.size() / std::max(1e-9, wall_s)
    << "\ntrack_ms mean " << h.sum / std::max(1L, h.n) << " p50 " << h.pct(50) << " p95 " << h.pct(95)
    << " p99 " << h.pct(99) << " p99.9 " << h.pct(99.9) << " max " << h.max
    << "\nover_33.33ms " << h.over << "\nrelease_to_done_ms p99 "
    << (e2e.empty() ? 0 : e2e[(size_t)(0.99 * (e2e.size() - 1))]) << " max " << (e2e.empty() ? 0 : e2e.back())
    << "\nstate_not_ok " << notok << "\nno_pose " << nopose << "\nalign_ms_mean " << cam.align_ms_mean
    << "\nviz_handoff_ms_mean " << viz_mean << "\njpeg_ms_mean " << jpeg_mean << "\n";
  return o.str();
}

// ---------------------------------------------------------------- run

struct LiveOptions {
  std::string host = "127.0.0.1", web = LIVE_WEB_DIR;
  int port = 8080, jpeg_q = 80, limit = 0;
  double rate = 30.0;
  size_t qdepth = 5;
  bool wait_browser = true, exit_at_end = false;
  std::vector<std::string> rest;  // positional arguments and anything the backend parses itself
};

static const char* LIVE_COMMON_FLAGS =
    "[--port 8080] [--host 127.0.0.1] [--rate 30] [--queue 5] [--web DIR] [--jpeg-quality 80]\n"
    "        [--limit N] [--no-wait] [--exit-at-end]";

static LiveOptions parse_live_options(int argc, char** argv) {
  LiveOptions o;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto val = [&]() { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--port") o.port = std::stoi(val());
    else if (a == "--host") o.host = val();
    else if (a == "--rate") o.rate = std::stod(val());
    else if (a == "--queue") o.qdepth = std::stoul(val());
    else if (a == "--web") o.web = val();
    else if (a == "--jpeg-quality") o.jpeg_q = std::stoi(val());
    else if (a == "--limit") o.limit = std::stoi(val());
    else if (a == "--no-wait") o.wait_browser = false;
    else if (a == "--exit-at-end") o.exit_at_end = true;
    else o.rest.push_back(a);
  }
  return o;
}

static std::string json_escape(const std::string& s) {
  std::string e;
  for (char ch : s) {
    if (ch == '\n') e += "\\n";
    else if (ch == '"' || ch == '\\') { e += '\\'; e += ch; }
    else e += ch;
  }
  return e;
}

// Runs one bag through `make()`'s backend at camera rate while serving the viewer.
// `slam` and `config` only label the run on the page.
static int run_live(const LiveOptions& opt, const std::string& bag, const std::string& out, const std::string& slam,
                    const std::string& config, const std::function<std::unique_ptr<SlamBackend>(const BagCamera&)>& make) {
  std::signal(SIGINT, [](int) {
    if (g_quit) _exit(130);
    if (g_stop) g_quit = true;
    g_stop = true;
  });
  std::signal(SIGTERM, [](int) {
    if (g_quit) _exit(143);
    g_stop = g_quit = true;
  });

  BagCamera cam;
  if (!cam.open(bag)) { std::cerr << "cannot open " << bag << "\n"; return 1; }
  const std::string bag_name = bag.substr(bag.find_last_of('/') + 1);

  // ---- state a browser needs on (re)connect
  Recorder rec;
  if (!rec.open(out)) { std::cerr << "cannot write the stream recording into " << out << "\n"; return 1; }
  std::mutex hist_m;
  std::vector<float> traj;  // per tracked frame: idx, state, has_pose, x, y, z, track_ms, e2e_ms
  std::string status = "loading";
  std::string final_summary;
  Msg last_map, last_frame;
  auto hello = [&]() {
    std::ostringstream j;
    j << "{\"type\":\"hello\",\"slam\":\"" << json_escape(slam) << "\",\"bag\":\"" << json_escape(bag_name)
      << "\",\"settings\":\"" << json_escape(config) << "\",\"rate\":" << opt.rate << ",\"queue\":" << opt.qdepth
      << ",\"width\":" << cam.width() << ",\"height\":" << cam.height() << ",\"deadline_ms\":" << DEADLINE_MS
      << ",\"status\":\"" << status << "\"}";
    return ws_frame(j.str(), false);
  };
  // every tracked frame so far (state, timing, position) plus which frames have a map snapshot
  auto history = [&]() {
    std::vector<float> t;
    { std::lock_guard<std::mutex> l(hist_m); t = traj; }
    std::ostringstream j;
    j << "{\"type\":\"history\",\"traj\":[" << std::setprecision(5);
    for (size_t i = 0; i < t.size(); i += 8) {  // [idx, state, track_ms, e2e_ms(, x, y, z)]
      j << (i ? ",[" : "[") << int(t[i]) << ',' << int(t[i + 1]) << ',' << t[i + 6] << ',' << t[i + 7];
      if (t[i + 2] > 0) j << ',' << t[i + 3] << ',' << t[i + 4] << ',' << t[i + 5];
      j << ']';
    }
    j << "],\"maps\":[";
    const auto mf = rec.map_frames();
    for (size_t k = 0; k < mf.size(); k++) j << (k ? "," : "") << mf[k];
    j << "]}";
    return ws_frame(j.str(), false);
  };
  auto end_msg = [&]() {
    return ws_frame("{\"type\":\"end\",\"summary\":\"" + json_escape(final_summary) + "\"}", false);
  };
  LiveServer server(opt.web, [&]() {
    std::vector<Msg> v;
    std::string st;
    Msg map, frame;
    {
      std::lock_guard<std::mutex> l(hist_m);
      v.push_back(hello());
      st = status;
      map = last_map;
      frame = last_frame;
    }
    v.push_back(history());
    if (map) v.push_back(map);
    if (frame) v.push_back(frame);  // so a reload shows the current image straight away
    if (st == "finished") v.push_back(end_msg());
    return v;
  });
  // GET /rec/frame/<i> and /rec/map/<k>: the recorded payloads, same layout as the live messages
  server.set_route([&](const std::string& path, std::string& body) {
    int n = 0;
    if (sscanf(path.c_str(), "/rec/frame/%d", &n) == 1) return rec.frame(n, body);
    if (sscanf(path.c_str(), "/rec/map/%d", &n) == 1) return rec.map(n, body);
    return false;
  });
  if (!server.start(opt.host, opt.port)) return 1;
  std::cout << "viewer: http://" << opt.host << ":" << opt.port << "/" << std::endl;

  std::unique_ptr<SlamBackend> backend = make(cam);
  if (!backend) return 1;
  {
    std::lock_guard<std::mutex> l(hist_m);
    status = "running";
  }
  if (opt.wait_browser) {
    std::cout << "waiting for a browser to connect before starting the camera ..." << std::endl;
    while (!g_stop && server.clients() == 0) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // let the page settle
  }
  server.broadcast(hello());
  std::cout << "camera started" << std::endl;

  // ---- publisher: every snapshot -> colour + depth JPEG, pose, stats; the map periodically.
  // It works through a queue rather than taking only the latest, so the recording is complete;
  // if it ever falls 3 s behind, the oldest snapshots are skipped and counted.
  std::mutex snap_m;
  std::condition_variable snap_cv;
  std::deque<std::shared_ptr<Snapshot>> pending;
  bool pub_done = false;
  long pub_skipped = 0;
  double jpeg_sum = 0;
  long jpeg_n = 0;
  // [u8 2, u8 has_rgb, pad2][u32 n] xyz float32 * 3n (rgb u8 * 3n)
  auto send_map = [&](const std::vector<float>& xyz, const std::vector<uint8_t>& rgb, int frame_idx) {
    const size_t n = xyz.size() / 3;
    const bool has_rgb = rgb.size() == n * 3 && n > 0;
    std::string b;
    b.reserve(8 + n * (has_rgb ? 15 : 12));
    b.push_back(2); b.push_back(has_rgb ? 1 : 0); b.append(2, '\0');
    put_u32(b, n);
    b.append((const char*)xyz.data(), n * 12);
    if (has_rgb) b.append((const char*)rgb.data(), n * 3);
    rec.add_map(frame_idx, b);
    Msg m = ws_frame(b, true);
    { std::lock_guard<std::mutex> l(hist_m); last_map = m; }
    server.broadcast(m, 2, 1);
  };
  std::thread publisher([&] {
    pthread_set_qos_class_self_np(QOS_CLASS_UTILITY, 0);  // below the tracker and camera
    auto last_map_t = Clock::now() - std::chrono::seconds(60);
    const auto map_period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(backend->map_period_s()));
    const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, opt.jpeg_q};
    std::vector<uchar> jpg, djpg;
    std::vector<float> map_xyz;
    std::vector<uint8_t> map_rgb;
    cv::Mat img, d8, dcol, dsmall;
    while (true) {
      std::shared_ptr<Snapshot> s;
      {
        std::unique_lock<std::mutex> l(snap_m);
        snap_cv.wait(l, [&] { return !pending.empty() || pub_done; });
        if (pending.empty()) return;
        s = std::move(pending.front());
        pending.pop_front();
      }
      auto tj = Clock::now();
      s->bgr.copyTo(img);
      for (auto& p : s->kp) {
        cv::rectangle(img, p - cv::Point2f(4, 4), p + cv::Point2f(4, 4), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        cv::circle(img, p, 1, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
      }
      cv::imencode(".jpg", img, jpg, params);
      // depth as fed to the tracker: 0-6 m, near = red, no depth = black, half resolution
      s->depth.convertTo(d8, CV_8U, -255.0 / 6000.0, 255.0);
      cv::applyColorMap(d8, dcol, cv::COLORMAP_TURBO);
      dcol.setTo(cv::Scalar::all(0), s->depth == 0);
      cv::resize(dcol, dsmall, cv::Size(dcol.cols / 2, dcol.rows / 2), 0, 0, cv::INTER_NEAREST);
      cv::imencode(".jpg", dsmall, djpg, params);
      jpeg_sum += ms_between(tj, Clock::now());
      jpeg_n++;

      std::ostringstream j;
      j << std::fixed << std::setprecision(3) << "{\"i\":" << s->idx << ",\"stamp\":" << s->stamp
        << ",\"state\":" << s->state << ",\"track_ms\":" << s->track_ms << ",\"wait_ms\":" << s->wait_ms
        << ",\"e2e_ms\":" << s->e2e_ms << ",\"hz\":" << s->hz << ",\"n\":" << s->n << ",\"mean\":" << s->mean
        << ",\"p99\":" << s->p99 << ",\"max\":" << s->max << ",\"over\":" << s->over << ",\"drops\":" << s->drops
        << ",\"viz_ms\":" << s->viz_ms << ",\"jpeg_ms\":" << jpeg_sum / jpeg_n << ",\"n_kp\":" << s->kp.size()
        << ",\"rel_wall_ms\":" << s->rel_wall_ms << ",\"T\":";
      if (s->has_pose) {
        j << std::setprecision(5) << '[';
        for (int r = 0; r < 3; r++)
          for (int c = 0; c < 4; c++) j << (r || c ? "," : "") << s->Twc(r, c);
        j << ']';
      } else {
        j << "null";
      }
      j << '}';
      // [u8 1, pad3][u32 json][u32 points][u32 colour jpeg][u32 depth jpeg] json, xyz float32 * 3n, jpegs
      const std::string js = j.str();
      std::string b;
      b.reserve(20 + js.size() + s->xyz.size() * 4 + jpg.size() + djpg.size());
      b.push_back(1); b.append(3, '\0');
      put_u32(b, js.size());
      put_u32(b, s->xyz.size() / 3);
      put_u32(b, jpg.size());
      put_u32(b, djpg.size());
      b += js;
      b.append((const char*)s->xyz.data(), s->xyz.size() * 4);
      b.append((const char*)jpg.data(), jpg.size());
      b.append((const char*)djpg.data(), djpg.size());
      rec.add_frame(s->idx, b);
      Msg m = ws_frame(b, true);
      { std::lock_guard<std::mutex> l(hist_m); last_frame = m; }
      server.broadcast(m, 1, 2);

      if (Clock::now() - last_map_t >= map_period) {
        last_map_t = Clock::now();
        backend->map(map_xyz, map_rgb);
        send_map(map_xyz, map_rgb, s->idx);
      }
    }
  });

  // ---- tracker
  FrameSink sink(opt.qdepth);
  cam.start(sink, opt.rate, opt.limit);
  pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
  std::vector<Row> rows;
  rows.reserve(9000);
  LatencyHist hist;
  std::deque<Clock::time_point> recent;  // completions in the last second -> live tracker rate
  double viz_ms = 0, viz_sum = 0;
  auto wall0 = Clock::now();
  Frame f;
  while (sink.pop(f)) {
    auto t0 = Clock::now();
    const TrackResult tr = backend->track(f);
    auto t1 = Clock::now();
    const double ms = ms_between(t0, t1);
    const bool nopose = !tr.has_pose;
    rows.push_back({f.idx, f.stamp, ms, ms_between(f.t_rel, t0), ms_between(f.t_rel, t1), tr.state, nopose ? 1 : 0});
    hist.add(ms);
    backend->track_time(ms);
    recent.push_back(t1);
    while (t1 - recent.front() > std::chrono::seconds(1)) recent.pop_front();

    // everything below is visualisation hand-off; its cost is measured and reported as viz_ms
    auto s = std::make_shared<Snapshot>();
    s->idx = f.idx; s->state = tr.state; s->has_pose = !nopose; s->stamp = f.stamp;
    s->track_ms = ms; s->wait_ms = rows.back().wait_ms; s->e2e_ms = rows.back().e2e_ms;
    s->hz = ms_between(wall0, t1) >= 1000 ? double(recent.size()) : 0;
    s->n = hist.n; s->mean = hist.sum / hist.n; s->max = hist.max; s->over = hist.over;
    s->p99 = hist.pct(99);
    s->drops = sink.dropped(); s->viz_ms = viz_ms; s->rel_wall_ms = f.rel_wall_ms;
    s->bgr = f.bgr;  // shared, not copied
    s->depth = f.depth;
    s->Twc = tr.Twc;
    backend->observations(s->kp, s->xyz);
    {
      std::lock_guard<std::mutex> l(hist_m);
      traj.insert(traj.end(), {float(f.idx), float(tr.state), nopose ? 0.0f : 1.0f, tr.Twc(0, 3), tr.Twc(1, 3),
                               tr.Twc(2, 3), float(ms), float(rows.back().e2e_ms)});
    }
    {
      std::lock_guard<std::mutex> l(snap_m);
      if (pending.size() >= 90) { pending.pop_front(); pub_skipped++; }
      pending.push_back(std::move(s));
    }
    snap_cv.notify_one();
    viz_ms = ms_between(t1, Clock::now());
    viz_sum += viz_ms;

    if (rows.size() % 600 == 0)
      std::cerr << "[live] " << rows.size() << " tracked, last " << std::fixed << std::setprecision(2) << ms
                << " ms, p99 " << hist.pct(99) << " ms, dropped " << sink.dropped() << ", browsers "
                << server.clients() << "\n";
  }
  const double wall_s = ms_between(wall0, Clock::now()) / 1000.0;
  cam.join();
  {
    std::lock_guard<std::mutex> l(snap_m);
    pub_done = true;
  }
  snap_cv.notify_all();
  publisher.join();

  backend->finish(out);
  std::vector<float> xyz;
  std::vector<uint8_t> rgb;
  backend->map(xyz, rgb);
  send_map(xyz, rgb, rows.empty() ? 0 : rows.back().idx);
  {
    std::ofstream pf(out + "/map_points.pcd");
    pf << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
       << "COUNT 1 1 1\nWIDTH " << xyz.size() / 3 << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << xyz.size() / 3
       << "\nDATA ascii\n" << std::fixed << std::setprecision(5);
    for (size_t i = 0; i + 2 < xyz.size(); i += 3) pf << xyz[i] << " " << xyz[i + 1] << " " << xyz[i + 2] << "\n";
  }
  {
    std::ofstream tf(out + "/TrackPerFrame.csv");
    tf << "frame,stamp,track_ms,state,no_pose,queue_wait_ms,release_to_done_ms\n" << std::fixed << std::setprecision(6);
    for (auto& r : rows)
      tf << r.idx << "," << r.stamp << "," << r.track_ms << "," << r.state << "," << r.lost << ","
         << r.wait_ms << "," << r.e2e_ms << "\n";
  }
  const std::string sum = "slam " + slam + "\n" +
                          summary_text(hist, cam, sink.dropped(), wall_s, viz_sum / std::max<size_t>(1, rows.size()),
                                       jpeg_n ? jpeg_sum / jpeg_n : 0, rows) +
                          "publisher_skipped " + std::to_string(pub_skipped) + "\n" + backend->summary();
  std::ofstream(out + "/summary.txt") << sum;
  std::cout << "==== SUMMARY ====\n" << sum << std::flush;
  {
    std::lock_guard<std::mutex> l(hist_m);
    status = "finished";
    final_summary = sum;
  }
  server.broadcast(history());  // complete, for browsers that missed frames while live
  server.broadcast(end_msg());

  if (!opt.exit_at_end && !g_quit) {
    std::cout << (g_stop ? "tracking stopped early" : "bag finished") << "; the viewer (with replay) stays up at http://"
              << opt.host << ":" << opt.port << "/ (Ctrl+C to quit)" << std::endl;
    while (!g_quit) std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));  // let the end message leave
  return 0;
}
