// raw_session.h — shared by slam_stream (ORB-SLAM3) and rtab_stream (RTAB-Map).
//
// Reads a RAW RealSense SDK session (librealsense rosbag2 *.db3, /device_0/... topics, plus
// rgbd_timestamp_associations.json), gives every association its frame clock (colour metadata
// "timestamp", or the association host stamp), aligns depth to the colour frame (exact port of
// align() in sam6d_realtime/data/convert_recording.py), and provides the newline-JSON TCP line
// used by the live protocol. No SLAM or Eigen dependency, so both backends can include it even
// though they pin different Eigen versions.
#pragma once

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iterator>
#include <map>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <sqlite3.h>

using json = nlohmann::json;

// ---------------------------------------------------------------- raw bag reading
struct Blob {
  std::vector<uint8_t> d;
};

class Bag {
 public:
  explicit Bag(const std::string& path) {
    if (sqlite3_open_v2(path.c_str(), &db_, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK)
      throw std::runtime_error("cannot open " + path);
    sqlite3_prepare_v2(db_, "SELECT data FROM messages WHERE id=?", -1, &by_id_, nullptr);
  }
  ~Bag() {
    sqlite3_finalize(by_id_);
    sqlite3_close(db_);
  }
  bool by_id(long long id, Blob& out) {
    sqlite3_reset(by_id_);
    sqlite3_bind_int64(by_id_, 1, id);
    if (sqlite3_step(by_id_) != SQLITE_ROW) return false;
    const uint8_t* p = static_cast<const uint8_t*>(sqlite3_column_blob(by_id_, 0));
    int n = sqlite3_column_bytes(by_id_, 0);
    out.d.assign(p, p + n);
    return true;
  }
  bool first_of_topic(const std::string& topic, Blob& out) {
    sqlite3_stmt* st;
    sqlite3_prepare_v2(db_,
                       "SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
                       "WHERE t.name=? LIMIT 1", -1, &st, nullptr);
    sqlite3_bind_text(st, 1, topic.c_str(), -1, SQLITE_TRANSIENT);
    bool ok = sqlite3_step(st) == SQLITE_ROW;
    if (ok) {
      const uint8_t* p = static_cast<const uint8_t*>(sqlite3_column_blob(st, 0));
      out.d.assign(p, p + sqlite3_column_bytes(st, 0));
    }
    sqlite3_finalize(st);
    return ok;
  }

 private:
  sqlite3* db_ = nullptr;
  sqlite3_stmt* by_id_ = nullptr;
};

static uint32_t u32(const std::vector<uint8_t>& b, size_t off) {
  uint32_t v;
  std::memcpy(&v, b.data() + off, 4);
  return v;
}

// read_image_msg() from convert_recording.py
struct ImageView {
  uint32_t w = 0, h = 0;
  std::string enc;
  const uint8_t* data = nullptr;
  uint32_t len = 0;
};
static ImageView read_image_msg(const std::vector<uint8_t>& blob) {
  ImageView im;
  size_t off = 4 + 8;
  uint32_t flen = u32(blob, off); off += 4; off += flen; off = (off + 3) & ~size_t(3);
  im.h = u32(blob, off); im.w = u32(blob, off + 4); off += 8;
  uint32_t elen = u32(blob, off); off += 4;
  im.enc.assign(reinterpret_cast<const char*>(blob.data() + off), elen - 1); off += elen;
  off += 1; off = (off + 3) & ~size_t(3);
  off += 4;  // step
  im.len = u32(blob, off); off += 4;
  im.data = blob.data() + off;
  return im;
}

static std::string read_str_msg(const std::vector<uint8_t>& blob) {
  uint32_t slen = u32(blob, 4);
  return std::string(reinterpret_cast<const char*>(blob.data() + 8), slen - 1);
}

static std::map<std::string, std::string> parse_kv(const std::string& s) {
  std::map<std::string, std::string> d;
  std::stringstream ss(s);
  std::string kv;
  while (std::getline(ss, kv, ';')) {
    auto p = kv.find('=');
    if (p == std::string::npos) continue;
    auto trim = [](std::string x) {
      x.erase(0, x.find_first_not_of(" \t"));
      x.erase(x.find_last_not_of(" \t") + 1);
      return x;
    };
    d[trim(kv.substr(0, p))] = trim(kv.substr(p + 1));
  }
  return d;
}

static std::vector<double> parse_csv_doubles(const std::string& s) {
  std::vector<double> v;
  std::stringstream ss(s);
  std::string x;
  while (std::getline(ss, x, ',')) v.push_back(std::stod(x));
  return v;
}

// ---------------------------------------------------------------- depth alignment
struct Calib {
  int W, H;
  double fxd, fyd, ppxd, ppyd, fxc, fyc, ppxc, ppyc;
  double k1, k2, p1, p2, k3;
  double R[9], t[3];
  double depth_units;
};

class Aligner {
 public:
  explicit Aligner(const Calib& c) : c_(c) {
    xnd_.resize(c.W);
    ynd_.resize(c.H);
    for (int u = 0; u < c.W; ++u) xnd_[u] = (u - c.ppxd) / c.fxd;
    for (int v = 0; v < c.H; ++v) ynd_[v] = (v - c.ppyd) / c.fyd;
  }
  // Exact port of convert_recording.py align(): numpy rounding is half-to-even, which
  // std::nearbyint reproduces under the default FE_TONEAREST mode; the far-first
  // overwrite there is equivalent to keeping the minimum rounded mm per target pixel.
  void align(const uint16_t* depth, cv::Mat& out) const {
    const Calib& c = c_;
    out.create(c.H, c.W, CV_16UC1);
    out.setTo(0);
    uint16_t* o = out.ptr<uint16_t>();
    for (int v = 0; v < c.H; ++v) {
      for (int u = 0; u < c.W; ++u) {
        uint16_t raw = depth[v * c.W + u];
        if (raw == 0) continue;
        double Z = raw * c.depth_units;
        double X = xnd_[u] * Z, Y = ynd_[v] * Z;
        double Xc = c.R[0] * X + c.R[1] * Y + c.R[2] * Z + c.t[0];
        double Yc = c.R[3] * X + c.R[4] * Y + c.R[5] * Z + c.t[1];
        double Zc = c.R[6] * X + c.R[7] * Y + c.R[8] * Z + c.t[2];
        if (!(Zc > 0)) continue;
        double x = Xc / Zc, y = Yc / Zc;
        double r2 = x * x + y * y;
        double f = 1 + c.k1 * r2 + c.k2 * r2 * r2 + c.k3 * r2 * r2 * r2;
        double xd = x * f + 2 * c.p1 * x * y + c.p2 * (r2 + 2 * x * x);
        double yd = y * f + 2 * c.p2 * x * y + c.p1 * (r2 + 2 * y * y);
        double ucf = c.fxc * xd + c.ppxc, vcf = c.fyc * yd + c.ppyc;
        double iuf = std::nearbyint(ucf), ivf = std::nearbyint(vcf);
        if (!(iuf >= 0 && iuf < c.W && ivf >= 0 && ivf < c.H)) continue;
        int iu = int(iuf), iv = int(ivf);
        uint16_t zmm = uint16_t(int64_t(std::nearbyint(Zc * 1000.0)));
        uint16_t& dst = o[iv * c.W + iu];
        if (dst == 0 || zmm < dst) dst = zmm;
      }
    }
  }

 private:
  Calib c_;
  std::vector<double> xnd_, ynd_;
};

// ---------------------------------------------------------------- session
struct FrameRef {
  long long color_id, depth_id, t_ns;
};

struct Session {
  std::string dir, db3;
  std::vector<FrameRef> frames;  // raw: association order; converted: pair index in time order
  Calib calib;
  std::string time_source, time_domain;
  long long non_monotonic = 0;   // count of t_ns[i] <= t_ns[i-1]
  // "raw_sdk" (librealsense /device_0 topics + associations json) or "converted" (standard
  // /camera/camera/... topics, depth already aligned to colour, header stamps = frame clock).
  std::string input_kind = "raw_sdk";
  bool depth_aligned = false;
  std::string distortion_model;
  long long unpaired_color = 0, unpaired_depth = 0, inexact_pairs = 0;
  double max_pair_dt_ms = 0;
};

// Depth to colour frame: re-project raw SDK depth, or pass converted (already aligned) depth through.
static void depth_to_color(const Session& s, const Aligner& aligner, const uint16_t* raw, cv::Mat& out) {
  if (s.depth_aligned) {
    out.create(s.calib.H, s.calib.W, CV_16UC1);
    std::memcpy(out.data, raw, size_t(s.calib.W) * s.calib.H * 2);
  } else {
    aligner.align(raw, out);
  }
}

static const char* CONV_COLOR = "/camera/camera/color/image_raw";
static const char* CONV_DEPTH = "/camera/camera/aligned_depth_to_color/image_raw";
static const char* CONV_CINFO = "/camera/camera/color/camera_info";

// Minimal CDR reader (little/big endian encapsulation; alignment relative to the 4-byte header).
struct CdrReader {
  const std::vector<uint8_t>& b;
  size_t o = 4;
  bool le;
  explicit CdrReader(const std::vector<uint8_t>& blob) : b(blob) { le = blob.size() > 1 && blob[1] == 1; }
  void align(size_t n) { size_t r = (o - 4) % n; if (r) o += n - r; }
  template <typename T> T get() {
    align(sizeof(T));
    T v;
    std::memcpy(&v, b.data() + o, sizeof(T));
    if (!le) {
      uint8_t tmp[sizeof(T)];
      std::memcpy(tmp, &v, sizeof(T));
      std::reverse(tmp, tmp + sizeof(T));
      std::memcpy(&v, tmp, sizeof(T));
    }
    o += sizeof(T);
    return v;
  }
  std::string str() {
    uint32_t n = get<uint32_t>();
    std::string s(reinterpret_cast<const char*>(b.data() + o), n ? n - 1 : 0);
    o += n;
    return s;
  }
};

static bool has_topic(const std::string& db3, const char* topic) {
  sqlite3* db = nullptr;
  if (sqlite3_open_v2(db3.c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) return false;
  sqlite3_stmt* st;
  sqlite3_prepare_v2(db, "SELECT 1 FROM topics WHERE name=?", -1, &st, nullptr);
  sqlite3_bind_text(st, 1, topic, -1, SQLITE_TRANSIENT);
  bool ok = sqlite3_step(st) == SQLITE_ROW;
  sqlite3_finalize(st);
  sqlite3_close(db);
  return ok;
}

// (message id, header stamp ns) for every message of an Image topic, reading only the header bytes.
static std::vector<std::pair<long long, long long>> header_stamps(const std::string& db3, const char* topic) {
  std::vector<std::pair<long long, long long>> out;
  sqlite3* db = nullptr;
  sqlite3_open_v2(db3.c_str(), &db, SQLITE_OPEN_READONLY, nullptr);
  sqlite3_stmt* st;
  sqlite3_prepare_v2(db,
                     "SELECT m.id, substr(m.data,1,16) FROM messages m JOIN topics t ON t.id=m.topic_id "
                     "WHERE t.name=? ORDER BY m.id", -1, &st, nullptr);
  sqlite3_bind_text(st, 1, topic, -1, SQLITE_TRANSIENT);
  while (sqlite3_step(st) == SQLITE_ROW) {
    const uint8_t* p = static_cast<const uint8_t*>(sqlite3_column_blob(st, 1));
    int n = sqlite3_column_bytes(st, 1);
    if (n < 12) continue;
    std::vector<uint8_t> hb(p, p + n);
    CdrReader r(hb);
    int32_t sec = r.get<int32_t>();
    uint32_t nsec = r.get<uint32_t>();
    out.push_back({sqlite3_column_int64(st, 0), (long long)sec * 1000000000LL + nsec});
  }
  sqlite3_finalize(st);
  sqlite3_close(db);
  return out;
}

static void load_converted(Session& s, long long offset_ns) {
  s.input_kind = "converted";
  s.depth_aligned = true;
  s.time_source = "header";
  s.time_domain = "colour header stamp";
  auto colors = header_stamps(s.db3, CONV_COLOR);
  auto depths = header_stamps(s.db3, CONV_DEPTH);
  if (colors.empty() || depths.empty()) throw std::runtime_error("converted bag without colour/depth images: " + s.db3);
  std::sort(colors.begin(), colors.end(), [](auto& x, auto& y) { return x.second < y.second || (x.second == y.second && x.first < y.first); });
  std::sort(depths.begin(), depths.end(), [](auto& x, auto& y) { return x.second < y.second || (x.second == y.second && x.first < y.first); });
  std::vector<char> used(depths.size(), 0);
  const long long tol = 20000000LL;  // 20 ms
  for (auto& cimg : colors) {
    auto it = std::lower_bound(depths.begin(), depths.end(), cimg.second,
                               [](const std::pair<long long, long long>& d, long long t) { return d.second < t; });
    long long best = -1, best_dt = LLONG_MAX;
    for (auto jt : {it, it == depths.begin() ? depths.end() : std::prev(it)}) {
      if (jt == depths.end()) continue;
      size_t k = size_t(jt - depths.begin());
      if (used[k]) continue;
      long long dt = std::llabs(jt->second - cimg.second);
      if (dt < best_dt) { best_dt = dt; best = (long long)k; }
    }
    if (best < 0 || best_dt > tol) { ++s.unpaired_color; continue; }
    used[size_t(best)] = 1;
    if (best_dt) ++s.inexact_pairs;
    s.max_pair_dt_ms = std::max(s.max_pair_dt_ms, best_dt / 1e6);
    s.frames.push_back({cimg.first, depths[size_t(best)].first, cimg.second + offset_ns});
  }
  s.unpaired_depth = (long long)std::count(used.begin(), used.end(), 0);

  Bag bag(s.db3);
  Blob b;
  if (!bag.first_of_topic(CONV_CINFO, b)) throw std::runtime_error(std::string("missing ") + CONV_CINFO);
  CdrReader r(b.d);
  r.get<int32_t>(); r.get<uint32_t>(); r.str();  // header
  Calib& c = s.calib;
  c.H = int(r.get<uint32_t>());
  c.W = int(r.get<uint32_t>());
  s.distortion_model = r.str();
  uint32_t nd = r.get<uint32_t>();
  std::vector<double> d;
  for (uint32_t i = 0; i < nd; ++i) d.push_back(r.get<double>());
  double K[9];
  for (double& v : K) v = r.get<double>();
  c.fxc = K[0]; c.ppxc = K[2]; c.fyc = K[4]; c.ppyc = K[5];
  c.fxd = c.fxc; c.fyd = c.fyc; c.ppxd = c.ppxc; c.ppyd = c.ppyc;
  d.resize(5, 0.0);
  c.k1 = d[0]; c.k2 = d[1]; c.p1 = d[2]; c.p2 = d[3]; c.k3 = d[4];
  for (int i = 0; i < 9; ++i) c.R[i] = (i % 4 == 0) ? 1.0 : 0.0;
  c.t[0] = c.t[1] = c.t[2] = 0;
  c.depth_units = 0.001;  // 16UC1 millimetres
}

static Session load_session(const std::string& dir_in, long long offset_ns, const std::string& time_source) {
  Session s;
  s.dir = dir_in;
  while (s.dir.size() > 1 && s.dir.back() == '/') s.dir.pop_back();
  std::vector<std::string> cands;
  if (DIR* d = opendir(s.dir.c_str())) {
    while (dirent* e = readdir(d)) {
      std::string n = e->d_name;
      if (n.size() > 4 && n.substr(n.size() - 4) == ".db3") cands.push_back(n);
    }
    closedir(d);
  }
  if (cands.size() != 1) throw std::runtime_error("expected exactly one .db3 in " + s.dir);
  s.db3 = s.dir + "/" + cands[0];

  if (access((s.dir + "/rgbd_timestamp_associations.json").c_str(), R_OK) != 0 && has_topic(s.db3, CONV_COLOR)) {
    load_converted(s, offset_ns);
    for (size_t k = 1; k < s.frames.size(); ++k)
      if (s.frames[k].t_ns <= s.frames[k - 1].t_ns) ++s.non_monotonic;
    return s;
  }

  std::ifstream jf(s.dir + "/rgbd_timestamp_associations.json");
  if (!jf) throw std::runtime_error("missing rgbd_timestamp_associations.json in " + s.dir);
  json j = json::parse(jf);
  for (auto& a : j.at("associations")) {
    s.frames.push_back({a.at("color_message_id").get<long long>(),
                        a.at("depth_message_id").get<long long>(),
                        a.at("associated_host_epoch_timestamp_ns").get<long long>() + offset_ns});
  }
  // Frames stay in association order (frame_idx = association index); never re-sorted.
  s.time_source = time_source;
  if (time_source == "frame") {
    // Colour metadata "timestamp" (ms, float) is the per-frame clock. Metadata messages
    // are 1:1 with colour image messages in id order; map association -> colour id ->
    // position among colour ids -> metadata at that position (as objpose/pc/clock.py).
    auto topic_ids = [&](const char* topic, bool with_data, std::vector<long long>& ids,
                         std::vector<std::string>* payloads) {
      sqlite3* db = nullptr;
      sqlite3_open_v2(s.db3.c_str(), &db, SQLITE_OPEN_READONLY, nullptr);
      sqlite3_stmt* st;
      std::string q = std::string("SELECT m.id") + (with_data ? ", m.data" : "") +
                      " FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY m.id";
      sqlite3_prepare_v2(db, q.c_str(), -1, &st, nullptr);
      sqlite3_bind_text(st, 1, topic, -1, SQLITE_TRANSIENT);
      while (sqlite3_step(st) == SQLITE_ROW) {
        ids.push_back(sqlite3_column_int64(st, 0));
        if (with_data) {
          const uint8_t* p = static_cast<const uint8_t*>(sqlite3_column_blob(st, 1));
          std::vector<uint8_t> b(p, p + sqlite3_column_bytes(st, 1));
          payloads->push_back(read_str_msg(b));
        }
      }
      sqlite3_finalize(st);
      sqlite3_close(db);
    };
    std::vector<long long> color_ids, meta_ids;
    std::vector<std::string> metas;
    topic_ids("/device_0/sensor_1/Color_0/image/data", false, color_ids, nullptr);
    topic_ids("/device_0/sensor_1/Color_0/image/metadata", true, meta_ids, &metas);
    if (color_ids.size() != metas.size())
      throw std::runtime_error("colour frames (" + std::to_string(color_ids.size()) + ") != metadata messages (" +
                               std::to_string(metas.size()) + ")");
    std::map<long long, size_t> pos;
    for (size_t k = 0; k < color_ids.size(); ++k) pos[color_ids[k]] = k;
    for (auto& f : s.frames) {
      auto it = pos.find(f.color_id);
      if (it == pos.end()) throw std::runtime_error("association colour id not in bag: " + std::to_string(f.color_id));
      auto kv = parse_kv(metas[it->second]);
      if (!kv.count("timestamp")) throw std::runtime_error("metadata without timestamp: " + metas[it->second]);
      f.t_ns = (long long)std::nearbyint(std::stod(kv["timestamp"]) * 1e6) + offset_ns;
      if (s.time_domain.empty()) s.time_domain = kv.count("timestamp_domain") ? kv["timestamp_domain"] : "?";
    }
  } else if (time_source != "assoc") {
    throw std::runtime_error("--time-source must be assoc or frame");
  } else {
    s.time_domain = "associated_host_epoch";
  }
  for (size_t k = 1; k < s.frames.size(); ++k)
    if (s.frames[k].t_ns <= s.frames[k - 1].t_ns) ++s.non_monotonic;

  Bag bag(s.db3);
  Blob b;
  auto need = [&](const std::string& topic) {
    if (!bag.first_of_topic(topic, b)) throw std::runtime_error("missing topic " + topic);
    return read_str_msg(b.d);
  };
  auto dci = parse_kv(need("/device_0/sensor_0/Depth_0/camera_info"));
  auto cci = parse_kv(need("/device_0/sensor_1/Color_0/camera_info"));
  Calib& c = s.calib;
  c.fxd = std::stod(dci["fx"]); c.fyd = std::stod(dci["fy"]);
  c.ppxd = std::stod(dci["ppx"]); c.ppyd = std::stod(dci["ppy"]);
  c.fxc = std::stod(cci["fx"]); c.fyc = std::stod(cci["fy"]);
  c.ppxc = std::stod(cci["ppx"]); c.ppyc = std::stod(cci["ppy"]);
  auto kc = parse_csv_doubles(cci["coeffs"]);
  c.k1 = kc.at(0); c.k2 = kc.at(1); c.p1 = kc.at(2); c.p2 = kc.at(3); c.k3 = kc.at(4);
  c.W = std::stoi(cci["width"]); c.H = std::stoi(cci["height"]);
  std::string tf = need("/device_0/sensor_1/Color_0/tf/ref_0");
  std::vector<double> rot, trans;
  for (auto& kv : [&] {
         std::vector<std::string> parts;
         std::stringstream ss(tf);
         std::string p;
         while (std::getline(ss, p, ';')) parts.push_back(p);
         return parts;
       }()) {
    if (kv.rfind("rotation=", 0) == 0) rot = parse_csv_doubles(kv.substr(9));
    if (kv.rfind("translation=", 0) == 0) trans = parse_csv_doubles(kv.substr(12));
  }
  if (rot.size() != 9 || trans.size() != 3) throw std::runtime_error("bad tf/ref_0: " + tf);
  std::copy(rot.begin(), rot.end(), c.R);
  std::copy(trans.begin(), trans.end(), c.t);
  c.depth_units = std::stod(need("/device_0/sensor_0/option/Depth_Units/value"));
  return s;
}

static std::string absolute_path(const std::string& p) {
  if (p.empty() || p[0] == '/') return p;
  char buf[PATH_MAX];
  if (!getcwd(buf, sizeof(buf))) throw std::runtime_error("getcwd failed");
  return std::string(buf) + "/" + p;
}

// ---------------------------------------------------------------- tcp
class Line {
 public:
  bool connect_to(const std::string& hostport) {
    auto p = hostport.rfind(':');
    std::string host = hostport.substr(0, p);
    int port = std::stoi(hostport.substr(p + 1));
    fd_ = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    inet_pton(AF_INET, host.c_str(), &a.sin_addr);
    for (int i = 0; i < 100; ++i) {
      if (::connect(fd_, (sockaddr*)&a, sizeof(a)) == 0) {
        int one = 1;
        setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        return true;
      }
      close(fd_);
      fd_ = socket(AF_INET, SOCK_STREAM, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
  }
  bool send_json(const json& j) { return send_raw(j.dump()); }
  bool send_raw(const std::string& body) {
    std::string s = body + "\n";
    size_t sent = 0;
    while (sent < s.size()) {
      ssize_t n = ::send(fd_, s.data() + sent, s.size() - sent, 0);
      if (n <= 0) return false;
      sent += size_t(n);
    }
    return true;
  }
  bool read_json(json& out) {
    std::string line;
    char ch;
    while (true) {
      ssize_t n = ::recv(fd_, &ch, 1, 0);
      if (n <= 0) return false;
      if (ch == '\n') break;
      line.push_back(ch);
    }
    out = json::parse(line);
    return true;
  }
  ~Line() {
    if (fd_ >= 0) close(fd_);
  }

 private:
  int fd_ = -1;
};

// ---------------------------------------------------------------- helpers
static double pct(std::vector<double> v, double p) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  size_t i = std::min(v.size() - 1, size_t(std::ceil(p / 100.0 * v.size())) - (p > 0 ? 1 : 0));
  return v[i];
}

