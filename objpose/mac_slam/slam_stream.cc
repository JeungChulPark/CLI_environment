// slam_stream — ROS-free ORB-SLAM3 RGB-D runner for RAW RealSense SDK sessions.
//
// Reads <session>/*.db3 (librealsense rosbag2, /device_0/... topics) and
// <session>/rgbd_timestamp_associations.json directly, aligns depth to the colour
// frame (exact port of align() in sam6d_realtime/data/convert_recording.py), and runs
// ORB-SLAM3 RGB-D.
//
//   --mode offline --out traj.txt     all frames, as fast as possible, TUM T_world_camera
//   --mode live --connect HOST:PORT   real-time paced, newline-delimited JSON poses over TCP
//
// Timestamps: t_ns = associated_host_epoch_timestamp_ns + --offset-ns.

#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <numeric>
#include <regex>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <dirent.h>
#include <signal.h>
#include <climits>
#include <cstdlib>
#include <libgen.h>

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <sqlite3.h>

#include <iomanip>
#include "raw_session.h"
#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "KeyFrame.h"
#include "Tracking.h"

// Atlas/current-map id is only reachable through System's private mpAtlas. Explicit
// template instantiation is exempt from access checks, which gives a read-only accessor
// without patching ORB-SLAM3 (reports exact map switches to the hub).
template <typename Tag, typename Tag::type M>
struct PrivateAccess {
  friend typename Tag::type get(Tag) { return M; }
};
struct SystemAtlasTag {
  typedef ORB_SLAM3::Atlas* ORB_SLAM3::System::*type;
  friend type get(SystemAtlasTag);
};
template struct PrivateAccess<SystemAtlasTag, &ORB_SLAM3::System::mpAtlas>;
struct SystemTrackerTag {
  typedef ORB_SLAM3::Tracking* ORB_SLAM3::System::*type;
  friend type get(SystemTrackerTag);
};
template struct PrivateAccess<SystemTrackerTag, &ORB_SLAM3::System::mpTracker>;
struct TrackingVOTag {  // Tracking::mbVO (protected): localization fell back to visual odometry
  typedef bool ORB_SLAM3::Tracking::*type;
  friend type get(TrackingVOTag);
};
template struct PrivateAccess<TrackingVOTag, &ORB_SLAM3::Tracking::mbVO>;

using Clock = std::chrono::steady_clock;

static const char* STATE_NAMES[] = {"SYSTEM_NOT_READY", "NO_IMAGES_YET", "NOT_INITIALIZED",
                                    "OK", "RECENTLY_LOST", "LOST", "OK_KLT"};
static std::string state_name(int s) {
  int i = s + 1;
  return (i >= 0 && i < 7) ? STATE_NAMES[i] : "UNKNOWN";
}

struct Args {
  std::string session, settings, vocab, mode = "offline", out, connect;
  int features = 0;
  long long offset_ns = 0;
  double rate = 1.0;
  double pace = 0.0;  // offline: 0 = as fast as possible, R = sleep to R x real time
  int max_frames = 0;
  long long dump_idx = -1;
  std::string dump_path;
  std::string save_atlas, load_atlas;
  std::string time_source = "frame";
  std::string print_times;
  bool localization = false;
};

static void usage() {
  std::cerr << "slam_stream --session DIR --settings YAML --vocab ORBvoc.txt [--features N]\n"
               "            [--offset-ns NS] [--max-frames N] [--pace R (offline real-time pacing)]\n"
               "            [--save-atlas PATH | --load-atlas PATH --localization]   (PATH without .osa)\n"
               "            [--time-source frame|assoc (default frame)] [--print-times i,j,k]\n"
               "            --mode offline --out TUM.txt | --mode live --connect HOST:PORT [--rate R]\n";
}

// ---------------------------------------------------------------- settings override
// Template settings with Camera1.* / Camera.width/height replaced by the session's colour camera.
static std::string settings_with_camera(const std::string& path, const Session& s) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot read settings " + path);
  std::stringstream ss;
  ss << in.rdbuf();
  std::string txt = ss.str();
  const Calib& c = s.calib;
  auto set = [&](const std::string& key, const std::string& value) {
    std::regex re("(^|\\n)" + std::regex_replace(key, std::regex(R"(\.)"), R"(\.)") + ":[^\\n]*");
    if (!std::regex_search(txt, re)) throw std::runtime_error("settings template lacks " + key);
    txt = std::regex_replace(txt, re, "$1" + key + ": " + value);
  };
  char b[64];
  auto f = [&](double v, const char* fmtv) { snprintf(b, sizeof(b), fmtv, v); return std::string(b); };
  set("Camera1.fx", f(c.fxc, "%.6f")); set("Camera1.fy", f(c.fyc, "%.6f"));
  set("Camera1.cx", f(c.ppxc, "%.6f")); set("Camera1.cy", f(c.ppyc, "%.6f"));
  set("Camera1.k1", f(c.k1, "%.6f")); set("Camera1.k2", f(c.k2, "%.6f"));
  set("Camera1.p1", f(c.p1, "%.6f")); set("Camera1.p2", f(c.p2, "%.6f"));
  set("Camera1.k3", f(c.k3, "%.6f"));
  set("Camera.width", std::to_string(c.W)); set("Camera.height", std::to_string(c.H));
  txt = "# camera block generated by slam_stream from " + s.db3 + " " + CONV_CINFO + " (" + s.distortion_model + ")\n" + txt;
  // keep %YAML header on the first line
  auto pos = txt.find("%YAML:1.0");
  if (pos != std::string::npos && pos != 0) {
    txt.erase(pos, 9);
    txt = "%YAML:1.0\n" + txt;
  }
  std::string tmp = "/tmp/slam_stream_" + std::to_string(getpid()) + "_camera.yaml";
  std::ofstream(tmp) << txt;
  return tmp;
}

static std::string settings_override(const std::string& path, int features, const std::string& save_name,
                                     const std::string& load_name) {
  if (features <= 0 && save_name.empty() && load_name.empty()) return path;
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot read settings " + path);
  std::stringstream ss;
  ss << in.rdbuf();
  std::string txt = ss.str();
  if (features > 0) {
    std::regex re(R"((^|\n)ORBextractor\.nFeatures:[^\n]*)");
    if (!std::regex_search(txt, re)) throw std::runtime_error("settings has no ORBextractor.nFeatures");
    txt = std::regex_replace(txt, re, "$1ORBextractor.nFeatures: " + std::to_string(features));
  }
  // Drop any existing atlas keys, then set exactly what was asked for.
  txt = std::regex_replace(txt, std::regex(R"((^|\n)System\.(Load|Save)AtlasFromFile:[^\n]*)"), "$1");
  txt = std::regex_replace(txt, std::regex(R"((^|\n)System\.(Load|Save)AtlasToFile:[^\n]*)"), "$1");
  if (!load_name.empty()) txt += "\nSystem.LoadAtlasFromFile: \"" + load_name + "\"\n";
  if (!save_name.empty()) txt += "\nSystem.SaveAtlasToFile: \"" + save_name + "\"\n";
  std::string tmp = "/tmp/slam_stream_" + std::to_string(getpid()) + "_settings.yaml";
  std::ofstream(tmp) << txt;
  return tmp;
}

// ORB-SLAM3 reads/writes the atlas as "./" + NAME + ".osa", i.e. relative to the cwd.
// Returns the directory to chdir into and the bare name to put in the settings.
static std::pair<std::string, std::string> split_atlas_path(const std::string& p) {
  std::string copy1 = p, copy2 = p;
  std::string dir = dirname(&copy1[0]);
  std::string base = basename(&copy2[0]);
  if (base.size() > 4 && base.substr(base.size() - 4) == ".osa") base = base.substr(0, base.size() - 4);
  char buf[PATH_MAX];
  if (!realpath(dir.c_str(), buf)) throw std::runtime_error("atlas directory does not exist: " + dir);
  return {buf, base};
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
    else if (k == "--settings") a.settings = val();
    else if (k == "--vocab") a.vocab = val();
    else if (k == "--mode") a.mode = val();
    else if (k == "--out") a.out = val();
    else if (k == "--connect") a.connect = val();
    else if (k == "--features") a.features = std::stoi(val());
    else if (k == "--offset-ns") a.offset_ns = std::stoll(val());
    else if (k == "--rate") a.rate = std::stod(val());
    else if (k == "--pace") a.pace = std::stod(val());
    else if (k == "--max-frames") a.max_frames = std::stoi(val());
    else if (k == "--time-source") a.time_source = val();
    else if (k == "--print-times") a.print_times = val();
    else if (k == "--save-atlas") a.save_atlas = val();
    else if (k == "--load-atlas") a.load_atlas = val();
    else if (k == "--localization") a.localization = true;
    else if (k == "--dump-aligned") { a.dump_idx = std::stoll(val()); a.dump_path = val(); }
    else { usage(); return 2; }
  }
  if (a.session.empty() || a.settings.empty() || a.vocab.empty() ||
      (a.mode == "offline" && a.out.empty()) || (a.mode == "live" && a.connect.empty()) ||
      (a.mode != "offline" && a.mode != "live")) {
    usage();
    return 2;
  }

  Session s = load_session(a.session, a.offset_ns, a.time_source);
  fprintf(stderr, "[slam_stream] time-source=%s domain=%s offset_ns=%lld non_monotonic=%lld\n", s.time_source.c_str(),
          s.time_domain.c_str(), a.offset_ns, s.non_monotonic);
  if (s.non_monotonic)
    fprintf(stderr, "[slam_stream] WARNING: %lld frame times are not strictly increasing (association order kept)\n",
            s.non_monotonic);
  if (!a.print_times.empty()) {  // verification aid: t_ns of chosen frame_idx values, then exit
    std::stringstream ss(a.print_times);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      size_t k = std::stoull(tok);
      printf("frame_idx %zu t_ns %lld\n", k, s.frames.at(k).t_ns);
    }
    return 0;
  }
  if (a.max_frames > 0 && int(s.frames.size()) > a.max_frames) s.frames.resize(a.max_frames);
  const Calib& c = s.calib;
  fprintf(stderr, "[slam_stream] %s | %zu frames | color %dx%d fx=%.3f | depth fx=%.3f | t=(%.6f,%.6f,%.6f) | units=%g\n",
          s.db3.c_str(), s.frames.size(), c.W, c.H, c.fxc, c.fxd, c.t[0], c.t[1], c.t[2], c.depth_units);
  if (s.input_kind == "converted")
    fprintf(stderr, "[slam_stream] converted bag: %zu pairs (inexact %lld, max dt %.3f ms, unpaired colour %lld depth %lld) "
                    "K fx=%.3f fy=%.3f cx=%.3f cy=%.3f D=[%.6f %.6f %.6f %.6f %.6f] %s, depth pre-aligned\n",
            s.frames.size(), s.inexact_pairs, s.max_pair_dt_ms, s.unpaired_color, s.unpaired_depth, c.fxc, c.fyc, c.ppxc,
            c.ppyc, c.k1, c.k2, c.p1, c.p2, c.k3, s.distortion_model.c_str());
  Aligner aligner(c);
  if (a.dump_idx >= 0) {  // verification aid: raw uint16 aligned depth of one frame, no SLAM
    Bag dbag(s.db3);
    Blob b;
    const FrameRef& f = s.frames.at(size_t(a.dump_idx));
    if (!dbag.by_id(f.depth_id, b)) throw std::runtime_error("missing depth blob");
    ImageView di = read_image_msg(b.d);
    std::vector<uint16_t> raw(c.W * c.H);
    std::memcpy(raw.data(), di.data, raw.size() * 2);
    cv::Mat out;
    depth_to_color(s, aligner, raw.data(), out);
    std::ofstream(a.dump_path, std::ios::binary).write((const char*)out.data, out.total() * 2);
    fprintf(stderr, "[slam_stream] dumped aligned depth of frame %lld (t_ns %lld) to %s\n", a.dump_idx, f.t_ns,
            a.dump_path.c_str());
    return 0;
  }
  if (a.localization && a.load_atlas.empty()) throw std::runtime_error("--localization requires --load-atlas");
  if (!a.load_atlas.empty() && !a.save_atlas.empty()) throw std::runtime_error("use --save-atlas or --load-atlas, not both");
  if (a.localization) a.save_atlas.clear();  // never overwrite the prior map
  a.out = absolute_path(a.out);
  a.vocab = absolute_path(a.vocab);
  std::string atlas_dir, save_name, load_name;
  if (!a.save_atlas.empty()) std::tie(atlas_dir, save_name) = split_atlas_path(a.save_atlas);
  if (!a.load_atlas.empty()) {
    std::tie(atlas_dir, load_name) = split_atlas_path(a.load_atlas);
    if (access((atlas_dir + "/" + load_name + ".osa").c_str(), R_OK) != 0)
      throw std::runtime_error("atlas not found: " + atlas_dir + "/" + load_name + ".osa");
  }
  std::string base_settings = absolute_path(a.settings);
  std::string cam_settings;
  if (s.input_kind == "converted") {
    // Converted bags carry their own calibration: rewrite the template's Camera1.* from camera_info.
    cam_settings = settings_with_camera(base_settings, s);
    base_settings = cam_settings;
  }
  std::string settings = settings_override(base_settings, a.features, save_name, load_name);
  if (!atlas_dir.empty()) {
    if (chdir(atlas_dir.c_str()) != 0) throw std::runtime_error("chdir failed: " + atlas_dir);
    fprintf(stderr, "[slam_stream] atlas %s: %s/%s.osa\n", load_name.empty() ? "save" : "load", atlas_dir.c_str(),
            (load_name.empty() ? save_name : load_name).c_str());
  }

  Bag bag(s.db3);
  ORB_SLAM3::System slam(a.vocab, settings, ORB_SLAM3::System::RGBD, false);
  if (a.localization) {
    slam.ActivateLocalizationMode();  // as rgbd_node: no LocalMapping/LoopClosing, prior map fixed
    fprintf(stderr, "[slam_stream] LOCALIZATION mode on loaded atlas\n");
  }
  ORB_SLAM3::Tracking* tracker = slam.*get(SystemTrackerTag());

  auto current_map_id = [&]() -> long long {
    ORB_SLAM3::Atlas* atlas = slam.*get(SystemAtlasTag());
    ORB_SLAM3::Map* m = atlas ? atlas->GetCurrentMap() : nullptr;
    return m ? (long long)m->GetId() : -1;
  };

  // Decodes, aligns and tracks one frame. Returns false if the blobs are missing.
  Blob cb, db;
  cv::Mat color, depth_aligned;
  struct Result {
    int state; bool ok; Sophus::SE3f Twc; double track_ms, prep_ms; long long map_id; bool map_changed;
    int n_tracked_mp; bool vo;
  };
  auto process = [&](const FrameRef& f, Result& r) -> bool {
    auto t0 = Clock::now();
    if (!bag.by_id(f.color_id, cb) || !bag.by_id(f.depth_id, db)) return false;
    ImageView ci = read_image_msg(cb.d), di = read_image_msg(db.d);
    if (int(ci.w) != c.W || int(ci.h) != c.H || int(di.w) != c.W || int(di.h) != c.H ||
        ci.len != ci.w * ci.h * 3 || di.len != di.w * di.h * 2)
      throw std::runtime_error("unexpected image geometry/size");
    if (ci.enc == "bgr8") {
      color = cv::Mat(c.H, c.W, CV_8UC3, (void*)ci.data).clone();
    } else if (ci.enc == "rgb8") {  // match rgbd_node: hand ORB-SLAM3 BGR
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
    depth_to_color(s, aligner, raw.data(), depth_aligned);
    auto t1 = Clock::now();
    Sophus::SE3f Tcw = slam.TrackRGBD(color, depth_aligned, double(f.t_ns) * 1e-9);
    auto t2 = Clock::now();
    r.state = slam.GetTrackingState();
    r.ok = (r.state == 2) && !Tcw.matrix().isZero(0);
    r.Twc = Tcw.inverse();
    r.prep_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.track_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    r.map_id = current_map_id();
    r.map_changed = slam.MapChanged();
    std::vector<ORB_SLAM3::MapPoint*> tracked = slam.GetTrackedMapPoints();
    r.n_tracked_mp = int(std::count_if(tracked.begin(), tracked.end(), [](ORB_SLAM3::MapPoint* m) { return m != nullptr; }));
    r.vo = tracker ? tracker->*get(TrackingVOTag()) : false;
    return true;
  };

  std::vector<double> track_ms, prep_ms;
  std::map<std::string, int> states;
  int processed = 0, ok_count = 0, dropped = 0;
  auto wall0 = Clock::now();

  if (a.mode == "offline") {
    std::ofstream tum(a.out);
    tum.setf(std::ios::fixed);
    std::ofstream fcsv(a.out + ".frames.csv");
    fcsv << "t_ns,state,ok,map_id,track_ms,prep_ms,map_changed,n_tracked_mp,vo_only,tx,ty,tz,qx,qy,qz,qw\n";
    int ok_map_count = 0;
    json loc_segments = json::array();  // runs of consecutive OK && !vo_only frames
    bool in_loc = false;
    json segments = json::array();
    long long seg_map = -2;
    for (size_t i = 0; i < s.frames.size(); ++i) {
      if (a.pace > 0) {  // keep LocalMapping in step with tracking, as in live operation
        double due_s = double(s.frames[i].t_ns - s.frames.front().t_ns) * 1e-9 / a.pace;
        double el = std::chrono::duration<double>(Clock::now() - wall0).count();
        if (due_s > el) std::this_thread::sleep_for(std::chrono::duration<double>(due_s - el));
      }
      Result r;
      if (!process(s.frames[i], r)) continue;
      ++processed;
      track_ms.push_back(r.track_ms);
      prep_ms.push_back(r.prep_ms);
      states[state_name(r.state)]++;
      fcsv << s.frames[i].t_ns << "," << state_name(r.state) << "," << int(r.ok) << "," << r.map_id << ","
           << r.track_ms << "," << r.prep_ms << "," << int(r.map_changed) << "," << r.n_tracked_mp << ","
           << int(r.vo);
      if (r.ok) {
        Eigen::Vector3f p = r.Twc.translation();
        Eigen::Quaternionf q = r.Twc.unit_quaternion();
        char buf[200];
        snprintf(buf, sizeof(buf), ",%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f", p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
        fcsv << buf << "\n";
      } else {
        fcsv << ",,,,,,,\n";
      }
      bool anchored = r.ok && !r.vo;
      if (anchored) {
        ++ok_map_count;
        if (!in_loc) loc_segments.push_back({{"first_t_ns", s.frames[i].t_ns}, {"last_t_ns", s.frames[i].t_ns}, {"frames", 0}});
        loc_segments.back()["last_t_ns"] = s.frames[i].t_ns;
        loc_segments.back()["frames"] = loc_segments.back()["frames"].get<int>() + 1;
      }
      in_loc = anchored;
      if (r.ok) {
        ++ok_count;
        Eigen::Vector3f p = r.Twc.translation();
        Eigen::Quaternionf q = r.Twc.unit_quaternion();
        tum << std::setprecision(9) << double(s.frames[i].t_ns) * 1e-9 << " " << std::setprecision(6) << p.x() << " "
            << p.y() << " " << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        if (r.map_id != seg_map) {
          segments.push_back({{"map_id", r.map_id}, {"first_t_ns", s.frames[i].t_ns}, {"last_t_ns", s.frames[i].t_ns}, {"ok_frames", 0}});
          seg_map = r.map_id;
        }
        segments.back()["last_t_ns"] = s.frames[i].t_ns;
        segments.back()["ok_frames"] = segments.back()["ok_frames"].get<int>() + 1;
      }
      if (processed % 500 == 0)
        fprintf(stderr, "[slam_stream] %d/%zu ok=%d track mean %.2f ms\n", processed, s.frames.size(), ok_count,
                std::accumulate(track_ms.begin(), track_ms.end(), 0.0) / track_ms.size());
    }
    double wall = std::chrono::duration<double>(Clock::now() - wall0).count();
    json side = {
        {"session", s.dir}, {"db3", s.db3}, {"settings", a.settings}, {"features", a.features},
        {"offset_ns", a.offset_ns}, {"input_kind", s.input_kind}, {"time_source", s.time_source}, {"time_domain", s.time_domain}, {"non_monotonic", s.non_monotonic}, {"pace", a.pace}, {"frames", s.frames.size()}, {"processed", processed},
        {"ok_frames", ok_count}, {"ok_ratio", processed ? double(ok_count) / processed : 0.0},
        {"track_ms", {{"mean", track_ms.empty() ? 0 : std::accumulate(track_ms.begin(), track_ms.end(), 0.0) / track_ms.size()},
                      {"p50", pct(track_ms, 50)}, {"p95", pct(track_ms, 95)}, {"p99", pct(track_ms, 99)},
                      {"max", track_ms.empty() ? 0 : *std::max_element(track_ms.begin(), track_ms.end())},
                      {"over_33ms", std::count_if(track_ms.begin(), track_ms.end(), [](double x) { return x > 33.333; })}}},
        {"prep_ms_mean", prep_ms.empty() ? 0 : std::accumulate(prep_ms.begin(), prep_ms.end(), 0.0) / prep_ms.size()},
        {"states", states}, {"map_segments", segments}, {"wall_s", wall},
        {"save_atlas", save_name.empty() ? "" : atlas_dir + "/" + save_name + ".osa"},
        {"load_atlas", load_name.empty() ? "" : atlas_dir + "/" + load_name + ".osa"},
        {"localization", a.localization},
        {"ok_not_vo_frames", ok_map_count},
        {"ok_not_vo_ratio", processed ? double(ok_map_count) / processed : 0.0},
        {"anchored_segments", loc_segments},
        {"trajectory", "TUM T_world_camera, state OK only, clock = time_source (+ offset_ns)"}};
    std::ofstream(a.out + ".json") << side.dump(1) << "\n";
    fprintf(stderr, "[slam_stream] done: %d processed, %d ok, wall %.1f s\n", processed, ok_count, wall);
  } else {
    Line line;
    if (!line.connect_to(a.connect)) {
      fprintf(stderr, "[slam_stream] cannot connect to %s\n", a.connect.c_str());
      return 1;
    }
    line.send_json({{"type", "hello"}, {"source", "orbslam3"}, {"features", a.features}, {"session", s.dir},
                    {"first_ns", s.frames.front().t_ns}, {"last_ns", s.frames.back().t_ns},
                    {"n_frames", s.frames.size()}, {"time_source", s.time_source}, {"time_domain", s.time_domain},
                    {"kf_updates", true}});
    json start;
    do {
      if (!line.read_json(start)) {
        fprintf(stderr, "[slam_stream] server closed before start\n");
        slam.Shutdown();
        return 1;
      }
    } while (start.value("type", "") != "start");
    long long t0_ns = start.value("t0_ns", s.frames.front().t_ns);
    double rate = start.value("rate", a.rate);
    fprintf(stderr, "[slam_stream] start t0_ns=%lld rate=%.2f\n", t0_ns, rate);

    // ---- keyframe-anchored object support -------------------------------------------
    // 6-decimal fixed formatting keeps kf_update compact (nlohmann would print 17 digits).
    auto fmt6 = [](double v) {
      char b[32];
      snprintf(b, sizeof(b), "%.6f", v);
      std::string s(b);
      if (s == "-0.000000") s = "0.000000";
      return s;
    };
    std::vector<double> kf_build_ms;
    std::vector<size_t> kf_bytes;
    long long last_t_ns = t0_ns;
    auto send_kf_update = [&](const char* reason) -> bool {
      auto tb = Clock::now();
      ORB_SLAM3::Atlas* atlas = slam.*get(SystemAtlasTag());
      ORB_SLAM3::Map* m = atlas ? atlas->GetCurrentMap() : nullptr;
      std::string body;
      int n = 0;
      std::string kfs;
      if (m) {
        std::vector<ORB_SLAM3::KeyFrame*> all = m->GetAllKeyFrames();
        std::sort(all.begin(), all.end(), [](ORB_SLAM3::KeyFrame* x, ORB_SLAM3::KeyFrame* y) { return x->mnId < y->mnId; });
        kfs.reserve(all.size() * 140);
        for (ORB_SLAM3::KeyFrame* kf : all) {
          if (!kf || kf->isBad()) continue;
          Eigen::Matrix4f T = kf->GetPoseInverse().matrix();
          if (n) kfs += ",";
          kfs += "[" + std::to_string(kf->mnId);
          for (int rr = 0; rr < 3; ++rr)
            for (int cc = 0; cc < 4; ++cc) kfs += "," + fmt6(T(rr, cc));
          kfs += "]";
          ++n;
        }
      }
      body = "{\"type\":\"kf_update\",\"t_ns\":" + std::to_string(last_t_ns) + ",\"map_id\":" +
             std::to_string(m ? (long long)m->GetId() : -1) + ",\"reason\":\"" + reason + "\",\"n\":" +
             std::to_string(n);
      double build = std::chrono::duration<double, std::milli>(Clock::now() - tb).count();
      body += ",\"build_ms\":" + fmt6(build) + ",\"kfs\":[" + kfs + "]}";
      kf_build_ms.push_back(build);
      kf_bytes.push_back(body.size() + 1);
      return line.send_raw(body);
    };
    const double kf_period_s = 1.0;
    auto last_periodic = Clock::now();
    std::vector<Clock::time_point> kf_followups;  // after map_changed: +0.5 s and +3 s
    auto service_kf_updates = [&]() -> bool {
      auto now = Clock::now();
      bool ok = true;
      bool sent_followup = false;
      for (auto it = kf_followups.begin(); it != kf_followups.end();) {
        if (*it <= now) {
          if (!sent_followup) {  // two due at once -> one message
            ok = ok && send_kf_update("map_changed");
            sent_followup = true;
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
    wall0 = Clock::now();
    last_periodic = wall0;
    bool alive = true;
    while (alive && idx < s.frames.size()) {
      if (!service_kf_updates()) { alive = false; break; }
      double elapsed = std::chrono::duration<double>(Clock::now() - wall0).count();
      double due_s = double(s.frames[idx].t_ns - t0_ns) * 1e-9 / rate;
      if (due_s > elapsed) {
        std::this_thread::sleep_for(std::chrono::duration<double>(std::min(due_s - elapsed, 0.005)));
        continue;
      }
      size_t j = idx;  // newest frame already due
      while (j + 1 < s.frames.size() && double(s.frames[j + 1].t_ns - t0_ns) * 1e-9 / rate <= elapsed) ++j;
      dropped += int(j - idx);
      Result r;
      if (process(s.frames[j], r)) {
        ++processed;
        last_t_ns = s.frames[j].t_ns;
        track_ms.push_back(r.track_ms);
        // Reference keyframe used for this frame's pose (the one SaveTrajectoryTUM anchors to),
        // read in the tracking thread right after TrackRGBD.
        ORB_SLAM3::KeyFrame* ref = tracker ? tracker->mCurrentFrame.mpReferenceKF : nullptr;
        json pose = {{"type", "pose"}, {"t_ns", s.frames[j].t_ns}, {"state", state_name(r.state)},
                     {"track_ms", std::round(r.track_ms * 100) / 100}, {"prep_ms", std::round(r.prep_ms * 100) / 100},
                     {"frame_idx", j}, {"dropped", dropped}, {"map_changed", r.map_changed}, {"map_id", r.map_id}};
        if (r.ok) {
          Eigen::Matrix4f M = r.Twc.matrix();
          std::vector<double> v;
          for (int rr = 0; rr < 4; ++rr)
            for (int cc = 0; cc < 4; ++cc) v.push_back(M(rr, cc));
          pose["T_wc"] = v;
        } else {
          pose["T_wc"] = nullptr;
        }
        std::string body = pose.dump();
        body.pop_back();  // splice fixed-precision keyframe fields before the closing brace
        if (ref) {
          Eigen::Matrix4f K = ref->GetPoseInverse().matrix();
          ORB_SLAM3::Map* km = ref->GetMap();
          body += ",\"ref_kf\":" + std::to_string(ref->mnId) + ",\"kf_map_id\":" +
                  (km ? std::to_string(km->GetId()) : std::string("null")) + ",\"T_w_kf\":[";
          for (int rr = 0; rr < 4; ++rr)
            for (int cc = 0; cc < 4; ++cc) body += (rr || cc ? "," : "") + fmt6(K(rr, cc));
          body += "]}";
        } else {
          body += ",\"ref_kf\":null,\"kf_map_id\":null,\"T_w_kf\":null}";
        }
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
    if (!kf_build_ms.empty())
      fprintf(stderr, "[slam_stream] kf_update: %zu sent, build mean %.2f p99 %.2f max %.2f ms, size mean %.0f max %zu bytes\n",
              kf_build_ms.size(), std::accumulate(kf_build_ms.begin(), kf_build_ms.end(), 0.0) / kf_build_ms.size(),
              pct(kf_build_ms, 99), *std::max_element(kf_build_ms.begin(), kf_build_ms.end()),
              std::accumulate(kf_bytes.begin(), kf_bytes.end(), 0.0) / kf_bytes.size(),
              *std::max_element(kf_bytes.begin(), kf_bytes.end()));
    fprintf(stderr, "[slam_stream] live done: processed %d dropped %d track mean %.2f p99 %.2f ms%s\n", processed,
            dropped, track_ms.empty() ? 0 : std::accumulate(track_ms.begin(), track_ms.end(), 0.0) / track_ms.size(),
            pct(track_ms, 99), alive ? "" : " (server disconnected)");
  }

  slam.Shutdown();
  if (a.mode == "offline" && !a.localization) {
    // Live per-frame poses never receive later loop-closure / GBA corrections; the
    // optimized trajectory (keyframe-anchored, after shutdown) does. Use it for RT fits.
    slam.SaveTrajectoryTUM(a.out + ".optimized.txt");
  }
  if (settings != a.settings) unlink(settings.c_str());
  if (!cam_settings.empty() && cam_settings != settings) unlink(cam_settings.c_str());
  return 0;
}
