// ORB-SLAM3 RGB-D over a BOP YCB-V scene folder (rgb/NNNNNN.png, depth/NNNNNN.png, 30 fps).
// usage: ycbv_rgbd <vocab> <settings.yaml> <scene_dir> <n_frames> <out_prefix> [fps=30]
// Per-frame lines are flushed at once: ORB-SLAM3 can crash in SaveTrajectoryTUM after tracking was lost (several maps),
// which used to lose the buffered per-frame file.
#include <System.h>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cstdio>
#include <fstream>
int main(int argc, char** argv) {
  if (argc != 6 && argc != 7) { fprintf(stderr, "usage\n"); return 1; }
  double fps = argc == 7 ? atof(argv[6]) : 30.0;
  std::string scene = argv[3]; int n = atoi(argv[4]); std::string out = argv[5];
  ORB_SLAM3::System slam(argv[1], argv[2], ORB_SLAM3::System::RGBD, false);
  std::ofstream per((out + "_frames.txt").c_str());   // per-frame tracked pose (camera->world, TUM)
  std::ofstream tt((out + "_timing.txt").c_str());
  char b[32];
  for (int i = 1; i <= n; ++i) {
    snprintf(b, sizeof b, "/%06d.png", i);
    cv::Mat rgb = cv::imread(scene + "/rgb" + b, cv::IMREAD_UNCHANGED);
    cv::Mat d = cv::imread(scene + "/depth" + b, cv::IMREAD_UNCHANGED);
    if (rgb.empty() || d.empty()) { fprintf(stderr, "missing %d\n", i); continue; }
    double t = (i - 1) / fps;
    auto t0 = std::chrono::steady_clock::now();
    Sophus::SE3f Tcw = slam.TrackRGBD(rgb, d, t);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    int st = slam.GetTrackingState();
    tt << i << " " << ms << " " << st << std::endl;
    if (st == 2) {  // OK
      Sophus::SE3f Twc = Tcw.inverse();
      Eigen::Vector3f p = Twc.translation(); Eigen::Quaternionf q = Twc.unit_quaternion();
      per << std::fixed; per.precision(6);
      per << t << " " << p.x() << " " << p.y() << " " << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
  }
  per.close(); tt.close();
  slam.Shutdown();
  slam.SaveTrajectoryTUM(out + "_final.txt");        // all frames, after loop closure / BA
  slam.SaveKeyFrameTrajectoryTUM(out + "_kf.txt");
  return 0;
}
