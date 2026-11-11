// Basalt-Win: run stereo-inertial odometry on a EuRoC sequence, save the
// trajectory and an evaluation report.
//
// usage: bw_vio --dataset <EuRoC sequence folder> [--out <dir>] [--config <file>]
//               [--calib <file>] [--max-frames N] [--name MH_01]
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "bw/eval.h"
#include "bw/optical_flow.h"
#include "bw/report.h"
#include "bw/vio_estimator.h"

#ifndef BW_SOURCE_DIR
#define BW_SOURCE_DIR "."
#endif

namespace fs = std::filesystem;
using namespace bw;

static std::string buildInfo() {
  std::ostringstream ss;
#if defined(_MSC_VER)
  ss << "MSVC " << _MSC_VER;
#elif defined(__clang__)
  ss << "clang " << __clang_major__ << "." << __clang_minor__;
#elif defined(__GNUC__)
  ss << "GCC " << __GNUC__ << "." << __GNUC_MINOR__;
#endif
#ifdef NDEBUG
  ss << ", Release";
#else
  ss << ", Debug";
#endif
#ifdef _OPENMP
  ss << ", OpenMP";
#endif
#ifdef EIGEN_VERSION_STRING
  ss << ", Eigen " << EIGEN_VERSION_STRING;
#else
  ss << ", Eigen " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION;
#endif
  ss << ", OpenCV " << CV_VERSION << ", Windows x64";
  return ss.str();
}

int main(int argc, char** argv) {
  std::string dataset_path, out_dir, name;
  std::string config_path = std::string(BW_SOURCE_DIR) + "/config/euroc_config.txt";
  std::string calib_path = std::string(BW_SOURCE_DIR) + "/config/euroc_ds_calib.txt";
  long max_frames = -1;
  bool mono = false;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--dataset") dataset_path = next();
    else if (a == "--out") out_dir = next();
    else if (a == "--config") config_path = next();
    else if (a == "--calib") calib_path = next();
    else if (a == "--max-frames") max_frames = std::stol(next());
    else if (a == "--name") name = next();
    else if (a == "--mono") mono = true;  // left camera only (monocular VIO)
    else {
      std::cerr << "usage: bw_vio --dataset <EuRoC sequence> [--out dir] [--config file] "
                   "[--calib file] [--max-frames N] [--name MH_01]\n";
      return 1;
    }
  }
  if (dataset_path.empty()) {
    std::cerr << "--dataset is required\n";
    return 1;
  }
  if (name.empty()) name = fs::path(dataset_path).filename().string();
  if (out_dir.empty()) out_dir = std::string(BW_SOURCE_DIR) + "/results/" + name;
  fs::create_directories(out_dir);

  VioConfig config;
  if (!config.load(config_path)) std::cout << "Using built-in config (no " << config_path << ")\n";
  Calibration calib;
  if (!calib.load(calib_path)) return 1;
  if (mono) calib.num_cams = 1;

  EurocDataset ds;
  if (!ds.open(dataset_path)) return 1;

  FrameToFrameOpticalFlow flow(config, calib);
  VioEstimator vio(config, calib);

  const Vec3 accel_cov = calib.discreteAccelNoiseStd().array().square();
  const Vec3 gyro_cov = calib.discreteGyroNoiseStd().array().square();
  auto calibrated = [&](ImuSample s) {
    calib.calibrateImu(s.gyro, s.accel);
    return s;
  };

  const auto& imu = ds.imu();
  size_t imu_idx = 0;
  int64_t prev_t = -1;
  std::vector<GtPose> traj;
  RunInfo info;
  info.dataset_path = fs::absolute(dataset_path).string();
  info.sequence_name = name;
  info.build_info = buildInfo();

  std::ofstream fstats((fs::path(out_dir) / "frame_stats.csv").string());
  fstats << "t_ns,of_ms,vio_ms,tracked0,tracked1,landmarks,observations,kfs,is_kf,lm_iterations,"
            "px,py,pz,bgx,bgy,bgz,bax,bay,baz\n";

  const size_t n_frames =
      max_frames > 0 ? std::min<size_t>(ds.numFrames(), size_t(max_frames)) : ds.numFrames();
  const int64_t t_first = ds.frameTime(0);
  auto wall_start = std::chrono::steady_clock::now();
  size_t num_kf = 0;

  for (size_t i = 0; i < n_frames; i++) {
    const int64_t t = ds.frameTime(i);
    cv::Mat l, r;
    if (!ds.loadStereo(i, l, r)) {
      std::cerr << "Failed to load frame " << i << std::endl;
      continue;
    }
    auto t0 = std::chrono::steady_clock::now();
    if (mono) r = cv::Mat();
    OpticalFlowResult::Ptr of = flow.processFrame(t, l, r);
    auto t1 = std::chrono::steady_clock::now();

    if (!vio.initialized()) {
      while (imu_idx < imu.size() && imu[imu_idx].t_ns < t) imu_idx++;
      if (imu_idx >= imu.size()) break;
      vio.initialize(t, calibrated(imu[imu_idx]).accel);
      vio.measure(of, nullptr);
    } else {
      const NavState& last = vio.lastState();
      auto meas = std::make_shared<ImuPreintegration>(prev_t, last.bias_gyro, last.bias_accel);
      while (imu_idx < imu.size() && imu[imu_idx].t_ns <= prev_t) imu_idx++;
      while (imu_idx < imu.size() && imu[imu_idx].t_ns <= t) {
        meas->integrate(calibrated(imu[imu_idx]), accel_cov, gyro_cov);
        imu_idx++;
      }
      if (meas->startT() + meas->dtNs() < t) {
        if (imu_idx >= imu.size()) break;
        ImuSample s = calibrated(imu[imu_idx]);
        s.t_ns = t;
        meas->integrate(s, accel_cov, gyro_cov);
      }
      vio.measure(of, meas);
    }
    auto t2 = std::chrono::steady_clock::now();
    prev_t = t;

    const NavState& st = vio.lastState();
    if (!st.T_w_i.t.allFinite()) {
      std::cerr << "Estimator diverged (NaN) at frame " << i << std::endl;
      break;
    }
    traj.push_back({t, st.T_w_i});
    const double of_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double vio_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    const VioStats& vs = vio.stats();
    if (vs.is_kf) num_kf++;
    info.frame_t_s.push_back((t - t_first) * 1e-9);
    info.of_ms.push_back(of_ms);
    info.vio_ms.push_back(vio_ms);
    info.num_landmarks.push_back(vs.num_landmarks);
    info.num_tracked.push_back(static_cast<int>(of->observations[0].size()));
    info.bias_gyro.push_back(st.bias_gyro);
    info.bias_accel.push_back(st.bias_accel);
    fstats << t << "," << of_ms << "," << vio_ms << "," << of->observations[0].size() << ","
           << of->observations[1].size() << "," << vs.num_landmarks << "," << vs.num_obs << ","
           << vs.num_kfs << "," << int(vs.is_kf) << "," << vs.lm_iterations << ","
           << st.T_w_i.t.x() << "," << st.T_w_i.t.y() << "," << st.T_w_i.t.z() << ","
           << st.bias_gyro.x() << "," << st.bias_gyro.y() << "," << st.bias_gyro.z() << ","
           << st.bias_accel.x() << "," << st.bias_accel.y() << "," << st.bias_accel.z() << "\n";

    if (i % 200 == 0 || i + 1 == n_frames) {
      std::cout << "frame " << std::setw(5) << i << "/" << n_frames << "  t=" << std::fixed
                << std::setprecision(2) << (t - t_first) * 1e-9 << "s  p=[" << std::setprecision(3)
                << st.T_w_i.t.transpose() << "]  lms=" << vs.num_landmarks
                << "  of=" << std::setprecision(1) << of_ms << "ms  vio=" << vio_ms << "ms"
                << std::endl;
    }
  }
  info.wall_time_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  info.num_frames = traj.size();
  info.num_keyframes = num_kf;

  const std::string traj_file = (fs::path(out_dir) / "trajectory_tum.txt").string();
  saveTum(traj_file, traj);
  info.trajectory_file = fs::absolute(traj_file).string();
  std::cout << "Saved trajectory: " << traj_file << " (" << traj.size() << " poses)" << std::endl;

  if (!ds.groundTruth().empty()) {
    EvalResult ev = evaluateTrajectory(traj, ds.groundTruth());
    std::cout << std::setprecision(4) << "\n===== EuRoC " << name << " evaluation =====\n"
              << "associated poses : " << ev.ate_se3.count << "\n"
              << "ATE RMSE (SE3)   : " << ev.ate_se3.rmse << " m\n"
              << "ATE RMSE (Sim3)  : " << ev.ate_sim3.rmse << " m (scale " << ev.sim3_scale << ")\n"
              << "RPE trans (1 s)  : " << ev.rpe_trans_1s.rmse << " m\n"
              << "RPE rot   (1 s)  : " << ev.rpe_rot_1s_deg.rmse << " deg\n"
              << "GT path length   : " << ev.gt_length_m << " m\n"
              << "wall time        : " << info.wall_time_s << " s\n";
    writeReport(out_dir, info, ev, traj);
    std::cout << "Report: " << fs::absolute(fs::path(out_dir) / "report.html").string() << std::endl;
  }
  return 0;
}
