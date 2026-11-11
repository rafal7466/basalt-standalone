#include "bw/dataset.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

#include <opencv2/imgcodecs.hpp>

namespace fs = std::filesystem;

namespace bw {

static std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(0, item.find_first_not_of(" \t"));
    item.erase(item.find_last_not_of(" \t\r") + 1);
    out.push_back(item);
  }
  return out;
}

static bool readImageList(const fs::path& dir, std::map<int64_t, std::string>& out) {
  std::ifstream f(dir / "data.csv");
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    auto v = splitCsv(line);
    if (v.size() < 2) continue;
    out[std::stoll(v[0])] = (dir / "data" / v[1]).string();
  }
  return !out.empty();
}

bool loadEurocGroundTruth(const std::string& csv, std::vector<GtPose>& gt) {
  std::ifstream f(csv);
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    auto v = splitCsv(line);
    if (v.size() < 8) continue;
    GtPose p;
    p.t_ns = std::stoll(v[0]);
    Vec3 t(std::stod(v[1]), std::stod(v[2]), std::stod(v[3]));
    Quat q(std::stod(v[4]), std::stod(v[5]), std::stod(v[6]), std::stod(v[7]));
    p.T_w_i = SE3(q, t);
    gt.push_back(p);
  }
  return !gt.empty();
}

bool loadTumTrajectory(const std::string& path, std::vector<GtPose>& traj) {
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::stringstream ss(line);
    double t, x, y, z, qx, qy, qz, qw;
    if (!(ss >> t >> x >> y >> z >> qx >> qy >> qz >> qw)) continue;
    GtPose p;
    p.t_ns = static_cast<int64_t>(std::llround(t * 1e9));
    p.T_w_i = SE3(Quat(qw, qx, qy, qz), Vec3(x, y, z));
    traj.push_back(p);
  }
  return !traj.empty();
}

bool EurocDataset::open(const std::string& path) {
  fs::path root(path);
  if (fs::exists(root / "mav0")) root /= "mav0";
  root_ = root.string();

  std::map<int64_t, std::string> left, right;
  if (!readImageList(root / "cam0", left) || !readImageList(root / "cam1", right)) {
    std::cerr << "Cannot read cam0/cam1 data.csv in " << root_ << std::endl;
    return false;
  }
  for (const auto& kv : left) {
    auto it = right.find(kv.first);
    if (it == right.end()) continue;
    image_t_ns_.push_back(kv.first);
    left_files_.push_back(kv.second);
    right_files_.push_back(it->second);
  }

  std::ifstream f((root / "imu0" / "data.csv").string());
  if (!f) {
    std::cerr << "Cannot read imu0/data.csv" << std::endl;
    return false;
  }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    auto v = splitCsv(line);
    if (v.size() < 7) continue;
    ImuSample s;
    s.t_ns = std::stoll(v[0]);
    s.gyro = Vec3(std::stod(v[1]), std::stod(v[2]), std::stod(v[3]));
    s.accel = Vec3(std::stod(v[4]), std::stod(v[5]), std::stod(v[6]));
    imu_.push_back(s);
  }

  fs::path gt_csv = root / "state_groundtruth_estimate0" / "data.csv";
  if (fs::exists(gt_csv)) loadEurocGroundTruth(gt_csv.string(), gt_);

  std::cout << "Dataset: " << root_ << "\n  stereo frames: " << image_t_ns_.size()
            << "\n  imu samples: " << imu_.size() << "\n  gt poses: " << gt_.size()
            << std::endl;
  return !image_t_ns_.empty() && !imu_.empty();
}

bool EurocDataset::loadStereo(size_t i, cv::Mat& l, cv::Mat& r) const {
  l = cv::imread(left_files_[i], cv::IMREAD_GRAYSCALE);
  r = cv::imread(right_files_[i], cv::IMREAD_GRAYSCALE);
  return !l.empty() && !r.empty();
}

}  // namespace bw
