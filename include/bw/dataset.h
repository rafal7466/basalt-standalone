// Basalt-Win: EuRoC (ASL) dataset reader. Works with both <seq>/mav0/... and
// a flattened <seq>/cam0, <seq>/imu0 layout.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "bw/lie.h"

namespace bw {

struct ImuSample {
  int64_t t_ns;
  Vec3 gyro;
  Vec3 accel;
};

struct GtPose {
  int64_t t_ns;
  SE3 T_w_i;
};

class EurocDataset {
 public:
  bool open(const std::string& path);

  size_t numFrames() const { return image_t_ns_.size(); }
  int64_t frameTime(size_t i) const { return image_t_ns_[i]; }
  // Loads both stereo images (8-bit grayscale)
  bool loadStereo(size_t i, cv::Mat& left, cv::Mat& right) const;

  const std::vector<ImuSample>& imu() const { return imu_; }
  const std::vector<GtPose>& groundTruth() const { return gt_; }
  const std::string& root() const { return root_; }

 private:
  std::string root_;
  std::vector<int64_t> image_t_ns_;
  std::vector<std::string> left_files_, right_files_;
  std::vector<ImuSample> imu_;
  std::vector<GtPose> gt_;
};

bool loadEurocGroundTruth(const std::string& csv, std::vector<GtPose>& gt);
bool loadTumTrajectory(const std::string& path, std::vector<GtPose>& traj);

}  // namespace bw
