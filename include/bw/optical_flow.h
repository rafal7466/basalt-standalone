// Basalt-Win: frame-to-frame sparse patch optical flow (port of
// basalt/optical_flow/frame_to_frame_optical_flow.h + patch.h).
// - SE(2) inverse-compositional patch alignment on an image pyramid
// - 52-point pattern (scaled by 0.5 = Basalt "pattern 51"), mean-normalized
// - forward/backward consistency check
// - one FAST corner per empty 50x50 grid cell
// - left->right stereo tracking with epipolar check
#pragma once
#include <map>
#include <memory>
#include <vector>

#include <opencv2/core.hpp>

#include "bw/config.h"

namespace bw {

using KeypointId = int;

struct OpticalFlowResult {
  using Ptr = std::shared_ptr<OpticalFlowResult>;
  int64_t t_ns = 0;
  std::map<KeypointId, Vec2> observations[2];  // per camera
};

struct ImagePyramid {
  std::vector<cv::Mat> lvl;  // CV_32F, level 0 = full resolution
  void build(const cv::Mat& gray8, int levels);
};

class FrameToFrameOpticalFlow {
 public:
  FrameToFrameOpticalFlow(const VioConfig& config, const Calibration& calib);
  // img1 may be empty (monocular). R_c1_c0 (optional, per camera): rotation from the previous
  // to the current camera frame (gyro), used as initial guess when
  // optical_flow_imu_rotation_prediction is on.
  OpticalFlowResult::Ptr processFrame(int64_t t_ns, const cv::Mat& img0, const cv::Mat& img1,
                                      const Mat3* R_c1_c0 = nullptr);

 private:
  void trackPoints(const ImagePyramid& pyr_1, const ImagePyramid& pyr_2,
                   const std::map<KeypointId, Vec2>& in, std::map<KeypointId, Vec2>& out,
                   int cam = -1, const Mat3* R_21 = nullptr) const;
  bool trackPoint(const ImagePyramid& old_pyr, const ImagePyramid& pyr, const Vec2& old_pos,
                  Vec2& pos) const;
  void addPoints();
  void filterPoints();

  VioConfig config_;
  Calibration calib_;
  Mat3 E_;  // essential matrix cam0 <- cam1 (normalized translation)
  KeypointId last_keypoint_id_ = 0;
  OpticalFlowResult::Ptr transforms_;
  std::shared_ptr<std::vector<ImagePyramid>> pyramid_, old_pyramid_;
  Eigen::Matrix<double, 2, Eigen::Dynamic> pattern_;
};

}  // namespace bw
