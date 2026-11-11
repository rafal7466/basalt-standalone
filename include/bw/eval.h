// Basalt-Win: trajectory evaluation (ATE with SE3/Sim3 Umeyama alignment, RPE)
#pragma once
#include <string>
#include <vector>

#include "bw/dataset.h"

namespace bw {

struct ErrorStats {
  double rmse = 0, mean = 0, median = 0, std = 0, min = 0, max = 0;
  size_t count = 0;
};
ErrorStats computeStats(std::vector<double> v);

struct EvalResult {
  // associated pairs
  std::vector<int64_t> t_ns;
  std::vector<Vec3> p_est_aligned;  // SE3-aligned estimate
  std::vector<Vec3> p_gt;
  std::vector<double> ate_err;  // per pose translational error after SE3 align
  ErrorStats ate_se3, ate_sim3;
  double sim3_scale = 1;
  SE3 T_gt_est;  // SE3 alignment
  ErrorStats rpe_trans_1s, rpe_rot_1s_deg;
  double gt_length_m = 0;
  double duration_s = 0;
};

// Linear interpolation of ground truth at time t (within max_gap_ns)
bool interpolateGt(const std::vector<GtPose>& gt, int64_t t_ns, SE3& T, int64_t max_gap_ns = 20000000);

EvalResult evaluateTrajectory(const std::vector<GtPose>& est, const std::vector<GtPose>& gt);

void saveTum(const std::string& path, const std::vector<GtPose>& traj);

}  // namespace bw
