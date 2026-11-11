// Basalt-Win: evaluate any TUM-format trajectory against EuRoC ground truth.
// usage: bw_eval <groundtruth data.csv> <trajectory_tum.txt>
#include <iomanip>
#include <iostream>

#include "bw/eval.h"

using namespace bw;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: bw_eval <state_groundtruth_estimate0/data.csv> <trajectory_tum.txt>\n";
    return 1;
  }
  std::vector<GtPose> gt, est;
  if (!loadEurocGroundTruth(argv[1], gt) || !loadTumTrajectory(argv[2], est)) {
    std::cerr << "failed to load inputs\n";
    return 1;
  }
  EvalResult ev = evaluateTrajectory(est, gt);
  std::cout << std::fixed << std::setprecision(4) << "poses " << ev.ate_se3.count
            << "\nATE_SE3_RMSE " << ev.ate_se3.rmse << "\nATE_SIM3_RMSE " << ev.ate_sim3.rmse
            << "\nSIM3_SCALE " << ev.sim3_scale << "\nRPE_TRANS_1S_RMSE " << ev.rpe_trans_1s.rmse
            << "\nRPE_ROT_1S_RMSE_DEG " << ev.rpe_rot_1s_deg.rmse << std::endl;
  return 0;
}
