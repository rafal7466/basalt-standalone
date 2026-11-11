// Basalt-Win: IMU preintegration (equivalent of basalt IntegratedImuMeasurement)
// and full navigation state.
#pragma once
#include <memory>

#include "bw/dataset.h"
#include "bw/lie.h"

namespace bw {

// Increment layout for a full state (15): [dp(3), dphi(3), dv(3), dbg(3), dba(3)]
// Pose perturbation (as in Basalt PoseStateWithLin): t <- t + dp,
// R <- Exp(dphi) * R  (rotation perturbed in the world frame -> dphi_z = yaw).
constexpr int POSE_SIZE = 6;
constexpr int POSE_VEL_BIAS_SIZE = 15;

struct NavState {
  int64_t t_ns = 0;
  SE3 T_w_i;
  Vec3 vel_w_i = Vec3::Zero();
  Vec3 bias_gyro = Vec3::Zero();
  Vec3 bias_accel = Vec3::Zero();

  void applyInc(const Eigen::Matrix<double, 15, 1>& inc) {
    T_w_i.t += inc.segment<3>(0);
    T_w_i.R = expSO3(inc.segment<3>(3)) * T_w_i.R;
    vel_w_i += inc.segment<3>(6);
    bias_gyro += inc.segment<3>(9);
    bias_accel += inc.segment<3>(12);
  }
  void applyPoseInc(const Eigen::Matrix<double, 6, 1>& inc) {
    T_w_i.t += inc.segment<3>(0);
    T_w_i.R = expSO3(inc.segment<3>(3)) * T_w_i.R;
  }
};

class ImuPreintegration {
 public:
  using Ptr = std::shared_ptr<ImuPreintegration>;
  using Mat9 = Eigen::Matrix<double, 9, 9>;
  using Vec9 = Eigen::Matrix<double, 9, 1>;

  ImuPreintegration() = default;
  ImuPreintegration(int64_t start_t_ns, const Vec3& bg, const Vec3& ba)
      : start_t_ns_(start_t_ns), bg_lin_(bg), ba_lin_(ba) {}

  // Integrates a sample over (last_t, sample.t_ns] using the sample value.
  void integrate(const ImuSample& s, const Vec3& accel_cov, const Vec3& gyro_cov);

  // Predict the state at the end of the interval.
  void predictState(const NavState& s0, const Vec3& g, NavState& s1) const;

  // Residual r = [r_phi, r_v, r_p] and Jacobians wrt the two states
  // (15-dim increments each). Returns residual.
  Vec9 residual(const NavState& s0, const NavState& s1, const Vec3& g,
                Eigen::Matrix<double, 9, 15>* J0 = nullptr,
                Eigen::Matrix<double, 9, 15>* J1 = nullptr) const;

  int64_t startT() const { return start_t_ns_; }
  int64_t dtNs() const { return dt_ns_; }
  double dt() const { return dt_ns_ * 1e-9; }
  const Mat9& cov() const { return cov_; }
  const Mat3& deltaR() const { return dR_; }  // rotation i(start) -> i(end) at the linearization bias
  Mat9 info() const;

 private:
  int64_t start_t_ns_ = 0;
  int64_t dt_ns_ = 0;
  Vec3 bg_lin_ = Vec3::Zero(), ba_lin_ = Vec3::Zero();
  Mat3 dR_ = Mat3::Identity();
  Vec3 dv_ = Vec3::Zero(), dp_ = Vec3::Zero();
  Mat3 dR_dbg_ = Mat3::Zero(), dv_dbg_ = Mat3::Zero(), dv_dba_ = Mat3::Zero();
  Mat3 dp_dbg_ = Mat3::Zero(), dp_dba_ = Mat3::Zero();
  Mat9 cov_ = Mat9::Zero();  // error state ordering [phi, v, p]
};

}  // namespace bw
