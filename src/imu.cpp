#include "bw/imu.h"

namespace bw {

void ImuPreintegration::integrate(const ImuSample& s, const Vec3& accel_cov,
                                  const Vec3& gyro_cov) {
  const int64_t last_t = start_t_ns_ + dt_ns_;
  const double dt = (s.t_ns - last_t) * 1e-9;
  if (dt <= 0) return;

  const Vec3 w = s.gyro - bg_lin_;
  const Vec3 a = s.accel - ba_lin_;
  const Vec3 wdt = w * dt;
  const Mat3 dR_step = expSO3(wdt);
  const Mat3 Jr = rightJacobianSO3(wdt);
  const Mat3 a_hat = hat(a);

  // covariance propagation, error state [phi, v, p]
  Mat9 A = Mat9::Identity();
  A.block<3, 3>(0, 0) = dR_step.transpose();
  A.block<3, 3>(3, 0) = -dR_ * a_hat * dt;
  A.block<3, 3>(6, 0) = -0.5 * dR_ * a_hat * dt * dt;
  A.block<3, 3>(6, 3) = Mat3::Identity() * dt;
  Eigen::Matrix<double, 9, 3> Bg = Eigen::Matrix<double, 9, 3>::Zero();
  Bg.block<3, 3>(0, 0) = Jr * dt;
  Eigen::Matrix<double, 9, 3> Ba = Eigen::Matrix<double, 9, 3>::Zero();
  Ba.block<3, 3>(3, 0) = dR_ * dt;
  Ba.block<3, 3>(6, 0) = 0.5 * dR_ * dt * dt;
  cov_ = A * cov_ * A.transpose() + Bg * gyro_cov.asDiagonal() * Bg.transpose() +
         Ba * accel_cov.asDiagonal() * Ba.transpose();

  // bias Jacobians (use old dR_ and dR_dbg_)
  dp_dba_ += dv_dba_ * dt - 0.5 * dR_ * dt * dt;
  dp_dbg_ += dv_dbg_ * dt - 0.5 * dR_ * a_hat * dR_dbg_ * dt * dt;
  dv_dba_ += -dR_ * dt;
  dv_dbg_ += -dR_ * a_hat * dR_dbg_ * dt;
  dR_dbg_ = dR_step.transpose() * dR_dbg_ - Jr * dt;

  // mean
  dp_ += dv_ * dt + 0.5 * dR_ * a * dt * dt;
  dv_ += dR_ * a * dt;
  dR_ = dR_ * dR_step;
  dR_ = Quat(dR_).normalized().toRotationMatrix();

  dt_ns_ = s.t_ns - start_t_ns_;
}

ImuPreintegration::Mat9 ImuPreintegration::info() const {
  Mat9 I = Mat9::Identity();
  Mat9 c = cov_;
  c.diagonal().array() += 1e-12;
  return c.ldlt().solve(I);
}

void ImuPreintegration::predictState(const NavState& s0, const Vec3& g, NavState& s1) const {
  const double dt = this->dt();
  const Vec3 dbg = s0.bias_gyro - bg_lin_;
  const Vec3 dba = s0.bias_accel - ba_lin_;
  const Mat3 dR = dR_ * expSO3(dR_dbg_ * dbg);
  const Vec3 dv = dv_ + dv_dbg_ * dbg + dv_dba_ * dba;
  const Vec3 dp = dp_ + dp_dbg_ * dbg + dp_dba_ * dba;
  const Mat3& R0 = s0.T_w_i.R;
  s1 = s0;
  s1.t_ns = s0.t_ns + dt_ns_;
  s1.T_w_i.R = R0 * dR;
  s1.T_w_i.normalize();
  s1.T_w_i.t = s0.T_w_i.t + s0.vel_w_i * dt + 0.5 * g * dt * dt + R0 * dp;
  s1.vel_w_i = s0.vel_w_i + g * dt + R0 * dv;
}

ImuPreintegration::Vec9 ImuPreintegration::residual(const NavState& s0, const NavState& s1,
                                                    const Vec3& g,
                                                    Eigen::Matrix<double, 9, 15>* J0,
                                                    Eigen::Matrix<double, 9, 15>* J1) const {
  const double dt = this->dt();
  const Vec3 dbg = s0.bias_gyro - bg_lin_;
  const Vec3 dba = s0.bias_accel - ba_lin_;
  const Mat3 dR = dR_ * expSO3(dR_dbg_ * dbg);
  const Vec3 dv = dv_ + dv_dbg_ * dbg + dv_dba_ * dba;
  const Vec3 dp = dp_ + dp_dbg_ * dbg + dp_dba_ * dba;

  const Mat3& Ri = s0.T_w_i.R;
  const Mat3& Rj = s1.T_w_i.R;
  const Mat3 RiT = Ri.transpose();
  const Vec3 V = s1.vel_w_i - s0.vel_w_i - g * dt;
  const Vec3 P = s1.T_w_i.t - s0.T_w_i.t - s0.vel_w_i * dt - 0.5 * g * dt * dt;

  Vec9 r;
  const Vec3 r_phi = logSO3(dR.transpose() * RiT * Rj);
  r.segment<3>(0) = r_phi;
  r.segment<3>(3) = RiT * V - dv;
  r.segment<3>(6) = RiT * P - dp;

  if (J0 || J1) {
    const Mat3 Jrinv = rightJacobianInvSO3(r_phi);
    if (J0) {
      J0->setZero();
      // phi
      J0->block<3, 3>(0, 3) = -Jrinv * Rj.transpose();
      J0->block<3, 3>(0, 9) =
          -Jrinv * expSO3(r_phi).transpose() * rightJacobianSO3(dR_dbg_ * dbg) * dR_dbg_;
      // v
      J0->block<3, 3>(3, 3) = RiT * hat(V);
      J0->block<3, 3>(3, 6) = -RiT;
      J0->block<3, 3>(3, 9) = -dv_dbg_;
      J0->block<3, 3>(3, 12) = -dv_dba_;
      // p
      J0->block<3, 3>(6, 0) = -RiT;
      J0->block<3, 3>(6, 3) = RiT * hat(P);
      J0->block<3, 3>(6, 6) = -RiT * dt;
      J0->block<3, 3>(6, 9) = -dp_dbg_;
      J0->block<3, 3>(6, 12) = -dp_dba_;
    }
    if (J1) {
      J1->setZero();
      J1->block<3, 3>(0, 3) = Jrinv * Rj.transpose();
      J1->block<3, 3>(3, 6) = RiT;
      J1->block<3, 3>(6, 0) = RiT;
    }
  }
  return r;
}

}  // namespace bw
