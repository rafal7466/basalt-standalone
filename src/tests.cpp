// Basalt-Win: numerical Jacobian checks for camera, IMU and reprojection factors.
#include <iostream>
#include <random>

#include "bw/vio_estimator.h"

namespace bw {

struct VioTestAccess {
  static int run(const Calibration& calib) {
    VioConfig cfg;
    VioEstimator e(cfg, calib);
    std::mt19937 rng(7);
    std::normal_distribution<double> n(0, 1);
    auto rv = [&]() { return Vec3(n(rng), n(rng), n(rng)); };
    int failures = 0;
    for (int trial = 0; trial < 20; trial++) {
      NavState h, t;
      h.T_w_i = SE3(expSO3(0.1 * rv()), 0.3 * rv());
      t.T_w_i = SE3(expSO3(0.1 * rv()) * h.T_w_i.R, h.T_w_i.t + 0.2 * rv());
      e.frame_poses_.clear();
      e.frame_poses_[1] = h;
      e.frame_poses_[2] = t;
      Landmark lm;
      lm.host = TimeCamId(1, trial % 2);
      Vec3 dir_c(0.2 * n(rng), 0.2 * n(rng), 1.0);
      lm.dir = Stereographic::project(dir_c);
      lm.inv_dist = 0.3 + 0.1 * std::abs(n(rng));
      const TimeCamId tgt(2, (trial / 2) % 2);
      Vec2 r0;
      Eigen::Matrix<double, 2, 3> Jl;
      Eigen::Matrix<double, 2, 6> Jh, Jt;
      if (!e.linearizePoint(lm, tgt, Vec2::Zero(), r0, &Jl, &Jh, &Jt)) continue;
      const double eps = 1e-6;
      Eigen::Matrix<double, 2, 3> Jl_n;
      Eigen::Matrix<double, 2, 6> Jh_n, Jt_n;
      for (int k = 0; k < 3; k++) {
        Landmark l2 = lm;
        if (k < 2) l2.dir[k] += eps; else l2.inv_dist += eps;
        Vec2 r;
        e.linearizePoint(l2, tgt, Vec2::Zero(), r, nullptr, nullptr, nullptr);
        Jl_n.col(k) = (r - r0) / eps;
      }
      for (int k = 0; k < 6; k++) {
        Eigen::Matrix<double, 6, 1> inc = Eigen::Matrix<double, 6, 1>::Zero();
        inc[k] = eps;
        Vec2 r;
        e.frame_poses_[1].applyPoseInc(inc);
        e.linearizePoint(lm, tgt, Vec2::Zero(), r, nullptr, nullptr, nullptr);
        Jh_n.col(k) = (r - r0) / eps;
        e.frame_poses_[1] = h;
        e.frame_poses_[2].applyPoseInc(inc);
        e.linearizePoint(lm, tgt, Vec2::Zero(), r, nullptr, nullptr, nullptr);
        Jt_n.col(k) = (r - r0) / eps;
        e.frame_poses_[2] = t;
      }
      const double el = (Jl - Jl_n).norm() / (1 + Jl.norm());
      const double eh = (Jh - Jh_n).norm() / (1 + Jh.norm());
      const double et = (Jt - Jt_n).norm() / (1 + Jt.norm());
      if (el > 1e-4 || eh > 1e-4 || et > 1e-4) {
        std::cout << "  reprojection jac mismatch trial " << trial << ": " << el << " " << eh << " "
                  << et << "\n";
        failures++;
      }
    }
    std::cout << "[reprojection jacobians] " << (failures ? "FAIL" : "ok") << std::endl;
    return failures;
  }
};

}  // namespace bw

using namespace bw;

int main(int argc, char** argv) {
  Calibration calib;
  std::string path = argc > 1 ? argv[1] : std::string(BW_SOURCE_DIR) + "/config/euroc_ds_calib.txt";
  if (!calib.load(path)) return 1;
  int failures = 0;
  std::mt19937 rng(1);
  std::normal_distribution<double> n(0, 1);
  auto rv = [&]() { return Vec3(n(rng), n(rng), n(rng)); };

  // camera
  {
    int f = 0;
    for (int i = 0; i < 200; i++) {
      Vec3 p(0.8 * n(rng), 0.6 * n(rng), 1.0 + std::abs(n(rng)));
      Vec2 uv;
      Eigen::Matrix<double, 2, 3> J, Jn;
      if (!calib.cam[0].project(p, uv, &J)) continue;
      for (int k = 0; k < 3; k++) {
        Vec3 q = p;
        q[k] += 1e-6;
        Vec2 uv2;
        calib.cam[0].project(q, uv2);
        Jn.col(k) = (uv2 - uv) / 1e-6;
      }
      Vec3 b;
      calib.cam[0].unproject(uv, b);
      if ((J - Jn).norm() > 1e-3 * (1 + J.norm()) || (b - p.normalized()).norm() > 1e-6) f++;
    }
    std::cout << "[camera project/unproject/jacobian] " << (f ? "FAIL" : "ok") << std::endl;
    failures += f;
  }

  // pinhole + radial-tangential (EuRoC cam0 radtan calibration)
  {
    Calibration ck;
    int f = 0;
    if (ck.load(std::string(BW_SOURCE_DIR) + "/config/test_pinhole_radtan_calib.txt")) {
      for (int i = 0; i < 200; i++) {
        Vec3 p(0.15 * n(rng), 0.12 * n(rng), 1.0 + std::abs(n(rng)));
        Vec2 uv;
        Eigen::Matrix<double, 2, 3> J, Jn;
        if (!ck.cam[0].project(p, uv, &J)) continue;
        for (int k = 0; k < 3; k++) {
          Vec3 q = p;
          q[k] += 1e-6;
          Vec2 uv2;
          ck.cam[0].project(q, uv2);
          Jn.col(k) = (uv2 - uv) / 1e-6;
        }
        Vec3 b;
        ck.cam[0].unproject(uv, b);
        if ((J - Jn).norm() > 1e-3 * (1 + J.norm()) || (b - p.normalized()).norm() > 1e-8) f++;
      }
      std::cout << "[pinhole-radtan project/unproject/jacobian] " << (f ? "FAIL" : "ok") << std::endl;
      failures += f;
    }
  }

  // IMU preintegration residual Jacobians
  {
    int f = 0;
    for (int trial = 0; trial < 10; trial++) {
      NavState s0;
      s0.T_w_i = SE3(expSO3(0.5 * rv()), rv());
      s0.vel_w_i = rv();
      s0.bias_gyro = 0.01 * rv();
      s0.bias_accel = 0.1 * rv();
      ImuPreintegration pre(0, Vec3::Zero(), Vec3::Zero());
      for (int k = 1; k <= 10; k++) {
        ImuSample s{k * 5000000LL, 0.5 * rv(), Vec3(0, 0, 9.81) + rv()};
        pre.integrate(s, Vec3::Constant(1e-3), Vec3::Constant(1e-5));
      }
      const Vec3 g(0, 0, -9.81);
      NavState s1;
      pre.predictState(s0, g, s1);
      Eigen::Matrix<double, 15, 1> d;
      for (int k = 0; k < 15; k++) d[k] = 0.01 * n(rng);
      s1.applyInc(d);
      s1.bias_gyro += 0.003 * rv();
      Eigen::Matrix<double, 9, 15> J0, J1, J0n, J1n;
      const auto r0 = pre.residual(s0, s1, g, &J0, &J1);
      const double eps = 1e-7;
      for (int k = 0; k < 15; k++) {
        Eigen::Matrix<double, 15, 1> inc = Eigen::Matrix<double, 15, 1>::Zero();
        inc[k] = eps;
        NavState a = s0;
        a.applyInc(inc);
        J0n.col(k) = (pre.residual(a, s1, g) - r0) / eps;
        NavState b = s1;
        b.applyInc(inc);
        J1n.col(k) = (pre.residual(s0, b, g) - r0) / eps;
      }
      const double e0 = (J0 - J0n).norm() / (1 + J0.norm());
      const double e1 = (J1 - J1n).norm() / (1 + J1.norm());
      if (e0 > 1e-3 || e1 > 1e-3) {
        std::cout << "  imu jac mismatch " << e0 << " " << e1 << "\n";
        if (e0 > 1e-3) std::cout << "J0\n" << J0 << "\nJ0n\n" << J0n << "\n";
        f++;
      }
    }
    std::cout << "[imu residual jacobians] " << (f ? "FAIL" : "ok") << std::endl;
    failures += f;
  }

  failures += VioTestAccess::run(calib);
  std::cout << (failures ? "TESTS FAILED" : "ALL TESTS PASSED") << std::endl;
  return failures ? 1 : 0;
}
