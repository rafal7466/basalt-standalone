// Basalt-Win: minimal SO(3)/SE(3) utilities (replacement for Sophus).
#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace bw {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat2 = Eigen::Matrix2d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using MatX = Eigen::MatrixXd;
using VecX = Eigen::VectorXd;
using Quat = Eigen::Quaterniond;

inline Mat3 hat(const Vec3& v) {
  Mat3 m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

inline Mat3 expSO3(const Vec3& w) {
  const double th = w.norm();
  if (th < 1e-10) return Mat3::Identity() + hat(w);
  return Eigen::AngleAxisd(th, w / th).toRotationMatrix();
}

inline Vec3 logSO3(const Mat3& R) {
  Eigen::AngleAxisd aa(R);
  double a = aa.angle();
  if (a > M_PI) a -= 2 * M_PI;
  return aa.axis() * a;
}

// Right Jacobian of SO(3)
inline Mat3 rightJacobianSO3(const Vec3& w) {
  const double th2 = w.squaredNorm();
  const Mat3 W = hat(w);
  if (th2 < 1e-10) return Mat3::Identity() - 0.5 * W;
  const double th = std::sqrt(th2);
  return Mat3::Identity() - (1 - std::cos(th)) / th2 * W +
         (th - std::sin(th)) / (th2 * th) * W * W;
}

inline Mat3 rightJacobianInvSO3(const Vec3& w) {
  const double th2 = w.squaredNorm();
  const Mat3 W = hat(w);
  if (th2 < 1e-10) return Mat3::Identity() + 0.5 * W;
  const double th = std::sqrt(th2);
  return Mat3::Identity() + 0.5 * W +
         (1.0 / th2 - (1 + std::cos(th)) / (2 * th * std::sin(th))) * W * W;
}

struct SE3 {
  Mat3 R = Mat3::Identity();
  Vec3 t = Vec3::Zero();

  SE3() = default;
  SE3(const Mat3& R_, const Vec3& t_) : R(R_), t(t_) {}
  SE3(const Quat& q, const Vec3& t_) : R(q.normalized().toRotationMatrix()), t(t_) {}

  SE3 inverse() const { return SE3(R.transpose(), -R.transpose() * t); }
  SE3 operator*(const SE3& o) const { return SE3(R * o.R, R * o.t + t); }
  Vec3 operator*(const Vec3& p) const { return R * p + t; }
  Quat q() const { return Quat(R).normalized(); }
  Mat4 matrix() const {
    Mat4 m = Mat4::Identity();
    m.topLeftCorner<3, 3>() = R;
    m.topRightCorner<3, 1>() = t;
    return m;
  }
  // Re-orthonormalize rotation (numerical hygiene)
  void normalize() { R = Quat(R).normalized().toRotationMatrix(); }
};

// Stereographic parametrization of bearing vectors (as in Basalt)
struct Stereographic {
  // unit bearing -> 2D
  static Vec2 project(const Vec3& p) {
    const Vec3 n = p.normalized();
    const double s = 1.0 + n.z();
    return Vec2(n.x() / s, n.y() / s);
  }
  // 2D -> unit bearing, with Jacobian d bearing / d uv (3x2)
  static Vec3 unproject(const Vec2& uv, Eigen::Matrix<double, 3, 2>* J = nullptr) {
    const double x = uv.x(), y = uv.y();
    const double n = x * x + y * y;
    const double d = 1.0 / (1.0 + n);
    Vec3 b(2 * x * d, 2 * y * d, (1 - n) * d);
    if (J) {
      const double d2 = d * d;
      (*J)(0, 0) = 2 * d - 4 * x * x * d2;
      (*J)(0, 1) = -4 * x * y * d2;
      (*J)(1, 0) = -4 * x * y * d2;
      (*J)(1, 1) = 2 * d - 4 * y * y * d2;
      (*J)(2, 0) = -2 * x * d - 2 * x * (1 - n) * d2;
      (*J)(2, 1) = -2 * y * d - 2 * y * (1 - n) * d2;
    }
    return b;
  }
};

}  // namespace bw
