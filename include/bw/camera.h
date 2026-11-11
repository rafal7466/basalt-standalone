// Basalt-Win: camera models.
//  * Double Sphere (Usenko et al. 2018) -- Basalt's EuRoC calibration
//  * Pinhole + radial-tangential distortion (k1, k2, p1, p2; OpenCV / MATLAB
//    cameraIntrinsics convention)
#pragma once
#include "bw/lie.h"

namespace bw {

struct Camera {
  enum class Type { DoubleSphere, PinholeRadtan };
  Type type = Type::DoubleSphere;
  double fx = 0, fy = 0, cx = 0, cy = 0;
  double xi = 0, alpha = 0;            // double sphere
  double k1 = 0, k2 = 0, p1 = 0, p2 = 0;  // radial-tangential
  int width = 752, height = 480;

  bool project(const Vec3& p, Vec2& uv, Eigen::Matrix<double, 2, 3>* J = nullptr) const {
    return type == Type::DoubleSphere ? projectDS(p, uv, J) : projectRT(p, uv, J);
  }
  bool unproject(const Vec2& uv, Vec3& b) const {
    return type == Type::DoubleSphere ? unprojectDS(uv, b) : unprojectRT(uv, b);
  }
  bool inImage(const Vec2& uv, double border = 0) const {
    return uv.x() >= border && uv.y() >= border && uv.x() < width - border &&
           uv.y() < height - border;
  }

  // ---------------- Double Sphere
  bool projectDS(const Vec3& p, Vec2& uv, Eigen::Matrix<double, 2, 3>* J) const {
    const double x = p.x(), y = p.y(), z = p.z();
    const double r2 = x * x + y * y;
    const double d1 = std::sqrt(r2 + z * z);
    const double k = xi * d1 + z;
    const double d2 = std::sqrt(r2 + k * k);
    const double norm = alpha * d2 + (1 - alpha) * k;
    const double w1 = alpha <= 0.5 ? alpha / (1 - alpha) : (1 - alpha) / alpha;
    const double w2 = (w1 + xi) / std::sqrt(2 * w1 * xi + xi * xi + 1);
    if (!(z > -w2 * d1) || norm < 1e-9 || d1 < 1e-9) return false;
    const double inv = 1.0 / norm;
    uv.x() = fx * x * inv + cx;
    uv.y() = fy * y * inv + cy;
    if (J) {
      const double d1x = x / d1, d1y = y / d1, d1z = z / d1;
      const double kx = xi * d1x, ky = xi * d1y, kz = xi * d1z + 1.0;
      const double d2x = (x + k * kx) / d2, d2y = (y + k * ky) / d2, d2z = (k * kz) / d2;
      const double nx = alpha * d2x + (1 - alpha) * kx;
      const double ny = alpha * d2y + (1 - alpha) * ky;
      const double nz = alpha * d2z + (1 - alpha) * kz;
      const double inv2 = inv * inv;
      (*J)(0, 0) = fx * (inv - x * nx * inv2);
      (*J)(0, 1) = -fx * x * ny * inv2;
      (*J)(0, 2) = -fx * x * nz * inv2;
      (*J)(1, 0) = -fy * y * nx * inv2;
      (*J)(1, 1) = fy * (inv - y * ny * inv2);
      (*J)(1, 2) = -fy * y * nz * inv2;
    }
    return true;
  }
  bool unprojectDS(const Vec2& uv, Vec3& b) const {
    const double mx = (uv.x() - cx) / fx;
    const double my = (uv.y() - cy) / fy;
    const double r2 = mx * mx + my * my;
    if (alpha > 0.5 && r2 >= 1.0 / (2 * alpha - 1)) return false;
    const double t = 1.0 - (2 * alpha - 1) * r2;
    if (t < 0) return false;
    const double mz = (1 - alpha * alpha * r2) / (alpha * std::sqrt(t) + 1 - alpha);
    const double mz2 = mz * mz;
    const double s = mz2 + (1 - xi * xi) * r2;
    if (s < 0) return false;
    const double k = (mz * xi + std::sqrt(s)) / (mz2 + r2);
    b = Vec3(k * mx, k * my, k * mz - xi);
    b.normalize();
    return std::isfinite(b.x());
  }

  // ---------------- Pinhole + radial-tangential
  void distort(double x, double y, double& xd, double& yd, Mat2* Jd = nullptr) const {
    const double r2 = x * x + y * y;
    const double rad = 1 + k1 * r2 + k2 * r2 * r2;
    xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    if (Jd) {
      const double drx = 2 * k1 * x + 4 * k2 * r2 * x;  // d rad / dx
      const double dry = 2 * k1 * y + 4 * k2 * r2 * y;
      (*Jd)(0, 0) = rad + x * drx + 2 * p1 * y + 6 * p2 * x;
      (*Jd)(0, 1) = x * dry + 2 * p1 * x + 2 * p2 * y;
      (*Jd)(1, 0) = y * drx + 2 * p1 * x + 2 * p2 * y;
      (*Jd)(1, 1) = rad + y * dry + 6 * p1 * y + 2 * p2 * x;
    }
  }
  bool projectRT(const Vec3& p, Vec2& uv, Eigen::Matrix<double, 2, 3>* J) const {
    if (!(p.z() > 1e-6 * p.norm()) || p.z() <= 0) return false;
    const double iz = 1.0 / p.z();
    const double x = p.x() * iz, y = p.y() * iz;
    if (x * x + y * y > 4.0) return false;  // far outside the narrow FOV
    double xd, yd;
    Mat2 Jd;
    distort(x, y, xd, yd, J ? &Jd : nullptr);
    uv.x() = fx * xd + cx;
    uv.y() = fy * yd + cy;
    if (J) {
      Eigen::Matrix<double, 2, 3> Jn;
      Jn << iz, 0, -x * iz, 0, iz, -y * iz;
      Mat2 F = Mat2::Zero();
      F(0, 0) = fx;
      F(1, 1) = fy;
      *J = F * Jd * Jn;
    }
    return true;
  }
  bool unprojectRT(const Vec2& uv, Vec3& b) const {
    const Vec2 m((uv.x() - cx) / fx, (uv.y() - cy) / fy);
    Vec2 xy = m;
    for (int it = 0; it < 20; it++) {
      double xd, yd;
      Mat2 Jd;
      distort(xy.x(), xy.y(), xd, yd, &Jd);
      const Vec2 r(xd - m.x(), yd - m.y());
      if (r.squaredNorm() < 1e-24) break;
      xy -= Jd.inverse() * r;
    }
    b = Vec3(xy.x(), xy.y(), 1.0).normalized();
    return b.allFinite();
  }
};

using DoubleSphereCamera = Camera;  // backwards compatible name

}  // namespace bw
