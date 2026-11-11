#include "bw/eval.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>

namespace bw {

ErrorStats computeStats(std::vector<double> v) {
  ErrorStats s;
  s.count = v.size();
  if (v.empty()) return s;
  double sum = 0, sum2 = 0;
  for (double x : v) {
    sum += x;
    sum2 += x * x;
  }
  s.mean = sum / v.size();
  s.rmse = std::sqrt(sum2 / v.size());
  s.std = std::sqrt(std::max(0.0, sum2 / v.size() - s.mean * s.mean));
  std::sort(v.begin(), v.end());
  s.min = v.front();
  s.max = v.back();
  s.median = v.size() % 2 ? v[v.size() / 2] : 0.5 * (v[v.size() / 2 - 1] + v[v.size() / 2]);
  return s;
}

bool interpolateGt(const std::vector<GtPose>& gt, int64_t t, SE3& T, int64_t max_gap_ns) {
  auto it = std::lower_bound(gt.begin(), gt.end(), t,
                             [](const GtPose& p, int64_t tt) { return p.t_ns < tt; });
  if (it == gt.end()) return false;
  if (it->t_ns == t) {
    T = it->T_w_i;
    return true;
  }
  if (it == gt.begin()) return false;
  auto prev = std::prev(it);
  if (it->t_ns - prev->t_ns > max_gap_ns) return false;
  const double a = double(t - prev->t_ns) / double(it->t_ns - prev->t_ns);
  T.t = (1 - a) * prev->T_w_i.t + a * it->T_w_i.t;
  T.R = prev->T_w_i.q().slerp(a, it->T_w_i.q()).toRotationMatrix();
  return true;
}

// Umeyama alignment: finds s, R, t minimizing || gt - (s R est + t) ||
static void umeyama(const std::vector<Vec3>& est, const std::vector<Vec3>& gt, bool with_scale,
                    Mat3& R, Vec3& t, double& s) {
  const size_t n = est.size();
  Vec3 me = Vec3::Zero(), mg = Vec3::Zero();
  for (size_t i = 0; i < n; i++) {
    me += est[i];
    mg += gt[i];
  }
  me /= double(n);
  mg /= double(n);
  Mat3 C = Mat3::Zero();
  double var_e = 0;
  for (size_t i = 0; i < n; i++) {
    C += (gt[i] - mg) * (est[i] - me).transpose();
    var_e += (est[i] - me).squaredNorm();
  }
  C /= double(n);
  var_e /= double(n);
  Eigen::JacobiSVD<Mat3> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat3 S = Mat3::Identity();
  if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0) S(2, 2) = -1;
  R = svd.matrixU() * S * svd.matrixV().transpose();
  s = with_scale ? (svd.singularValues().asDiagonal() * S).trace() / var_e : 1.0;
  t = mg - s * R * me;
}

EvalResult evaluateTrajectory(const std::vector<GtPose>& est, const std::vector<GtPose>& gt) {
  EvalResult r;
  std::vector<Vec3> pe, pg;
  std::vector<SE3> Te, Tg;
  for (const auto& e : est) {
    SE3 T;
    if (!interpolateGt(gt, e.t_ns, T)) continue;
    r.t_ns.push_back(e.t_ns);
    pe.push_back(e.T_w_i.t);
    pg.push_back(T.t);
    Te.push_back(e.T_w_i);
    Tg.push_back(T);
  }
  if (pe.size() < 3) return r;

  Mat3 R;
  Vec3 t;
  double s;
  umeyama(pe, pg, false, R, t, s);
  r.T_gt_est = SE3(R, t);
  std::vector<double> err;
  for (size_t i = 0; i < pe.size(); i++) {
    const Vec3 a = R * pe[i] + t;
    r.p_est_aligned.push_back(a);
    r.p_gt.push_back(pg[i]);
    err.push_back((a - pg[i]).norm());
  }
  r.ate_err = err;
  r.ate_se3 = computeStats(err);

  umeyama(pe, pg, true, R, t, s);
  r.sim3_scale = s;
  std::vector<double> err_s;
  for (size_t i = 0; i < pe.size(); i++) err_s.push_back((s * R * pe[i] + t - pg[i]).norm());
  r.ate_sim3 = computeStats(err_s);

  for (size_t i = 1; i < pg.size(); i++) r.gt_length_m += (pg[i] - pg[i - 1]).norm();
  r.duration_s = (r.t_ns.back() - r.t_ns.front()) * 1e-9;

  // RPE over 1 s
  std::vector<double> rt, rr;
  size_t j = 0;
  for (size_t i = 0; i < Te.size(); i++) {
    const int64_t target = r.t_ns[i] + 1000000000LL;
    while (j < Te.size() && r.t_ns[j] < target) j++;
    if (j >= Te.size()) break;
    if (r.t_ns[j] - target > 50000000LL) continue;
    const SE3 dg = Tg[i].inverse() * Tg[j];
    const SE3 de = Te[i].inverse() * Te[j];
    const SE3 E = dg.inverse() * de;
    rt.push_back(E.t.norm());
    rr.push_back(std::abs(Eigen::AngleAxisd(E.R).angle()) * 180.0 / M_PI);
  }
  r.rpe_trans_1s = computeStats(rt);
  r.rpe_rot_1s_deg = computeStats(rr);
  return r;
}

void saveTum(const std::string& path, const std::vector<GtPose>& traj) {
  std::ofstream f(path);
  f << "# timestamp tx ty tz qx qy qz qw\n" << std::fixed;
  for (const auto& p : traj) {
    const Quat q = p.T_w_i.q();
    f << std::setprecision(9) << p.t_ns * 1e-9 << " " << p.T_w_i.t.x() << " " << p.T_w_i.t.y()
      << " " << p.T_w_i.t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
      << "\n";
  }
}

}  // namespace bw
