#include "bw/vio_estimator.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace bw {

namespace {

using Mat23 = Eigen::Matrix<double, 2, 3>;
using Mat26 = Eigen::Matrix<double, 2, 6>;
using Mat63 = Eigen::Matrix<double, 6, 3>;
using Mat66 = Eigen::Matrix<double, 6, 6>;

// Basalt triangulation (DLT); returns [bearing in frame 0; inverse distance]
Vec4 triangulate(const Vec3& f0, const Vec3& f1, const SE3& T_0_1) {
  Eigen::Matrix<double, 3, 4> P1 = Eigen::Matrix<double, 3, 4>::Zero();
  P1.leftCols<3>().setIdentity();
  const SE3 T_1_0 = T_0_1.inverse();
  Eigen::Matrix<double, 3, 4> P2;
  P2.leftCols<3>() = T_1_0.R;
  P2.col(3) = T_1_0.t;
  Mat4 A;
  A.row(0) = f0[0] * P1.row(2) - f0[2] * P1.row(0);
  A.row(1) = f0[1] * P1.row(2) - f0[2] * P1.row(1);
  A.row(2) = f1[0] * P2.row(2) - f1[2] * P2.row(0);
  A.row(3) = f1[1] * P2.row(2) - f1[2] * P2.row(1);
  Eigen::JacobiSVD<Mat4> svd(A, Eigen::ComputeFullV);
  Vec4 v = svd.matrixV().col(3);
  if (v.head<3>().dot(f0) < 0) v = -v;
  return v / v.head<3>().norm();
}

inline double huberWeight(double e, double k) { return e < k ? 1.0 : k / e; }

}  // namespace

VioEstimator::VioEstimator(const VioConfig& config, const Calibration& calib)
    : config_(config), calib_(calib) {
  gyro_bias_weight_ = calib.gyro_bias_std.array().square().inverse();
  accel_bias_weight_ = calib.accel_bias_std.array().square().inverse();
  g_ = Vec3(0, 0, -calib.gravity);
}

void VioEstimator::initialize(int64_t t_ns, const Vec3& accel) {
  NavState s;
  s.t_ns = t_ns;
  s.T_w_i.R = Quat::FromTwoVectors(accel, Vec3::UnitZ()).toRotationMatrix();
  initialize(s);
}

void VioEstimator::initialize(const NavState& state, const Eigen::Matrix<double, 15, 1>& prior_info) {
  initialize(state);
  marg_H_.setZero();
  marg_H_.diagonal() = prior_info;
}

void VioEstimator::initialize(const NavState& s) {
  const int64_t t_ns = s.t_ns;
  frame_states_.clear();
  frame_states_[t_ns] = s;
  last_state_t_ns_ = t_ns;

  marg_order_ = AbsOrderMap();
  marg_order_.abs_order_map[t_ns] = {0, POSE_VEL_BIAS_SIZE};
  marg_order_.total_size = POSE_VEL_BIAS_SIZE;
  marg_H_.setZero(POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE);
  marg_b_.setZero(POSE_VEL_BIAS_SIZE);
  // prior on position and yaw, small prior on biases (same indices as Basalt)
  marg_H_.diagonal().head<3>().setConstant(config_.vio_init_pose_weight);
  marg_H_(5, 5) = config_.vio_init_pose_weight;
  marg_H_.diagonal().segment<3>(9).setConstant(config_.vio_init_ba_weight);
  marg_H_.diagonal().segment<3>(12).setConstant(config_.vio_init_bg_weight);
  if (config_.vio_init_gyro_bias_weight > 0)
    marg_H_.diagonal().segment<3>(9).setConstant(config_.vio_init_gyro_bias_weight);
  if (config_.vio_init_accel_bias_weight > 0)
    marg_H_.diagonal().segment<3>(12).setConstant(config_.vio_init_accel_bias_weight);
  marg_lin_.clear();
  marg_lin_[t_ns] = s;

  initialized_ = true;
  std::cout << "VIO initialized at t_ns " << t_ns << "\nR_w_i\n" << s.T_w_i.R
            << "\nv_w_i " << s.vel_w_i.transpose() << "\nbg " << s.bias_gyro.transpose()
            << "\nba " << s.bias_accel.transpose() << std::endl;
}

SE3 VioEstimator::getPose(int64_t id) const {
  auto it = frame_states_.find(id);
  if (it != frame_states_.end()) return it->second.T_w_i;
  return frame_poses_.at(id).T_w_i;
}

bool VioEstimator::measure(const OpticalFlowResult::Ptr& of, const ImuPreintegration::Ptr& meas,
                           const AidingMeasurement* aid) {
  stats_ = VioStats();
  if (meas) {
    NavState next;
    meas->predictState(frame_states_.at(last_state_t_ns_), g_, next);
    next.t_ns = of->t_ns;
    last_state_t_ns_ = of->t_ns;
    frame_states_[last_state_t_ns_] = next;
    imu_meas_[meas->startT()] = *meas;
  }

  prev_opt_flow_res_[of->t_ns] = of;

  if (aid) {  // gating against the IMU-predicted state (fixed for the life of the factor)
    AidingMeasurement a = *aid;
    const Vec3& v = frame_states_.at(last_state_t_ns_).vel_w_i;
    const double vh = v.head<2>().norm();
    const bool in_flight = vh > config_.aid_min_speed && a.airspeed > config_.aid_min_speed;
    a.has_speed = a.has_speed && in_flight;
    if (a.has_course) {
      double innov = std::atan2(v.x(), v.y()) - a.course;
      innov = std::atan2(std::sin(innov), std::cos(innov));
      const bool gate_ok = std::abs(innov) < config_.aid_course_max_innov_deg * M_PI / 180;
      // re-acquisition: after 10 consecutive gate
      // rejections (1 s at 10 Hz) the course is used again (the Huber loss limits its pull)
      if (in_flight && !gate_ok) course_rejected_++;
      if (gate_ok) course_rejected_ = 0;
      a.has_course = in_flight && (gate_ok || course_rejected_ >= 10);
    }
    if (a.has_baro || a.has_speed || a.has_course) aid_[last_state_t_ns_] = a;
    stats_.aid_baro = a.has_baro;
    stats_.aid_speed = a.has_speed;
    stats_.aid_course = a.has_course;
  }

  // new residuals for existing landmarks
  int connected0 = 0;
  std::map<int64_t, int> num_points_connected;
  std::unordered_set<KeypointId> unconnected_obs0;
  for (int i = 0; i < 2; i++) {
    TimeCamId tcid_target(of->t_ns, i);
    for (const auto& kv : of->observations[i]) {
      auto it = landmarks_.find(kv.first);
      if (it != landmarks_.end()) {
        it->second.obs[tcid_target] = kv.second;
        num_points_connected[it->second.host.frame_id]++;
        if (i == 0) connected0++;
      } else if (i == 0) {
        unconnected_obs0.insert(kv.first);
      }
    }
  }

  if (double(connected0) / double(connected0 + unconnected_obs0.size()) <
          config_.vio_new_kf_keypoints_thresh &&
      frames_after_kf_ > config_.vio_min_frames_after_kf)
    take_kf_ = true;

  if (take_kf_) {
    take_kf_ = false;
    frames_after_kf_ = 0;
    kf_ids_.insert(last_state_t_ns_);
    stats_.is_kf = true;
    TimeCamId tcidl(of->t_ns, 0);
    const double min_triang_distance2 =
        config_.vio_min_triangulation_dist * config_.vio_min_triangulation_dist;
    int num_points_added = 0;
    stats_.triang_candidates = static_cast<int>(unconnected_obs0.size());
    for (KeypointId lm_id : unconnected_obs0) {
      std::map<TimeCamId, Vec2> kp_obs;
      for (const auto& kv : prev_opt_flow_res_) {
        for (int k = 0; k < 2; k++) {
          auto it = kv.second->observations[k].find(lm_id);
          if (it != kv.second->observations[k].end()) kp_obs[TimeCamId(kv.first, k)] = it->second;
        }
      }
      bool valid_kp = false;
      bool has_baseline = false;
      const Vec2 p0 = of->observations[0].at(lm_id);
      Vec3 f0;
      if (!calib_.cam[0].unproject(p0, f0)) continue;
      for (const auto& kv_obs : kp_obs) {
        if (valid_kp) break;
        const TimeCamId& tcido = kv_obs.first;
        Vec3 f1;
        if (!calib_.cam[tcido.cam_id].unproject(kv_obs.second, f1)) continue;
        const SE3 T_i0_i1 = getPose(tcidl.frame_id).inverse() * getPose(tcido.frame_id);
        const SE3 T_0_1 = calib_.T_i_c[0].inverse() * T_i0_i1 * calib_.T_i_c[tcido.cam_id];
        if (T_0_1.t.squaredNorm() < min_triang_distance2) continue;
        has_baseline = true;
        const Vec4 p = triangulate(f0, f1, T_0_1);
        if (p.allFinite() && p[3] > 0 && p[3] < 3.0) {
          Landmark lm;
          lm.host = tcidl;
          lm.dir = Stereographic::project(p.head<3>());
          lm.inv_dist = p[3];
          landmarks_[lm_id] = lm;
          num_points_added++;
          valid_kp = true;
        }
      }
      if (!has_baseline) stats_.triang_no_baseline++;
      if (valid_kp) {
        auto& lm = landmarks_[lm_id];
        for (const auto& kv_obs : kp_obs) lm.obs[kv_obs.first] = kv_obs.second;
      }
    }
    num_points_kf_[of->t_ns] = num_points_added;
    stats_.new_landmarks = num_points_added;
  } else {
    frames_after_kf_++;
  }

  std::unordered_set<KeypointId> lost_landmarks;
  if (config_.vio_marg_lost_landmarks) {
    for (const auto& kv : landmarks_) {
      if (!of->observations[0].count(kv.first) && !of->observations[1].count(kv.first))
        lost_landmarks.insert(kv.first);
    }
  }

  optimize();
  marginalize(num_points_connected, lost_landmarks);

  stats_.num_landmarks = static_cast<int>(landmarks_.size());
  int nobs = 0;
  for (const auto& kv : landmarks_) nobs += static_cast<int>(kv.second.obs.size());
  stats_.num_obs = nobs;
  stats_.num_kfs = static_cast<int>(kf_ids_.size());
  return true;
}

bool VioEstimator::linearizePoint(const Landmark& lm, const TimeCamId& tcid_t, const Vec2& z,
                                  Vec2& res, Mat23* Jl, Mat26* Jh, Mat26* Jt) const {
  const double rho = lm.inv_dist;
  if (!(rho > 0) || !std::isfinite(rho)) return false;
  const SE3 Th = getPose(lm.host.frame_id);
  const SE3 Tt = getPose(tcid_t.frame_id);
  const SE3& Tic_h = calib_.T_i_c[lm.host.cam_id];
  const SE3& Tic_t = calib_.T_i_c[tcid_t.cam_id];

  Eigen::Matrix<double, 3, 2> Jb;
  const Vec3 b = Stereographic::unproject(lm.dir, Jl ? &Jb : nullptr);
  const Vec3 q_h = Tic_h.R * b + Tic_h.t * rho;
  const Vec3 Rq = Th.R * q_h;
  const Vec3 Pw = Rq + rho * Th.t;
  const Vec3 D = Pw - rho * Tt.t;
  const Mat3 RtT = Tt.R.transpose();
  const Mat3 A = Tic_t.R.transpose();
  const Vec3 pc = A * (RtT * D - rho * Tic_t.t);

  Vec2 uv;
  Mat23 Jp;
  const bool need_j = Jl || Jh || Jt;
  if (!calib_.cam[tcid_t.cam_id].project(pc, uv, need_j ? &Jp : nullptr)) return false;
  res = uv - z;
  if (!res.allFinite()) return false;

  if (need_j) {
    const Mat3 ART = A * RtT;
    const Mat23 JpART = Jp * ART;
    if (Jl) {
      Jl->leftCols<2>() = JpART * Th.R * Tic_h.R * Jb;
      Jl->col(2) = JpART * (Th.R * Tic_h.t + Th.t - Tt.t) - Jp * A * Tic_t.t;
    }
    const bool same_frame = lm.host.frame_id == tcid_t.frame_id;
    if (Jh) {
      if (same_frame) {
        Jh->setZero();
      } else {
        Jh->leftCols<3>() = JpART * rho;
        Jh->rightCols<3>() = -JpART * hat(Rq);
      }
    }
    if (Jt) {
      if (same_frame) {
        Jt->setZero();
      } else {
        Jt->leftCols<3>() = -JpART * rho;
        Jt->rightCols<3>() = JpART * hat(D);
      }
    }
  }
  return true;
}

VecX VioEstimator::priorDelta() const {
  VecX d = VecX::Zero(marg_order_.total_size);
  for (const auto& kv : marg_order_.abs_order_map) {
    const int off = kv.second.first, sz = kv.second.second;
    const NavState& lin = marg_lin_.at(kv.first);
    const NavState* cur = nullptr;
    auto its = frame_states_.find(kv.first);
    if (its != frame_states_.end())
      cur = &its->second;
    else
      cur = &frame_poses_.at(kv.first);
    d.segment<3>(off) = cur->T_w_i.t - lin.T_w_i.t;
    d.segment<3>(off + 3) = logSO3(cur->T_w_i.R * lin.T_w_i.R.transpose());
    if (sz == POSE_VEL_BIAS_SIZE) {
      d.segment<3>(off + 6) = cur->vel_w_i - lin.vel_w_i;
      d.segment<3>(off + 9) = cur->bias_gyro - lin.bias_gyro;
      d.segment<3>(off + 12) = cur->bias_accel - lin.bias_accel;
    }
  }
  return d;
}

void VioEstimator::addPrior(const AbsOrderMap& aom, MatX& H, VecX& b, double* error) const {
  const VecX d = priorDelta();
  const VecX bm = marg_b_ + marg_H_ * d;
  if (error) *error += 0.5 * d.dot(marg_H_ * d) + marg_b_.dot(d);
  for (const auto& kv1 : marg_order_.abs_order_map) {
    const auto& a1 = aom.abs_order_map.at(kv1.first);
    const int o1 = kv1.second.first, s1 = kv1.second.second;
    if (a1.second != s1) {
      std::cerr << "prior/aom size mismatch" << std::endl;
      continue;
    }
    b.segment(a1.first, s1) += bm.segment(o1, s1);
    for (const auto& kv2 : marg_order_.abs_order_map) {
      const auto& a2 = aom.abs_order_map.at(kv2.first);
      const int o2 = kv2.second.first, s2 = kv2.second.second;
      H.block(a1.first, a2.first, s1, s2) += marg_H_.block(o1, o2, s1, s2);
    }
  }
}

void VioEstimator::addImu(const AbsOrderMap& aom, MatX* H, VecX* b, double& error) const {
  for (const auto& kv : imu_meas_) {
    const ImuPreintegration& m = kv.second;
    const int64_t t0 = m.startT(), t1 = m.startT() + m.dtNs();
    auto i0 = aom.abs_order_map.find(t0), i1 = aom.abs_order_map.find(t1);
    if (i0 == aom.abs_order_map.end() || i1 == aom.abs_order_map.end()) continue;
    auto s0 = frame_states_.find(t0), s1 = frame_states_.find(t1);
    if (s0 == frame_states_.end() || s1 == frame_states_.end()) continue;

    Eigen::Matrix<double, 9, 15> J0, J1;
    const auto r = m.residual(s0->second, s1->second, g_, H ? &J0 : nullptr, H ? &J1 : nullptr);
    const Eigen::Matrix<double, 9, 9> W = m.info();
    error += 0.5 * r.dot(W * r);

    const double dt = m.dt();
    const Vec3 res_bg = s1->second.bias_gyro - s0->second.bias_gyro;
    const Vec3 res_ba = s1->second.bias_accel - s0->second.bias_accel;
    const Vec3 wg = gyro_bias_weight_ / dt, wa = accel_bias_weight_ / dt;
    error += 0.5 * res_bg.dot(wg.asDiagonal() * res_bg) + 0.5 * res_ba.dot(wa.asDiagonal() * res_ba);

    if (H) {
      const int o0 = i0->second.first, o1 = i1->second.first;
      H->block<15, 15>(o0, o0) += J0.transpose() * W * J0;
      H->block<15, 15>(o0, o1) += J0.transpose() * W * J1;
      H->block<15, 15>(o1, o0) += J1.transpose() * W * J0;
      H->block<15, 15>(o1, o1) += J1.transpose() * W * J1;
      b->segment<15>(o0) += J0.transpose() * W * r;
      b->segment<15>(o1) += J1.transpose() * W * r;
      // bias random walk: r = b1 - b0
      for (int k = 0; k < 3; k++) {
        H->coeffRef(o0 + 9 + k, o0 + 9 + k) += wg[k];
        H->coeffRef(o1 + 9 + k, o1 + 9 + k) += wg[k];
        H->coeffRef(o0 + 9 + k, o1 + 9 + k) -= wg[k];
        H->coeffRef(o1 + 9 + k, o0 + 9 + k) -= wg[k];
        (*b)[o0 + 9 + k] -= wg[k] * res_bg[k];
        (*b)[o1 + 9 + k] += wg[k] * res_bg[k];
        H->coeffRef(o0 + 12 + k, o0 + 12 + k) += wa[k];
        H->coeffRef(o1 + 12 + k, o1 + 12 + k) += wa[k];
        H->coeffRef(o0 + 12 + k, o1 + 12 + k) -= wa[k];
        H->coeffRef(o1 + 12 + k, o0 + 12 + k) -= wa[k];
        (*b)[o0 + 12 + k] -= wa[k] * res_ba[k];
        (*b)[o1 + 12 + k] += wa[k] * res_ba[k];
      }
    }
  }
}

void VioEstimator::addAiding(const AbsOrderMap& aom, MatX* H, VecX* b, double& error,
                            const std::set<int64_t>* aid_only) const {
  const double k = config_.aid_huber;
  auto add = [&](int o, const Eigen::Matrix<double, 1, 15>& J, double r, double sigma) {
    const double e = std::abs(r) / sigma;
    const double hw = e < k ? 1.0 : k / e;
    error += 0.5 * hw * (2 - hw) * e * e;
    if (H) {
      const double w = hw / (sigma * sigma);
      H->block<15, 15>(o, o) += J.transpose() * w * J;
      b->segment<15>(o) += J.transpose() * w * r;
    }
  };
  for (const auto& kv : aid_) {
    if (aid_only && !aid_only->count(kv.first)) continue;
    auto ia = aom.abs_order_map.find(kv.first);
    auto is = frame_states_.find(kv.first);
    if (ia == aom.abs_order_map.end() || is == frame_states_.end() || ia->second.second != POSE_VEL_BIAS_SIZE)
      continue;
    const int o = ia->second.first;
    const NavState& s = is->second;
    const AidingMeasurement& a = kv.second;
    Eigen::Matrix<double, 1, 15> J;
    if (a.has_baro) {
      J.setZero();
      J(2) = 1;
      add(o, J, s.T_w_i.t.z() - a.baro_up, config_.aid_baro_sigma);
    }
    if (a.has_speed) {  // |v_h - wind| - TAS
      const Vec2 av(s.vel_w_i.x() - config_.aid_wind_e, s.vel_w_i.y() - config_.aid_wind_n);
      const double n = std::max(av.norm(), 1.0);
      J.setZero();
      J(6) = av.x() / n;
      J(7) = av.y() / n;
      add(o, J, av.norm() - a.airspeed, config_.aid_speed_sigma);
    }
    if (a.has_course) {  // atan2(vE, vN) - course
      const double ve = s.vel_w_i.x(), vn = s.vel_w_i.y(), h2 = std::max(ve * ve + vn * vn, 1.0);
      double r = std::atan2(ve, vn) - a.course;
      r = std::atan2(std::sin(r), std::cos(r));
      J.setZero();
      J(6) = vn / h2;
      J(7) = -ve / h2;
      add(o, J, r, config_.aid_course_sigma_deg * M_PI / 180);
    }
  }
}

void VioEstimator::linearize(const AbsOrderMap& aom, const std::vector<KeypointId>& lm_ids,
                             LinSystem& ls, const std::set<int64_t>* aid_only) const {
  const int n = aom.total_size;
  ls.H.setZero(n, n);
  ls.b.setZero(n);
  ls.error = 0;
  ls.lms.clear();
  ls.lms.reserve(lm_ids.size());

  const double sigma2 = config_.vio_obs_std_dev * config_.vio_obs_std_dev;
  const double k = config_.vio_obs_huber_thresh;

  for (KeypointId id : lm_ids) {
    const Landmark& lm = landmarks_.at(id);
    auto ih = aom.abs_order_map.find(lm.host.frame_id);
    if (ih == aom.abs_order_map.end()) continue;
    LmLin ll;
    ll.id = id;
    int nres = 0;
    for (const auto& ob : lm.obs) {
      auto it = aom.abs_order_map.find(ob.first.frame_id);
      if (it == aom.abs_order_map.end()) continue;
      Vec2 r;
      Mat23 Jl;
      Mat26 Jh, Jt;
      if (!linearizePoint(lm, ob.first, ob.second, r, &Jl, &Jh, &Jt)) continue;
      const double e = r.norm();
      const double hw = huberWeight(e, k);
      const double w = hw / sigma2;
      ls.error += 0.5 * hw * (2 - hw) * e * e / sigma2;
      nres++;

      ll.Hll += Jl.transpose() * w * Jl;
      ll.bl += Jl.transpose() * w * r;
      if (ob.first.frame_id != lm.host.frame_id) {
        const int oh = ih->second.first, ot = it->second.first;
        ls.H.block<6, 6>(oh, oh) += Jh.transpose() * w * Jh;
        ls.H.block<6, 6>(oh, ot) += Jh.transpose() * w * Jt;
        ls.H.block<6, 6>(ot, oh) += Jt.transpose() * w * Jh;
        ls.H.block<6, 6>(ot, ot) += Jt.transpose() * w * Jt;
        ls.b.segment<6>(oh) += Jh.transpose() * w * r;
        ls.b.segment<6>(ot) += Jt.transpose() * w * r;
        ll.block(oh) += Jh.transpose() * w * Jl;
        ll.block(ot) += Jt.transpose() * w * Jl;
      }
    }
    if (nres > 0) ls.lms.push_back(std::move(ll));
  }

  addImu(aom, &ls.H, &ls.b, ls.error);
  addAiding(aom, &ls.H, &ls.b, ls.error, aid_only);
  addPrior(aom, ls.H, ls.b, &ls.error);
}

double VioEstimator::visualError(
    std::map<KeypointId, std::vector<std::pair<TimeCamId, double>>>* outliers,
    double outlier_threshold) const {
  const double sigma2 = config_.vio_obs_std_dev * config_.vio_obs_std_dev;
  const double k = config_.vio_obs_huber_thresh;
  double error = 0;
  for (const auto& kv : landmarks_) {
    const Landmark& lm = kv.second;
    for (const auto& ob : lm.obs) {
      Vec2 r;
      if (!linearizePoint(lm, ob.first, ob.second, r, nullptr, nullptr, nullptr)) {
        if (outliers) (*outliers)[kv.first].emplace_back(ob.first, -2.0);
        continue;
      }
      const double e = r.norm();
      const double hw = huberWeight(e, k);
      error += 0.5 * hw * (2 - hw) * e * e / sigma2;
      if (outliers && e > outlier_threshold)
        (*outliers)[kv.first].emplace_back(ob.first, ob.first != lm.host ? e : -2.0);
    }
  }
  return error;
}

double VioEstimator::totalError() const {
  AbsOrderMap aom;
  for (const auto& kv : frame_poses_) {
    aom.abs_order_map[kv.first] = {aom.total_size, POSE_SIZE};
    aom.total_size += POSE_SIZE;
  }
  for (const auto& kv : frame_states_) {
    aom.abs_order_map[kv.first] = {aom.total_size, POSE_VEL_BIAS_SIZE};
    aom.total_size += POSE_VEL_BIAS_SIZE;
  }
  double error = visualError(nullptr, 0);
  addImu(aom, nullptr, nullptr, error);
  addAiding(aom, nullptr, nullptr, error, nullptr);
  const VecX d = priorDelta();
  error += 0.5 * d.dot(marg_H_ * d) + marg_b_.dot(d);
  return error;
}

void VioEstimator::backup() {
  backup_states_ = frame_states_;
  backup_poses_ = frame_poses_;
  for (auto& kv : landmarks_) {
    kv.second.dir_backup = kv.second.dir;
    kv.second.inv_dist_backup = kv.second.inv_dist;
  }
}

void VioEstimator::restore() {
  frame_states_ = backup_states_;
  frame_poses_ = backup_poses_;
  for (auto& kv : landmarks_) {
    kv.second.dir = kv.second.dir_backup;
    kv.second.inv_dist = kv.second.inv_dist_backup;
  }
}

void VioEstimator::removeObservations(KeypointId id, const std::set<TimeCamId>& obs) {
  auto it = landmarks_.find(id);
  if (it == landmarks_.end()) return;
  for (const auto& o : obs) it->second.obs.erase(o);
  if (it->second.obs.size() < 2) landmarks_.erase(it);
}

void VioEstimator::filterOutliers(double threshold, int min_num_obs) {
  std::map<KeypointId, std::vector<std::pair<TimeCamId, double>>> outliers;
  visualError(&outliers, threshold);
  for (const auto& kv : outliers) {
    const int num_obs = static_cast<int>(landmarks_.at(kv.first).obs.size());
    const int num_outliers = static_cast<int>(kv.second.size());
    bool remove = num_obs - num_outliers < min_num_obs;
    for (const auto& o : kv.second)
      if (o.second == -2.0) remove = true;
    if (remove) {
      landmarks_.erase(kv.first);
    } else {
      std::set<TimeCamId> s;
      for (const auto& o : kv.second) s.insert(o.first);
      removeObservations(kv.first, s);
    }
  }
}

void VioEstimator::optimize() {
  if (!(opt_started_ || frame_states_.size() > 4)) return;
  opt_started_ = true;

  AbsOrderMap aom;
  for (const auto& kv : frame_poses_) {
    aom.abs_order_map[kv.first] = {aom.total_size, POSE_SIZE};
    aom.total_size += POSE_SIZE;
  }
  for (const auto& kv : frame_states_) {
    aom.abs_order_map[kv.first] = {aom.total_size, POSE_VEL_BIAS_SIZE};
    aom.total_size += POSE_VEL_BIAS_SIZE;
  }
  const int n = aom.total_size;

  double lambda = config_.vio_lm_lambda_initial;
  const double min_lambda = config_.vio_lm_lambda_min;
  const double max_lambda = config_.vio_lm_lambda_max;
  double lambda_vee = 2.0;

  LinSystem ls;
  bool need_lin = true;
  bool terminated = false;
  bool filtered = false;
  int it = 0;
  double error_total = 0;
  std::vector<KeypointId> lm_ids;

  while (it <= config_.vio_max_iterations && !terminated) {
    if (it >= config_.vio_filter_iteration && !filtered) {
      filterOutliers(config_.vio_outlier_threshold, 4);
      filtered = true;
      need_lin = true;
    }
    if (need_lin) {
      lm_ids.clear();
      for (const auto& kv : landmarks_) lm_ids.push_back(kv.first);
      linearize(aom, lm_ids, ls);
      error_total = ls.error;
      need_lin = false;
    }

    // --- damped Schur complement on landmarks
    MatX Hs = ls.H;
    VecX bs = ls.b;
    std::vector<Mat3> Hll_inv(ls.lms.size());
    for (size_t l = 0; l < ls.lms.size(); l++) {
      const LmLin& ll = ls.lms[l];
      Mat3 Hd = ll.Hll;
      Hd.diagonal() += (ll.Hll.diagonal() * lambda).cwiseMax(min_lambda);
      Hll_inv[l] = Hd.inverse();
      const Mat3& Hi = Hll_inv[l];
      for (const auto& pi : ll.Hpl) {
        const Mat63 HiHi = pi.second * Hi;
        bs.segment<6>(pi.first) -= HiHi * ll.bl;
        for (const auto& pj : ll.Hpl)
          Hs.block<6, 6>(pi.first, pj.first) -= HiHi * pj.second.transpose();
      }
    }
    Hs.diagonal() += (Hs.diagonal() * lambda).cwiseMax(min_lambda);
    VecX inc = -Hs.ldlt().solve(bs);
    if (!inc.allFinite()) {
      lambda = lambda_vee * lambda;
      lambda_vee *= 2;
      it++;
      if (lambda > max_lambda) terminated = true;
      continue;
    }

    // back-substitute landmarks + compute model cost change (undamped model)
    std::vector<Vec3> inc_l(ls.lms.size());
    double model = ls.b.dot(inc) + 0.5 * inc.dot(ls.H * inc);
    for (size_t l = 0; l < ls.lms.size(); l++) {
      const LmLin& ll = ls.lms[l];
      Vec3 rhs = ll.bl;
      Vec3 Hlp_dp = Vec3::Zero();
      for (const auto& p : ll.Hpl) Hlp_dp += p.second.transpose() * inc.segment<6>(p.first);
      rhs += Hlp_dp;
      inc_l[l] = -Hll_inv[l] * rhs;
      model += ll.bl.dot(inc_l[l]) + inc_l[l].dot(Hlp_dp) + 0.5 * inc_l[l].dot(ll.Hll * inc_l[l]);
    }
    const double l_diff = -model;

    backup();
    for (auto& kv : frame_poses_)
      kv.second.applyPoseInc(inc.segment<6>(aom.abs_order_map.at(kv.first).first));
    for (auto& kv : frame_states_)
      kv.second.applyInc(inc.segment<15>(aom.abs_order_map.at(kv.first).first));
    for (size_t l = 0; l < ls.lms.size(); l++) {
      Landmark& lm = landmarks_.at(ls.lms[l].id);
      lm.dir += inc_l[l].head<2>();
      lm.inv_dist += inc_l[l][2];
    }
    double step_norminf = inc.size() ? inc.cwiseAbs().maxCoeff() : 0.0;

    const double after_error = totalError();
    const double f_diff = error_total - after_error;
    const double rel = f_diff / l_diff;
    if (l_diff > 0 && rel > 0 && std::isfinite(after_error)) {
      lambda *= std::max(1.0 / 3, 1 - std::pow(2 * rel - 1, 3));
      lambda = std::max(min_lambda, lambda);
      lambda_vee = 2.0;
      it++;
      need_lin = true;
      if ((f_diff > 0 && f_diff < 1e-6) || step_norminf < 1e-4) terminated = true;
    } else {
      lambda = lambda_vee * lambda;
      lambda_vee *= 2;
      restore();
      it++;
      if (lambda > max_lambda) terminated = true;
    }
  }
  (void)n;
  stats_.lm_iterations = it;
  stats_.final_error = totalError();
}

void VioEstimator::marginalize(const std::map<int64_t, int>& num_points_connected,
                               const std::unordered_set<KeypointId>& lost_landmarks) {
  if (!opt_started_) return;
  const int max_states = config_.vio_max_states;
  const size_t max_kfs = static_cast<size_t>(config_.vio_max_kfs);
  if (!(frame_poses_.size() > max_kfs || static_cast<int>(frame_states_.size()) >= max_states))
    return;

  const int states_to_remove = static_cast<int>(frame_states_.size()) - max_states + 1;
  auto sit = frame_states_.cbegin();
  for (int i = 0; i < states_to_remove; i++) sit++;
  const int64_t last_state_to_marg = sit->first;

  AbsOrderMap aom;
  std::set<int64_t> poses_to_marg;
  for (const auto& kv : frame_poses_) {
    aom.abs_order_map[kv.first] = {aom.total_size, POSE_SIZE};
    if (kf_ids_.count(kv.first) == 0) poses_to_marg.insert(kv.first);
    aom.total_size += POSE_SIZE;
  }
  std::set<int64_t> states_to_marg_vel_bias, states_to_marg_all;
  for (const auto& kv : frame_states_) {
    if (kv.first > last_state_to_marg) break;
    if (kv.first != last_state_to_marg) {
      if (kf_ids_.count(kv.first) > 0)
        states_to_marg_vel_bias.insert(kv.first);
      else
        states_to_marg_all.insert(kv.first);
    }
    aom.abs_order_map[kv.first] = {aom.total_size, POSE_VEL_BIAS_SIZE};
    aom.total_size += POSE_VEL_BIAS_SIZE;
  }

  // keyframe selection (Basalt / DSO heuristic)
  std::set<int64_t> kfs_to_marg;
  while (kf_ids_.size() > max_kfs && !states_to_marg_vel_bias.empty()) {
    int64_t id_to_marg = -1;
    if (kf_ids_.size() > 2) {
      auto end_minus_2 = std::prev(kf_ids_.end(), 2);
      for (auto it = kf_ids_.begin(); it != end_minus_2; ++it) {
        auto c = num_points_connected.find(*it);
        auto nk = num_points_kf_.find(*it);
        const double npk = nk != num_points_kf_.end() ? std::max(1, nk->second) : 1;
        if (c == num_points_connected.end() || c->second / npk < config_.vio_kf_marg_feature_ratio) {
          id_to_marg = *it;
          break;
        }
      }
    }
    if (kf_ids_.size() > 2 && id_to_marg < 0) {
      auto end_minus_2 = std::prev(kf_ids_.end(), 2);
      const int64_t last_kf = *kf_ids_.crbegin();
      double min_score = std::numeric_limits<double>::max();
      int64_t min_id = -1;
      for (auto it1 = kf_ids_.begin(); it1 != end_minus_2; ++it1) {
        double denom = 0;
        for (auto it2 = kf_ids_.begin(); it2 != end_minus_2; ++it2)
          denom += 1 / ((getPose(*it1).t - getPose(*it2).t).norm() + 1e-5);
        const double score = std::sqrt((getPose(*it1).t - getPose(last_kf).t).norm()) * denom;
        if (score < min_score) {
          min_id = *it1;
          min_score = score;
        }
      }
      id_to_marg = min_id;
    }
    if (id_to_marg < 0) break;
    kfs_to_marg.insert(id_to_marg);
    poses_to_marg.insert(id_to_marg);
    kf_ids_.erase(id_to_marg);
  }

  // a selected keyframe that is still a full state is marginalized completely
  for (int64_t id : kfs_to_marg) {
    if (states_to_marg_vel_bias.erase(id)) states_to_marg_all.insert(id);
  }

  // linearize: prior + IMU + landmarks hosted in marginalized KFs + lost landmarks
  std::vector<KeypointId> lm_ids;
  for (const auto& kv : landmarks_) {
    if (kfs_to_marg.count(kv.second.host.frame_id) || lost_landmarks.count(kv.first))
      lm_ids.push_back(kv.first);
  }
  // aiding factors: only those of the states that leave the window as full states (their information
  // goes into the prior); the factors of last_state_to_marg stay in the optimization
  std::set<int64_t> aid_marg = states_to_marg_all;
  aid_marg.insert(states_to_marg_vel_bias.begin(), states_to_marg_vel_bias.end());
  LinSystem ls;
  linearize(aom, lm_ids, ls, &aid_marg);
  MatX H = ls.H;
  VecX b = ls.b;
  for (const LmLin& ll : ls.lms) {
    Mat3 Hll = ll.Hll;
    Hll.diagonal().array() += 1e-9;
    Eigen::LDLT<Mat3> ldlt(Hll);
    if (ldlt.info() != Eigen::Success || !(ldlt.vectorD().array() > 1e-12).all()) continue;
    const Mat3 Hi = ldlt.solve(Mat3::Identity());
    if (!Hi.allFinite()) continue;
    for (const auto& pi : ll.Hpl) {
      const Mat63 HiHi = pi.second * Hi;
      b.segment<6>(pi.first) -= HiHi * ll.bl;
      for (const auto& pj : ll.Hpl) H.block<6, 6>(pi.first, pj.first) -= HiHi * pj.second.transpose();
    }
  }

  // indices to keep / marginalize
  std::vector<int> idx_keep, idx_marg;
  for (const auto& kv : aom.abs_order_map) {
    const int start = kv.second.first;
    if (kv.second.second == POSE_SIZE) {
      for (int i = 0; i < POSE_SIZE; i++)
        (poses_to_marg.count(kv.first) ? idx_marg : idx_keep).push_back(start + i);
    } else if (states_to_marg_all.count(kv.first)) {
      for (int i = 0; i < POSE_VEL_BIAS_SIZE; i++) idx_marg.push_back(start + i);
    } else if (states_to_marg_vel_bias.count(kv.first)) {
      for (int i = 0; i < POSE_SIZE; i++) idx_keep.push_back(start + i);
      for (int i = POSE_SIZE; i < POSE_VEL_BIAS_SIZE; i++) idx_marg.push_back(start + i);
    } else {
      for (int i = 0; i < POSE_VEL_BIAS_SIZE; i++) idx_keep.push_back(start + i);
    }
  }
  std::sort(idx_keep.begin(), idx_keep.end());
  const int nk = static_cast<int>(idx_keep.size()), nm = static_cast<int>(idx_marg.size());
  MatX Hkk(nk, nk), Hkm(nk, nm), Hmm(nm, nm);
  VecX bk(nk), bmv(nm);
  for (int i = 0; i < nk; i++) {
    bk[i] = b[idx_keep[i]];
    for (int j = 0; j < nk; j++) Hkk(i, j) = H(idx_keep[i], idx_keep[j]);
    for (int j = 0; j < nm; j++) Hkm(i, j) = H(idx_keep[i], idx_marg[j]);
  }
  for (int i = 0; i < nm; i++) {
    bmv[i] = b[idx_marg[i]];
    for (int j = 0; j < nm; j++) Hmm(i, j) = H(idx_marg[i], idx_marg[j]);
  }
  MatX H_new = Hkk;
  VecX b_new = bk;
  if (nm > 0) {
    // robust inverse of Hmm (eigen-decomposition, drop null directions)
    Eigen::SelfAdjointEigenSolver<MatX> es(0.5 * (Hmm + Hmm.transpose()));
    VecX ev = es.eigenvalues();
    const double emax = std::max(ev.maxCoeff(), 1e-12);
    VecX evinv(nm);
    for (int i = 0; i < nm; i++) evinv[i] = ev[i] > emax * 1e-12 ? 1.0 / ev[i] : 0.0;
    const MatX Hmm_inv = es.eigenvectors() * evinv.asDiagonal() * es.eigenvectors().transpose();
    H_new -= Hkm * Hmm_inv * Hkm.transpose();
    b_new -= Hkm * Hmm_inv * bmv;
  }
  H_new = 0.5 * (H_new + H_new.transpose());

  // --- update window
  for (int64_t id : aid_marg) aid_.erase(id);
  for (int64_t id : states_to_marg_all) {
    frame_states_.erase(id);
    imu_meas_.erase(id);
    prev_opt_flow_res_.erase(id);
  }
  for (int64_t id : states_to_marg_vel_bias) {
    frame_poses_[id] = frame_states_.at(id);
    frame_states_.erase(id);
    imu_meas_.erase(id);
  }
  for (int64_t id : poses_to_marg) {
    frame_poses_.erase(id);
    prev_opt_flow_res_.erase(id);
    num_points_kf_.erase(id);
  }
  // landmark database: remove landmarks hosted in marginalized KFs and
  // observations in removed frames
  for (auto it = landmarks_.begin(); it != landmarks_.end();) {
    if (kfs_to_marg.count(it->second.host.frame_id)) {
      it = landmarks_.erase(it);
      continue;
    }
    for (auto o = it->second.obs.begin(); o != it->second.obs.end();) {
      const int64_t f = o->first.frame_id;
      if (poses_to_marg.count(f) || states_to_marg_all.count(f) || kfs_to_marg.count(f))
        o = it->second.obs.erase(o);
      else
        ++o;
    }
    if (it->second.obs.size() < 2)
      it = landmarks_.erase(it);
    else
      ++it;
  }
  if (config_.vio_marg_lost_landmarks)
    for (KeypointId id : lost_landmarks) landmarks_.erase(id);

  // new prior: all keyframe poses + last_state_to_marg, linearized at the
  // current estimate (b_new is the gradient at the current estimate)
  AbsOrderMap order_new;
  marg_lin_.clear();
  for (const auto& kv : frame_poses_) {
    order_new.abs_order_map[kv.first] = {order_new.total_size, POSE_SIZE};
    order_new.total_size += POSE_SIZE;
    marg_lin_[kv.first] = kv.second;
  }
  order_new.abs_order_map[last_state_to_marg] = {order_new.total_size, POSE_VEL_BIAS_SIZE};
  order_new.total_size += POSE_VEL_BIAS_SIZE;
  marg_lin_[last_state_to_marg] = frame_states_.at(last_state_to_marg);

  if (order_new.total_size != nk) {
    std::cerr << "Marginalization order mismatch " << order_new.total_size << " vs " << nk
              << std::endl;
  }
  marg_H_ = H_new;
  marg_b_ = b_new;
  marg_order_ = order_new;
}

}  // namespace bw
