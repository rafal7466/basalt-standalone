// Basalt-Win: sliding-window keypoint VIO (port of the logic in
// basalt/vi_estimator/sqrt_keypoint_vio.cpp) without TBB/Sophus/Pangolin.
//
//  * states: full [pose, vel, bg, ba] for the newest frames (max_states),
//    pose-only keyframes (max_kfs)
//  * landmarks hosted in keyframes: stereographic bearing + inverse distance
//  * factors: reprojection (Huber), IMU preintegration, bias random walk,
//    marginalization prior
//  * Levenberg-Marquardt with Schur complement on landmarks
//  * marginalization of old states / keyframes / lost landmarks
#pragma once
#include <map>
#include <set>
#include <unordered_set>
#include <vector>

#include "bw/config.h"
#include "bw/imu.h"
#include "bw/optical_flow.h"

namespace bw {

struct TimeCamId {
  int64_t frame_id = 0;
  int cam_id = 0;
  TimeCamId() = default;
  TimeCamId(int64_t f, int c) : frame_id(f), cam_id(c) {}
  bool operator<(const TimeCamId& o) const {
    return frame_id < o.frame_id || (frame_id == o.frame_id && cam_id < o.cam_id);
  }
  bool operator==(const TimeCamId& o) const { return frame_id == o.frame_id && cam_id == o.cam_id; }
  bool operator!=(const TimeCamId& o) const { return !(*this == o); }
};

struct Landmark {
  TimeCamId host;
  Vec2 dir = Vec2::Zero();  // stereographic bearing in host camera
  double inv_dist = 0;
  std::map<TimeCamId, Vec2> obs;  // includes the host observation
  Vec2 dir_backup = Vec2::Zero();
  double inv_dist_backup = 0;
};

// aiding measurements attached to a frame (world frame ENU: x east, y north, z up)
struct AidingMeasurement {
  bool has_baro = false, has_speed = false, has_course = false;
  double baro_up = 0;   // m, height above the origin (barometer)
  double airspeed = 0;  // m/s, true airspeed (horizontal)
  double course = 0;    // rad, clockwise from north
};

struct AbsOrderMap {
  std::map<int64_t, std::pair<int, int>> abs_order_map;  // frame -> (offset, size)
  int total_size = 0;
};

struct VioStats {
  int num_landmarks = 0;
  int num_obs = 0;
  int num_kfs = 0;
  int lm_iterations = 0;
  double final_error = 0;
  bool is_kf = false;
  int new_landmarks = 0;      // landmarks created in this frame (keyframe)
  int triang_candidates = 0;  // unconnected points considered for triangulation
  int triang_no_baseline = 0; // ... without an observation with enough baseline
  int aid_baro = 0, aid_speed = 0, aid_course = 0;  // aiding factors accepted for this frame
};

class VioEstimator {
 public:
  VioEstimator(const VioConfig& config, const Calibration& calib);

  // static initialization from the first (bias corrected) accelerometer sample
  void initialize(int64_t t_ns, const Vec3& accel);
  // initialization from a complete state (attitude, velocity, biases), e.g. from
  // a static window with a known heading or an in-flight start
  void initialize(const NavState& state);
  // ... with a full diagonal prior (1/sigma^2 for the 15 increments [p, phi(world), v, bg, ba]),
  // e.g. the covariance of a hand-off state, instead of Basalt's position + yaw + bias prior
  void initialize(const NavState& state, const Eigen::Matrix<double, 15, 1>& prior_info);
  bool measure(const OpticalFlowResult::Ptr& of, const ImuPreintegration::Ptr& meas,
               const AidingMeasurement* aid = nullptr);

  bool initialized() const { return initialized_; }
  const NavState& lastState() const { return frame_states_.at(last_state_t_ns_); }
  const VioStats& stats() const { return stats_; }
  const Vec3& gravity() const { return g_; }

 private:
  friend struct VioTestAccess;  // unit tests (src/tests.cpp)

  struct LmLin {
    KeypointId id;
    Mat3 Hll = Mat3::Zero();
    Vec3 bl = Vec3::Zero();
    std::vector<std::pair<int, Eigen::Matrix<double, 6, 3>>> Hpl;  // (offset, block)
    Eigen::Matrix<double, 6, 3>& block(int off) {
      for (auto& p : Hpl)
        if (p.first == off) return p.second;
      Hpl.emplace_back(off, Eigen::Matrix<double, 6, 3>::Zero());
      return Hpl.back().second;
    }
  };
  struct LinSystem {
    MatX H;
    VecX b;
    std::vector<LmLin> lms;
    double error = 0;
  };

  SE3 getPose(int64_t frame_id) const;
  bool linearizePoint(const Landmark& lm, const TimeCamId& tcid_t, const Vec2& z, Vec2& res,
                      Eigen::Matrix<double, 2, 3>* Jl, Eigen::Matrix<double, 2, 6>* Jh,
                      Eigen::Matrix<double, 2, 6>* Jt) const;
  void linearize(const AbsOrderMap& aom, const std::vector<KeypointId>& lm_ids, LinSystem& ls,
                 const std::set<int64_t>* aid_only = nullptr) const;
  // aiding factors of the states in aom (only those in aid_only, if given)
  void addAiding(const AbsOrderMap& aom, MatX* H, VecX* b, double& error,
                 const std::set<int64_t>* aid_only) const;
  void addPrior(const AbsOrderMap& aom, MatX& H, VecX& b, double* error) const;
  void addImu(const AbsOrderMap& aom, MatX* H, VecX* b, double& error) const;
  double visualError(std::map<KeypointId, std::vector<std::pair<TimeCamId, double>>>* outliers,
                     double outlier_threshold) const;
  double totalError() const;
  VecX priorDelta() const;

  void optimize();
  void marginalize(const std::map<int64_t, int>& num_points_connected,
                   const std::unordered_set<KeypointId>& lost_landmarks);
  void filterOutliers(double threshold, int min_num_obs);
  void backup();
  void restore();

  void removeObservations(KeypointId id, const std::set<TimeCamId>& obs);

  VioConfig config_;
  Calibration calib_;
  Vec3 g_ = Vec3(0, 0, -9.81);

  bool initialized_ = false;
  bool opt_started_ = false;
  bool take_kf_ = true;
  int frames_after_kf_ = 0;
  int64_t last_state_t_ns_ = 0;

  std::map<int64_t, NavState> frame_states_;  // full states
  std::map<int64_t, NavState> frame_poses_;   // pose-only keyframes
  std::map<int64_t, NavState> backup_states_, backup_poses_;
  std::map<int64_t, ImuPreintegration> imu_meas_;  // keyed by start time
  std::set<int64_t> kf_ids_;
  std::map<int64_t, OpticalFlowResult::Ptr> prev_opt_flow_res_;
  std::map<int64_t, int> num_points_kf_;
  std::map<KeypointId, Landmark> landmarks_;
  std::map<int64_t, AidingMeasurement> aid_;  // per full state
  int course_rejected_ = 0;                   // consecutive course measurements outside the gate

  // marginalization prior: E(x) = 0.5 d^T H d + b^T d, d = x [-] x_lin
  MatX marg_H_;
  VecX marg_b_;
  AbsOrderMap marg_order_;
  std::map<int64_t, NavState> marg_lin_;

  Vec3 gyro_bias_weight_, accel_bias_weight_;  // 1/std^2 (per second)
  VioStats stats_;
};

}  // namespace bw
