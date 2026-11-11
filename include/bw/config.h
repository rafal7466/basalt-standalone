// Basalt-Win: configuration + calibration (mirrors basalt/data/euroc_config.json
// and basalt/data/euroc_ds_calib.json).
#pragma once
#include <map>
#include <string>
#include <vector>

#include "bw/camera.h"

namespace bw {

struct VioConfig {
  // optical flow
  int optical_flow_detection_grid_size = 50;
  double optical_flow_max_recovered_dist2 = 0.04;
  int optical_flow_max_iterations = 5;
  double optical_flow_epipolar_error = 0.005;
  int optical_flow_levels = 3;
  double optical_flow_pattern_scale = 0.5;  // pattern 51 = 0.5 * pattern 52

  // Basalt-Win extension (not in upstream frame_to_frame flow): predict the
  // feature positions with the gyro rotation between frames (initial guess of
  // the patch tracking). Needed for fast rotations at high resolution.
  bool optical_flow_imu_rotation_prediction = false;

  // estimator
  int vio_max_states = 3;
  int vio_max_kfs = 7;
  int vio_min_frames_after_kf = 5;
  double vio_new_kf_keypoints_thresh = 0.7;
  double vio_obs_std_dev = 0.5;
  double vio_obs_huber_thresh = 1.0;
  double vio_min_triangulation_dist = 0.05;
  double vio_outlier_threshold = 3.0;
  int vio_filter_iteration = 4;
  int vio_max_iterations = 7;
  double vio_lm_lambda_initial = 1e-4;
  double vio_lm_lambda_min = 1e-6;
  double vio_lm_lambda_max = 1e2;
  double vio_init_pose_weight = 1e8;
  double vio_init_ba_weight = 1e1;
  double vio_init_bg_weight = 1e2;
  // Basalt-Win: explicit initial bias priors (1/sigma^2). When > 0 they replace the
  // Basalt weights above (which Basalt applies to the state blocks 9..11 = gyro bias
  // and 12..14 = accelerometer bias, i.e. with swapped names).
  double vio_init_gyro_bias_weight = 0;
  double vio_init_accel_bias_weight = 0;
  bool vio_marg_lost_landmarks = true;
  double vio_kf_marg_feature_ratio = 0.1;
  bool vio_debug = false;

  // Basalt-Win extension (not used for EuRoC): optional aiding factors on the full
  // states of the window - barometric height, airspeed and course of a fixed-wing aircraft
  double aid_wind_n = 0, aid_wind_e = 0;  // m/s, air mass velocity (for the airspeed factor)
  double aid_speed_sigma = 1.118;         // m/s, airspeed (1 m/s sensor + 0.5 m/s wind)
  double aid_course_sigma_deg = 5;        // deg
  double aid_baro_sigma = 2;              // m
  double aid_min_speed = 15;              // m/s: airspeed/course only in flight
  double aid_course_max_innov_deg = 25;   // deg: course innovation gate
  double aid_huber = 2.5;                 // Huber threshold in sigmas

  bool load(const std::string& path);  // optional key: value overrides
};

struct Calibration {
  int num_cams = 2;  // 1 = monocular, 2 = stereo
  SE3 T_i_c[2];
  Camera cam[2];
  double gravity = 9.81;            // m/s^2 (magnitude, world z up)
  int64_t cam_time_offset_ns = 0;   // true exposure time = image timestamp + offset
  // optional IMU calibration of the raw data (identity by default):
  //   gyro  = Mw^-1 (Rwa^T w_raw - Aw a_raw)   accel = Ma^-1 a_raw
  Mat3 imu_Mw = Mat3::Identity(), imu_Aw = Mat3::Zero(), imu_Rwa = Mat3::Identity(),
       imu_Ma = Mat3::Identity();
  Vec3 calib_accel_bias = Vec3::Zero();
  Vec3 calib_gyro_bias = Vec3::Zero();
  Vec3 accel_noise_std = Vec3::Constant(0.016);
  Vec3 gyro_noise_std = Vec3::Constant(0.000282);
  Vec3 accel_bias_std = Vec3::Constant(0.001);
  Vec3 gyro_bias_std = Vec3::Constant(0.0001);
  double imu_update_rate = 200.0;

  Vec3 discreteAccelNoiseStd() const { return accel_noise_std * std::sqrt(imu_update_rate); }
  Vec3 discreteGyroNoiseStd() const { return gyro_noise_std * std::sqrt(imu_update_rate); }

  bool load(const std::string& path);
  // applies the IMU calibration matrices and subtracts the static biases
  void calibrateImu(Vec3& gyro, Vec3& accel) const {
    const Vec3 w = imu_Mw.inverse() * (imu_Rwa.transpose() * gyro - imu_Aw * accel);
    const Vec3 a = imu_Ma.inverse() * accel;
    gyro = w - calib_gyro_bias;
    accel = a - calib_accel_bias;
  }
};

// "key: v1 v2 v3" text format; '#' starts a comment.
std::map<std::string, std::vector<std::string>> readKeyValueFile(const std::string& path);

}  // namespace bw
