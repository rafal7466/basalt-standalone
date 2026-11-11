#include "bw/config.h"

#include <fstream>
#include <iostream>
#include <sstream>

namespace bw {

std::map<std::string, std::vector<std::string>> readKeyValueFile(const std::string& path) {
  std::map<std::string, std::vector<std::string>> kv;
  std::ifstream f(path);
  if (!f) return kv;
  std::string line;
  while (std::getline(f, line)) {
    auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    auto colon = line.find(':');
    if (colon == std::string::npos) continue;
    std::string key = line.substr(0, colon);
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t\r") + 1);
    std::stringstream ss(line.substr(colon + 1));
    std::vector<std::string> vals;
    std::string v;
    while (ss >> v) vals.push_back(v);
    if (!key.empty()) kv[key] = vals;
  }
  return kv;
}

#define BW_READ_D(name) \
  if (kv.count(#name)) name = std::stod(kv[#name].at(0));
#define BW_READ_I(name) \
  if (kv.count(#name)) name = std::stoi(kv[#name].at(0));
#define BW_READ_B(name) \
  if (kv.count(#name)) name = (kv[#name].at(0) == "true" || kv[#name].at(0) == "1");

bool VioConfig::load(const std::string& path) {
  auto kv = readKeyValueFile(path);
  if (kv.empty()) return false;
  BW_READ_I(optical_flow_detection_grid_size);
  BW_READ_D(optical_flow_max_recovered_dist2);
  BW_READ_I(optical_flow_max_iterations);
  BW_READ_D(optical_flow_epipolar_error);
  BW_READ_I(optical_flow_levels);
  BW_READ_D(optical_flow_pattern_scale);
  BW_READ_B(optical_flow_imu_rotation_prediction);
  BW_READ_I(vio_max_states);
  BW_READ_I(vio_max_kfs);
  BW_READ_I(vio_min_frames_after_kf);
  BW_READ_D(vio_new_kf_keypoints_thresh);
  BW_READ_D(vio_obs_std_dev);
  BW_READ_D(vio_obs_huber_thresh);
  BW_READ_D(vio_min_triangulation_dist);
  BW_READ_D(vio_outlier_threshold);
  BW_READ_I(vio_filter_iteration);
  BW_READ_I(vio_max_iterations);
  BW_READ_D(vio_lm_lambda_initial);
  BW_READ_D(vio_lm_lambda_min);
  BW_READ_D(vio_lm_lambda_max);
  BW_READ_D(vio_init_pose_weight);
  BW_READ_D(vio_init_ba_weight);
  BW_READ_D(vio_init_bg_weight);
  BW_READ_D(vio_init_gyro_bias_weight);
  BW_READ_D(vio_init_accel_bias_weight);
  BW_READ_B(vio_marg_lost_landmarks);
  BW_READ_D(vio_kf_marg_feature_ratio);
  BW_READ_B(vio_debug);
  BW_READ_D(aid_wind_n);
  BW_READ_D(aid_wind_e);
  BW_READ_D(aid_speed_sigma);
  BW_READ_D(aid_course_sigma_deg);
  BW_READ_D(aid_baro_sigma);
  BW_READ_D(aid_min_speed);
  BW_READ_D(aid_course_max_innov_deg);
  BW_READ_D(aid_huber);
  return true;
}

static Vec3 vec3(const std::vector<std::string>& v) {
  return Vec3(std::stod(v.at(0)), std::stod(v.at(1)), std::stod(v.at(2)));
}

bool Calibration::load(const std::string& path) {
  auto kv = readKeyValueFile(path);
  if (kv.empty()) {
    std::cerr << "Cannot read calibration " << path << std::endl;
    return false;
  }
  auto mat3 = [&](const std::string& key, Mat3& M) {
    if (!kv.count(key)) return false;
    const auto& v = kv.at(key);
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) M(r, c) = std::stod(v.at(3 * r + c));
    return true;
  };
  if (kv.count("num_cams")) num_cams = std::stoi(kv["num_cams"].at(0));
  for (int i = 0; i < num_cams; i++) {
    const std::string s = std::to_string(i);
    if (kv.count("T_imu_cam" + s)) {
      const auto& T = kv.at("T_imu_cam" + s);  // px py pz qx qy qz qw
      Quat q(std::stod(T.at(6)), std::stod(T.at(3)), std::stod(T.at(4)), std::stod(T.at(5)));
      T_i_c[i] = SE3(q, Vec3(std::stod(T[0]), std::stod(T[1]), std::stod(T[2])));
    } else {
      // rotation matrix camera -> IMU (row major) + camera position in the IMU frame
      Mat3 R;
      if (!mat3("R_imu_cam" + s, R) || !kv.count("p_imu_cam" + s)) {
        std::cerr << "missing extrinsics of cam" << s << std::endl;
        return false;
      }
      T_i_c[i] = SE3(Quat(R).normalized(), vec3(kv["p_imu_cam" + s]));
    }
    std::string type = kv.count("camera_type_cam" + s) ? kv["camera_type_cam" + s].at(0) : "ds";
    const auto& I = kv.at("intrinsics_cam" + s);
    cam[i].fx = std::stod(I.at(0));
    cam[i].fy = std::stod(I.at(1));
    cam[i].cx = std::stod(I.at(2));
    cam[i].cy = std::stod(I.at(3));
    if (type == "pinhole-radtan") {
      cam[i].type = Camera::Type::PinholeRadtan;  // fx fy cx cy k1 k2 p1 p2
      cam[i].k1 = std::stod(I.at(4));
      cam[i].k2 = std::stod(I.at(5));
      cam[i].p1 = std::stod(I.at(6));
      cam[i].p2 = std::stod(I.at(7));
    } else {
      cam[i].type = Camera::Type::DoubleSphere;  // fx fy cx cy xi alpha
      cam[i].xi = std::stod(I.at(4));
      cam[i].alpha = std::stod(I.at(5));
    }
    if (kv.count("resolution")) {
      cam[i].width = std::stoi(kv["resolution"].at(0));
      cam[i].height = std::stoi(kv["resolution"].at(1));
    }
  }
  if (kv.count("gravity")) gravity = std::stod(kv["gravity"].at(0));
  if (kv.count("cam_time_offset_ns")) cam_time_offset_ns = std::stoll(kv["cam_time_offset_ns"].at(0));
  mat3("imu_Mw", imu_Mw);
  mat3("imu_Aw", imu_Aw);
  mat3("imu_Rwa", imu_Rwa);
  mat3("imu_Ma", imu_Ma);
  if (kv.count("calib_accel_bias")) calib_accel_bias = vec3(kv["calib_accel_bias"]);
  if (kv.count("calib_gyro_bias")) calib_gyro_bias = vec3(kv["calib_gyro_bias"]);
  if (kv.count("accel_noise_std")) accel_noise_std = vec3(kv["accel_noise_std"]);
  if (kv.count("gyro_noise_std")) gyro_noise_std = vec3(kv["gyro_noise_std"]);
  if (kv.count("accel_bias_std")) accel_bias_std = vec3(kv["accel_bias_std"]);
  if (kv.count("gyro_bias_std")) gyro_bias_std = vec3(kv["gyro_bias_std"]);
  if (kv.count("imu_update_rate")) imu_update_rate = std::stod(kv["imu_update_rate"].at(0));
  return true;
}

}  // namespace bw
