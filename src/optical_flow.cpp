#include "bw/optical_flow.h"

#include <cmath>
#include <limits>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

namespace bw {

namespace {

constexpr int kPatternSize = 52;
constexpr double kPattern52[kPatternSize][2] = {
    {-3, 7},  {-1, 7},  {1, 7},   {3, 7},                                  //
    {-5, 5},  {-3, 5},  {-1, 5},  {1, 5},   {3, 5},  {5, 5},               //
    {-7, 3},  {-5, 3},  {-3, 3},  {-1, 3},  {1, 3},  {3, 3},  {5, 3},  {7, 3},
    {-7, 1},  {-5, 1},  {-3, 1},  {-1, 1},  {1, 1},  {3, 1},  {5, 1},  {7, 1},
    {-7, -1}, {-5, -1}, {-3, -1}, {-1, -1}, {1, -1}, {3, -1}, {5, -1}, {7, -1},
    {-7, -3}, {-5, -3}, {-3, -3}, {-1, -3}, {1, -3}, {3, -3}, {5, -3}, {7, -3},
    {-5, -5}, {-3, -5}, {-1, -5}, {1, -5},  {3, -5}, {5, -5},              //
    {-3, -7}, {-1, -7}, {1, -7},  {3, -7}};

constexpr int kEdgeThreshold = 19;

using VecP = Eigen::Matrix<double, kPatternSize, 1>;
using MatP3 = Eigen::Matrix<double, kPatternSize, 3>;
using Mat3P = Eigen::Matrix<double, 3, kPatternSize>;
using Mat2P = Eigen::Matrix<double, 2, kPatternSize>;

inline bool inBounds(const cv::Mat& img, const Vec2& p, double border) {
  return p.x() >= border && p.y() >= border && p.x() < img.cols - border - 1 &&
         p.y() < img.rows - border - 1;
}

inline double interp(const cv::Mat& img, double x, double y) {
  const int ix = static_cast<int>(x), iy = static_cast<int>(y);
  const double dx = x - ix, dy = y - iy;
  const float* r0 = img.ptr<float>(iy) + ix;
  const float* r1 = img.ptr<float>(iy + 1) + ix;
  return (1 - dy) * ((1 - dx) * r0[0] + dx * r0[1]) + dy * ((1 - dx) * r1[0] + dx * r1[1]);
}

// value, d/dx, d/dy (central differences of bilinear interpolation)
inline Vec3 interpGrad(const cv::Mat& img, const Vec2& p) {
  const double x = p.x(), y = p.y();
  Vec3 r;
  r[0] = interp(img, x, y);
  r[1] = 0.5 * (interp(img, x + 1, y) - interp(img, x - 1, y));
  r[2] = 0.5 * (interp(img, x, y + 1) - interp(img, x, y - 1));
  return r;
}

struct Patch {
  VecP data;
  Mat3P H_inv_JT;
  bool valid = false;

  Patch(const cv::Mat& img, const Vec2& pos, const Mat2P& pattern) {
    int n_valid = 0;
    double sum = 0;
    Vec3 grad_sum = Vec3::Zero();
    MatP3 J;
    Eigen::Matrix<double, 2, 3> Jw;
    Jw.setZero();
    Jw(0, 0) = Jw(1, 1) = 1;
    for (int i = 0; i < kPatternSize; i++) {
      Vec2 p = pos + pattern.col(i);
      Jw(0, 2) = -pattern(1, i);
      Jw(1, 2) = pattern(0, i);
      if (inBounds(img, p, 2)) {
        Vec3 vg = interpGrad(img, p);
        data[i] = vg[0];
        sum += vg[0];
        J.row(i) = vg.tail<2>().transpose() * Jw;
        grad_sum += J.row(i).transpose();
        n_valid++;
      } else {
        data[i] = -1;
      }
    }
    if (n_valid == 0 || sum < std::numeric_limits<double>::epsilon()) return;
    const double mean_inv = n_valid / sum;
    for (int i = 0; i < kPatternSize; i++) {
      if (data[i] >= 0) {
        J.row(i) -= grad_sum.transpose() * data[i] / sum;
        data[i] *= mean_inv;
      } else {
        J.row(i).setZero();
      }
    }
    J *= mean_inv;
    Mat3 H = J.transpose() * J;
    Mat3 Hinv = Mat3::Identity();
    H.ldlt().solveInPlace(Hinv);
    H_inv_JT = Hinv * J.transpose();
    valid = H_inv_JT.allFinite() && data.allFinite();
  }

  bool residual(const cv::Mat& img, const Mat2P& tp, VecP& res) const {
    double sum = 0;
    int n_valid = 0;
    for (int i = 0; i < kPatternSize; i++) {
      if (inBounds(img, tp.col(i), 2)) {
        res[i] = interp(img, tp(0, i), tp(1, i));
        sum += res[i];
        n_valid++;
      } else {
        res[i] = -1;
      }
    }
    if (sum < std::numeric_limits<double>::epsilon()) {
      res.setZero();
      return false;
    }
    int n_res = 0;
    for (int i = 0; i < kPatternSize; i++) {
      if (res[i] >= 0 && data[i] >= 0) {
        res[i] = n_valid * res[i] / sum - data[i];
        n_res++;
      } else {
        res[i] = 0;
      }
    }
    return n_res > kPatternSize / 2;
  }
};

// SE(2) exponential: (tx, ty, theta) -> (R, t)
inline void se2Exp(const Vec3& inc, Mat2& R, Vec2& t) {
  const double th = inc[2];
  const double c = std::cos(th), s = std::sin(th);
  R << c, -s, s, c;
  double a, b;
  if (std::abs(th) < 1e-10) {
    a = 1 - th * th / 6.0;
    b = 0.5 * th;
  } else {
    a = s / th;
    b = (1 - c) / th;
  }
  Mat2 V;
  V << a, -b, b, a;
  t = V * inc.head<2>();
}

}  // namespace

void ImagePyramid::build(const cv::Mat& gray8, int levels) {
  lvl.resize(levels + 1);
  gray8.convertTo(lvl[0], CV_32F);
  for (int i = 1; i <= levels; i++) cv::pyrDown(lvl[i - 1], lvl[i]);
}

FrameToFrameOpticalFlow::FrameToFrameOpticalFlow(const VioConfig& config,
                                                 const Calibration& calib)
    : config_(config), calib_(calib) {
  pattern_.resize(2, kPatternSize);
  for (int i = 0; i < kPatternSize; i++) {
    pattern_(0, i) = kPattern52[i][0] * config.optical_flow_pattern_scale;
    pattern_(1, i) = kPattern52[i][1] * config.optical_flow_pattern_scale;
  }
  E_.setZero();
  if (calib.num_cams > 1) {
    const SE3 T_0_1 = calib.T_i_c[0].inverse() * calib.T_i_c[1];
    E_ = hat(T_0_1.t.normalized()) * T_0_1.R;
  }
}

OpticalFlowResult::Ptr FrameToFrameOpticalFlow::processFrame(int64_t t_ns, const cv::Mat& img0,
                                                             const cv::Mat& img1,
                                                             const Mat3* R_c1_c0) {
  const int ncam = img1.empty() ? 1 : 2;
  old_pyramid_ = pyramid_;
  pyramid_ = std::make_shared<std::vector<ImagePyramid>>(ncam);
  const cv::Mat* imgs[2] = {&img0, &img1};
#pragma omp parallel for
  for (int i = 0; i < ncam; i++) (*pyramid_)[i].build(*imgs[i], config_.optical_flow_levels);

  auto new_transforms = std::make_shared<OpticalFlowResult>();
  new_transforms->t_ns = t_ns;
  if (transforms_ && old_pyramid_ && static_cast<int>(old_pyramid_->size()) == ncam) {
    for (int i = 0; i < ncam; i++) {
      const Mat3* R = (config_.optical_flow_imu_rotation_prediction && R_c1_c0) ? &R_c1_c0[i] : nullptr;
      trackPoints((*old_pyramid_)[i], (*pyramid_)[i], transforms_->observations[i],
                  new_transforms->observations[i], i, R);
    }
  }
  transforms_ = new_transforms;
  addPoints();
  if (ncam > 1) filterPoints();

  // return a copy (the tracker keeps its own state)
  return std::make_shared<OpticalFlowResult>(*transforms_);
}

void FrameToFrameOpticalFlow::trackPoints(const ImagePyramid& pyr_1, const ImagePyramid& pyr_2,
                                          const std::map<KeypointId, Vec2>& in,
                                          std::map<KeypointId, Vec2>& out, int cam,
                                          const Mat3* R_21) const {
  std::vector<KeypointId> ids;
  std::vector<Vec2> init;
  ids.reserve(in.size());
  init.reserve(in.size());
  for (const auto& kv : in) {
    ids.push_back(kv.first);
    init.push_back(kv.second);
  }
  // rotation-only prediction of a pixel (points far away compared to the translation)
  auto rotate = [&](const Vec2& p, const Mat3& R, Vec2& q) {
    Vec3 b;
    if (!calib_.cam[cam].unproject(p, b)) return false;
    return calib_.cam[cam].project(R * b, q) && calib_.cam[cam].inImage(q, 20);
  };
  std::vector<Vec2> result(ids.size());
  std::vector<char> ok(ids.size(), 0);
  const int n = static_cast<int>(ids.size());
#pragma omp parallel for schedule(dynamic, 8)
  for (int r = 0; r < n; r++) {
    Vec2 p2 = init[r];
    if (R_21 && cam >= 0 && !rotate(init[r], *R_21, p2)) continue;
    if (!trackPoint(pyr_1, pyr_2, init[r], p2)) continue;
    Vec2 p1_rec = p2;
    if (R_21 && cam >= 0 && !rotate(p2, R_21->transpose(), p1_rec)) continue;
    if (!trackPoint(pyr_2, pyr_1, p2, p1_rec)) continue;
    if ((init[r] - p1_rec).squaredNorm() < config_.optical_flow_max_recovered_dist2) {
      result[r] = p2;
      ok[r] = 1;
    }
  }
  out.clear();
  for (int r = 0; r < n; r++)
    if (ok[r]) out[ids[r]] = result[r];
}

bool FrameToFrameOpticalFlow::trackPoint(const ImagePyramid& old_pyr, const ImagePyramid& pyr,
                                         const Vec2& old_pos, Vec2& pos) const {
  // pos holds the initial guess on input
  Mat2 A = Mat2::Identity();
  Vec2 t = pos;
  bool valid = true;
  for (int level = config_.optical_flow_levels; level >= 0 && valid; level--) {
    const double scale = static_cast<double>(1 << level);
    t /= scale;
    Patch p(old_pyr.lvl[level], old_pos / scale, pattern_);
    valid &= p.valid;
    if (valid) {
      const cv::Mat& img = pyr.lvl[level];
      for (int it = 0; valid && it < config_.optical_flow_max_iterations; it++) {
        Mat2P tp = A * pattern_;
        tp.colwise() += t;
        VecP res;
        valid &= p.residual(img, tp, res);
        if (!valid) break;
        const Vec3 inc = -p.H_inv_JT * res;
        valid &= inc.allFinite() && inc.lpNorm<Eigen::Infinity>() < 1e6;
        if (!valid) break;
        Mat2 Ri;
        Vec2 ti;
        se2Exp(inc, Ri, ti);
        t = A * ti + t;
        A = A * Ri;
        valid &= inBounds(img, t, 2);
      }
    }
    t *= scale;
  }
  pos = t;
  return valid;
}

void FrameToFrameOpticalFlow::addPoints() {
  const cv::Mat& img = (*pyramid_)[0].lvl[0];
  const int PS = config_.optical_flow_detection_grid_size;
  const int w = img.cols, h = img.rows;
  const int x_start = (w % PS) / 2, x_stop = x_start + PS * (w / PS - 1);
  const int y_start = (h % PS) / 2, y_stop = y_start + PS * (h / PS - 1);

  Eigen::MatrixXi cells = Eigen::MatrixXi::Zero(h / PS + 1, w / PS + 1);
  for (const auto& kv : transforms_->observations[0]) {
    const Vec2& p = kv.second;
    if (p.x() >= x_start && p.y() >= y_start && p.x() < x_stop + PS && p.y() < y_stop + PS) {
      int cx = static_cast<int>((p.x() - x_start) / PS);
      int cy = static_cast<int>((p.y() - y_start) / PS);
      cells(cy, cx) += 1;
    }
  }

  cv::Mat img8;
  img.convertTo(img8, CV_8U);
  std::map<KeypointId, Vec2> new_poses0, new_poses1;

  for (int x = x_start; x <= x_stop; x += PS) {
    for (int y = y_start; y <= y_stop; y += PS) {
      if (cells((y - y_start) / PS, (x - x_start) / PS) > 0) continue;
      cv::Mat sub = img8(cv::Rect(x, y, PS, PS)).clone();
      int points_added = 0;
      int threshold = 40;
      while (points_added < 1 && threshold >= 5) {
        std::vector<cv::KeyPoint> pts;
        cv::FAST(sub, pts, threshold);
        std::sort(pts.begin(), pts.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
          return a.response > b.response;
        });
        for (size_t i = 0; i < pts.size() && points_added < 1; i++) {
          const double px = x + pts[i].pt.x, py = y + pts[i].pt.y;
          if (px >= kEdgeThreshold && py >= kEdgeThreshold && px < w - kEdgeThreshold &&
              py < h - kEdgeThreshold) {
            transforms_->observations[0][last_keypoint_id_] = Vec2(px, py);
            new_poses0[last_keypoint_id_] = Vec2(px, py);
            last_keypoint_id_++;
            points_added++;
          }
        }
        threshold /= 2;
      }
    }
  }

  if (pyramid_->size() > 1) {
    trackPoints((*pyramid_)[0], (*pyramid_)[1], new_poses0, new_poses1);
    for (const auto& kv : new_poses1) transforms_->observations[1].emplace(kv);
  }
}

void FrameToFrameOpticalFlow::filterPoints() {
  std::vector<KeypointId> to_remove;
  for (const auto& kv : transforms_->observations[1]) {
    auto it = transforms_->observations[0].find(kv.first);
    if (it == transforms_->observations[0].end()) continue;
    Vec3 b0, b1;
    bool v0 = calib_.cam[0].unproject(it->second, b0);
    bool v1 = calib_.cam[1].unproject(kv.second, b1);
    if (!v0 || !v1 || std::abs(b0.transpose() * E_ * b1) > config_.optical_flow_epipolar_error)
      to_remove.push_back(kv.first);
  }
  for (KeypointId id : to_remove) transforms_->observations[1].erase(id);
}

}  // namespace bw
