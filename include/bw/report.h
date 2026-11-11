// Basalt-Win: HTML + Markdown evaluation report with inline SVG plots
#pragma once
#include <string>
#include <vector>

#include "bw/eval.h"

namespace bw {

struct Series {
  std::vector<double> x, y;
  std::string color, label;
  double width = 1.6;
  bool dashed = false;
};
// SVG line plot (theme colors via CSS variables, e.g. "var(--est)")
std::string svgPlot(const std::vector<Series>& series, const std::string& title,
                    const std::string& xlabel, const std::string& ylabel, bool equal_aspect,
                    int W = 640, int H = 420);
std::string fmt(double v, int prec = 3);
// shared page style of the reports
std::string reportCss();

struct RunInfo {
  std::string dataset_path;
  std::string sequence_name;
  std::string trajectory_file;
  size_t num_frames = 0;
  size_t num_keyframes = 0;
  double wall_time_s = 0;
  std::vector<double> frame_t_s;   // frame time relative to start
  std::vector<double> of_ms;       // optical flow time per frame
  std::vector<double> vio_ms;      // estimator time per frame
  std::vector<int> num_landmarks;  // landmarks in window per frame
  std::vector<int> num_tracked;    // tracked points cam0 per frame
  std::vector<Vec3> bias_gyro, bias_accel;
  std::string build_info;
};

void writeReport(const std::string& out_dir, const RunInfo& info, const EvalResult& ev,
                 const std::vector<GtPose>& est_traj);

}  // namespace bw
